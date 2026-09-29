"""ROS2 桥接层：封装订阅/发布/Action/服务，通过 Qt 信号与 GUI 通信。"""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.serialization import deserialize_message
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import BatteryState, LaserScan, Range, PointCloud2, Imu
from sensor_msgs_py.point_cloud2 import read_points
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from PyQt5.QtCore import QObject, pyqtSignal


# 超声话题名 → 机器人系方位（0=正前，正值=向左），与 sensor_panel.SENSOR_SPECS 一致。
# 桥接层不反向依赖 UI 模块，故在此单独维护一份。
SONAR_DIR = {
    'sonar_fl': '前左', 'sonar_fr': '前右',
    'sonar_sl': '左', 'sonar_sr': '右',
    'sonar_rl': '后左', 'sonar_rr': '后右',
}

# 方位名 → 展示用方位词（用于"停车原因 / 下一步建议"文案）
DIR_DISPLAY = {
    '前': '前方', '左前': '左前方', '左': '左侧', '左后': '左后方',
    '后': '后方', '右后': '右后方', '右': '右侧', '右前': '右前方',
}


class GuiSignals(QObject):
    """跨线程信号集合（ROS 回调 -> GUI）。"""
    map_received = pyqtSignal(object)
    scan_received = pyqtSignal(object)
    path_received = pyqtSignal(object)
    pose_updated = pyqtSignal(float, float, float)   # x, y, yaw
    cmd_vel_updated = pyqtSignal(float, float)        # linear, angular
    nav_state = pyqtSignal(str)                       # 导航状态文本
    goal_finished = pyqtSignal(str)                   # 'succeeded' | 'cancelled' | 'failed' | 'rejected'
    battery_updated = pyqtSignal(float, float, int)   # voltage, percentage(0-1), power_status
    chassis_updated = pyqtSignal(str)                 # 底盘状态文本（真机）
    network_updated = pyqtSignal(str)                 # 网络状态文本（预留，实机）
    log = pyqtSignal(str)
    # 传感器监控
    sonar_updated = pyqtSignal(dict)                  # {name: {'range': m, 'active': bool, 'obstacle_dist': m}}
    camera_cloud_updated = pyqtSignal(dict)           # {name: {'min_dist': m, 'active': bool}}
    imu_updated = pyqtSignal(float, float)            # angular_z(°/s), linear_accel_x
    nav_decision = pyqtSignal(str)                    # 导航决策文本
    avoid_decision = pyqtSignal(str)                  # 避障决策文本
    stop_reason = pyqtSignal(str)                     # 停车原因文本
    next_action = pyqtSignal(str)                     # 下一步建议文本
    route_status = pyqtSignal(dict)                   # route_follower 巡线进度回报


class RosBridge(Node):
    def __init__(self, use_sim_time: bool = True):
        # 关键：必须与发布 TF 的节点（robot_state_publisher）使用同一时钟。
        # 仿真走 /clock（仿真时间），实机走系统时间，否则 TF 时间戳不一致
        # 会持续刷 TF_OLD_DATA 警告并导致位姿查询异常。
        super().__init__('dt01_nav_gui', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, bool(use_sim_time)),
        ])
        self.signals = GuiSignals()

        # ---- 订阅 ----
        # 全部用 raw=True + Python 侧反序列化 + try/except：绕过 rclpy 的 C++
        # 反序列化路径（其 pybind11 会对个别消息抛
        #   RuntimeError: Unable to convert call argument to Python object，
        # 一旦抛出会污染 executor，导致后续所有回调不再处理 → 面板全 0Hz）。
        def _subscribe_raw(msg_type, topic, handler, qos):
            def _cb(raw):
                try:
                    msg = deserialize_message(raw, msg_type)
                except Exception as e:
                    self.get_logger().error(
                        f'反序列化失败 {topic} ({type(e).__name__})，已跳过该帧')
                    return
                handler(msg)
            self.create_subscription(msg_type, topic, _cb, qos, raw=True)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        _subscribe_raw(OccupancyGrid, '/map', self._on_map, map_qos)
        _subscribe_raw(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        _subscribe_raw(Path, '/plan', self._on_path, 10)
        _subscribe_raw(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        # 位姿：优先用 AMCL 直接发布的 /amcl_pose，TF 作为兜底
        _subscribe_raw(PoseWithCovarianceStamped, '/amcl_pose',
                       self._on_amcl_pose, 10)
        _subscribe_raw(Odometry, '/odom', self._on_odom, 10)
        # 传感器：超声 ×6
        self._sonar_topics = [
            '/sonar_fl', '/sonar_fr', '/sonar_sl',
            '/sonar_sr', '/sonar_rl', '/sonar_rr']
        self._sonar_state = {name: {'range': float('inf'),
                                    'active': False,
                                    'obstacle_dist': float('inf')}
                             for name in self._sonar_topics}
        for topic in self._sonar_topics:
            # 超声由 Gazebo ray sensor 以 sensor_data(best-effort) QoS 发布，
            # 订阅端必须同样用 best-effort，否则 reliable 订阅收不到任何数据
            _subscribe_raw(Range, topic,
                           lambda msg, t=topic: self._on_sonar(t, msg),
                           qos_profile_sensor_data)
        # 传感器：RGB-D 点云 —— 订阅【永久禁用】。
        # 根因：gazebo_ros_camera 发布的 PointCloud2 字段格式，本环境 rclpy 无法
        # 反序列化，任何分辨率都会抛
        #   RuntimeError: Unable to convert call argument to Python object
        # 每 20ms 打断一次 spin_once，把 /scan、/sonar_*、/imu 全部饿死（面板全 0Hz）。
        # 相机玻璃检测属锦上添花，等换成 C++ 桥接或 rosbag 回放再恢复。
        # 恢复时把下面订阅循环解除注释即可（逻辑键 camera_l/camera_r 的其余代码已就绪）：
        self._camera_topics = {
            'camera_l': '/camera_l_sensor/points',
            'camera_r': '/camera_r_sensor/points',
        }
        self._camera_state = {name: {'min_dist': float('inf'),
                                     'active': False,
                                     'points': 0}
                              for name in self._camera_topics}
        self._camera_last_proc = {name: 0.0 for name in self._camera_topics}
        self._camera_max_points = 5000
        # for name, topic in self._camera_topics.items():
        #     self.create_subscription(PointCloud2, topic,
        #                              lambda msg, n=name: self._on_camera_cloud(n, msg),
        #                              qos_profile_sensor_data)
        # 传感器：IMU（两种车型都发布到 /imu/data，而不是 /imu）
        _subscribe_raw(Imu, '/imu/data', self._on_imu, qos_profile_sensor_data)

        # ---- 遥测（真机：dt01_driver 发布；仿真：无发布方时显示"未接入"）----
        _subscribe_raw(BatteryState, '/battery_state', self._on_battery, 10)
        _subscribe_raw(String, '/dt01/chassis_status', self._on_chassis, 10)
        _subscribe_raw(String, '/dt01/network', self._on_network, 10)

        # ---- 发布 ----
        # 遥控话题在真机上必须是 /cmd_vel_teleop：
        # 真机的 /cmd_vel 由 safety_mux 独占输出，GUI 直接发 /cmd_vel 会
        # 绕过安全仲裁（急停/防撞/激光急停全失效），还会和 Nav2 抢底盘。
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        # 巡线（route_follower 节点跑在工控机上）：GUI 只发路线/心跳/急停
        self.route_goal_pub = self.create_publisher(Path, '/dt01/route/goal', 10)
        self.route_ctrl_pub = self.create_publisher(String, '/dt01/route/ctrl', 10)
        self.route_hb_pub = self.create_publisher(String, '/dt01/route/heartbeat', 10)
        _subscribe_raw(String, '/dt01/route/status', self._on_route_status, 10)

        # ---- 链路心跳（真机：判断工控机数据有没有真的过来）----
        self._last_seen = {'odom': 0.0, 'scan': 0.0, 'chassis': 0.0,
                           'battery': 0.0, 'imu': 0.0}

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Action：导航 ----
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        self._navigating = False

        # ---- 传感器/决策缓存 ----
        self._latest_scan = None
        self._latest_cmd_vel = (0.0, 0.0)
        # 底盘真实速度（来自 /odom），用于判断"车是否真的在动"
        self._odom_vel = (0.0, 0.0)
        self._last_odom_move = 0.0
        self._latest_pose = None
        self._amcl_pose_count = 0
        self._tf_fail_count = 0
        self._last_nav_text = None
        self._last_avoid_text = None
        self._last_stop_text = None
        self._last_next_text = None
        self._last_goal_text = None
        self._last_feedback_log = 0.0
        self._last_feedback_dist = None

        # ---- 服务：保存地图（slam_toolbox）----
        self._save_map_client = None
        try:
            from slam_toolbox.srv import SaveMap
            self._SaveMap = SaveMap
            self._save_map_client = self.create_client(
                SaveMap, '/slam_toolbox/save_map')
        except ImportError:
            self._SaveMap = None

    # ================= 订阅回调 =================
    def _on_map(self, msg: OccupancyGrid):
        self.signals.map_received.emit(msg)

    def _on_scan(self, msg: LaserScan):
        self._last_seen['scan'] = time.monotonic()
        self._latest_scan = msg
        self.signals.scan_received.emit(msg)
        self._emit_decisions()

    def _on_path(self, msg: Path):
        self.signals.path_received.emit(msg)

    def _on_cmd_vel(self, msg: Twist):
        self._latest_cmd_vel = (msg.linear.x, msg.angular.z)
        self.signals.cmd_vel_updated.emit(msg.linear.x, msg.angular.z)
        self._emit_decisions()

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        """AMCL 直接发布的 map 系位姿，比 TF 查询更稳定。"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._latest_pose = (p.x, p.y, yaw)
        self._amcl_pose_count += 1
        self.signals.pose_updated.emit(p.x, p.y, yaw)

    def _on_odom(self, msg: Odometry):
        """里程计位姿：仅在 /amcl_pose 尚未到达时兜底。

        同时记录底盘真实速度 —— 这是判断"车到底动没动"最可靠的依据。
        （/cmd_vel 是指令，车被急停/堵住时指令非零但实际没动；
         反过来底盘也可能在阻尼滑行。）
        """
        self._last_seen['odom'] = time.monotonic()
        self._odom_vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self._last_odom_move = time.monotonic()
        if self._amcl_pose_count == 0:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self._latest_pose = (p.x, p.y, yaw)
            self.signals.pose_updated.emit(p.x, p.y, yaw)

    def _on_sonar(self, topic, msg: Range):
        """超声 Range 回调：记录量程、是否有效、最近障碍距离。"""
        r = float(msg.range)
        # Range 规范：超出 [min_range, max_range] 视为无有效回波
        active = msg.min_range <= r <= msg.max_range
        self._sonar_state[topic] = {
            'range': float(msg.max_range),
            'min_range': float(msg.min_range),
            'active': bool(active),
            'obstacle_dist': r if active else float('inf'),
        }
        self.signals.sonar_updated.emit({topic: self._sonar_state[topic]})
        self._emit_decisions()

    def _on_camera_cloud(self, name, msg: PointCloud2):
        """RGB-D 点云回调：取前方扇区最近点作为障碍距离。

        name 为逻辑键 'camera_l' / 'camera_r'（对应 /camera_*_sensor/points）。

        点云 320×240@30Hz，Python 全量处理会吃掉大量 CPU。
        这里做两件事：① 5Hz 降频；② 只处理前 self._camera_max_points 个
        有效点，足以估计最近障碍距离。
        """
        now = time.monotonic()
        if now - self._camera_last_proc[name] < 0.18:
            return
        self._camera_last_proc[name] = now

        min_dist = float('inf')
        count = 0
        try:
            for x, y, z in read_points(msg, field_names=('x', 'y', 'z'),
                                       skip_nans=True):
                count += 1
                if count > self._camera_max_points:
                    break
                # 仅关注 base_link 前方 ±50°、高度 0.05~0.8m 的点（忽略地面/天花板）
                if not (0.05 <= z <= 0.8):
                    continue
                if abs(math.atan2(y, x)) > math.radians(50):
                    continue
                d = math.hypot(x, y)
                if d < min_dist:
                    min_dist = d
        except Exception:
            pass
        self._camera_state[name] = {
            'active': count > 0,
            'min_dist': min_dist,
            'points': count,
        }
        self.signals.camera_cloud_updated.emit(
            {name: self._camera_state[name]})
        self._emit_decisions()

    def _on_imu(self, msg: Imu):
        """IMU 回调：偏航角速度与纵向加速度（不测距，仅状态显示）。"""
        wz = math.degrees(msg.angular_velocity.z)
        ax = msg.linear_acceleration.x
        self.signals.imu_updated.emit(wz, ax)

    # ================= 遥测回调 =================
    def _on_battery(self, msg: BatteryState):
        self._last_seen['battery'] = time.monotonic()
        self.signals.battery_updated.emit(
            msg.voltage, msg.percentage, msg.power_supply_status)

    def _on_chassis(self, msg: String):
        self._last_seen['chassis'] = time.monotonic()
        self.signals.chassis_updated.emit(msg.data)

    def _on_network(self, msg: String):
        self.signals.network_updated.emit(msg.data)

    # ================= 真机链路状态 =================
    def get_link_status(self, timeout=3.0):
        """判断工控机的数据有没有真的过来。

        真机最常见的假象：ros2 topic list 里能看到话题名，但一个数据都没有
        （驱动没起来 / DDS 只通了发现没通数据）。这里按"最近是否收到过数据"
        判断，比只看话题是否存在靠谱得多。

        返回 dict：{'connected': bool, 'odom': bool, 'scan': bool,
                    'chassis': bool, 'detail': str}
        """
        now = time.monotonic()
        st = {k: (now - v) < timeout for k, v in self._last_seen.items()}
        connected = st['odom'] and st['scan']
        missing = [k for k, v in st.items() if not v and k in ('odom', 'scan')]
        detail = '已连接' if connected else ('未收到：' + '/'.join(missing)
                                             if missing else '未连接')
        return {'connected': connected, 'odom': st['odom'], 'scan': st['scan'],
                'chassis': st['chassis'], 'detail': detail}

    # ================= 机器人位姿 =================
    def get_robot_pose(self):
        """返回机器人在 map 系的 (x, y, yaw)。

        优先级：/amcl_pose 话题 > TF(map->base_link) > /odom 兜底。
        原先仅依赖 TF：AMCL 的 map->odom 由话题发布，若 GUI 的 spin_once 频率
        不足或 TF 缓存未及时填充，查询会失败并一直返回 None（机器人图标卡住）。
        """
        if self._amcl_pose_count > 0 and self._latest_pose is not None:
            return self._latest_pose
        t = None
        for child in ('base_link', 'base_footprint'):
            try:
                t = self.tf_buffer.lookup_transform(
                    'map', child, rclpy.time.Time())
                break
            except Exception:
                continue
        if t is None:
            self._tf_fail_count += 1
            return self._latest_pose
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._latest_pose = (x, y, yaw)
        return (x, y, yaw)

    def get_robot_odom_pose(self):
        """查询 odom -> base_link（回退 base_footprint），返回 (x, y, yaw) 或 None。

        用于在 map 坐标系尚未建立时（AMCL 还没收到初始位姿）获取机器人的里程计位姿，
        作为「自动设定初始位姿」的依据。odom 由底盘驱动/仿真持续发布，几乎总是可用。
        """
        t = None
        for child in ('base_link', 'base_footprint'):
            try:
                t = self.tf_buffer.lookup_transform(
                    'odom', child, rclpy.time.Time())
                break
            except Exception:
                continue
        if t is None:
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (x, y, yaw)

    # ================= 手动控制 =================
    def set_cmd_vel_topic(self, topic: str):
        """切换遥控发布话题（仿真 /cmd_vel，真机 /cmd_vel_teleop）。"""
        if getattr(self, '_cmd_topic', None) == topic:
            return
        self._cmd_topic = topic
        try:
            self.destroy_publisher(self.cmd_pub)
        except Exception:
            pass
        self.cmd_pub = self.create_publisher(Twist, topic, 10)
        self.get_logger().info(f'遥控话题已切换为 {topic}')

    def publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    # ================= 初始位姿 =================
    def publish_initial_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        # 零时间戳：避免 GUI 墙钟与仿真时钟不一致，AMCL 使用最新位姿
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # 协方差放宽到 1.0（stddev 1m）：容忍手动"大概点一下"的偏差，
        # AMCL 会用激光扫描自动收敛到精确位姿。太紧(0.25)时点偏一点就定位失败。
        msg.pose.covariance[0] = 1.0
        msg.pose.covariance[7] = 1.0
        msg.pose.covariance[35] = 0.5
        self.init_pose_pub.publish(msg)
        self.signals.log.emit(f'已设定初始位姿: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')

    def publish_global_localization(self):
        """全局定位：以极大协方差发布初始位姿，让 AMCL 均匀撒粒子自动搜索位姿。

        用于机器人完全不知道自己在哪、又不想手动点选时的兜底方案。
        AMCL 会在大范围内分布粒子并用激光收敛，通常 10~30 秒完成。
        """
        pose = self._latest_pose or (0.0, 0.0, 0.0)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        msg.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        msg.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        # x/y 方差 100m²（stddev 10m），yaw 方差 π²（全圆）：等价"我不知道在哪"
        msg.pose.covariance[0] = 100.0
        msg.pose.covariance[7] = 100.0
        msg.pose.covariance[35] = math.pi * math.pi
        self.init_pose_pub.publish(msg)
        self.signals.log.emit('已请求全局定位：AMCL 正在自动搜索机器人位姿（约 10~30 秒，请勿移动机器人）')

    # ================= 导航 Action =================
    def send_goal(self, x: float, y: float, yaw: float):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.signals.nav_state.emit('导航服务器未就绪（请先启动导航模式）')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        # 零时间戳：导航栈按最新 TF 规划，避免 GUI 墙钟与仿真时钟不一致
        goal.pose.header.stamp = rclpy.time.Time().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._last_goal_text = f'已下发目标 ({x:.2f}, {y:.2f})，等待到达'
        self.signals.nav_state.emit(f'目标已发送: ({x:.2f}, {y:.2f}) 规划中...')
        self._emit_decisions()   # 立即刷新"导航决策"，避免面板仍停留在"空闲"
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._on_nav_feedback)
        future.add_done_callback(self._on_goal_response)
        return True

    def _on_goal_response(self, future):
        try:
            self._goal_handle = future.result()
        except Exception as e:
            self.signals.nav_state.emit(f'目标发送失败: {e}')
            return
        if not self._goal_handle.accepted:
            self.signals.nav_state.emit('目标被拒绝')
            self._goal_handle = None
            self.signals.goal_finished.emit('rejected')
            return
        self._navigating = True
        self.signals.nav_state.emit('导航中...')
        self._emit_decisions()
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_feedback(self, feedback_msg):
        """导航反馈：节流到 1Hz 或距离变化 >0.1m 才上报。

        原实现按反馈频率（约 10Hz）无脑 emit，主窗口每条都写日志，
        导致日志被"剩余 X m"刷满，真正有用的告警（No valid trajectories 等）
        被淹没——这也是"导航卡死却看不出原因"的元凶之一。
        """
        try:
            dist = feedback_msg.feedback.distance_remaining
        except Exception:
            return
        now = time.monotonic()
        changed = (self._last_feedback_dist is None
                   or abs(dist - self._last_feedback_dist) >= 0.1)
        if not (changed and now - self._last_feedback_log >= 1.0):
            return
        self._last_feedback_log = now
        self._last_feedback_dist = dist
        self.signals.nav_state.emit(f'导航中... 剩余 {dist:.2f} m')

    def _on_nav_result(self, future):
        self._navigating = False
        self._goal_handle = None
        self._last_goal_text = None
        self._last_feedback_dist = None
        self._emit_decisions()
        try:
            result = future.result()
            status = result.status
            # action_msgs/GoalStatus: 4=SUCCEEDED
            if status == 4:
                self.signals.nav_state.emit('已到达目标点')
                self.signals.goal_finished.emit('succeeded')
            elif status == 5:
                self.signals.nav_state.emit('导航已取消')
                self.signals.goal_finished.emit('cancelled')
            elif status == 6:
                self.signals.nav_state.emit('导航失败（目标不可达）')
                self.signals.goal_finished.emit('failed')
            else:
                self.signals.nav_state.emit(f'导航结束，状态码 {status}')
        except Exception as e:
            self.signals.nav_state.emit(f'导航结果异常: {e}')

    def cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self.signals.nav_state.emit('正在取消导航...')
        else:
            self.signals.nav_state.emit('当前没有进行中的导航')

    # ================= 巡线（route_follower 节点，跑在工控机上）=================
    def send_route(self, points):
        """下发巡线路线 [(x, y, yaw), ...]。节点收到 'start' 后开始执行。"""
        msg = Path()
        msg.header.frame_id = 'map'
        # 零时间戳：避免 GUI 墙钟与工控机时钟不一致
        msg.header.stamp = rclpy.time.Time().to_msg()
        for x, y, yaw in points:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(ps)
        self.route_goal_pub.publish(msg)

    def send_route_ctrl(self, cmd: str):
        """巡线控制指令：'start' / 'stop' / 'vmax:0.30'（设速度）。"""
        m = String()
        m.data = cmd
        self.route_ctrl_pub.publish(m)

    def send_route_heartbeat(self):
        """2Hz 心跳：节点超时收不到即认为 GUI 断链，自动零速停车。"""
        m = String()
        m.data = 'hb'
        self.route_hb_pub.publish(m)

    def _on_route_status(self, msg: String):
        """解析 route_follower 的 JSON 进度回报，转发给 GUI。"""
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        if isinstance(d, dict):
            self.signals.route_status.emit(d)

    @property
    def is_navigating(self):
        return self._navigating

    # ================= 保存地图 =================
    def save_map_ready(self) -> bool:
        """slam_toolbox 的保存服务是否在线（不管建图是谁启动的）。"""
        if self._save_map_client is None:
            return False
        try:
            return self._save_map_client.service_is_ready()
        except Exception:                                       # noqa: BLE001
            return False

    def save_map(self, path_without_ext: str) -> bool:
        """调用 slam_toolbox 保存地图服务。返回是否成功发起。"""
        if self._save_map_client is None:
            self.signals.log.emit('slam_toolbox 服务不可用')
            return False
        if not self._save_map_client.wait_for_service(timeout_sec=1.0):
            self.signals.log.emit('保存地图服务未就绪（SLAM 未运行？）')
            return False
        req = self._SaveMap.Request()
        req.name.data = path_without_ext
        future = self._save_map_client.call_async(req)
        future.add_done_callback(self._on_save_map_done)
        return True

    def _on_save_map_done(self, future):
        try:
            future.result()
            self.signals.log.emit('地图保存成功')
        except Exception as e:
            self.signals.log.emit(f'地图保存失败: {e}')

    # ================= 激光转世界坐标 =================
    def scan_frame_transform(self, scan_frame):
        """查 map -> 雷达坐标系 的 TF，返回 (x, y, yaw)；失败返回 None。

        为什么要查"雷达"而不是"车体"：
          scan_to_points 过去用 车体位姿 + 雷达角度 直接推算世界坐标，
          这**默认了雷达与车头同向**。真机上雷达装在车顶支架上，安装
          偏转（180°/±90°）是常态 —— 漏掉这个偏转，画出来的雷达点就会
          "左右反、甚至前后反"。而且它依赖 /amcl_pose 是否已到，存在竞态：
          重启 GUI 后 TF 缓存填充顺序不同，偶尔"碰巧对了"，所以以前
          "退出重进"能暂时解决 —— 本质是没走 TF。
          正确做法：查 map -> base_scan（雷达帧）的完整变换，一次到位，
          与安装偏转无关，也不依赖 /amcl_pose。
        """
        for frame in (scan_frame, 'base_scan', 'laser', 'base_link',
                      'base_footprint'):
            if not frame:
                continue
            try:
                t = self.tf_buffer.lookup_transform(
                    'map', frame, rclpy.time.Time())
            except Exception:
                continue
            tr = t.transform.translation
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (tr.x, tr.y, yaw)
        return None

    @staticmethod
    def scan_to_points(msg, robot_pose, max_range=6.0):
        """把激光数据转换为 map 坐标系点列表（用雷达帧位姿，含安装偏转）。

        必须同时排除：NaN/inf、range_min 以内的自身回波、以及等于 range_max 的
        无回波值。Gazebo 在没有命中任何物体时返回 range_max（而不是 inf），
        原实现只过滤 <range_max，导致空旷环境下整个圆环都是"障碍物"。
        """
        if robot_pose is None:
            return []
        rx, ry, ryaw = robot_pose
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        # 有效回波：有限值、在量程内、且严格小于最大量程（射空）
        valid = np.isfinite(ranges)
        valid &= (ranges >= max(msg.range_min, 0.0))
        valid &= (ranges < min(msg.range_max, max_range))
        pts = []
        for r, a in zip(ranges[valid], angles[valid]):
            pts.append((rx + r * math.cos(ryaw + a),
                        ry + r * math.sin(ryaw + a)))
        return pts

    # ================= 激光扇区分析（障碍距离可视化）=================
    @staticmethod
    def scan_sector_distances(msg, sector_count=12, max_range=6.0):
        """把一帧激光按扇区分箱，返回每个扇区的最近有效障碍距离。

        返回 (dists, valid_mask)：
          dists[i]      第 i 个扇区的最近距离（无回波为 inf）
          valid_mask[i] 该扇区是否真的有回波（False = 射空，不是障碍）
        角度从 -pi 开始逆时针，第 0 扇区对应机器人右后方。
        """
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        n = len(ranges)
        if n == 0:
            return np.full(sector_count, np.inf), np.zeros(sector_count, bool)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        valid = np.isfinite(ranges)
        valid &= (ranges >= max(msg.range_min, 0.0))
        valid &= (ranges < min(msg.range_max, max_range))
        idx = ((angles + math.pi) / (2.0 * math.pi) * sector_count).astype(int)
        idx = np.clip(idx, 0, sector_count - 1)
        dists = np.full(sector_count, np.inf)
        mask = np.zeros(sector_count, dtype=bool)
        for i in range(sector_count):
            sel = valid & (idx == i)
            if np.any(sel):
                dists[i] = float(np.min(ranges[sel]))
                mask[i] = True
        return dists, mask

    def get_sensor_snapshot(self):
        """汇总当前所有传感器状态，供可视化面板使用。"""
        scan = self._latest_scan
        sectors = np.full(12, np.inf)
        mask = np.zeros(12, dtype=bool)
        if scan is not None:
            sectors, mask = self.scan_sector_distances(scan)
        return {
            'scan': scan,
            'sectors': sectors,
            'sector_valid': mask,
            'sonar': dict(self._sonar_state),
            'camera': dict(self._camera_state),
            'pose': self._latest_pose,
            'cmd_vel': self._latest_cmd_vel,
            'navigating': self._navigating,
            'amcl_ok': self._amcl_pose_count > 0,
        }

    # ================= 导航/避障决策输出 =================
    def _should_emit_decisions(self):
        """是否处于"需要考虑避障"的状态。

        判断依据是**底盘真实是否在动**，而不是"有没有指令"：
          · /odom 的实测速度（最可靠 —— 急停时指令非零但车不动）
          · 或 /cmd_vel 指令速度（遥控/导航刚下发、车还没响应的瞬间）
          · 或正在导航中

        只看指令是不够的：safety_mux 会持续输出零速帧，而车静止时
        雷达照样有回波（墙壁/家具），于是终端不停刷
        "减速警戒（右侧 0.77m）" —— 车根本没动，这些输出全是噪音。

        速度带 0.05 死区：轮速编码器静止时也有微小抖动，避免误判。
        """
        if self._navigating:
            return True
        odom_lin, odom_ang = self._odom_vel
        if abs(odom_lin) > 0.05 or abs(odom_ang) > 0.05:
            return True
        cmd_lin, cmd_ang = self._latest_cmd_vel
        return abs(cmd_lin) > 0.05 or abs(cmd_ang) > 0.05

    def _emit_decisions(self):
        """根据当前传感器与速度状态生成导航决策、避障决策、停车原因、下一步建议。

        节流：与上次文本相同则不再发送，避免刷屏。
        避障类输出仅在运动/导航中生成（见 _should_emit_decisions）。
        """
        nav_text = self._nav_decision_text()
        if nav_text != self._last_nav_text:
            self._last_nav_text = nav_text
            self.signals.nav_decision.emit(nav_text)

        # 空闲静止：避障/停车/建议不参与播报，收敛为"空闲"一条，避免刷屏。
        if not self._should_emit_decisions():
            for attr, sig in (('_last_avoid_text', self.signals.avoid_decision),
                              ('_last_stop_text', self.signals.stop_reason),
                              ('_last_next_text', self.signals.next_action)):
                if getattr(self, attr) != '空闲（未运行）':
                    setattr(self, attr, '空闲（未运行）')
                    sig.emit('空闲（未运行）')
            return

        # 周边障碍只分析一次，供避障决策/停车原因/下一步建议共用
        surr = self._analyze_surroundings()

        avoid_text = self._avoid_decision_text(surr)
        if avoid_text != self._last_avoid_text:
            self._last_avoid_text = avoid_text
            self.signals.avoid_decision.emit(avoid_text)

        stop_text = self._stop_reason_text(surr)
        if stop_text != self._last_stop_text:
            self._last_stop_text = stop_text
            self.signals.stop_reason.emit(stop_text)

        next_text = self._next_action_text(surr)
        if next_text != self._last_next_text:
            self._last_next_text = next_text
            self.signals.next_action.emit(next_text)

    def _nav_decision_text(self):
        if self._navigating:
            lin, ang = self._latest_cmd_vel
            if abs(lin) < 0.01 and abs(ang) < 0.01:
                action = '停滞（等待重规划/清除代价地图）'
            elif abs(ang) > 0.3 and abs(lin) < 0.05:
                action = '原地转向对正路径'
            elif lin > 0:
                action = f'前进 {lin:.2f} m/s'
            else:
                action = f'后退 {lin:.2f} m/s'
            return f'导航中 → {action}'
        if self._last_goal_text:
            return self._last_goal_text
        return '空闲（未下发导航目标）'

    @staticmethod
    def _angle_to_dir(angle):
        """机器人系角度（0=正前，正值=向左）→ 8 方位名。"""
        a = math.degrees(angle) % 360.0
        idx = int(round(a / 45.0)) % 8
        return ('前', '左前', '左', '左后', '后', '右后', '右', '右前')[idx]

    def _analyze_surroundings(self, near_thresh=0.8):
        """汇总激光/超声/RGB-D 的近距障碍，按（来源, 方位）取最近距离。

        返回 dict：
          threats: [{'source','dir','dist'}, ...]  距离升序
          front/left/right/back: 各方位的最近距离（inf=该方位无近距障碍）
        """
        agg = {}   # (source, dir) -> min dist

        # 激光：8 方位分箱，仅统计近距回波（射空=range_max 已被过滤）
        scan = self._latest_scan
        if scan is not None:
            r = np.asarray(scan.ranges, dtype=np.float64)
            if r.size:
                angles = scan.angle_min + np.arange(r.size) * scan.angle_increment
                valid = np.isfinite(r)
                valid &= (r >= max(scan.range_min, 0.0))
                valid &= (r < min(scan.range_max, 8.0))
                for a, d in zip(angles[valid], r[valid]):
                    if d < near_thresh:
                        key = ('激光雷达', self._angle_to_dir(a))
                        if d < agg.get(key, np.inf):
                            agg[key] = float(d)

        # 超声：每路一个方位
        for topic, st in self._sonar_state.items():
            if not st.get('active'):
                continue
            d = st.get('obstacle_dist', np.inf)
            if np.isfinite(d) and d < near_thresh:
                key_name = topic.split('/')[-1]
                key = ('超声', SONAR_DIR.get(key_name, key_name))
                if d < agg.get(key, np.inf):
                    agg[key] = float(d)

        # RGB-D：左右各一路（外八 ±45°）
        for name, st in self._camera_state.items():
            if not st.get('active'):
                continue
            d = st.get('min_dist', np.inf)
            if np.isfinite(d) and d < near_thresh:
                key = ('RGB-D', '左前' if name == 'camera_l' else '右前')
                if d < agg.get(key, np.inf):
                    agg[key] = float(d)

        threats = [{'source': s, 'dir': dr, 'dist': d}
                   for (s, dr), d in agg.items()]
        threats.sort(key=lambda t: t['dist'])

        def _min(*dirs):
            ds = [t['dist'] for t in threats if t['dir'] in dirs]
            return min(ds) if ds else float('inf')

        return {
            'threats': threats,
            'front': _min('前', '左前', '右前'),
            'left': _min('左', '左前', '左后'),
            'right': _min('右', '右前', '右后'),
            'back': _min('后', '左后', '右后'),
        }

    # 避障距离显示粒度（米）。
    # 为什么要量化：雷达回波本身有噪声，同一面墙连续两帧可能是
    # 0.77 / 0.76 / 0.75 —— 文本每次都不同，节流失效，终端被刷屏。
    # 量化到 5cm 后，只有距离真的变化超过 5cm 才会更新文本。
    AVOID_DIST_QUANT = 0.05

    @classmethod
    def _qdist(cls, d):
        """把距离量化到 AVOID_DIST_QUANT 的网格上（消除亚厘米抖动）。"""
        q = cls.AVOID_DIST_QUANT
        return round(round(d / q) * q, 2)

    def _avoid_decision_text(self, surr):
        """综合激光/超声/RGB-D 给出避障决策（含方位与距离）。"""
        threats = surr['threats']
        if not threats:
            return '无障碍 → 全速通行'
        nearest = threats[0]
        dist = nearest['dist']
        if dist < 0.35:
            level = '急停'
        elif dist < 0.6:
            level = '强减速+绕行'
        else:
            level = '减速警戒'
        detail = '、'.join(
            f"{t['source']}{DIR_DISPLAY[t['dir']]} {self._qdist(t['dist']):.2f}m"
            for t in threats[:4])
        where = DIR_DISPLAY[nearest['dir']]
        qd = self._qdist(dist)
        return f'{level}（最近 {nearest["source"]} {where} {qd:.2f}m）| {detail}'

    def _stop_reason_text(self, surr):
        """说明机器人当前是否停车，以及停车是否由传感器近距障碍导致。

        用户核心诉求：机器人不动时，能一眼看出「是因为雷达/超声波在四周
        某某距离发现了障碍物」，而不是只显示一个模糊的"停滞"。
        """
        lin, ang = self._latest_cmd_vel
        moving = abs(lin) > 0.01 or abs(ang) > 0.01
        threats = surr['threats']
        if moving:
            if threats:
                return '行驶中（正在减速/绕行避障）'
            return '行驶中'
        # 停车状态
        if not threats:
            if self._navigating:
                return '停车：未检测到近距障碍（可能在重规划/等待路径）'
            return '停车：未检测到近距障碍（空闲/手动停止）'
        parts = [
            f"{t['source']}在{DIR_DISPLAY[t['dir']]} {t['dist']:.2f} m 发现障碍"
            for t in threats[:4]]
        return '停车：' + '；'.join(parts)

    def _next_action_text(self, surr):
        """根据周边障碍方位，给出下一步建议（绕行/倒车/等待/通行）。"""
        front, left = surr['front'], surr['left']
        right, back = surr['right'], surr['back']

        def blocked(d):
            return np.isfinite(d) and d < 0.8

        if not any(blocked(d) for d in (front, left, right, back)):
            return '四周畅通 → 全速通行'

        if blocked(front):
            if not blocked(left) and not blocked(right):
                return '前方受阻，左右均畅通 → 建议左转或右转绕行'
            if not blocked(left):
                return f'前方 {front:.2f}m 受阻，左侧畅通 → 建议左转绕行'
            if not blocked(right):
                return f'前方 {front:.2f}m 受阻，右侧畅通 → 建议右转绕行'
            if not blocked(back):
                return '前方及两侧均受阻，后方畅通 → 建议倒车脱困'
            return '四周均被障碍包围 → 建议原地停车等待/人工介入'

        # 前方畅通，仅有侧向/后方障碍
        sides = []
        if blocked(left):
            sides.append(f'左侧 {left:.2f}m')
        if blocked(right):
            sides.append(f'右侧 {right:.2f}m')
        if sides:
            return '前方畅通，注意' + '、'.join(sides) + ' → 保持行驶，控制横向间距'
        if blocked(back):
            return f'后方 {back:.2f}m 有障碍 → 保持前进，勿倒车'
        return '前方畅通 → 全速通行'

    # ================= 时钟 =================
    def set_use_sim_time(self, flag: bool):
        """切换时钟来源：True=仿真时间（/clock），False=系统时间。

        必须与发布 TF 的 robot_state_publisher 保持一致，
        否则 TF 时间戳与本地时钟相差巨大，会持续刷 TF_OLD_DATA 警告。
        """
        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, bool(flag))])
        self.get_logger().info(
            'use_sim_time 已切换为 %s' % ('true' if flag else 'false'))
