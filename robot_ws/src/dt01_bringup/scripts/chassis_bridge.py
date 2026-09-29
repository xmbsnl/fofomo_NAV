#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""chassis_bridge —— 厂家底盘话题 → 上层统一遥测话题。

为什么需要这一层
----------------
厂家驱动 dt_ros2 发布的是 /dt/* 系列自定义话题（robot_ros2_msgs），
而 dijun-nav GUI / 上层业务订阅的是通用话题。两边直接对接会全空。

本节点做三件事：
  1. 转发电量：/dt/battery_state (sensor_msgs/BatteryState) -> /battery_state
  2. 合成状态串：/dt/state_info + /dt/velocity_info + 各急停/碰撞 Bool
                -> /dt01/chassis_status (std_msgs/String, "k=v k=v ..." 格式)
                GUI 依赖的键：mode / vx / wz / estop / error / soc / v
  3. 暴露安全原始量：-> /dt01/safety (std_msgs/String)
     真正的安全仲裁在 safety_mux 节点，这里只做只读展示与调试。

设计约束：
  - 所有厂家消息用 try/except 导入。robot_ros2_msgs 没编译出来时节点不会崩，
    只是遥测为空（驱动本身仍然工作），便于分步联调。
  - 状态串固定 10Hz 节流发布，避免 50Hz 底盘数据刷屏 GUI。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String, Bool
from sensor_msgs.msg import BatteryState as RosBatteryState

# ---- 厂家消息（缺失时优雅降级）----
try:
    from robot_ros2_msgs.msg import (ChassisState, ChassisVelocity,
                                    RemoteControl, BatteryState as DtBatteryState,
                                    ChassisParameter)
    HAS_DT_MSGS = True
except ImportError:                                    # pragma: no cover
    HAS_DT_MSGS = False
    ChassisState = ChassisVelocity = RemoteControl = None
    DtBatteryState = ChassisParameter = None

# 控制模式枚举（ChassisState.control_mode）
CONTROL_MODE = {0: '待机', 1: '遥控', 2: 'ROS', 3: '回充', 4: '未知'}


class ChassisBridge(Node):
    def __init__(self):
        super().__init__('chassis_bridge')

        self.declare_parameter('status_topic', '/dt01/chassis_status')
        self.declare_parameter('safety_topic', '/dt01/safety')
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('publish_rate', 10.0)

        status_topic = self.get_parameter('status_topic').value
        safety_topic = self.get_parameter('safety_topic').value
        battery_topic = self.get_parameter('battery_topic').value
        rate = float(self.get_parameter('publish_rate').value)

        # 底盘状态帧是 50Hz 的高频自定义消息，用 best_effort 更省带宽
        sensor_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)

        self.pub_status = self.create_publisher(String, status_topic, 10)
        self.pub_safety = self.create_publisher(String, safety_topic, 10)
        self.pub_battery = self.create_publisher(RosBatteryState,
                                                 battery_topic, 10)

        # ---- 缓存的最新底盘数据 ----
        self._state = None
        self._velocity = None
        self._remote = None
        self._battery_dt = None
        self._params = None
        self._stop_button = False
        self._software_stop = False
        self._front_collision = False
        self._rear_collision = False

        if HAS_DT_MSGS:
            self.create_subscription(ChassisState, '/dt/state_info',
                                     self._on_state, sensor_qos)
            self.create_subscription(ChassisVelocity, '/dt/velocity_info',
                                     self._on_velocity, sensor_qos)
            self.create_subscription(RemoteControl, '/dt/remote_control_info',
                                     self._on_remote, sensor_qos)
            self.create_subscription(DtBatteryState, '/dt/battery_info',
                                     self._on_battery_dt, sensor_qos)
            self.create_subscription(ChassisParameter, '/dt/parameter_info',
                                     self._on_params, 10)
            self.get_logger().info('robot_ros2_msgs 已加载，遥测桥接全部启用')
        else:
            self.get_logger().warn(
                '未找到 robot_ros2_msgs，底盘状态遥测将为空。'
                '请先 colcon build robot_ros2_msgs 并 source install/setup.bash')

        for topic, setter in (
                ('/dt/std_stop_button_info', '_stop_button'),
                ('/dt/std_software_stop_info', '_software_stop'),
                ('/dt/std_front_collision_info', '_front_collision'),
                ('/dt/std_rear_collision_info', '_rear_collision')):
            self.create_subscription(
                Bool, topic,
                lambda msg, s=setter: setattr(self, s, bool(msg.data)),
                10)

        # 标准电量话题直接转发（GUI 只认这个名字）
        self.create_subscription(RosBatteryState, '/dt/battery_state',
                                 self._on_battery_ros, sensor_qos)

        self.create_timer(1.0 / rate, self._tick)

    # ==================== 回调 ====================
    def _on_state(self, msg):
        self._state = msg

    def _on_velocity(self, msg):
        self._velocity = msg

    def _on_remote(self, msg):
        self._remote = msg

    def _on_battery_dt(self, msg):
        self._battery_dt = msg

    def _on_params(self, msg):
        self._params = msg
        self.get_logger().info(
            f'底盘参数：轮距={msg.track_width}mm 轴距={msg.wheel_base}mm '
            f'轮径={msg.wheel_diameter}mm 减速比={msg.gear_ratio} '
            f'编码器线数={msg.encoder_line}', once=True)

    def _on_battery_ros(self, msg):
        # 厂家已经给了标准的 sensor_msgs/BatteryState，原样转发出去即可。
        # 若厂家未填 percentage，用 dt 自定义帧里的 percent 补上。
        out = msg
        if not (0.0 < msg.percentage <= 1.0) and self._battery_dt is not None:
            out.percentage = max(0.0, min(1.0, self._battery_dt.percent / 100.0))
        if out.voltage <= 0.0 and self._battery_dt is not None:
            out.voltage = float(self._battery_dt.voltage)
        self.pub_battery.publish(out)

    # ==================== 周期发布 ====================
    def _tick(self):
        self.pub_status.publish(String(data=self._status_text()))
        self.pub_safety.publish(String(data=self._safety_text()))

    def _status_text(self):
        """合成 'k=v k=v ...' 状态串（GUI 按等号切分解析）。"""
        st, vel, bt, rc = self._state, self._velocity, self._battery_dt, self._remote
        mode = st.control_mode if st is not None else -1
        # vx 单位保持 mm/s：GUI 用 _chassis_num 抽数字后自己拼单位
        vx_mm = (vel.linear * 1000.0) if vel is not None else 0.0
        wz = vel.angular if vel is not None else 0.0

        estop = 1 if (self._stop_button
                      or (st is not None and (st.stop_button
                                              or st.remote_control_stop))) else 0
        error = 1 if (st is not None
                      and (st.motor_drive_error or st.motor_encoder_error)) else 0
        coll = 1 if (self._front_collision or self._rear_collision
                     or (st is not None
                         and (st.front_collision or st.rear_collision))) else 0
        rc_online = 1 if (rc is not None and rc.online) else (
            1 if (st is not None and st.remote_control_online) else 0)

        parts = [
            f'mode={mode}',
            f'vx={vx_mm:.0f}mm/s',
            f'wz={wz:.2f}rad/s',
            f'soc={bt.percent if bt is not None else -1}%',
            f'v={bt.voltage if bt is not None else 0.0:.1f}V',
            f'estop={estop}',
            f'error={error}',
            f'collision={coll}',
            f'rc={rc_online}',
        ]
        return ' '.join(parts)

    def _safety_text(self):
        """只读安全快照，便于日志与远程排障。"""
        st = self._state
        flags = []
        if self._stop_button or (st is not None and st.stop_button):
            flags.append('急停按钮')
        if st is not None and st.remote_control_stop:
            flags.append('遥控急停')
        if self._software_stop or (st is not None and st.software_stop):
            flags.append('软件急停')
        if self._front_collision or (st is not None and st.front_collision):
            flags.append('前防撞')
        if self._rear_collision or (st is not None and st.rear_collision):
            flags.append('后防撞')
        if st is not None and st.motor_drive_error:
            flags.append('驱动器故障')
        if st is not None and st.motor_encoder_error:
            flags.append('编码器故障')
        if st is not None and not st.motor_drive_online:
            flags.append('驱动器离线')
        return ('正常' if not flags else '⚠ ' + '/'.join(flags))


def main():
    rclpy.init()
    node = ChassisBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
