#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dt01_check —— 真机数据链路一键体检。

用法（工控机上，驱动已启动）：
  ros2 run dt01_bringup check_system.py
  ros2 run dt01_bringup check_system.py --ros-args -p sample_time:=5.0
  ros2 run dt01_bringup check_system.py --ros-args -p check_map:=true   # 导航模式

检查四件事，任一项 FAIL 都说明后面不用往下测了：
  1. 节点是否都活着
  2. 关键话题是否真的在出数据（有话题≠有数据）
  3. TF 树是否完整（真机最常见的坑：base_link 双父帧）
  4. 底盘安全位（急停/防撞/故障）是否为 0

退出码：0=全部通过，1=有 FAIL（方便脚本里串起来用）。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Imu, BatteryState
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

try:
    from robot_ros2_msgs.msg import ChassisState
    HAS_DT_MSGS = True
except ImportError:
    HAS_DT_MSGS = False
    ChassisState = None


GREEN = '\033[32m'
YELLOW = '\033[33m'
RED = '\033[31m'
RESET = '\033[0m'

# ---- 期望存在的话题：(话题, 类型, 最低频率 Hz, 是否必需) ----
TOPIC_SPECS = [
    ('/odom', Odometry, 30.0, True),
    ('/scan', LaserScan, 5.0, True),
    ('/imu/data', Imu, 20.0, True),
    ('/dt/state_info', ChassisState, 20.0, True),
    ('/battery_state', BatteryState, 1.0, False),
    ('/dt01/chassis_status', String, 5.0, True),
    ('/dt01/safety_mux', String, 0.0, False),
    ('/odometry/filtered', Odometry, 20.0, False),   # 开 EKF 时才有
]

# ---- 期望存活的节点：(节点名, 是否必需) ----
NODE_SPECS = [
    ('dt_ros2_node', True),
    ('bluesea_node', True),
    ('ahrs_driver_node', True),
    ('robot_state_publisher', True),
    ('odom_to_tf', True),
    ('chassis_bridge', True),
    ('safety_mux', True),
    ('ekf_filter_node', False),
    ('imu_preprocess', False),
    ('link_watchdog', False),
]

# ---- 期望的 TF 边（parent, child, 是否必需）----
TF_EDGES = [
    ('odom', 'base_footprint', True),
    ('base_footprint', 'base_link', True),
    ('base_link', 'base_scan', True),
    ('base_link', 'imu_link', True),
]


class CheckSystem(Node):
    def __init__(self):
        super().__init__('dt01_check')
        self.declare_parameter('sample_time', 3.0)
        self.declare_parameter('check_map', False)
        self._sample_time = float(self.get_parameter('sample_time').value)
        self._check_map = bool(self.get_parameter('check_map').value)

        self._counts = {}
        sensor_qos = QoSProfile(depth=50,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        latch_qos = QoSProfile(depth=1,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)

        for topic, msg_type, _hz, _req in TOPIC_SPECS:
            if msg_type is None:
                continue
            self._counts[topic] = 0
            qos = latch_qos if topic in ('/dt01/safety_mux',
                                         '/dt01/chassis_status') else sensor_qos
            self.create_subscription(
                msg_type, topic,
                lambda _msg, t=topic: self._counts.__setitem__(
                    t, self._counts[t] + 1),
                qos)

        self._chassis = None
        if HAS_DT_MSGS:
            self.create_subscription(ChassisState, '/dt/state_info',
                                     self._on_state, sensor_qos)
        self._status_text = ''
        self.create_subscription(String, '/dt01/chassis_status',
                                 self._on_status, latch_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._failures = 0

    def _on_state(self, msg):
        self._chassis = msg

    def _on_status(self, msg):
        self._status_text = msg.data

    # ---------------- 报告 ----------------
    def _ok(self, text):
        print(f'  {GREEN}[PASS]{RESET} {text}')

    def _warn(self, text):
        print(f'  {YELLOW}[WARN]{RESET} {text}')

    def _fail(self, text):
        self._failures += 1
        print(f'  {RED}[FAIL]{RESET} {text}')

    def run(self):
        print()
        print('=' * 62)
        print(' DT-01 真机数据链路体检')
        print(f' 采样窗口：{self._sample_time:.1f}s（期间请保持机器人静止）')
        print('=' * 62)

        # ---- 1. 节点 ----
        print('\n[1] 节点存活')
        alive = set(self.get_node_names())
        for name, required in NODE_SPECS:
            # 节点名可能带命名空间前缀，用后缀匹配
            hit = any(n == name or n.endswith('/' + name) for n in alive)
            if hit:
                self._ok(name)
            elif required:
                self._fail(f'{name} 未运行（必需）')
            else:
                self._warn(f'{name} 未运行（可选）')

        # ---- 采样 ----
        t0 = time.monotonic()
        start_counts = dict(self._counts)
        while time.monotonic() - t0 < self._sample_time:
            rclpy.spin_once(self, timeout_sec=0.1)
        dt = time.monotonic() - t0

        # ---- 2. 话题频率 ----
        print('\n[2] 话题数据（有话题 ≠ 有数据，这里看的是实际频率）')
        for topic, msg_type, min_hz, required in TOPIC_SPECS:
            if msg_type is None:
                self._warn(f'{topic} 跳过（robot_ros2_msgs 未编译）')
                continue
            n = self._counts.get(topic, 0) - start_counts.get(topic, 0)
            hz = n / dt if dt > 0 else 0.0
            if n == 0:
                if required:
                    self._fail(f'{topic} 无任何数据')
                else:
                    self._warn(f'{topic} 无数据（可选）')
            elif hz < min_hz:
                self._fail(f'{topic} 频率过低 {hz:.1f}Hz < {min_hz:.1f}Hz')
            else:
                self._ok(f'{topic} {hz:.1f}Hz')

        # ---- 3. TF 树 ----
        print('\n[3] TF 树')
        edges = list(TF_EDGES)
        if self._check_map:
            edges.insert(0, ('map', 'odom', True))
        for parent, child, required in edges:
            try:
                self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0))
                self._ok(f'{parent} -> {child}')
            except (LookupException, ConnectivityException,
                    ExtrapolationException) as e:
                if required:
                    self._fail(f'{parent} -> {child} 查不到：{type(e).__name__}')
                else:
                    self._warn(f'{parent} -> {child} 查不到（可选）')

        # ---- 4. 底盘安全位 ----
        print('\n[4] 底盘安全位')
        st = self._chassis
        if st is None:
            self._warn('未收到 /dt/state_info，跳过（robot_ros2_msgs 未编译？）')
        else:
            checks = [
                ('急停按钮', st.stop_button),
                ('遥控急停', st.remote_control_stop),
                ('软件急停', st.software_stop),
                ('前防撞', st.front_collision),
                ('后防撞', st.rear_collision),
                ('驱动器故障', st.motor_drive_error),
                ('编码器故障', st.motor_encoder_error),
            ]
            for name, bad in checks:
                if bad:
                    self._fail(f'{name} = 触发')
            if not any(bad for _n, bad in checks):
                self._ok('急停 / 防撞 / 故障 全部为 0')
            if not st.motor_drive_online:
                self._fail('驱动器离线')
            else:
                self._ok('驱动器在线')

        if self._status_text:
            print(f'\n  底盘状态串：{self._status_text}')

        # ---- 汇总 ----
        print('\n' + '=' * 62)
        if self._failures == 0:
            print(f' {GREEN}全部通过{RESET}：数据链路正常，可以进入建图/导航')
        else:
            print(f' {RED}发现 {self._failures} 项问题{RESET}，按上面顺序逐个解决')
            print(' 排障顺序：节点 -> 话题 -> TF -> 底盘安全位（前面的没过，')
            print(' 后面的检查没有意义）')
        print('=' * 62 + '\n')
        return 0 if self._failures == 0 else 1


def main():
    rclpy.init()
    node = CheckSystem()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
