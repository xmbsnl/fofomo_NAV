#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""safety_mux —— /cmd_vel 安全仲裁器（上真机前唯一必须经过的一关）。

为什么必须有它
--------------
底盘协议里虽然有 800ms 指令超时保护，但它只在"完全收不到串口数据"时生效。
下面这些情况它救不了你：
  * Nav2 进程还在、但卡在死循环里持续发同一个速度 -> 车会一直走
  * 上位机通过 WiFi 发指令，网络一抖，最后一条非零速度被卡住 -> 车一直走
  * 人挡在车前，DWB 因为代价地图更新慢还没反应过来 -> 撞上去

本节点串行在 Nav2/遥操 与底盘之间，每周期重新裁决一次该发什么：

  优先级（高 -> 低）
  ---------------------------------------------------------------
   1. 急停 / 防撞条 / 驱动器故障   -> 强制零速（可锁存）
   2. 遥控器接管                  -> 停止发布，把底盘交还给遥控器
   3. 激光急停扇区命中            -> 零速（或按比例减速）
   4. 遥操 /cmd_vel_teleop        -> 透传（限速）
   5. 自主 /cmd_vel_nav           -> 透传（限速）
   6. 全部超时                    -> 主动补发 1s 零速刹停，然后停发
                                     （触发底盘 800ms 超时保护，双保险）

另外它做两件"小事"但很关键：
  * 速度限幅：任何来源都逃不过 max_linear / max_angular
  * 状态外发：/dt01/safety_mux 文本，GUI / 日志一眼看出为什么停车

接线：Nav2(velocity_smoother) -> /cmd_vel_nav -> [safety_mux] -> /cmd_vel -> dt_ros2
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool, Empty

try:
    from robot_ros2_msgs.msg import ChassisState, RemoteControl
    HAS_DT_MSGS = True
except ImportError:                                    # pragma: no cover
    HAS_DT_MSGS = False
    ChassisState = RemoteControl = None


def _zero():
    return Twist()


class SafetyMux(Node):
    def __init__(self):
        super().__init__('safety_mux')

        # ---------- 参数 ----------
        self.declare_parameter('nav_topic', '/cmd_vel_nav')
        self.declare_parameter('teleop_topic', '/cmd_vel_teleop')
        self.declare_parameter('out_topic', '/cmd_vel')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('source_timeout', 0.5)   # 指令源超时（s）
        self.declare_parameter('brake_time', 1.0)       # 超时后补发零速时长（s）
        self.declare_parameter('max_linear', 1.0)       # m/s（DT-01 上限 1.5，先留余量）
        self.declare_parameter('max_angular', 1.5)      # rad/s
        self.declare_parameter('teleop_timeout', 0.3)   # 遥操停手后多久退回自主

        # 激光急停
        self.declare_parameter('use_laser_stop', True)
        self.declare_parameter('laser_topic', '/scan')
        self.declare_parameter('laser_stop_distance', 0.30)   # 以内 -> 停
        self.declare_parameter('laser_slow_distance', 0.60)   # 以内 -> 限速
        self.declare_parameter('laser_stop_angle', 0.9)       # ±rad 前向扇区
        self.declare_parameter('laser_max_useful', 8.0)       # 超过该距离忽略

        # 急停锁存：触发后必须等人工/话题解除，防止抖动中反复启动
        self.declare_parameter('estop_latch', True)
        self.declare_parameter('reset_topic', '/dt01/safety_reset')
        # 默认不开：防撞条松开就自动清除，若机器人仍顶着障碍物会立刻再次触发，
        # 甚至可能在没脱离接触时重新加速。需要自动恢复时再显式打开。
        self.declare_parameter('auto_clear_collision', False)

        p = lambda n: self.get_parameter(n).value
        self.p = p
        self._rate = float(p('rate'))
        self._last_log = 0.0
        self._last_reason = ''

        # ---------- 状态 ----------
        self._nav = (_zero(), 0.0)
        self._teleop = (_zero(), 0.0)
        # 原始位（分别存，最终取或）：急停按钮 / 软件急停 / 前防撞 / 后防撞
        self._estop_btn = False
        self._estop_soft = False
        self._coll_front = False
        self._coll_rear = False
        self._estop = False            # = 急停按钮 or 软件急停
        self._collision = False        # = 前防撞 or 后防撞
        self._fault = False
        self._rc_active = False
        self._estop_latched = False
        self._scan = None
        self._stop_since = None        # 开始补发零速的时刻

        # ---------- 订阅 ----------
        sensor_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        latch_qos = QoSProfile(depth=1,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Twist, p('nav_topic'), self._on_nav, 10)
        self.create_subscription(Twist, p('teleop_topic'), self._on_teleop, 10)
        self.create_subscription(LaserScan, p('laser_topic'),
                                 self._on_scan, sensor_qos)
        self.create_subscription(Empty, p('reset_topic'), self._on_reset, 10)

        for topic, key in (('/dt/std_stop_button_info', 'stop'),
                           ('/dt/std_software_stop_info', 'soft'),
                           ('/dt/std_front_collision_info', 'front'),
                           ('/dt/std_rear_collision_info', 'rear')):
            self.create_subscription(
                Bool, topic,
                lambda msg, k=key: self._on_bool(k, bool(msg.data)), 10)

        if HAS_DT_MSGS:
            self.create_subscription(ChassisState, '/dt/state_info',
                                     self._on_state, sensor_qos)
            self.create_subscription(RemoteControl, '/dt/remote_control_info',
                                     self._on_remote, sensor_qos)
        else:
            self.get_logger().warn(
                '未找到 robot_ros2_msgs：急停/碰撞/遥控接管检测将只依赖'
                '/dt/std_*_info 话题。请编译 robot_ros2_msgs 后重启。')

        # ---------- 发布 ----------
        self.pub_cmd = self.create_publisher(Twist, p('out_topic'), 10)
        self.pub_state = self.create_publisher(String, '/dt01/safety_mux',
                                               latch_qos)
        self.pub_collision_clean = self.create_publisher(
            Empty, '/dt/std_all_collision_clean', 10)
        self.pub_chassis_stop = self.create_publisher(Bool, '/dt/stop_ctrl', 10)

        self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f"safety_mux 就绪：{p('nav_topic')} + {p('teleop_topic')} "
            f"-> {p('out_topic')} @ {self._rate}Hz，"
            f"限速 {p('max_linear')} m/s / {p('max_angular')} rad/s")

    # ==================== 输入 ====================
    def _on_nav(self, msg):
        self._nav = (msg, time.monotonic())

    def _on_teleop(self, msg):
        self._teleop = (msg, time.monotonic())

    def _on_scan(self, msg):
        self._scan = msg

    def _on_reset(self, _msg):
        self._estop_latched = False
        self.get_logger().warn('收到安全复位指令，急停锁存已解除')

    def _on_bool(self, key, value):
        # 前/后防撞分开存，再取或：否则先撞前再撞后时，
        # 前防撞松开会被后面那次 False 回调错误地清掉。
        if key == 'front':
            self._coll_front = value
        elif key == 'rear':
            self._coll_rear = value
        elif key == 'stop':
            self._estop_btn = value
        elif key == 'soft':
            self._estop_soft = value
        self._estop = bool(self._estop_btn or self._estop_soft)
        self._collision = bool(self._coll_front or self._coll_rear)

    def _on_state(self, msg):
        # 只写原始位，合成量统一由 _refresh_flags() 计算，
        # 避免 /dt/state_info 与 /dt/std_*_info 两个来源互相覆盖。
        self._estop_btn = bool(msg.stop_button or msg.remote_control_stop)
        self._estop_soft = bool(msg.software_stop)
        self._coll_front = bool(msg.front_collision)
        self._coll_rear = bool(msg.rear_collision)
        self._fault = bool(msg.motor_drive_error or msg.motor_encoder_error
                           or not msg.motor_drive_online)
        self._refresh_flags()

    def _refresh_flags(self):
        self._estop = bool(self._estop_btn or self._estop_soft)
        self._collision = bool(self._coll_front or self._coll_rear)

    def _on_remote(self, msg):
        # 遥控器在线且任一摇杆离开中位 -> 判定人工接管
        dead = 50
        active = bool(msg.online) and any(
            abs(v) > dead for v in (msg.rocker_left_x, msg.rocker_left_y,
                                    msg.rocker_right_x, msg.rocker_right_y))
        if active != self._rc_active:
            self.get_logger().warn(
                '遥控器接管状态切换：%s' % ('接管中（ROS 停止下发）'
                                     if active else '已交还 ROS'))
        self._rc_active = active

    # ==================== 核心裁决 ====================
    def _tick(self):
        now = time.monotonic()
        reason = None
        out = None            # None = 本周期不发布

        # 1) 硬安全
        if self._estop:
            self._estop_latched = True
        if self._estop or self._estop_latched:
            reason = '急停触发（已锁存，发 /dt01/safety_reset 解除）'
            out = _zero()
        elif self._collision:
            reason = '防撞条触发'
            out = _zero()
        elif self._fault:
            reason = '驱动器/编码器故障'
            out = _zero()
        # 2) 遥控器接管：停发，让底盘只听遥控
        elif self._rc_active:
            reason = '遥控器接管中（ROS 不下发指令）'
            out = None
        else:
            # 3) 选源：遥操优先于自主
            t_toleop = now - self._teleop[1] < float(self.p('teleop_timeout'))
            t_nav = now - self._nav[1] < float(self.p('source_timeout'))
            src = None
            if t_toleop and self._is_moving(self._teleop[0]):
                src, name = self._teleop[0], '遥操'
            elif t_nav and self._is_moving(self._nav[0]):
                src, name = self._nav[0], '自主'
            elif t_toleop or t_nav:
                src, name = _zero(), '空闲（指令源在线但为零速）'
            else:
                src = name = None

            if src is not None:
                out = self._limit(src)
                # 4) 激光急停（对所有来源生效）
                dist = self._front_clearance(out.linear.x)
                stop_d = float(self.p('laser_stop_distance'))
                slow_d = float(self.p('laser_slow_distance'))
                if dist is not None and dist < stop_d:
                    reason = f'激光急停：前方 {dist:.2f}m < {stop_d:.2f}m'
                    out = _zero()
                elif dist is not None and dist < slow_d and out.linear.x > 0:
                    scale = max(0.0, (dist - stop_d) / max(1e-6, slow_d - stop_d))
                    out.linear.x *= scale
                    reason = f'激光减速：前方 {dist:.2f}m，限速至 {scale:.0%}'
                else:
                    reason = name
                self._stop_since = None
            else:
                # 5) 全部超时：主动补发零速刹停，再彻底停发交给底盘超时保护
                if self._stop_since is None:
                    self._stop_since = now
                    self.get_logger().warn(
                        '所有指令源超时，开始主动刹停')
                if now - self._stop_since < float(self.p('brake_time')):
                    reason = '指令源超时（主动刹停中）'
                    out = _zero()
                else:
                    reason = '指令源超时（已停发，底盘看门狗生效）'
                    out = None

        # ---------- 输出 ----------
        if out is not None:
            self.pub_cmd.publish(out)

        if self.p('auto_clear_collision') and not self._collision:
            self._clear_collision_once()

        # 状态文本节流到 2Hz，避免刷屏
        if reason != self._last_reason and now - self._last_log > 0.5:
            self._last_reason = reason
            self._last_log = now
            msg = String(data=str(reason))
            self.pub_state.publish(msg)
            if reason and ('急停' in reason or '故障' in reason
                           or '防撞' in reason):
                self.get_logger().error(f'[安全] {reason}')

    # ==================== 工具 ====================
    @staticmethod
    def _is_moving(msg: Twist):
        return (abs(msg.linear.x) > 1e-3 or abs(msg.linear.y) > 1e-3
                or abs(msg.angular.z) > 1e-3)

    def _limit(self, msg: Twist):
        out = Twist()
        out.linear.x = max(-float(self.p('max_linear')),
                           min(float(self.p('max_linear')), msg.linear.x))
        out.linear.y = msg.linear.y      # 差速底盘恒为 0
        out.angular.z = max(-float(self.p('max_angular')),
                            min(float(self.p('max_angular')), msg.angular.z))
        return out

    def _front_clearance(self, vx: float):
        """返回运动方向上的最近有效障碍距离（m）；无数据返回 None。

        前进看前向扇区，倒车看后向扇区 —— 否则倒车撞墙时根本拦不住。
        """
        if not self.p('use_laser_stop') or self._scan is None:
            return None
        scan = self._scan
        ranges = scan.ranges
        if not ranges:
            return None
        half = float(self.p('laser_stop_angle'))
        max_useful = float(self.p('laser_max_useful'))
        rmin = max(float(scan.range_min), 0.05)   # 滤掉车体自身回波
        rmax = min(float(scan.range_max), max_useful)

        # 目标方位：前进 -> 0（正前）；倒车 -> pi（正后）
        center = 0.0 if vx >= 0 else math.pi
        best = float('inf')
        for i, r in enumerate(ranges):
            if r != r or r < rmin or r > rmax:      # NaN 自检（r != r）
                continue
            a = scan.angle_min + i * scan.angle_increment
            d = abs(math.atan2(math.sin(a - center), math.cos(a - center)))
            if d <= half and r < best:
                best = r
        return best if best != float('inf') else None

    def _clear_collision_once(self):
        if getattr(self, '_cleared', False):
            return
        self._cleared = True
        self.pub_collision_clean.publish(Empty())


def main():
    rclpy.init()
    node = SafetyMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
