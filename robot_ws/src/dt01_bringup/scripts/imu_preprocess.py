#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""imu_preprocess —— IMU 数据清洗，喂给 EKF 之前必须先过这一道。

直接把厂家 IMU 话题接进 robot_localization 有两个高概率翻车点：

1. **协方差全零**
   很多厂家驱动发的 sensor_msgs/Imu 里 *_covariance 全是 0。
   在滤波器语义里"0"不是"未知"，而是"完美可信"。结果 EKF 会把 IMU 当成
   绝对真理，轮式里程计被完全压过，车一打滑位姿直接飞。
   本节点检测到全零协方差时，替换成参数里的合理默认值。

2. **轴向/符号不一致**
   fdilink 等航姿模块可能输出 NED（Z 朝下）而非 ROS 的 ENU，
   yaw 角速度符号也就反了。融合反了的角速度 = 越融越歪。
   用 yaw_rate_sign / frame_id 两个参数就能现场纠正，不用改驱动。

输出话题默认 /imu/data_fused，由 ekf.yaml 的 imu0 订阅。
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu


class ImuPreprocess(Node):
    def __init__(self):
        super().__init__('imu_preprocess')

        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/imu/data_fused')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('yaw_rate_sign', 1.0)
        self.declare_parameter('angular_velocity_covariance', 0.02)
        self.declare_parameter('linear_acceleration_covariance', 0.10)
        # orientation 默认不参与 EKF（odom 系融合只用角速度），
        # 给一个大值等同于"基本不信"，防止磁力计跳变污染航向。
        self.declare_parameter('orientation_covariance', 1.0e3)
        self.declare_parameter('force_covariance', False)  # true=无条件覆盖
        # ------------------------------------------------------------
        # 输出降采样（2026-09-15 新增，解决 CPU 打满）
        # 飞迪 N100 以 400Hz 发布，而这个节点是 Python 实现，每帧都要
        # 做协方差检测 + 数组构造 + 发布。400Hz 全量处理实测占用
        # **61.6% CPU**（RK3588），是系统负载 7.98 的最大单一来源，
        # 进而拖慢 SSH 响应、加剧超时。
        # EKF 本身只跑 50Hz，400Hz 输入纯属浪费 —— 降到 50Hz 后
        # 融合精度不变（EKF 采样率才是瓶颈），CPU 直降约 7/8。
        # 设为 0 或负数表示不降采样（全量转发，仅调试用）。
        # ------------------------------------------------------------
        self.declare_parameter('publish_rate', 50.0)
        # ------------------------------------------------------------
        # 陀螺 z 轴零偏（rad/s），发布前扣除。
        # 2026-09-18 实测：静止时 EKF yaw 以恒定 0.196°/s 漂移（30s 漂 5.9°），
        # 与 /imu/data_fused 的 wz 均值 0.003415 rad/s 完全一致 —— 静止漂移
        # 100% 来自零偏而非噪声（噪声 std 仅 0.0006）。扣除后验收 yaw 峰峰
        # 应 < 1°/30s。零偏随温度缓慢变化，若发现静止又开始匀速漂移，
        # 静止测 20 秒 wz 均值更新此值即可。
        # ------------------------------------------------------------
        self.declare_parameter('gyro_bias_z', 0.0)

        self._yaw_sign = float(self.get_parameter('yaw_rate_sign').value)
        self._cov_w = float(self.get_parameter('angular_velocity_covariance').value)
        self._cov_a = float(self.get_parameter('linear_acceleration_covariance').value)
        self._cov_o = float(self.get_parameter('orientation_covariance').value)
        self._force = bool(self.get_parameter('force_covariance').value)
        self._frame = self.get_parameter('frame_id').value
        self._rate = float(self.get_parameter('publish_rate').value)
        self._gyro_bias_z = float(self.get_parameter('gyro_bias_z').value)
        self._min_interval = (1.0 / self._rate) if self._rate > 0 else 0.0
        self._last_pub = 0.0

        sensor_qos = QoSProfile(depth=50,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(
            Imu, self.get_parameter('output_topic').value, sensor_qos)
        self.create_subscription(
            Imu, self.get_parameter('input_topic').value,
            self._on_imu, sensor_qos)
        self._fixed = False
        self.get_logger().info(
            f"imu_preprocess: {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value} "
            f"(yaw_rate_sign={self._yaw_sign})")

    @staticmethod
    def _all_zero(cov):
        return cov is None or all(abs(v) < 1e-12 for v in cov)

    def _on_imu(self, msg: Imu):
        # ---- 降采样：400Hz 输入 -> 按 publish_rate 节流输出 ----
        # 必须在最前面 return，否则协方差计算等开销照样发生，省不到 CPU。
        if self._min_interval > 0.0:
            now = time.monotonic()
            if now - self._last_pub < self._min_interval:
                return
            self._last_pub = now

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame or msg.header.frame_id

        out.orientation = msg.orientation
        out.angular_velocity.x = msg.angular_velocity.x
        out.angular_velocity.y = msg.angular_velocity.y
        # 先扣零偏再乘符号（零偏是在 IMU 自身坐标系实测的，与符号无关）
        out.angular_velocity.z = \
            (msg.angular_velocity.z - self._gyro_bias_z) * self._yaw_sign
        out.linear_acceleration = msg.linear_acceleration

        # 协方差：[0,4,8] 分别是 x/y/z 的对角元
        if self._force or self._all_zero(msg.angular_velocity_covariance):
            out.angular_velocity_covariance = [self._cov_w, 0.0, 0.0,
                                               0.0, self._cov_w, 0.0,
                                               0.0, 0.0, self._cov_w]
            if not self._fixed:
                self.get_logger().warn(
                    'IMU 角速度协方差为全零，已替换为 %s（否则 EKF 会把它当绝对真理）'
                    % self._cov_w)
                self._fixed = True
        else:
            out.angular_velocity_covariance = list(msg.angular_velocity_covariance)
            # z 轴单独兜底：有些驱动只填了 x/y
            if out.angular_velocity_covariance[8] < 1e-12:
                out.angular_velocity_covariance[8] = self._cov_w

        if self._force or self._all_zero(msg.linear_acceleration_covariance):
            out.linear_acceleration_covariance = [self._cov_a, 0.0, 0.0,
                                                  0.0, self._cov_a, 0.0,
                                                  0.0, 0.0, self._cov_a]
        else:
            out.linear_acceleration_covariance = \
                list(msg.linear_acceleration_covariance)

        if self._force or self._all_zero(msg.orientation_covariance):
            out.orientation_covariance = [self._cov_o, 0.0, 0.0,
                                          0.0, self._cov_o, 0.0,
                                          0.0, 0.0, self._cov_o]
        else:
            out.orientation_covariance = list(msg.orientation_covariance)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuPreprocess()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
