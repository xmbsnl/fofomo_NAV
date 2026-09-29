#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""odom_to_tf —— 把里程计消息广播为 TF（odom -> base_footprint）。

为什么要单独写这个节点（以及为什么不能按 child_frame_id 原样转发）
------------------------------------------------------------------
1. 厂家底盘驱动 dt_ros2 发布 nav_msgs/Odometry：
       header.frame_id    = 'odom'
       child_frame_id     = 'base_link'
   并且**它自己不广播任何 TF**。

2. URDF（robot_state_publisher）会广播静态链：
       base_footprint -> base_link -> base_scan / imu_link

3. 如果这里照抄 child_frame_id 广播 odom -> base_link，base_link 就会
   同时拥有 odom 与 base_footprint 两个父帧。tf2 不允许一个 frame 有
   多个 parent，结果是整个 TF 树断裂，AMCL / costmap / RViz 全部报
   "Frame base_link has multiple parents" 或查不到变换。

正确做法：本节点只广播动态段 odom -> base_footprint，静态段交给 URDF，
最终得到一棵单亲树：

    map -> odom -> base_footprint -> base_link -> base_scan / imu_link
           ^^^^^^^^^^^^^^^^^^^^^^^^^ 本节点
                                    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ URDF

参数
----
odom_topic  : 订阅的里程计话题。接 EKF 时应填 /odometry/filtered。
base_frame  : 发布的子帧，固定 base_footprint（不要改成 base_link）。
odom_frame  : 发布的父帧，默认取消息里的 header.frame_id。
publish_tf  : 关掉可让 EKF 自己发 TF（二选一，不要两个都发）。
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', '')        # 空 = 沿用消息自带
        self.declare_parameter('publish_tf', True)

        self._base_frame = self.get_parameter('base_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        odom_topic = self.get_parameter('odom_topic').value

        self.broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, 20)

        # 诊断用：超过 3s 没数据说明上游（驱动或 EKF）挂了
        self._last_stamp = None
        self._warned = False
        self.create_timer(1.0, self._watchdog)

        self.get_logger().info(
            f'odom_to_tf: {odom_topic} -> TF {self._odom_frame or "<msg.frame_id>"}'
            f' -> {self._base_frame} (publish_tf={self._publish_tf})')

    def _odom_callback(self, msg: Odometry):
        self._last_stamp = self.get_clock().now()
        self._warned = False
        if not self._publish_tf:
            return

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self._odom_frame or msg.header.frame_id or 'odom'
        t.child_frame_id = self._base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)

    def _watchdog(self):
        if self._last_stamp is None or self._warned:
            return
        if (self.get_clock().now() - self._last_stamp).nanoseconds > 3e9:
            self._warned = True
            self.get_logger().error(
                '超过 3s 没有收到里程计数据，TF 已停止更新！'
                '检查底盘驱动 / EKF 是否还在跑。')


def main():
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
