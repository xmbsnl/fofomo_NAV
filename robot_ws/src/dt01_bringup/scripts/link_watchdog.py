#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""link_watchdog —— 车↔上位机 无线链路体检，结果发到 /dt01/network。

真机上 90% 的"导航莫名其妙卡住"最后都查到网络：
  * WiFi 信号弱 -> /scan 丢帧 -> 代价地图出现空洞 -> DWB 判定不可行
  * 多播被路由器关了 -> 上位机根本看不见车的话题
  * 4G 路由器 NAT 抖动 -> 控制指令延迟几百毫秒 -> 机器人画龙

这个节点每 2s ping 一次上位机，把 RTT / 丢包率发出来，
GUI 的「网络」一栏就能直接看到链路质量，不用再去开终端敲 ping。

只做观测、不做控制：链路断了不会自动停车（那是 safety_mux 的职责），
因为本设计里导航栈跑在工控机上，断网时机器人应该自己把任务跑完。
"""
import subprocess
import shutil
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


class LinkWatchdog(Node):
    def __init__(self):
        super().__init__('link_watchdog')

        self.declare_parameter('host_ip', '')       # 留空=只报本机接口
        self.declare_parameter('interval', 2.0)
        self.declare_parameter('ping_count', 3)
        self.declare_parameter('topic', '/dt01/network')

        self._ip = self.get_parameter('host_ip').value
        interval = float(self.get_parameter('interval').value)
        self._count = int(self.get_parameter('ping_count').value)

        latch_qos = QoSProfile(depth=1,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(
            String, self.get_parameter('topic').value, latch_qos)

        if self._ip:
            self.get_logger().info(f'链路监测已启动：ping {self._ip}')
        else:
            self.get_logger().warn(
                '未设置 host_ip，只上报本机 IP。'
                '用法：ros2 run dt01_bringup link_watchdog.py '
                '--ros-args -p host_ip:=192.168.31.100')

        self.create_timer(interval, self._tick)
        self._tick()

    # ---------------- 工具 ----------------
    @staticmethod
    def _local_ip():
        try:
            out = subprocess.run(['hostname', '-I'], capture_output=True,
                                 text=True, timeout=2).stdout.split()
            return out[0] if out else '未知'
        except Exception:
            return '未知'

    def _ping(self, ip):
        """返回 (rtt_ms, loss_pct)，失败返回 (None, 100.0)。"""
        if not shutil.which('ping'):
            return None, None
        try:
            # -W 严格超时，避免网络不通时卡住整个定时器
            out = subprocess.run(
                ['ping', '-c', str(self._count), '-W', '1', '-q', ip],
                capture_output=True, text=True, timeout=self._count + 3
            ).stdout
        except Exception:
            return None, 100.0

        loss = 100.0
        m = re.search(r'([\d.]+)% packet loss', out)
        if m:
            loss = float(m.group(1))
        rtt = None
        m = re.search(r'(?:min/avg/max/\w+ =|rtt min/avg/max/mdev =)\s+'
                      r'[\d.]+/([\d.]+)/', out)
        if m:
            rtt = float(m.group(1))
        return rtt, loss

    def _tick(self):
        local = self._local_ip()
        if not self._ip:
            self.pub.publish(String(data=f'本机 {local}（未配置 host_ip）'))
            return

        rtt, loss = self._ping(self._ip)
        if loss is None:
            text = f'本机 {local} -> {self._ip}：ping 不可用'
        elif loss >= 100.0:
            text = f'本机 {local} -> {self._ip}：✗ 链路中断'
        else:
            level = '良好' if loss < 1 and (rtt or 99) < 10 else \
                    ('一般' if loss < 10 and (rtt or 99) < 50 else '差')
            rtt_s = f'{rtt:.1f}ms' if rtt is not None else '--'
            text = (f'本机 {local} -> {self._ip}：{level} '
                    f'RTT={rtt_s} 丢包={loss:.0f}%')
        self.pub.publish(String(data=text))


def main():
    rclpy.init()
    node = LinkWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
