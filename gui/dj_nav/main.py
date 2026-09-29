#!/usr/bin/env python3
"""DJ-NAV 1.0 入口（真机专用版，无仿真）。

用法：
  source /opt/ros/humble/setup.bash
  python3 dj_nav/main.py
或：
  bash run.sh
"""
import os
import sys

# 允许以脚本方式直接运行（python3 main.py）
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from PyQt5.QtWidgets import QApplication

from dj_nav.ros_bridge import RosBridge
from dj_nav.main_window import MainWindow, DARK_QSS


def main():
    rclpy.init()
    # 真机版：始终用系统时钟（仿真 /clock 已移除）
    bridge = RosBridge(use_sim_time=False)

    app = QApplication(sys.argv)
    app.setApplicationName('DJ-NAV')
    app.setStyleSheet(DARK_QSS)

    win = MainWindow(bridge)
    win.show()

    ret = app.exec_()

    bridge.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
