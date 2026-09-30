# -*- coding: utf-8 -*-
"""真机运行档案（Profile）—— DJ-NAV 1.0 专用，仅真机版本。

本文件是「真机差异」的唯一真相源：
  · 机器人本体（驱动+雷达+IMU+EKF+安全层）全部跑在工控机上，
    GUI 只通过 SSH 启停 + DDS 网络监控与下发，本机不起任何 ROS 进程。
  · 仿真相关内容已全部移除（原版见 01-simulation/dt01_sim/scripts/nav_gui）。

远程目标与工作空间改动时，只改本文件。
"""
import os

# 工程根：.../DJ-NAV1.0
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 地图目录：本地只是车上地图的"镜像副本"（供 GUI 显示与打点），
# 导航真正用的是工控机上的那份。首次连通时自动同步，无需手动拷贝。
MAPS_DIR = os.path.join(BASE_DIR, 'maps')
SAVED_MAPS_DIR = os.path.join(MAPS_DIR, 'saved')

# 点位文件：真机地图的打点坐标（与仿真完全不同坐标系，本版本无仿真）
POINTS_FILE = os.path.join(BASE_DIR, 'config', 'nav_points.json')

# 电子围栏文件：按地图分组的禁区多边形（GUI 画图生成，下发到工控机巡线节点）
KEEPOUT_FILE = os.path.join(BASE_DIR, 'config', 'keepout_zones.json')


class RealProfile:
    """真机档案：连工控机（SSH 启停 + DDS 监控与下发）。"""

    name = 'real'
    label = '真机'
    use_sim_time = False                 # 真机没有 /clock，用系统时钟
    is_remote = True                     # 恒为真：进程都在车上

    # 点位文件：真机地图的打点坐标
    points_file = POINTS_FILE

    # 电子围栏文件：按地图分组的禁区多边形
    keepout_file = KEEPOUT_FILE

    # 遥控话题：必须走 /cmd_vel_teleop —— 真机的 /cmd_vel 是 safety_mux
    # 的独占输出，直接发会绕过急停/防撞/激光急停，还会和 Nav2 抢底盘。
    cmd_vel_topic = '/cmd_vel_teleop'

    requires_hardware_link = True

    notes = '连工控机：GUI 通过 SSH 在车上启停 dt01_all.launch.py，本机只监控与下发。'

    # ---- 远程目标（SSH 免密登录必须已配好：ssh-copy-id teamhd@192.168.2.153）----
    remote_user = 'teamhd'
    remote_host = '192.168.2.153'
    remote_ws = '/home/teamhd/dt01_ws'
    remote_maps_dir = '/home/teamhd/dt01_maps'


# 本版本唯一的档案：界面不再有"仿真/实机"切换。
REAL = RealProfile()
