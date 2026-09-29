#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DT-01 一键启动：机器人（底盘/雷达/IMU/安全层）+ 建图 或 导航，一个命令全拉起。

用法（工控机上，任意目录）：

  # 导航模式（默认，地图默认 ~/dt01_maps/office.yaml）
  ros2 launch dt01_bringup dt01_all.launch.py

  # 指定别的地图
  ros2 launch dt01_bringup dt01_all.launch.py map_file:=/home/teamhd/dt01_maps/room2.yaml

  # 建图模式
  ros2 launch dt01_bringup dt01_all.launch.py mode:=slam

  # 底盘参数与 dt01_robot.launch.py 相同，按需覆盖
  ros2 launch dt01_bringup dt01_all.launch.py chassis_port:=/dev/ttyS7 \
      lidar_type:=udp imu_port:=/dev/dt01_imu
  # 临时关掉 IMU 融合（排查方向问题时用）
  ros2 launch dt01_bringup dt01_all.launch.py fuse_imu:=false

关于 fuse_imu（默认 false）
------------------------
EKF 一直开着（use_ekf 默认 true），fuse_imu 控制的是"要不要把 IMU 角速度喂给 EKF"。

【2026-09-18 已开启（默认 true）】
09-08 不敢开的原因：400Hz 全量灌 EKF 把 RK3588 打满（load 10.8），静止 yaw
8 秒漂 187°。两步整改后验收通过：
  1. imu_preprocess 降采样 400→50Hz（09-15 完成）：/imu/data_fused 实测 46.7Hz
  2. 扣除陀螺零偏 gyro_bias_z=0.003415 rad/s（09-18 实测：不扣则静止 yaw
     以恒定 0.196°/s 漂移，30s 漂 5.9°；扣后峰峰 0.10°/30s，PASS）
负载 15 分钟均值 ~4（标准 <5）。IMU 安装：重力 Z 朝上，yaw_rate_sign=+1.0。
临时关闭（排查方向问题时用）：
  ros2 launch dt01_bringup dt01_all.launch.py fuse_imu:=false

停止：在本终端按一次 Ctrl+C，所有节点一起退出。
注意：导航栈内部自带 3s 延迟启动（等 map_server 加载完），启动后稍等几秒再给初始位姿。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_dir = get_package_share_directory('dt01_bringup')
    launch_dir = os.path.join(pkg_dir, 'launch')
    default_map = os.path.join(os.path.expanduser('~'), 'dt01_maps', 'office.yaml')

    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map_file')

    # ---- 机器人（驱动 + 模型 + 安全层），所有模式都需要 ----
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dt01_robot.launch.py')),
        launch_arguments={
            'chassis_port': LaunchConfiguration('chassis_port'),
            'lidar_type': LaunchConfiguration('lidar_type'),
            'imu_port': LaunchConfiguration('imu_port'),
            'fuse_imu': LaunchConfiguration('fuse_imu'),
        }.items(),
    )

    # ---- 建图（mode:=slam 时启用）----
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dt01_slam.launch.py')),
        condition=IfCondition(PythonExpression(
            ["'", mode, "' == 'slam'"])),
    )

    # ---- 导航（默认；延迟 5s，等机器人端 TF/话题就绪）----
    nav = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'dt01_nav.launch.py')),
            launch_arguments={'map_file': map_file}.items(),
            condition=IfCondition(PythonExpression(
                ["'", mode, "' == 'nav'"])),
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='nav',
                              description='启动模式：nav（默认）或 slam'),
        DeclareLaunchArgument('map_file', default_value=default_map,
                              description='导航用地图 yaml（slam 模式忽略）'),
        # 底盘参数默认值按本机实际接线写死，一条命令即可启动
        DeclareLaunchArgument('chassis_port', default_value='/dev/ttyS7'),
        DeclareLaunchArgument('lidar_type', default_value='udp'),
        DeclareLaunchArgument('imu_port', default_value='/dev/dt01_imu'),
        # 默认 true：降采样 + 零偏扣除后验收通过（详见文件头"关于 fuse_imu"）。
        # 旋转建图的重影主要靠它（陀螺角速度）压制。
        DeclareLaunchArgument('fuse_imu', default_value='true'),
        robot,
        slam,
        nav,
    ])
