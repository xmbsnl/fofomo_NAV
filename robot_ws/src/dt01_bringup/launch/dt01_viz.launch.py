#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DT-01 上位机可视化入口 —— 跑在你自己的电脑上，不跑在工控机上。

前置（两台机器都要做）：
  1. 同一 WiFi / 同一网段
  2. 两边都执行过 `bash setup_dds.sh`（配置 CycloneDDS + ROS_DOMAIN_ID）
  3. 两边系统时间已同步（chrony）

用法（你的电脑上）：
  ros2 launch dt01_bringup dt01_viz.launch.py

跑起来先确认三件事，否则后面的排查全是白费：
  ros2 topic list            # 应能看到工控机上的 /scan /odom /imu/data /tf
  ros2 topic hz /scan        # 有频率才说明数据真的过来了
  ros2 topic echo /dt01/network --once   # 看链路 RTT / 丢包

看不到话题 = DDS 没通，回到 setup_dds.sh，而不是去怀疑驱动。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('dt01_bringup')
    default_rviz = os.path.join(pkg_dir, 'config', 'dt01_real.rviz')

    rviz_config = LaunchConfiguration('rviz_config')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=default_rviz,
                              description='RViz 配置文件'),
        rviz2,
    ])
