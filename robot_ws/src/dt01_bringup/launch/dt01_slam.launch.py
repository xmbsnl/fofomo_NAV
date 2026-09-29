#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DT-01 真机建图（SLAM Toolbox 在线异步建图）。

前置：dt01_robot.launch.py 已经在跑（雷达/里程计/TF 都有数据）。

用法（工控机上）：
  # 终端 1：机器人
  ros2 launch dt01_bringup dt01_robot.launch.py
  # 终端 2：建图
  ros2 launch dt01_bringup dt01_slam.launch.py
  # 终端 3：遥控（在你的电脑上，或用 SSH 里的 teleop）
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop
  # 建完保存（也可以在 RViz 里用 Save Map 插件）
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$HOME/dt01_maps/office'}}"

注意遥控话题：真机上必须发到 /cmd_vel_teleop（走 safety_mux），
不要直接发 /cmd_vel —— 那会绕过安全仲裁，且会和 Nav2 抢底盘。

建图要点（真机）：
  * 速度别超过 0.3 m/s，转弯半径别太小 —— 低成本雷达运动畸变很明显
  * 走闭合回路，让 SLAM 有机会回环检测（loop closure），否则地图会歪
  * 场地中央的空旷区域要绕一圈把边界扫出来
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('dt01_bringup')
    default_params = os.path.join(pkg_dir, 'config', 'mapper_params_real.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='SLAM Toolbox 参数文件'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='真机必须为 false'),
        slam_toolbox,
    ])
