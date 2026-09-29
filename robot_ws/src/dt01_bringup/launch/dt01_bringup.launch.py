#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""【已合并，保留兼容】真机一键启动 —— 现在等价于 dt01_robot.launch.py。

历史上这里直接罗列了驱动三件套 + TF + URDF。后来加入了 EKF 融合、
安全仲裁、遥测桥、链路体检，节点数从 5 个涨到 10 个，
统一收敛到 dt01_robot.launch.py。

本文件保留原来的参数名（chassis_port / lidar_type / lidar_port / imu_port），
老脚本不用改。新代码请直接用 dt01_robot.launch.py。

  ros2 launch dt01_bringup dt01_bringup.launch.py
  ros2 launch dt01_bringup dt01_bringup.launch.py chassis_port:=/dev/dt01_chassis
  ros2 launch dt01_bringup dt01_bringup.launch.py lidar_type:=udp
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_launch = PathJoinSubstitution(
        [FindPackageShare('dt01_bringup'), 'launch', 'dt01_robot.launch.py'])

    chassis_port = LaunchConfiguration('chassis_port')
    lidar_type = LaunchConfiguration('lidar_type')
    lidar_port = LaunchConfiguration('lidar_port')
    imu_port = LaunchConfiguration('imu_port')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('chassis_port', default_value='/dev/ttyUSB0',
                              description='底盘串口（兼容旧参数）'),
        DeclareLaunchArgument('lidar_type', default_value='uart',
                              description='雷达连接方式：uart / udp（兼容旧参数）'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1',
                              description='雷达串口（兼容旧参数）'),
        DeclareLaunchArgument('imu_port', default_value='/dev/fdilink_ahrs',
                              description='IMU 串口（兼容旧参数）'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='真机必须为 false（兼容旧参数）'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                'chassis_port': chassis_port,
                'lidar_type': lidar_type,
                'lidar_port': lidar_port,
                'imu_port': imu_port,
            }.items(),
        ),
    ])
