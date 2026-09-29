#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DT-01 真机导航栈（AMCL + Nav2）—— 默认跑在工控机上。

用法（工控机上，dt01_robot 已启动的前提下）：
  ros2 launch dt01_bringup dt01_nav.launch.py map_file:=/home/teamhd/maps/office.yaml
  ros2 launch dt01_bringup dt01_nav.launch.py map_file:=... params_file:=.../nav2_params_real.yaml

也可以跑在上位机（DDS 配好之后），此时 /scan /odom /tf 全部走 WiFi。
但**不建议**：网络一抖控制指令就断，机器人会失控。
设计上导航栈应该和驱动在同一台机器上，上位机只做 RViz/GUI 监控。

指令链（比仿真版多了一级 safety_mux）：

  controller_server ─┐
  behavior_server  ─┴─> /cmd_vel_nav_raw
                          └─> velocity_smoother -> /cmd_vel_nav
                                └─> [safety_mux] -> /cmd_vel -> 底盘

为什么不直接让 controller 发到 /cmd_vel_nav？
  因为 behavior_server 的 spin/backup 恢复动作也要经过平滑和安全仲裁，
  否则"卡死自动脱困"这一路会绕过安全层，倒车撞人。

注意：TF 话题必须保持默认 /tf 与 /tf_static，不要 remap 成 tf，
否则 tf2_ros.Buffer 和 Nav2 内部都收不到 map 坐标系。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('dt01_bringup')
    default_params = os.path.join(pkg_dir, 'config', 'nav2_params_real.yaml')
    default_map = os.path.join(os.path.expanduser('~'), 'dt01_maps', 'map.yaml')

    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Nav2 输出先汇总到 /cmd_vel_nav_raw，平滑后再经 safety_mux 到底盘
    cmd_remaps = [('cmd_vel', 'cmd_vel_nav_raw')]

    # ==== 定位 ====
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_file,
                                  'use_sim_time': use_sim_time}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # ==== 导航 ====
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=cmd_remaps,
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=cmd_remaps,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav_raw'),
                    ('cmd_vel_smoothed', 'cmd_vel_nav')],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Nav2 参数文件'),
        DeclareLaunchArgument('map_file', default_value=default_map,
                              description='地图 yaml 路径'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='真机必须为 false'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='自动拉起 Nav2 生命周期节点'),

        map_server,
        amcl,
        lifecycle_manager_localization,
        # 延迟 3s：等 map_server 把地图加载完、AMCL 完成初始化，
        # 否则导航节点会因为拿不到 map 坐标系而反复报变换失败
        TimerAction(
            period=3.0,
            actions=[
                controller_server,
                smoother_server,
                planner_server,
                behavior_server,
                bt_navigator,
                waypoint_follower,
                velocity_smoother,
                lifecycle_manager_navigation,
            ],
        ),
    ])
