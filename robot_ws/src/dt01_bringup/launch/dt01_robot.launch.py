#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DT-01 真机「机器人端」主入口 —— 跑在工控机（RK3588）上。

一次拉起 10 个节点，分四层：

  【驱动层】dt_ros2(底盘) + bluesea2(雷达) + fdilink_ahrs(IMU)
  【模型层】robot_state_publisher(URDF 静态 TF)
  【融合层】imu_preprocess -> ekf_node -> odom_to_tf（use_ekf 控制）
  【安全层】chassis_bridge(遥测) + safety_mux(指令仲裁) + link_watchdog(链路体检)

指令流（出问题时按这条链逐段排查）：

  Nav2 -> /cmd_vel_nav ─┐
  遥操 -> /cmd_vel_teleop ─┴─> [safety_mux] -> /cmd_vel -> dt_ros2 -> 串口 -> 底盘

TF 树（单亲，任何一环断了整棵树就废）：

  map -> odom -> base_footprint -> base_link -> base_scan / imu_link
         ^AMCL   ^odom_to_tf      ^URDF(robot_state_publisher)

用法（工控机上）：
  ros2 launch dt01_bringup dt01_robot.launch.py
  ros2 launch dt01_bringup dt01_robot.launch.py chassis_port:=/dev/dt01_chassis \\
      lidar_port:=/dev/dt01_lidar imu_port:=/dev/dt01_imu host_ip:=192.168.31.100
  # 网口雷达
  ros2 launch dt01_bringup dt01_robot.launch.py lidar_type:=udp
  # 第一次联调先关掉 IMU 融合，确认纯轮速里程计没问题再开
  ros2 launch dt01_bringup dt01_robot.launch.py fuse_imu:=false

注意：本文件只起「机器人」。建图/导航另起一个 launch（dt01_slam / dt01_nav），
两边都不会互相拉起对方，方便单独重启导航栈而不动驱动。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('dt01_bringup')
    urdf_xacro = os.path.join(pkg_dir, 'urdf', 'dt01_real.urdf.xacro')
    lidar_uart_yaml = os.path.join(pkg_dir, 'config', 'uart_lidar.yaml')
    lidar_udp_yaml = os.path.join(pkg_dir, 'config', 'udp_lidar.yaml')
    ekf_yaml = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    # ---------------- 参数 ----------------
    chassis_port = LaunchConfiguration('chassis_port')
    chassis_baud = LaunchConfiguration('chassis_baud')
    drive_type = LaunchConfiguration('drive_type')
    lidar_type = LaunchConfiguration('lidar_type')
    lidar_port = LaunchConfiguration('lidar_port')
    imu_port = LaunchConfiguration('imu_port')
    imu_baud = LaunchConfiguration('imu_baud')
    use_ekf = LaunchConfiguration('use_ekf')
    fuse_imu = LaunchConfiguration('fuse_imu')
    use_safety = LaunchConfiguration('use_safety')
    host_ip = LaunchConfiguration('host_ip')
    base_frame = LaunchConfiguration('base_frame')

    # ---------------- 1. 底盘驱动 dt_ros2 ----------------
    chassis_node = Node(
        package='dt_ros2',
        executable='dt_ros2_node',
        name='dt_ros2_node',
        output='screen',
        parameters=[{
            'dt_port': chassis_port,
            'dt_baudrate': chassis_baud,
            'dt_odom_enable': True,
            'dt_drive_type': drive_type,
            'dt_log_display': True,
            'dt_original_display': False,
        }],
        remappings=[
            ('/dt/velocity_ctrl', '/cmd_vel'),   # 订阅：safety_mux 的输出
            ('/dt/odom_info', '/odom'),          # 发布：轮式里程计
        ],
    )

    # ---------------- 2. 激光雷达 bluesea2 ----------------
    # 参数文件按连接方式二选一，公共字段在后面统一覆盖（后者优先）
    lidar_params_file = PythonExpression(
        ["'", lidar_udp_yaml, "' if '", lidar_type, "' == 'udp' else '",
         lidar_uart_yaml, "'"])
    lidar_node = Node(
        package='bluesea2',
        executable='bluesea2_node',
        name='bluesea_node',
        output='screen',
        parameters=[lidar_params_file, {
            'frame_id': 'base_scan',     # 必须匹配 URDF 中的雷达 link
            'scan_topic': 'scan',
            'cloud_topic': 'cloud',
            'output_scan': True,
            'output_cloud2': False,      # 单线雷达点云对导航无用，省带宽
            'min_dist': 0.1,
            'max_dist': 50.0,
            'type': lidar_type,
            'port': lidar_port,
        }],
    )

    # ---------------- 3. IMU fdilink_ahrs ----------------
    imu_node = Node(
        package='fdilink_ahrs',
        executable='ahrs_driver_node',
        name='ahrs_driver_node',
        output='screen',
        parameters=[{
            'if_debug_': False,
            'serial_port_': imu_port,
            'serial_baud_': imu_baud,
            'imu_topic': '/imu/data',
            'imu_frame_id_': 'imu_link',   # 匹配 URDF
            'device_type_': 1,
            'mag_pose_2d_topic': '/mag_pose_2d',
            'Magnetic_topic': '/magnetic',
            'Euler_angles_topic': '/euler_angles',
            'gps_topic': '/gps/fix',
            'twist_topic': '/system_speed',
            'NED_odom_topic': '/NED_odometry',
        }],
    )

    # ---------------- 4. URDF / 静态 TF ----------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_xacro]),
                value_type=str),
        }],
    )

    # ---------------- 5. IMU 清洗 ----------------
    # 只在 fuse_imu:=true 时启动。不融合 IMU 时启动它纯属浪费。
    # 它同时负责把厂家 IMU 的全零协方差换成合理值 —— 全零在 EKF 里
    # 表示"完美可信"，会直接压过轮式里程计。
    imu_preprocess = Node(
        package='dt01_bringup',
        executable='imu_preprocess.py',
        name='imu_preprocess',
        output='screen',
        condition=IfCondition(fuse_imu),
        parameters=[{
            'input_topic': '/imu/data',
            'output_topic': '/imu/data_fused',
            'frame_id': 'imu_link',
            # 若发现原地转圈时 /odometry/filtered 的 yaw 反向增长，
            # 说明 IMU 安装方向与 ROS 的 ENU 约定相反，把这里改成 -1.0
            'yaw_rate_sign': 1.0,
            # 陀螺 z 轴零偏（2026-09-18 静止实测 0.003415 rad/s ≈ 0.196°/s）。
            # 不扣的话 EKF 静止 yaw 每 30 秒漂 ~6°，转弯时重影明显。
            'gyro_bias_z': 0.003415,
        }],
    )

    # ---------------- 6. EKF 融合 ----------------
    # 节点名必须叫 ekf_filter_node，才能对上 ekf.yaml 的顶层键。
    # fuse_imu=false 时 imu_preprocess 不启动 -> /imu/data_fused 无人发布
    # -> EKF 自然只用轮式里程计，不需要改任何融合矩阵。
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[ekf_yaml],
    )

    # ---------------- 7. 里程计 TF 桥 ----------------
    # 只广播动态段 odom -> base_footprint；
    # 静态段 base_footprint -> base_link -> 传感器 由 URDF 提供。
    # 若这里直接广播 odom -> base_link，base_link 会有两个父帧，TF 树断裂。
    odom_tf_node = Node(
        package='dt01_bringup',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{
            'odom_topic': PythonExpression(
                ["'/odometry/filtered' if '", use_ekf, "' == 'true' else '/odom'"]),
            'base_frame': base_frame,     # 固定 base_footprint，不要改成 base_link
            'publish_tf': True,
        }],
    )

    # ---------------- 8. 底盘遥测桥 ----------------
    chassis_bridge = Node(
        package='dt01_bringup',
        executable='chassis_bridge.py',
        name='chassis_bridge',
        output='screen',
        parameters=[{
            'status_topic': '/dt01/chassis_status',
            'safety_topic': '/dt01/safety',
            'battery_topic': '/battery_state',
            'publish_rate': 10.0,
        }],
    )

    # ---------------- 9. 安全仲裁 ----------------
    safety_mux = Node(
        package='dt01_bringup',
        executable='safety_mux.py',
        name='safety_mux',
        output='screen',
        condition=IfCondition(use_safety),
        parameters=[{
            'nav_topic': '/cmd_vel_nav',
            'teleop_topic': '/cmd_vel_teleop',
            'out_topic': '/cmd_vel',
            'rate': 20.0,
            'source_timeout': 0.5,
            'max_linear': 1.0,
            'max_angular': 1.5,
            'use_laser_stop': True,
            'laser_stop_distance': 0.30,
            'laser_slow_distance': 0.60,
        }],
    )

    # ---------------- 10. 链路体检 ----------------
    link_watchdog = Node(
        package='dt01_bringup',
        executable='link_watchdog.py',
        name='link_watchdog',
        output='screen',
        condition=IfCondition(PythonExpression(["'", host_ip, "' != ''"])),
        parameters=[{'host_ip': host_ip, 'topic': '/dt01/network'}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('chassis_port', default_value='/dev/ttyUSB0',
                              description='底盘串口（建议用 udev 固定名 /dev/dt01_chassis）'),
        DeclareLaunchArgument('chassis_baud', default_value='115200',
                              description='底盘波特率（协议 115200 8N1）'),
        DeclareLaunchArgument('drive_type', default_value='SDFZ',
                              description='驱动器类型：SDFZ 或 HLS'),
        DeclareLaunchArgument('lidar_type', default_value='uart',
                              description='雷达连接方式：uart 串口 / udp 网口'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1',
                              description='雷达串口（uart 模式）'),
        DeclareLaunchArgument('imu_port', default_value='/dev/fdilink_ahrs',
                              description='IMU 串口（建议 udev 固定为 /dev/dt01_imu）'),
        DeclareLaunchArgument('imu_baud', default_value='921600',
                              description='IMU 波特率'),
        DeclareLaunchArgument('use_ekf', default_value='true',
                              description='是否启用 EKF 里程计融合'),
        DeclareLaunchArgument('fuse_imu', default_value='false',
                              description='EKF 是否融合 IMU 角速度。'
                                          '首次联调设 false，确认 IMU 方向后再开 true'),
        DeclareLaunchArgument('use_safety', default_value='true',
                              description='是否启用 safety_mux 安全仲裁。'
                                          '设 false 时 Nav2 速度指令无法到达底盘，'
                                          '只用于静止调试传感器'),
        DeclareLaunchArgument('host_ip', default_value='',
                              description='上位机 IP，用于链路体检；留空则不监测'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint',
                              description='TF 动态段的子帧，固定 base_footprint'),

        chassis_node,
        lidar_node,
        imu_node,
        robot_state_publisher,
        imu_preprocess,
        ekf_node,
        odom_tf_node,
        chassis_bridge,
        safety_mux,
        link_watchdog,
    ])
