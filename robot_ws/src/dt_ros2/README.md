# DT系列底盘 相关 ROS 包

ROS包 话题参数说明

## ROS包信息

包名: dt_ros2 <br>
程序名: dt_ros2_node <br>
节点名: dt_ros2_node

## 话题

发布话题名称: /dt/frame_info <br>
发布话题数据: dt_ros2::msg::Frame <br>
发布话题说明: 原始数据

发布话题名称: /dt/drive_left_error_info <br>
发布话题数据: dt_ros2::msg::DriveHollysysError 或 dt_ros2::msg::DriveSdfzError <br>
发布话题说明: 驱动器左路异常

发布话题名称: /dt/drive_right_error_info <br>
发布话题数据: dt_ros2::msg::DriveHollysysError 或 dt_ros2::msg::DriveSdfzError <br>
发布话题说明: 驱动器右路异常

发布话题名称: /dt/auto_charge_info <br>
发布话题数据: robot_ros2_msgs::msg::AutoCharge <br>
发布话题说明: 回充状态信息

发布话题名称: /dt/battery_info <br>
发布话题数据: robot_ros2_msgs::msg::BatteryState <br>
发布话题说明: 主板采集的电池信息

发布话题名称: /dt/bms_info <br>
发布话题数据: robot_ros2_msgs::msg::Bms <br>
发布话题说明: 主板采集的BMS信息

发布话题名称: /dt/date_info <br>
发布话题数据: robot_ros2_msgs::msg::ChassisDate <br>
发布话题说明: 底盘出货日期

发布话题名称: /dt/parameter_info <br>
发布话题数据: robot_ros2_msgs::msg::ChassisParameter <br>
发布话题说明: 底盘参数

发布话题名称: /dt/state_info <br>
发布话题数据: robot_ros2_msgs::msg::ChassisState <br>
发布话题说明: 底盘状态

发布话题名称: /dt/velocity_info <br>
发布话题数据: robot_ros2_msgs::msg::ChassisVelocity <br>
发布话题说明: 底盘 线速度 角速度

发布话题名称: /dt/hardware_version_info <br>
发布话题数据: robot_ros2_msgs::msg::HardwareVersion <br>
发布话题说明: 底盘硬件版本

发布话题名称: /dt/led_strip_mode_info <br>
发布话题数据: std_msgs::msg::UInt8 <br>
发布话题说明: 灯条控制模式

发布话题名称: /dt/led_strips_info <br>
发布话题数据: robot_ros2_msgs::msg::LedStrips <br>
发布话题说明: 所有灯条参数

发布话题名称: /dt/remote_control_info <br>
发布话题数据: robot_ros2_msgs::msg::RemoteControl <br>
发布话题说明: 遥控数据

发布话题名称: /dt/software_version_info <br>
发布话题数据: robot_ros2_msgs::msg::SoftwareVersion <br>
发布话题说明: 底盘软件版本

发布话题名称: /dt/current_info <br>
发布话题数据: robot_ros2_msgs::msg::TwoWheelDiffCurrent <br>
发布话题说明: 两轮差速底盘电流信息

发布话题名称: /dt/speed_info <br>
发布话题数据: robot_ros2_msgs::msg::TwoWheelDiffSpeed <br>
发布话题说明: 两轮差速底盘轮速信息

发布话题名称: /dt/odom_info <br>
发布话题数据: nav_msgs::msg::Odometry <br>
发布话题说明: 底盘里程信息

发布话题名称: /dt/battery_state <br>
发布话题数据: sensor_msgs::msg::BatteryState <br>
发布话题说明: 电池信息

发布话题名称: /dt/std_stop_button_info <br>
发布话题数据: std_msgs::msg::Bool <br>
发布话题说明: 统一急停按钮状态

发布话题名称: /dt/std_software_stop_info <br>
发布话题数据: std_msgs::msg::Bool <br>
发布话题说明: 统一软件急停状态

发布话题名称: /dt/std_front_collision_info <br>
发布话题数据: std_msgs::msg::Bool <br>
发布话题说明: 统一前碰撞状态

发布话题名称: /dt/std_rear_collision_info <br>
发布话题数据: std_msgs::msg::Bool <br>
发布话题说明: 统一后碰撞状态

---

订阅话题名称: /dt/velocity_ctrl <br>
订阅话题数据: geometry_msgs::msg::Twist <br>
订阅话题说明: 线速度 角速度 控制

订阅话题名称: /dt/speed_ctrl <br>
订阅话题数据: robot_ros2_msgs::msg::TwoWheelDiffSpeed <br>
订阅话题说明: 两轮差速底盘轮速控制

订阅话题名称: /dt/stop_ctrl <br>
订阅话题数据: std_msgs::msg::Bool <br>
订阅话题说明: 底盘急停控制, [true: 打开] [false: 关闭]

订阅话题名称: /dt/collision_clean <br>
订阅话题数据: std_msgs::msg::Empty <br>
订阅话题说明: 碰撞状态清除

订阅话题名称: /dt/fault_clean <br>
订阅话题数据: std_msgs::msg::Empty <br>
订阅话题说明: 故障尝试清除

订阅话题名称: /dt/auto_charge_ctrl <br>
订阅话题数据: std_msgs::UInt8 <br>
订阅话题说明: 回充开关, [0: 关闭回充] [1: 开启红外回充] [2: 开启激光回充]

订阅话题名称: /dt/led_strip_mode_ctrl <br>
订阅话题数据: std_msgs::msg::UInt8 <br>
订阅话题说明: 灯条控制模式切换, [0: 底盘控制] [1: 上位机控制]

订阅话题名称: /dt/led_strip_ctrl <br>
订阅话题数据: robot_ros2_msgs::msg::LedStrip <br>
订阅话题说明: 灯条数据设置

订阅话题名称: /dt/led_strips_ctrl <br>
订阅话题数据: robot_ros2_msgs::msg::LedStrips <br>
订阅话题说明: 多条灯条设置

订阅话题名称: /dt/odom_clean <br>
订阅话题数据: std_msgs::msg::Empty <br>
订阅话题说明: 里程计清零

订阅话题名称: /dt/std_stop_ctrl <br>
订阅话题数据: std_msgs::msg::Bool <br>
订阅话题说明: 统一软件急停, [true: 有效] [false: 无效]

订阅话题名称: /dt/std_all_collision_clean <br>
订阅话题数据: std_msgs::msg::Empty <br>
订阅话题说明: 统一所有碰撞清除

订阅话题名称: /dt/std_front_collision_clean <br>
订阅话题数据: std_msgs::msg::Empty <br>
订阅话题说明: 统一前碰撞清除

订阅话题名称: /dt/std_rear_collision_clean <br>
订阅话题数据: std_msgs::msg::Empty <br>
订阅话题说明: 保留话题未生效，统一后碰撞清除

订阅话题名称: /dt/std_all_lamp_ctrl <br>
订阅话题数据: std_msgs::msg::Bool <br>
订阅话题说明: 保留话题未生效，统一所有照明灯开关, [true: 打开] [false: 关闭]

订阅话题名称: /dt/std_front_lamp_ctrl <br>
订阅话题数据: std_msgs::msg::Bool <br>
订阅话题说明: 保留话题未生效，统一前照明灯开关, [true: 打开] [false: 关闭]

订阅话题名称: /dt/std_rear_lamp_ctrl <br>
订阅话题数据: std_msgs::msg::Bool <br>
订阅话题说明: 保留话题未生效，统一后照明灯开关, [true: 打开] [false: 关闭]

## 参数

参数名: dt_port <br>
数据类型: string <br>
默认值: /dev/ttyUSB0 <br>
参数说明: 通讯接口

参数名: dt_baudrate <br>
数据类型: int <br>
默认值: 115200 <br>
参数说明: 通讯波特率

参数名: dt_odom_enable <br>
数据类型: bool <br>
默认值: true <br>
参数说明: 里程计开关

参数名: dt_drive_type <br>
数据类型: string <br>
默认值: SDFZ <br>
参数说明: 驱动器类型，"SDFZ" or "HLS"

参数名: dt_log_display <br>
数据类型: bool <br>
默认值: true <br>
参数说明: 日志信息显示开关

参数名: dt_original_display <br>
数据类型: bool <br>
默认值: false <br>
参数说明: 原始数据输出开关

## 版本信息

当前: V 1.0.4 <br>

V 1.0.4 <br>
++对应协议版本《DT-01(SDFZ)底盘CAN通讯协议 - V1.0.0.251126》《DT-01 Pro(SDFZ)底盘CAN通讯协议 - V1.0.2.251106》 <br>
++对应消息包版本【robot_ros2_msgs v1.0.2】 <br>
++随协议改动做出更新 <br>
++新增标准消息发布和订阅

V 1.0.3 <br>
++新增通用bms信息发布。 <br>
++新增ros电池信息发布。

V 1.0.2 <br>
++log信息改为使用ros官方log输出。

V 1.0.1 <br>
++修复发送粘包问题

V 1.0.0 <br>
++首次创建

## 其他
