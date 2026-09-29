# DT-01 Navigation Stack

> DT-01 差速底盘 · RK3588 工控机 · ROS 2 Humble
> 打点连线巡线 · 建图导航一体 · 真车安全架构
>
> 📖 快速了解：[`docs/DJ-NAV-介绍.html`](docs/DJ-NAV-介绍.html)（浏览器直接打开）

DT-01 是一台差速底盘移动机器人（蓝海 2D 激光雷达 + 飞迪 N100 IMU + 6 路超声），
本仓库包含它的**完整导航系统**：

- **打点连线巡线**（核心模式）：在地图上打点即自动连成最短直线，机器人
  先原地转向对正下一点，再严格沿画出的线行驶，仅避障时短暂偏线、过障自动回线。
  控制算法（Pure Pursuit 前视点追踪）跑在机器人本机节点里，上位机断网 2 秒自动停车。
- **建图与定位**：slam_toolbox 建图 + Nav2/AMCL 定位，初始位姿按地图保存自动复用。
- **真车安全架构**：全部运动指令经 safety_mux 安全仲裁；遥控与自主分话题输入；
  全局急停（硬件遥控器 + 软件一键）；定位翻面检测；切图强制换图防坐标系错位。
- **上位机控制台（GUI）**：地图显示/打点、遥控、传感器监控、状态监控、日志，
  通过 SSH 启停机器人本体、通过 DDS 下发指令——本机不跑任何 ROS 进程。

## 仓库结构

```
DT-01-Robot/
├── docs/                     # 手册（先读这三份）
│   ├── 算法手册.md           #   现阶段全部算法：预处理/EKF/SLAM/AMCL/safety_mux/巡线控制
│   ├── 操作手册.md           #   上电、遥控器、GUI 全流程、急停、故障处理
│   ├── 传感器指南.md         #   每个传感器的接口/驱动/话题/配置/标定
│   └── DJ-NAV-介绍.html      #   图文介绍页
├── robot_ws/src/             # 机器人端 ROS 2 工作空间源码（编译后跑在工控机）
│   ├── dt01_bringup/         #   整机 launch / Nav2 参数 / 安全与辅助节点 / URDF
│   ├── dt_ros2/              #   底盘串口驱动（C++）
│   ├── bluesea2/             #   激光雷达驱动（厂商提供）
│   ├── fdilink_ahrs/         #   IMU 驱动（厂商提供，已移除测试数据）
│   ├── base/                 #   底盘服务定义
│   └── robot_ros2_msgs/      #   底盘消息定义
└── gui/                      # 上位机控制台 DJ-NAV 1.0（PyQt5，跑在笔记本）
    ├── dj_nav/               #   GUI 源码
    ├── route_follower_node.py#   巡线算法节点（GUI 自动部署到工控机运行）
    ├── run.sh
    └── README.md
```

## 快速开始

### 1. 机器人端（工控机，Ubuntu 22.04 + ROS 2 Humble）

```bash
sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox \
                 ros-humble-robot-localization ros-humble-rmw-cyclonedds-cpp
mkdir -p ~/dt01_ws/src && cp -r robot_ws/src/* ~/dt01_ws/src/
cd ~/dt01_ws && colcon build && source install/setup.bash

# 环境脚本（DDS / NTP 时间同步 / 串口 udev / 自检）
bash src/dt01_bringup/scripts/setup_dds.sh
bash src/dt01_bringup/scripts/setup_ntp.sh
bash src/dt01_bringup/scripts/setup_udev.sh

# 整机启动
ros2 launch dt01_bringup dt01_all.launch.py mode:=idle    # 只起本体
ros2 launch dt01_bringup dt01_all.launch.py mode:=slam    # 建图
```

### 2. 上位机（笔记本，跑 GUI）

```bash
# 免密登录工控机 + 生成 DDS 配置（详见 docs/操作手册.md 第 2 节）
ssh-keygen -t ed25519 && ssh-copy-id <user>@<工控机IP>

cd gui && bash run.sh
```

### 3. 一次完整作业

```
启动机器人 → 建图模式 → 遥控绕场（≤0.3m/s 走闭合回路）→ 另存地图
→ 导航模式 → 等定位（自动用保存的初始位姿）→ ＋打点（自动连线）
→ ▶ 开始导航 → 机器人沿画出的线巡线
```

急停：遥控器 SWD（硬件级，任何时候有效）/ 键盘 X / 顶栏 E-STOP。

## 文档

| 文档 | 内容 |
|---|---|
| [docs/算法手册.md](docs/算法手册.md) | 数据流、IMU 预处理、EKF、SLAM/AMCL（含镜像退化分析）、safety_mux 优先级、巡线 Pure Pursuit 公式与避障策略、全部参数速查 |
| [docs/操作手册.md](docs/操作手册.md) | 上电与遥控器拨杆、上位机准备、GUI 每个按钮、建图/巡线全流程、急停手段、现场调参、故障表 |
| [docs/传感器指南.md](docs/传感器指南.md) | 网络与串口规划总表、底盘 RS232（交叉接线！）、雷达 UDP 网段、IMU udev 与零偏标定、超声话题、编译部署、第三方组件声明 |
| [gui/README.md](gui/README.md) | GUI 控制台详细说明（架构/巡线参数/安全机制/FAQ） |

## 硬件清单

| 部件 | 型号/接口 |
|---|---|
| 底盘 | DT-01 差速底盘（RS232 串口控制，115200 8N1，帧头 `ED DE`） |
| 工控机 | RK3588（Ubuntu 22.04，双网口） |
| 激光雷达 | 蓝海 LDS-50C-E-R（UDP，~15Hz） |
| IMU | 飞迪 N100（USB 串口，400Hz） |
| 超声 | 6 路（底盘板载） |
| 遥控 | 富斯 6 通道（含硬件急停回路） |

## 注意事项

- `robot_ws/src/bluesea2`、`fdilink_ahrs` 为传感器**厂商提供**的驱动，
  版权归厂商所有，商用前请自行确认授权（详见 docs/传感器指南.md 第 8 节）。
- Nav2 / slam_toolbox / robot_localization 为 ROS 2 官方生态组件，需自行安装。
- 工控机路径与用户名默认按 `teamhd@192.168.2.153` 书写，部署到自己的机器时
  按需修改（涉及文件已在 docs/传感器指南.md 第 7 节列出）。
- 编译路径必须纯英文无空格（雷达驱动在中文路径下编译失败）。

---
研发：fofomo · 2026-09
