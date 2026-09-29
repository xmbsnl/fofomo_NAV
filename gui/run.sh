#!/bin/bash
# DJ-NAV 1.0 启动脚本（真机专用版）
# 依赖：ROS 2 Humble + rmw_cyclonedds + PyQt5 + 免密登录工控机（ssh-copy-id）
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash

# DDS 与域配置（与工控机保持一致，缺了看不到车的话题）
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml

python3 dj_nav/main.py
