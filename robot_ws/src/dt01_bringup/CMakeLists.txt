cmake_minimum_required(VERSION 3.8)
project(dt01_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ---- Python 节点 ----
# 全部用 install(PROGRAMS) 装到 lib/dt01_bringup/，
# 之后就能 ros2 run dt01_bringup <脚本名>
install(PROGRAMS
  scripts/odom_to_tf.py
  scripts/chassis_bridge.py
  scripts/safety_mux.py
  scripts/imu_preprocess.py
  scripts/link_watchdog.py
  scripts/check_system.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Shell 工具脚本 ----
install(PROGRAMS
  scripts/setup_dds.sh
  scripts/setup_ntp.sh
  scripts/setup_udev.sh
  DESTINATION lib/${PROJECT_NAME}
)

# ---- launch / 参数 / URDF / RViz / udev ----
install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
