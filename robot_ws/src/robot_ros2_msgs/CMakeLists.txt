cmake_minimum_required(VERSION 3.8)
project(robot_ros2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChassisState.msg"
  "msg/BatteryState.msg"
  "msg/Bms.msg"
  "msg/ChassisVelocity.msg"
  "msg/FourWheelSteerAngle.msg"
  "msg/FourWheelSteerEncoder.msg"
  "msg/FourWheelSteerMotion.msg"
  "msg/TwoWheelDiffSpeed.msg"
  "msg/FourWheelDiffSpeed.msg"
  "msg/FourWheelSteerSpeed.msg"
  "msg/AckermannSpeed.msg"
  "msg/TwoWheelDiffCurrent.msg"
  "msg/FourWheelDiffCurrent.msg"
  "msg/FourWheelSteerCurrent.msg"
  "msg/AckermannCurrent.msg"
  "msg/AutoCharge.msg"
  "msg/RemoteControl.msg"
  "msg/ChassisParameter.msg"
  "msg/SoftwareVersion.msg"
  "msg/HardwareVersion.msg"
  "msg/LedStrip.msg"
  "msg/LedStrips.msg"
  "msg/ChassisDate.msg"
  "msg/DaliBms.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
