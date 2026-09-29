#ifndef DT_CONTROL_HPP
#define DT_CONTROL_HPP

#include <cstdio>
#include <memory>
#include <chrono>
#include <vector>
#include <deque>
#include <mutex>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "serial_driver/serial_driver.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/byte.hpp"
#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"

#include "dt_ros2/binary.h"

#include "dt_ros2/msg/drive_hollysys_error.hpp"
#include "dt_ros2/msg/drive_sdfz_error.hpp"
#include "dt_ros2/msg/frame.hpp"

#include "robot_ros2_msgs/msg/auto_charge.hpp"
#include "robot_ros2_msgs/msg/battery_state.hpp"
#include "robot_ros2_msgs/msg/bms.hpp"
#include "robot_ros2_msgs/msg/chassis_date.hpp"
#include "robot_ros2_msgs/msg/chassis_parameter.hpp"
#include "robot_ros2_msgs/msg/chassis_state.hpp"
#include "robot_ros2_msgs/msg/chassis_velocity.hpp"
#include "robot_ros2_msgs/msg/hardware_version.hpp"
#include "robot_ros2_msgs/msg/led_strip.hpp"
#include "robot_ros2_msgs/msg/led_strips.hpp"
#include "robot_ros2_msgs/msg/remote_control.hpp"
#include "robot_ros2_msgs/msg/software_version.hpp"
#include "robot_ros2_msgs/msg/two_wheel_diff_current.hpp"
#include "robot_ros2_msgs/msg/two_wheel_diff_speed.hpp"

namespace dt_control
{

using DriveHollysysError = dt_ros2::msg::DriveHollysysError;
using DriveSdfzError     = dt_ros2::msg::DriveSdfzError;
using Frame              = dt_ros2::msg::Frame;

using AutoCharge            = robot_ros2_msgs::msg::AutoCharge;
using BatteryState          = robot_ros2_msgs::msg::BatteryState;
using Bms                   = robot_ros2_msgs::msg::Bms;
using ChassisDate           = robot_ros2_msgs::msg::ChassisDate;
using ChassisParameter      = robot_ros2_msgs::msg::ChassisParameter;
using ChassisState          = robot_ros2_msgs::msg::ChassisState;
using ChassisVelocity       = robot_ros2_msgs::msg::ChassisVelocity;
using HardwareVersion       = robot_ros2_msgs::msg::HardwareVersion;
using LedStrip              = robot_ros2_msgs::msg::LedStrip;
using LedStrips             = robot_ros2_msgs::msg::LedStrips;
using RemoteControl         = robot_ros2_msgs::msg::RemoteControl;
using SoftwareVersion       = robot_ros2_msgs::msg::SoftwareVersion;
using TwoWheelDiffCurrent   = robot_ros2_msgs::msg::TwoWheelDiffCurrent;
using TwoWheelDiffSpeed     = robot_ros2_msgs::msg::TwoWheelDiffSpeed;

#if 0
#define logs_debug(...)     do{if(param_.log){printf("\033[33m[DT] [DEBUG] "); printf(__VA_ARGS__); printf("\033[0m\n"); fflush(stdout);}}while(0)
#define logs_info(...)      do{if(param_.log){printf(        "[DT] [INFO] ");  printf(__VA_ARGS__); printf("\n");        fflush(stdout);}}while(0)
#define logs_error(...)     do{if(param_.log){printf("\033[31m[DT] [ERROR] "); printf(__VA_ARGS__); printf("\033[0m\n"); fflush(stdout);}}while(0)

#define logs_debug_stream(args)    do{if(param_.log){std::cout << "\033[33m" << "[DT]" << " " << "[DEBUG]" << " " << args << "\033[0m" << std::endl; std::cout.flush();}}while(0)
#define logs_info_stream(args)     do{if(param_.log){std::cout <<               "[DT]" << " " << "[INFO]"  << " " << args              << std::endl; std::cout.flush();}}while(0)
#define logs_error_stream(args)    do{if(param_.log){std::cout << "\033[31m" << "[DT]" << " " << "[ERROR]" << " " << args << "\033[0m" << std::endl; std::cout.flush();}}while(0)
#endif

#if 1
#define logs_debug(...)     do{if(param_.log){RCLCPP_DEBUG(this->get_logger(), "[DT] " __VA_ARGS__);}}while(0)
#define logs_info(...)      do{if(param_.log){RCLCPP_INFO (this->get_logger(), "[DT] " __VA_ARGS__);}}while(0)
#define logs_error(...)     do{if(true      ){RCLCPP_ERROR(this->get_logger(), "[DT] " __VA_ARGS__);}}while(0)

#define logs_debug_stream(args)    do{if(param_.log){RCLCPP_DEBUG_STREAM(this->get_logger(), "[DT] " << args);}}while(0)
#define logs_info_stream(args)     do{if(param_.log){RCLCPP_INFO_STREAM (this->get_logger(), "[DT] " << args);}}while(0)
#define logs_error_stream(args)    do{if(true      ){RCLCPP_ERROR_STREAM(this->get_logger(), "[DT] " << args);}}while(0)
#endif

#define pkgInfoDisplay(name, max_cnt, str) \
do                                         \
{                                          \
  static uint8_t name##_cnt = 0;           \
                                           \
  name##_cnt++;                            \
                                           \
  if(name##_cnt == max_cnt)                \
  {                                        \
    name##_cnt = 0;                        \
    logs_info(str);                        \
  }                                        \
}                                          \
while(0)

enum class TransmitCmd
{
  state_upload       = 0x01,
  current_upload     = 0x02,
  auto_charge_upload = 0x03,
  remote_ctrl_upload = 0x04,
  led_strip_upload   = 0x05,
  battery_upload     = 0x06,

  set_velocity       = 0x20,
  set_speed          = 0x21,
  set_stop           = 0x22,
  collision_clean    = 0x23,
  fault_clean        = 0x24,
  set_auto_charge    = 0x25,
  set_led_strip_mode = 0x26,
  set_led_strip      = 0x27,
  set_led_strips     = 0x28,

  get_state       = 0x40,
  get_current     = 0x41,
  get_auto_charge = 0x42,
  get_remote_ctrl = 0x43,
  get_drive_error = 0x44,
  get_parameter   = 0x45,
  get_software    = 0x46,
  get_hardware    = 0x47,
  get_date        = 0x48,
  get_led_strip   = 0x49,
  get_battery     = 0x4A,
};

enum class ReceiveCmd
{
  state         = 0x80,
  current       = 0x81,
  auto_charge   = 0x82,
  remote_ctrl   = 0x83,
  led_strip     = 0x84,
  battery_info  = 0x85,
  battery_error = 0x86,

  drive_error = 0xA0,
  parameter   = 0xA1,
  software    = 0xA2,
  hardware    = 0xA3,
  date        = 0xA4,
};

enum class PkgGetCode
{
  send,
  get,
  wait,
  skip,
  timeout,
};

struct Param
{
  std::string port;
  int baudrate = 0;

  bool odom = true;
  std::string drive_type = "SDFZ";

  bool log = true;
  bool original = false;

  double sub_interval;
  rclcpp::Time sub_last_time;
};

struct Pub
{
  rclcpp::Publisher<Frame>::SharedPtr frame;
  rclcpp::Publisher<DriveHollysysError>::SharedPtr drive_hollysys_left_error;
  rclcpp::Publisher<DriveHollysysError>::SharedPtr drive_hollysys_right_error;
  rclcpp::Publisher<DriveSdfzError>::SharedPtr drive_sdfz_left_error;
  rclcpp::Publisher<DriveSdfzError>::SharedPtr drive_sdfz_right_error;

  rclcpp::Publisher<AutoCharge>::SharedPtr auto_charge;
  rclcpp::Publisher<BatteryState>::SharedPtr battery;
  rclcpp::Publisher<Bms>::SharedPtr bms;
  rclcpp::Publisher<ChassisDate>::SharedPtr date;
  rclcpp::Publisher<ChassisParameter>::SharedPtr parameter;
  rclcpp::Publisher<ChassisState>::SharedPtr state;
  rclcpp::Publisher<ChassisVelocity>::SharedPtr velocity;
  rclcpp::Publisher<HardwareVersion>::SharedPtr hardware_version;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr led_strip_mode;
  rclcpp::Publisher<LedStrips>::SharedPtr led_strips;
  rclcpp::Publisher<RemoteControl>::SharedPtr remote_ctrl;
  rclcpp::Publisher<SoftwareVersion>::SharedPtr software_version;
  rclcpp::Publisher<TwoWheelDiffCurrent>::SharedPtr current;
  rclcpp::Publisher<TwoWheelDiffSpeed>::SharedPtr speed;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr std_stop_button;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr std_software_stop;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr std_front_collision;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr std_rear_collision;
};

struct Sub
{
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity;
  rclcpp::Subscription<TwoWheelDiffSpeed>::SharedPtr speed;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_ctrl;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr collision_clean;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr fault_clean;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr auto_charge_ctrl;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr led_strip_mode;
  rclcpp::Subscription<LedStrip>::SharedPtr led_strip;
  rclcpp::Subscription<LedStrips>::SharedPtr led_strips;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr odom_clean;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr std_stop_ctrl;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr std_all_collision_clean;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr std_front_collision_clean;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr std_rear_collision_clean;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr std_all_lamp_ctrl;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr std_front_lamp_ctrl;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr std_rear_lamp_ctrl;
};

struct Tim
{
  rclcpp::TimerBase::SharedPtr serial_init;
  rclcpp::TimerBase::SharedPtr pkg_loop;
  rclcpp::TimerBase::SharedPtr pkg_receive;
};

struct Msg
{
  Frame frame;
  DriveHollysysError drive_hollysys_left_error;
  DriveHollysysError drive_hollysys_right_error;
  DriveSdfzError drive_sdfz_left_error;
  DriveSdfzError drive_sdfz_right_error;

  AutoCharge auto_charge;
  BatteryState battery_state;
  Bms bms;
  ChassisDate chassis_date;
  ChassisParameter chassis_parameter;
  ChassisState chassis_state;
  ChassisVelocity chassis_velocity;
  HardwareVersion hardware_version;
  std_msgs::msg::UInt8 led_strip_mode;
  LedStrips led_strips;
  RemoteControl remote_control;
  SoftwareVersion software_version;
  TwoWheelDiffCurrent two_wheel_diff_current;
  TwoWheelDiffSpeed two_wheel_diff_speed;

  nav_msgs::msg::Odometry odometry;
  sensor_msgs::msg::BatteryState battery_info;

  std_msgs::msg::Bool std_stop_button;
  std_msgs::msg::Bool std_software_stop;
  std_msgs::msg::Bool std_front_collision;
  std_msgs::msg::Bool std_rear_collision;
};

struct SerialStep
{
  bool create = false;
  bool close = false;
  bool open = false;

  void clean(void)
  {
    create = false;
    close = false;
    open = false;
  }

  bool finish(void)
  {
    return ((create && close && open) == true) ? true : false;
  }
};

struct SerialPort
{
  std::unique_ptr<drivers::common::IoContext> ctx{};
  std::unique_ptr<drivers::serial_driver::SerialDriver> driver;

  std::deque<uint8_t> rx;
  std::deque<std::vector<uint8_t>> tx;

  // 串口接收跑在 IoContext 线程（serialRead 里 push_back），
  // 协议解析跑在定时器线程（pkgReceive 里 at()/pop_front()）。
  // 以前完全无锁 → 并发修改 deque 偶发 std::out_of_range
  //（deque::_M_range_check）直接 abort，底盘驱动节点崩溃、/odom 断供。
  // 所有对 rx 的访问都必须在 rx_mtx 保护下（2026-09-18 修复）。
  std::mutex rx_mtx;
};

struct Odom
{
  double x = 0;
  double y = 0;
  double yaw = 0;
  rclcpp::Time last_time;

  void clean(nav_msgs::msg::Odometry &value)
  {
    x = 0; y = 0; yaw = 0;
    value.pose.pose.position.x = x;
    value.pose.pose.position.y = y;
    last_time = rclcpp::Clock().now();
  }
};

struct PkgFlag
{
  bool once = false;

  bool request = false;
  bool response = false;

  uint32_t timeout_cnt = 0;
};

struct PkgGet
{
  bool do_once = false;
  uint32_t timeout_ms = 0;

  PkgFlag flag;

  void setOnce(bool set) {do_once = set;}
  void setTimeout(uint32_t ms) {timeout_ms = ms;}
  void flagClean(void) {flag = PkgFlag();}

  void setResponse(void) {if(flag.request == true) {flag.response = true;}}
};

struct Pkg
{
  uint32_t cycle = 100;
  uint32_t step = 0;

  const size_t max_num = 11;

  /* get once */

  PkgGet get_software_version;
  PkgGet get_hardware_version;
  PkgGet get_parameter;
  PkgGet get_date;

  /* upload */

  PkgGet state_upload;
  PkgGet auto_charge_upload;
  PkgGet remote_ctrl_upload;
  PkgGet led_strip_upload;
  PkgGet battery_upload;

  /* loop get */

  PkgGet get_drive_error;

  /* not use */

  PkgGet current_upload;

  PkgGet get_state;
  PkgGet get_current;
  PkgGet get_auto_charge;
  PkgGet get_remote_ctrl;
  PkgGet get_led_strip;
  PkgGet get_battery;

};

class DtControl : public rclcpp::Node
{

public:

  DtControl(void);
  ~DtControl(void);

private:

  Param param_;
  Pub pub_;
  Sub sub_;
  Tim tim_;
  Msg msg_;
  SerialStep step_;
  SerialPort port_;
  Odom odom_;
  Pkg pkg_;

  bool subIntervalCheck(void);

  void serialInit(void);
  void serialReset(void);
  bool serialCreate(void);
  bool serialOpen(void);
  bool serialClose(void);
  void serialRead(const std::vector<uint8_t> &array, const size_t &size);
  void serialWrite(std::vector<uint8_t> &array);

  void pkgReceive(void);
  void pkgProcess(void);

  void pkgOdomCalculation(double linear, double radian);
  void pkgFormatFrame(TransmitCmd cmd, size_t data_len, uint8_t *data_buffe, std::vector<uint8_t> &tx_buffer);

  void pkgSetStateUpload(bool enable);
  void pkgSetCurrentUpload(bool enable);
  void pkgSetAutoChargeUpload(bool enable);
  void pkgSetRemoteCtrlUpload(bool enable);
  void pkgSetLedStripUpload(bool enable);
  void pkgSetBatteryUpload(bool enable);

  void pkgSetVelocity(double linear, double radian);
  void pkgSetSpeed(int16_t left_speed, int16_t right_speed);
  void pkgSetStop(bool enable);
  void pkgCollisionClean(void);
  void pkgFaultClean(void);
  void pkgSetAutoCharge(uint8_t charge_type);
  void pkgSetLedStripMode(uint8_t mode);
  void pkgSetLedStrip(LedStrip led_strip);
  void pkgSetLedStrips(LedStrips led_strips);

  void pkgGetState(void);
  void pkgGetCurrent(void);
  void pkgGetAutoCharge(void);
  void pkgGetRemoteCtrl(void);
  void pkgGetDriveError(void);
  void pkgGetLedStrip(void);
  void pkgGetBattery(void);

  void pkgGetParameter(void);
  void pkgGetSoftwareVersion(void);
  void pkgGetHardwareVersion(void);
  void pkgGetDate(void);

  void pkgLoopInit(void);
  void pkgLoopReset(void);
  void pkgGetLoop(void);
  PkgGetCode pkgGetCheck(PkgGet &pkg, uint32_t &step);

};

}

#endif