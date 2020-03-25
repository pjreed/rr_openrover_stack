#pragma once

#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <vector>
#include <stdint.h>
#include <string>
#include <fstream>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rr_openrover_driver_msgs/msg/raw_rr_openrover_driver_fast_rate_data.hpp>
#include <rr_openrover_driver_msgs/msg/raw_rr_openrover_driver_med_rate_data.hpp>
#include <rr_openrover_driver_msgs/msg/raw_rr_openrover_driver_slow_rate_data.hpp>
#include <rr_openrover_driver_msgs/msg/smart_battery_status.hpp>

#include <rr_openrover_driver/odom_control.hpp>
#include <rr_openrover_driver/constants.hpp>

namespace openrover
{
class OpenRover
{
public:
  explicit OpenRover(rclcpp::Node::SharedPtr nh);

  OdomControl left_controller_;
  OdomControl right_controller_;

  PidGains pidGains_;

  bool start();
  bool openComs();
  bool setupRobotParams();
  void updateMeasuredVelocities();
  void createTimeoutTimer();

  void robotDataFastCB();
  void robotDataMediumCB();
  void robotDataSlowCB();
  void timeoutCB();

  void serialManager();

  bool publish_fast_rate_values_;
  bool publish_med_rate_values_;
  bool publish_slow_rate_values_;
  bool closed_loop_control_on_;
  bool e_stop_on_;

private:
  // PID debug variables
  std::string l_pid_csv_file_;
  std::string r_pid_csv_file_;
  std::ofstream l_fs_;
  std::ofstream r_fs_;

  // ROS Parameters
  std::string port_;
  std::string drive_type_;

  float timeout_;  // Default to neutral motor values after timeout seconds

  // ROS node handlers
  rclcpp::Node::SharedPtr nh_;

  // ROS Timers
  rclcpp::TimerBase::SharedPtr fast_timer;
  rclcpp::TimerBase::SharedPtr medium_timer;
  rclcpp::TimerBase::SharedPtr slow_timer;
  rclcpp::TimerBase::SharedPtr timeout_timer;

  // ROS Publisher and Subscribers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_enc_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_charging_pub;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr motor_speeds_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr vel_calc_pub;

  rclcpp::Publisher<rr_openrover_driver_msgs::msg::RawRrOpenroverDriverFastRateData>::SharedPtr fast_rate_pub;
  rclcpp::Publisher<rr_openrover_driver_msgs::msg::RawRrOpenroverDriverMedRateData>::SharedPtr medium_rate_pub;
  rclcpp::Publisher<rr_openrover_driver_msgs::msg::RawRrOpenroverDriverSlowRateData>::SharedPtr slow_rate_pub;
  rclcpp::Publisher<rr_openrover_driver_msgs::msg::SmartBatteryStatus>::SharedPtr battery_status_a_pub, battery_status_b_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr battery_state_of_charge_pub;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fan_speed_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_reset_sub;

  // General Class variables
  int serial_baud_rate_;
  int serial_port_fd_{};
  int robot_data_[250]{};  // stores all received data from robot
  bool is_charging_{};
  int motor_speeds_commanded_[3];  // stores most recent commanded motor speeds
  const int LEFT_MOTOR_INDEX_;
  const int RIGHT_MOTOR_INDEX_;
  const int FLIPPER_MOTOR_INDEX_;
  bool is_serial_coms_open_;
  bool use_legacy_;
  double fast_rate_hz_;  // update rate for encoders, 10Hz recommended
  double medium_rate_hz_;
  double slow_rate_hz_;

  // drive dependent parameters
  float odom_encoder_coef_{};
  float odom_axle_track_{};
  float odom_angular_coef_{};
  float odom_traction_factor_{};
  float odom_covariance_0_{};
  float odom_covariance_35_{};

  // velocity feedback
  double left_vel_commanded_;
  double right_vel_commanded_;
  double left_vel_measured_;
  double right_vel_measured_;
  double left_vel_filtered_;
  double right_vel_filtered_;

  int motor_speed_linear_coef_{};
  int motor_speed_angular_coef_{};
  int motor_speed_flipper_coef_{};
  int motor_speed_deadband_{};

  float total_weight_{};  // in kg
  // int motor_speed_diff_max_; ---WIP
  geometry_msgs::msg::Twist cmd_vel_commanded_;

  std::vector<unsigned char> serial_fast_buffer_;
  std::vector<unsigned char> serial_medium_buffer_;
  std::vector<unsigned char> serial_slow_buffer_;
  std::vector<unsigned char> serial_fan_buffer_;

  // ROS Subscriber callback functions
  void cmdVelCB(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void fanSpeedCB(const std_msgs::msg::Int32::ConstSharedPtr msg);
  void eStopCB(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void eStopResetCB(const std_msgs::msg::Bool::ConstSharedPtr msg);

  // ROS Publish Functions (robot_data_[X] to ros topics)
  void publishFastRateData();
  void publishMedRateData();
  void publishSlowRateData();
  void publishOdometry(float left_vel, float right_vel);
  void publishMotorSpeeds();
  void publishWheelVels();

  // Serial Com Functions
  int getParameterData(int parameter);
  bool setParameterData(int param1, int param2);
  void updateRobotData(int parameter);
  bool sendCommand(int param1, int param2);
  int readCommand();
};

rr_openrover_driver_msgs::msg::SmartBatteryStatus interpret_battery_status(uint16_t bits);

}  // namespace openrover