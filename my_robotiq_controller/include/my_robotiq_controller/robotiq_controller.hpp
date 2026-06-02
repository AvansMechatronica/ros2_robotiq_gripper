#pragma once

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"

#include <robotiq_driver/default_driver.hpp>
#include <robotiq_driver/default_serial.hpp>

class RobotiqControllerNode : public rclcpp::Node
{
public:
  RobotiqControllerNode();

private:
  void timer_callback();

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::ParallelGripperCommand>> goal_handle);

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Server<control_msgs::action::ParallelGripperCommand>::SharedPtr action_server_;
  std::unique_ptr<robotiq_driver::DefaultDriver> driver_;
  std::string port_;
  int baudrate_;
  double timeout_;
  int slave_address_;
  double commanded_position_;
  double commanded_effort_;
};