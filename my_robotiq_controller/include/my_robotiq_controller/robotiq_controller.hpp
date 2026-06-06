#pragma once

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "my_robotiq_controller/srv/set_force.hpp"
#include "my_robotiq_controller/srv/set_speed.hpp"

#include <robotiq_driver/default_driver.hpp>
#include <robotiq_driver/default_serial.hpp>

class RobotiqControllerNode : public rclcpp::Node
{
public:
  RobotiqControllerNode();
  ~RobotiqControllerNode() override;

private:
  void timer_callback();
  void handle_set_joint_state(const std_msgs::msg::Float64::SharedPtr msg);

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::ParallelGripperCommand>> goal_handle);

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Server<control_msgs::action::ParallelGripperCommand>::SharedPtr action_server_;
  rclcpp::Service<my_robotiq_controller::srv::SetForce>::SharedPtr set_force_service_;
  rclcpp::Service<my_robotiq_controller::srv::SetSpeed>::SharedPtr set_speed_service_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr set_joint_state_subscriber_;
  std::unique_ptr<robotiq_driver::DefaultDriver> driver_;
  std::string port_;
  int baudrate_;
  double timeout_;
  double update_rate_;
  double max_joint_position_;
  int slave_address_;
  double commanded_position_;
  double commanded_effort_;
  std::mutex driver_mutex_;
};