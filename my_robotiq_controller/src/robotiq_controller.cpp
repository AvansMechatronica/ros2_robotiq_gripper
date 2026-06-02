#include <chrono>
#include <functional>
#include <thread>

#include "my_robotiq_controller/robotiq_controller.hpp"

using robotiq_driver::DefaultDriver;
using robotiq_driver::DefaultSerial;

RobotiqControllerNode::RobotiqControllerNode()
: Node("dummy_node")
{
  port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
  baudrate_ = declare_parameter<int>("baudrate", 115200);
  timeout_ = declare_parameter<double>("timeout", 0.5);
  slave_address_ = declare_parameter<int>("slave_address", 9);
  commanded_position_ = 0.0;
  commanded_effort_ = 0.0;

  auto serial = std::make_unique<DefaultSerial>();
  serial->set_port(port_);
  serial->set_baudrate(static_cast<uint32_t>(baudrate_));
  serial->set_timeout(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(timeout_)));

  driver_ = std::make_unique<DefaultDriver>(std::move(serial));
  driver_->set_slave_address(static_cast<uint8_t>(slave_address_));

  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  action_server_ = rclcpp_action::create_server<control_msgs::action::ParallelGripperCommand>(
    this,
    "/robotiq_gripper_controller/gripper_cmd",
    [this](const rclcpp_action::GoalUUID & uuid,
           std::shared_ptr<const control_msgs::action::ParallelGripperCommand::Goal> goal) {
      (void)uuid;
      if (goal->command.position.empty() || goal->command.effort.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received malformed gripper goal");
        return rclcpp_action::GoalResponse::REJECT;
      }
      RCLCPP_INFO(
        this->get_logger(),
        "Received gripper goal: position=%.4f effort=%.4f",
        goal->command.position[0],
        goal->command.effort[0]);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::ParallelGripperCommand>> goal_handle) {
      (void)goal_handle;
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::ParallelGripperCommand>> goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Accepted goal");
      std::thread{std::bind(&RobotiqControllerNode::execute, this, goal_handle)}.detach();
    });

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&RobotiqControllerNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "RobotiqControllerNode initialized");
}

void RobotiqControllerNode::timer_callback()
{
  auto joint_state = sensor_msgs::msg::JointState();
  joint_state.header.stamp = this->now();
  joint_state.name = {
    "robotiq_85_left_finger_tip_joint",
    "robotiq_85_left_inner_knuckle_joint",
    "robotiq_85_left_knuckle_joint",
    "robotiq_85_right_finger_tip_joint",
    "robotiq_85_right_inner_knuckle_joint",
    "robotiq_85_right_knuckle_joint"
  };
  joint_state.position = {
    commanded_position_,
    commanded_position_,
    commanded_position_,
    -commanded_position_,
    -commanded_position_,
    commanded_position_
  };
  joint_state.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  joint_state.effort = {commanded_effort_, commanded_effort_, commanded_effort_, commanded_effort_, commanded_effort_, commanded_effort_};
  joint_state_publisher_->publish(joint_state);
}

void RobotiqControllerNode::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::ParallelGripperCommand>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  if (goal->command.position.empty() || goal->command.effort.empty()) {
    auto result = std::make_shared<control_msgs::action::ParallelGripperCommand::Result>();
    result->stalled = true;
    result->reached_goal = false;
    result->state.header.stamp = this->now();
    goal_handle->abort(result);
    RCLCPP_WARN(this->get_logger(), "Rejected gripper goal because it did not contain position and effort");
    return;
  }

  driver_->set_gripper_position(0x00);


  commanded_position_ = goal->command.position[0];
  commanded_effort_ = goal->command.effort[0];
  commanded_position_ = std::max(0.0, std::min(0.8, commanded_position_));
  commanded_effort_ = std::max(0.0, std::min(100.0, commanded_effort_));
  driver_->set_gripper_position(static_cast<uint8_t>(commanded_position_ * 0xFF));
  driver_->set_force(static_cast<uint8_t>(commanded_effort_ / 100.0 * 0xFF));



  auto result = std::make_shared<control_msgs::action::ParallelGripperCommand::Result>();
  result->state.header.stamp = this->now();
  result->state.name = {
    "robotiq_85_left_finger_tip_joint",
    "robotiq_85_left_inner_knuckle_joint",
    "robotiq_85_left_knuckle_joint",
    "robotiq_85_right_finger_tip_joint",
    "robotiq_85_right_inner_knuckle_joint",
    "robotiq_85_right_knuckle_joint"
  };
  result->state.position = {
    commanded_position_,
    commanded_position_,
    commanded_position_,
    -commanded_position_,
    -commanded_position_,
    commanded_position_
  };
  result->state.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  result->state.effort = {commanded_effort_, commanded_effort_, commanded_effort_, commanded_effort_, commanded_effort_, commanded_effort_};
  result->stalled = false;
  result->reached_goal = true;

  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
    RCLCPP_INFO(this->get_logger(), "Gripper goal canceled");
    return;
  }

  goal_handle->succeed(result);
  RCLCPP_INFO(
    this->get_logger(),
    "Gripper goal succeeded: position=%.4f effort=%.4f",
    commanded_position_,
    commanded_effort_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotiqControllerNode>());
  rclcpp::shutdown();
  return 0;
}
