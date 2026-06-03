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
  update_rate_ = declare_parameter<double>("update_rate", 50.0);
  max_joint_position_ = declare_parameter<double>("max_joint_position", 0.085);

  if (update_rate_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Parameter 'update_rate' must be > 0.0. Falling back to 1.0 Hz.");
    update_rate_ = 1.0;
  }


  commanded_position_ = 0.0;
  commanded_effort_ = 0.0;
#if 1
  auto serial = std::make_unique<DefaultSerial>();
  serial->set_port(port_);
  serial->set_baudrate(static_cast<uint32_t>(baudrate_));
  serial->set_timeout(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(timeout_)));

  driver_ = std::make_unique<DefaultDriver>(std::move(serial));
  driver_->set_slave_address(static_cast<uint8_t>(slave_address_));
  const bool connected = driver_->connect();
  if (!connected) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to the gripper on port %s with baudrate %d", port_.c_str(), baudrate_);
    throw std::runtime_error("Failed to connect to the gripper");
  }

  {
    std::lock_guard<std::mutex> lock(driver_mutex_);
    driver_->activate();
  }
#endif

  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  set_force_service_ = this->create_service<my_robotiq_controller::srv::SetForce>(
    "set_gripper_force",
    [this](
      const std::shared_ptr<my_robotiq_controller::srv::SetForce::Request> request,
      std::shared_ptr<my_robotiq_controller::srv::SetForce::Response> response)
    {
      if (!driver_) {
        response->success = false;
        response->message = "Driver is not initialized";
        RCLCPP_ERROR(this->get_logger(), "set_gripper_force failed: driver is not initialized");
        return;
      }

      const auto force = request->force;
      {
        std::lock_guard<std::mutex> lock(driver_mutex_);
        driver_->set_force(force);
      }
      response->success = true;
      response->message = "Gripper force updated";
      RCLCPP_INFO(this->get_logger(), "Gripper force set to %u", static_cast<unsigned>(force));
    });

  set_speed_service_ = this->create_service<my_robotiq_controller::srv::SetSpeed>(
    "set_gripper_speed",
    [this](
      const std::shared_ptr<my_robotiq_controller::srv::SetSpeed::Request> request,
      std::shared_ptr<my_robotiq_controller::srv::SetSpeed::Response> response)
    {
      if (!driver_) {
        response->success = false;
        response->message = "Driver is not initialized";
        RCLCPP_ERROR(this->get_logger(), "set_gripper_speed failed: driver is not initialized");
        return;
      }

      const auto speed = request->speed;
      {
        std::lock_guard<std::mutex> lock(driver_mutex_);
        driver_->set_speed(speed);
      }
      response->success = true;
      response->message = "Gripper speed updated";
      RCLCPP_INFO(this->get_logger(), "Gripper speed set to %u", static_cast<unsigned>(speed));
    });

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
    std::chrono::duration<double>(1.0 / update_rate_),
    std::bind(&RobotiqControllerNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "RobotiqControllerNode initialized");
}

RobotiqControllerNode::~RobotiqControllerNode()
{
  if (!driver_) {
    return;
  }

  try {
    std::lock_guard<std::mutex> lock(driver_mutex_);
    driver_->deactivate();
    driver_->disconnect();
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "Destructor cleanup failed: %s", e.what());
  }
}

void RobotiqControllerNode::timer_callback()
{
  constexpr double kRawPositionMax = 255.0;

  double joint_position = commanded_position_;
  if (driver_) {
    std::lock_guard<std::mutex> lock(driver_mutex_);
    const auto raw_position = static_cast<double>(driver_->get_gripper_position());
    const auto mapped_position = std::clamp((raw_position / kRawPositionMax) * max_joint_position_, 0.0, max_joint_position_);
    joint_position = max_joint_position_ - mapped_position;
  }

  //RCLCPP_INFO(this->get_logger(), "Publishing joint states: position=%.4f effort=%.4f", joint_position, commanded_effort_);
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
    -joint_position,
    joint_position,
    joint_position,
    joint_position,
    -joint_position,
    -joint_position
  };
  joint_state.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  joint_state.effort = {commanded_effort_, commanded_effort_, commanded_effort_, commanded_effort_, commanded_effort_, commanded_effort_};
  joint_state_publisher_->publish(joint_state);
}

void RobotiqControllerNode::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::ParallelGripperCommand>> goal_handle)
{
  constexpr double kRawPositionMax = 255.0;

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

  if (!driver_) {
    auto result = std::make_shared<control_msgs::action::ParallelGripperCommand::Result>();
    result->stalled = true;
    result->reached_goal = false;
    result->state.header.stamp = this->now();
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "Cannot execute gripper goal: driver is not initialized");
    return;
  }

  commanded_position_ = goal->command.position[0];
  commanded_effort_ = goal->command.effort[0];
  commanded_position_ = std::clamp(commanded_position_, 0.0, max_joint_position_);
  commanded_effort_ = std::max(0.0, std::min(100.0, commanded_effort_));
  {
    const auto normalized_position = commanded_position_ / max_joint_position_;
    const auto raw_command = static_cast<uint8_t>((1.0 - normalized_position) * 0xFF);
    std::lock_guard<std::mutex> lock(driver_mutex_);
    driver_->set_gripper_position(raw_command);
    driver_->set_force(static_cast<uint8_t>(commanded_effort_ / 100.0 * 0xFF));
  }

  const auto wait_start = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    bool gripper_is_moving = false;
    {
      std::lock_guard<std::mutex> lock(driver_mutex_);
      gripper_is_moving = driver_->gripper_is_moving();
    }
    if (!gripper_is_moving) {
      break;
    }

    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<control_msgs::action::ParallelGripperCommand::Result>();
      result->state.header.stamp = this->now();
      result->stalled = false;
      result->reached_goal = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Gripper goal canceled while moving");
      return;
    }

    if ((std::chrono::steady_clock::now() - wait_start) > std::chrono::seconds(10)) {
      auto result = std::make_shared<control_msgs::action::ParallelGripperCommand::Result>();
      result->state.header.stamp = this->now();
      result->stalled = true;
      result->reached_goal = false;
      goal_handle->abort(result);
      RCLCPP_WARN(this->get_logger(), "Gripper motion timeout while waiting for movement to finish");
      return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  double raw_position = 0.0;
  {
    std::lock_guard<std::mutex> lock(driver_mutex_);
    raw_position = static_cast<double>(driver_->get_gripper_position());
  }
  const auto mapped_position = std::clamp(
    (raw_position / kRawPositionMax) * max_joint_position_, 0.0, max_joint_position_);
  const auto reached_position = max_joint_position_ - mapped_position;


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
    -reached_position,
    reached_position,
    reached_position,
    reached_position,
    -reached_position,
    -reached_position
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
    reached_position,
    commanded_effort_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotiqControllerNode>());
  rclcpp::shutdown();
  return 0;
}
