#include <chrono>
#include <functional>
#include <thread>

#include "my_robotiq_controller/robotiq_controller.hpp"

using robotiq_driver::DefaultDriver;
using robotiq_driver::DefaultSerial;

// This node bridges ROS 2 interfaces (services + action + joint states)
// with the Robotiq low-level driver:
// - Parameters configure serial transport and joint-model scaling.
// - Two services update speed/force setpoints at runtime.
// - A ParallelGripperCommand action receives open/close goals.
// - A periodic timer publishes JointState for visualization/controllers.
RobotiqControllerNode::RobotiqControllerNode()
: Node("dummy_node")
{
  // --- Runtime parameters -------------------------------------------------
  // `max_joint_position_` is the modeled "fully open" position (meters/radians
  // depending on URDF joint definition) used to map raw 8-bit hardware values
  // into ROS joint-space values.
  port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
  baudrate_ = declare_parameter<int>("baudrate", 115200);
  timeout_ = declare_parameter<double>("timeout", 0.5);
  slave_address_ = declare_parameter<int>("slave_address", 9);
  update_rate_ = declare_parameter<double>("update_rate", 50.0);
  max_joint_position_ = declare_parameter<double>("max_joint_position", 0.085);
  startup_homing_ = declare_parameter<bool>("startup_homing", true);
  startup_timeout_ = declare_parameter<double>("startup_timeout", 8.0);
  const auto startup_speed_param = declare_parameter<int>("startup_speed", 128);
  const auto startup_force_param = declare_parameter<int>("startup_force", 80);
  startup_speed_ = static_cast<uint8_t>(std::clamp<int>(startup_speed_param, 0, 255));
  startup_force_ = static_cast<uint8_t>(std::clamp<int>(startup_force_param, 0, 255));

  if (update_rate_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Parameter 'update_rate' must be > 0.0. Falling back to 1.0 Hz.");
    update_rate_ = 1.0;
  }

  RCLCPP_DEBUG(
    this->get_logger(),
    "Parameters loaded: port=%s baudrate=%d timeout=%.3f slave_address=%d update_rate=%.3f max_joint_position=%.4f",
    port_.c_str(),
    baudrate_,
    timeout_,
    slave_address_,
    update_rate_,
    max_joint_position_);


  // Internal command cache. These values are reused by `timer_callback()`
  // so state output remains coherent even if hardware polling is delayed.
  commanded_position_ = 0.0;
  commanded_effort_ = 0.0;

  // --- Driver bootstrap ---------------------------------------------------
  // Build serial transport, then construct and connect the Robotiq driver.
  // Failure to connect is treated as fatal because this node's purpose is
  // direct hardware control.
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

  activate_gripper();


  // Publish complete six-joint kinematic state compatible with the Robotiq
  // URDF chain used by RViz and downstream controllers.
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  // Service endpoint to update force register without issuing a full action goal.
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
      RCLCPP_DEBUG(this->get_logger(), "set_gripper_force request received: force=%u", static_cast<unsigned>(force));
      {
        std::lock_guard<std::mutex> lock(driver_mutex_);
        driver_->set_force(force);
      }
      response->success = true;
      response->message = "Gripper force updated";
      RCLCPP_INFO(this->get_logger(), "Gripper force set to %u", static_cast<unsigned>(force));
    });

  // Service endpoint to update speed register without issuing a full action goal.
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
      RCLCPP_DEBUG(this->get_logger(), "set_gripper_speed request received: speed=%u", static_cast<unsigned>(speed));
      {
        std::lock_guard<std::mutex> lock(driver_mutex_);
        driver_->set_speed(speed);
      }
      response->success = true;
      response->message = "Gripper speed updated";
      RCLCPP_INFO(this->get_logger(), "Gripper speed set to %u", static_cast<unsigned>(speed));
    });

  set_joint_state_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
    "/robotiq_gripper_controller/set_joint_state",
    10,
    std::bind(&RobotiqControllerNode::handle_set_joint_state, this, std::placeholders::_1));

  // Action server behavior:
  // - Goal callback validates payload shape and accepts executable goals.
  // - Cancel callback always accepts cancellation requests.
  // - Accepted callback executes goal asynchronously in a detached thread so
  //   the action server callback path remains non-blocking.
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
      RCLCPP_DEBUG(
        this->get_logger(),
        "Goal request: position=%.4f effort=%.4f",
        goal->command.position[0],
        goal->command.effort[0]);
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

  // Periodic publisher loop for joint state feedback.
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / update_rate_),
    std::bind(&RobotiqControllerNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "RobotiqControllerNode ready");
}

RobotiqControllerNode::~RobotiqControllerNode()
{
  // Best-effort shutdown sequence: if a driver exists, deactivate then
  // disconnect while holding the same mutex used during runtime I/O.
  // Exceptions are swallowed to avoid throwing from destructor context.
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

void RobotiqControllerNode::activate_gripper()
{
  RCLCPP_INFO(this->get_logger(), "Connecting to Robotiq gripper succeeded, activating...");

  {
    std::lock_guard<std::mutex> lock(driver_mutex_);
    driver_->deactivate();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  {
    std::lock_guard<std::mutex> lock(driver_mutex_);
    driver_->activate();
    driver_->set_speed(startup_speed_);
    driver_->set_force(startup_force_);
  }

  // Force one status read after activation. If activation only wrote registers
  // but the gripper is not responding correctly, this throws and the node exits.
  {
    std::lock_guard<std::mutex> lock(driver_mutex_);
    const auto raw_position = driver_->get_gripper_position();
    RCLCPP_INFO(
      this->get_logger(),
      "Robotiq activated, raw position feedback=%u",
      static_cast<unsigned>(raw_position));
  }

  if (startup_homing_) {
    run_startup_homing();
  }
}

void RobotiqControllerNode::run_startup_homing()
{
  RCLCPP_INFO(this->get_logger(), "Running Robotiq startup homing: open -> close -> open");
  command_raw_position(0x00, "startup open");
  command_raw_position(0xFF, "startup close");
  command_raw_position(0x00, "startup reopen");
  commanded_position_ = max_joint_position_;
  RCLCPP_INFO(this->get_logger(), "Robotiq startup homing completed");
}

void RobotiqControllerNode::command_raw_position(uint8_t raw_position, const std::string& label)
{
  {
    std::lock_guard<std::mutex> lock(driver_mutex_);
    driver_->set_gripper_position(raw_position);
  }

  if (!wait_for_position(raw_position, label)) {
    throw std::runtime_error("Robotiq " + label + " did not complete before timeout");
  }
}

bool RobotiqControllerNode::wait_for_position(uint8_t target_position, const std::string& label)
{
  constexpr int kTolerance = 8;
  const auto start_time = std::chrono::steady_clock::now();
  bool saw_motion = false;

  while (rclcpp::ok()) {
    uint8_t raw_position = 0;
    bool moving = false;
    {
      std::lock_guard<std::mutex> lock(driver_mutex_);
      raw_position = driver_->get_gripper_position();
      moving = driver_->gripper_is_moving();
    }

    saw_motion = saw_motion || moving;
    const auto error = std::abs(static_cast<int>(raw_position) - static_cast<int>(target_position));
    if (error <= kTolerance && (!moving || saw_motion)) {
      RCLCPP_INFO(
        this->get_logger(),
        "Robotiq %s reached raw position %u",
        label.c_str(),
        static_cast<unsigned>(raw_position));
      return true;
    }

    if (!moving && saw_motion) {
      RCLCPP_INFO(
        this->get_logger(),
        "Robotiq %s stopped at raw position %u",
        label.c_str(),
        static_cast<unsigned>(raw_position));
      return true;
    }

    if ((std::chrono::steady_clock::now() - start_time) > std::chrono::duration<double>(startup_timeout_)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Robotiq %s timed out. Last raw position=%u target=%u moving=%s",
        label.c_str(),
        static_cast<unsigned>(raw_position),
        static_cast<unsigned>(target_position),
        moving ? "true" : "false");
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return false;
}

void RobotiqControllerNode::timer_callback()
{
  constexpr double kRawPositionMax = 255.0;

  // Default to the last commanded value; overwrite with hardware feedback when
  // available. This gives stable output during startup/transient failures.
  double joint_position = commanded_position_;
  if (driver_) {
    std::lock_guard<std::mutex> lock(driver_mutex_);
    // Robotiq register semantics vs. ROS joint semantics:
    // - The hardware reports a raw position in [0, 255].
    // - On this gripper, larger raw values correspond to a more CLOSED gripper.
    // - Our exported ROS joint position is modeled as opening width in meters,
    //   where 0.0 means fully closed and `max_joint_position_` means fully open.
    // Therefore we:
    //   1) Scale raw [0, 255] -> [0, max_joint_position_] (`mapped_position`),
    //   2) Invert with `max_joint_position_ - mapped_position` so the published
    //      joint value matches the URDF/controller convention (open increases).
    // This same conversion/inversion is intentionally mirrored in `execute()`
    // when building the action result so command, feedback and state remain
    // consistent.
    const auto raw_position = static_cast<double>(driver_->get_gripper_position());
    const auto mapped_position = std::clamp((raw_position / kRawPositionMax) * max_joint_position_, 0.0, max_joint_position_);
    joint_position = max_joint_position_ - mapped_position;
    RCLCPP_DEBUG(
      this->get_logger(),
      "Timer feedback: raw_position=%.2f mapped_position=%.4f published_position=%.4f",
      raw_position,
      mapped_position,
      joint_position);
  }

  RCLCPP_DEBUG(
    this->get_logger(),
    "Publishing joint states: position=%.4f effort=%.4f",
    joint_position,
    commanded_effort_);
  // Build the coupled six-joint representation used by the Robotiq 2F model.
  // Sign conventions are URDF-dependent; paired joints move symmetrically.
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

void RobotiqControllerNode::handle_set_joint_state(const std_msgs::msg::Float64::SharedPtr msg)
{
  if (!driver_) {
    RCLCPP_ERROR(this->get_logger(), "set_joint_state ignored: driver is not initialized");
    return;
  }

  constexpr double kRawPositionMax = 255.0;
  const auto requested_position = std::clamp(msg->data, 0.0, max_joint_position_);
  commanded_position_ = requested_position;

  const auto normalized_position = requested_position / max_joint_position_;
  const auto raw_command = static_cast<uint8_t>((1.0 - normalized_position) * kRawPositionMax);
  RCLCPP_DEBUG(
    this->get_logger(),
    "set_joint_state request: requested_position=%.4f normalized_position=%.4f raw_command=%u",
    requested_position,
    normalized_position,
    static_cast<unsigned>(raw_command));

  {
    std::lock_guard<std::mutex> lock(driver_mutex_);
    driver_->set_gripper_position(raw_command);
  }
}

void RobotiqControllerNode::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::ParallelGripperCommand>> goal_handle)
{
  constexpr double kRawPositionMax = 255.0;

  // 1) Validate incoming action goal payload.
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

  // 2) Validate runtime dependencies.
  if (!driver_) {
    auto result = std::make_shared<control_msgs::action::ParallelGripperCommand::Result>();
    result->stalled = true;
    result->reached_goal = false;
    result->state.header.stamp = this->now();
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "Cannot execute gripper goal: driver is not initialized");
    return;
  }

  // 3) Sanitize and cache command.
  // `position` is clamped to modeled gripper range.
  // `effort` is interpreted as percentage [0, 100].
  commanded_position_ = goal->command.position[0];
  commanded_effort_ = goal->command.effort[0];
  commanded_position_ = std::clamp(commanded_position_, 0.0, max_joint_position_);
  commanded_effort_ = std::max(0.0, std::min(100.0, commanded_effort_));
  RCLCPP_DEBUG(
    this->get_logger(),
    "Sanitized goal: commanded_position=%.4f commanded_effort=%.4f max_joint_position=%.4f",
    commanded_position_,
    commanded_effort_,
    max_joint_position_);
  {
    // 4) Convert ROS position/effort into hardware register commands.
    // Position conversion is inverted because raw register values increase
    // toward "closed" while ROS opening position increases toward "open".
    const auto normalized_position = commanded_position_ / max_joint_position_;
    const auto raw_command = static_cast<uint8_t>((1.0 - normalized_position) * 0xFF);
    const auto force_command = static_cast<uint8_t>(commanded_effort_ / 100.0 * 0xFF);
    RCLCPP_DEBUG(
      this->get_logger(),
      "Hardware command: normalized_position=%.4f raw_position=%u force=%u",
      normalized_position,
      static_cast<unsigned>(raw_command),
      static_cast<unsigned>(force_command));
    std::lock_guard<std::mutex> lock(driver_mutex_);
    driver_->set_gripper_position(raw_command);
    driver_->set_force(force_command);
  }

  // 5) Wait until movement ends, cancellation arrives, or timeout is reached.
  // Polling interval keeps latency low without busy-waiting.
  const auto wait_start = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    bool gripper_is_moving = false;
    {
      std::lock_guard<std::mutex> lock(driver_mutex_);
      gripper_is_moving = driver_->gripper_is_moving();
    }
    RCLCPP_DEBUG(this->get_logger(), "Movement poll: gripper_is_moving=%s", gripper_is_moving ? "true" : "false");
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

  // 6) Read final hardware position and remap to ROS model space.
  double raw_position = 0.0;
  {
    std::lock_guard<std::mutex> lock(driver_mutex_);
    raw_position = static_cast<double>(driver_->get_gripper_position());
  }
  const auto mapped_position = std::clamp(
    (raw_position / kRawPositionMax) * max_joint_position_, 0.0, max_joint_position_);
  const auto reached_position = max_joint_position_ - mapped_position;
  RCLCPP_DEBUG(
    this->get_logger(),
    "Final hardware state: raw_position=%.2f mapped_position=%.4f reached_position=%.4f",
    raw_position,
    mapped_position,
    reached_position);


  // 7) Populate action result using the same six-joint convention as the
  // periodic JointState publisher.
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

  // 8) Finalize as canceled or succeeded depending on late cancellation state.
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
  // Standard ROS 2 node lifecycle.
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RobotiqControllerNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("robotiq_controller"), "Startup failed: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
