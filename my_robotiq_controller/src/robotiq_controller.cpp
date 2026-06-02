#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/action/fibonacci.hpp"

class RobotiqControllerNode : public rclcpp::Node
{
public:
  RobotiqControllerNode()
  : Node("dummy_node")
  {
    // Create a topic publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("dummy_topic", 10);
    
    // Create an action server using lambdas for callbacks
    action_server_ = rclcpp_action::create_server<example_interfaces::action::Fibonacci>(
      this,
      "dummy_action",
      [this](const rclcpp_action::GoalUUID & uuid,
             std::shared_ptr<const example_interfaces::action::Fibonacci::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle) {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Accepted goal");
        std::thread{std::bind(&RobotiqControllerNode::execute, this, goal_handle)}.detach();
      });
    
    // Create a timer to publish messages periodically
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RobotiqControllerNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "RobotiqControllerNode initialized");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello from dummy_node";
    publisher_->publish(message);
  }

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<example_interfaces::action::Fibonacci::Feedback>();
    auto result = std::make_shared<example_interfaces::action::Fibonacci::Result>();

    for (int i = 0; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if goal has been cancelled
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      feedback->sequence.push_back(i);
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Server<example_interfaces::action::Fibonacci>::SharedPtr action_server_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotiqControllerNode>());
  rclcpp::shutdown();
  return 0;
}
