#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64.hpp"

#include "franka_msgs/action/move.hpp"

class GripperManager : public rclcpp::Node
{
public:
  using GripperMove = franka_msgs::action::Move;
  using GripperMoveGoalHandle = rclcpp_action::ClientGoalHandle<GripperMove>;

  GripperManager() : Node("gripper_manager")
  {
    gripper_command_topic_ = this->declare_parameter<std::string>("gripper_command_topic", "/teleop/gripper_cmd");
    gripper_action_name_ = this->declare_parameter<std::string>("gripper_action_name", "/right_franka_gripper/move");
    gripper_speed_ = this->declare_parameter<double>("gripper_speed", 0.05);
    command_epsilon_ = this->declare_parameter<double>("gripper_command_epsilon", 1e-3);

    gripper_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      gripper_command_topic_, 10,
      std::bind(&GripperManager::gripperCommandCallback, this, std::placeholders::_1));

    gripper_client_ = rclcpp_action::create_client<GripperMove>(this, gripper_action_name_);

    RCLCPP_INFO(this->get_logger(), "Gripper manager started.");
    RCLCPP_INFO(this->get_logger(), "Gripper command topic: %s", gripper_command_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Gripper action name: %s", gripper_action_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Gripper speed: %.3f", gripper_speed_);
    RCLCPP_INFO(this->get_logger(), "Gripper command epsilon: %.6f", command_epsilon_);
  }

private:
  void gripperCommandCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    const double requested_width = msg->data;

    if (has_last_sent_width_ && std::abs(requested_width - last_sent_width_) <= command_epsilon_)
    {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Ignoring duplicate gripper command: requested=%.6f last_sent=%.6f eps=%.6f",
        requested_width,
        last_sent_width_,
        command_epsilon_);
      return;
    }

    if (goal_active_ && std::abs(requested_width - active_goal_width_) <= command_epsilon_)
    {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Ignoring gripper command while equivalent goal is active: requested=%.6f active=%.6f eps=%.6f",
        requested_width,
        active_goal_width_,
        command_epsilon_);
      return;
    }

    if (!gripper_client_->wait_for_action_server(std::chrono::milliseconds(200)))
    {
      RCLCPP_WARN(this->get_logger(), "Gripper action server not available.");
      return;
    }

    GripperMove::Goal goal;
    goal.width = requested_width;
    goal.speed = gripper_speed_;

    rclcpp_action::Client<GripperMove>::SendGoalOptions options;
    options.goal_response_callback =
      [this, width = requested_width](const GripperMoveGoalHandle::SharedPtr & handle)
      {
        if (!handle)
        {
          goal_active_ = false;
          RCLCPP_WARN(this->get_logger(), "Gripper goal rejected.");
        }
        else
        {
          last_sent_width_ = width;
          has_last_sent_width_ = true;
          RCLCPP_INFO(this->get_logger(), "Gripper goal accepted: width=%.3f speed=%.3f", width, gripper_speed_);
        }
      };

    options.result_callback =
      [this](const GripperMoveGoalHandle::WrappedResult & result)
      {
        goal_active_ = false;
        switch (result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Gripper goal succeeded.");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Gripper goal aborted.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Gripper goal canceled.");
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "Unknown gripper result code.");
            break;
        }
      };

    goal_active_ = true;
    active_goal_width_ = requested_width;
    gripper_client_->async_send_goal(goal, options);
  }

  std::string gripper_command_topic_;
  std::string gripper_action_name_;
  double gripper_speed_;
  double command_epsilon_ = 1e-3;
  bool has_last_sent_width_ = false;
  double last_sent_width_ = 0.0;
  bool goal_active_ = false;
  double active_goal_width_ = 0.0;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_sub_;
  rclcpp_action::Client<GripperMove>::SharedPtr gripper_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperManager>());
  rclcpp::shutdown();
  return 0;
}
