#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "franka_msgs/action/move.hpp"

class GripperManager : public rclcpp::Node
{
public:
  using GripperMove = franka_msgs::action::Move;
  using GripperMoveGoalHandle = rclcpp_action::ClientGoalHandle<GripperMove>;

  GripperManager() : Node("gripper_manager")
  {
    gripper_state_command_topic_ = this->declare_parameter<std::string>(
      "gripper_state_command_topic", "/teleop/gripper_state_cmd");
    gripper_command_topic_ = this->declare_parameter<std::string>(
      "gripper_command_topic", "/teleop/gripper_cmd");
    gripper_inhibit_topic_ = this->declare_parameter<std::string>(
      "gripper_inhibit_topic", "/teleop/gripper_inhibit");
    teleop_control_topic_ = this->declare_parameter<std::string>(
      "teleop_control_topic", "/teleop/control");
    gripper_action_name_ = this->declare_parameter<std::string>("gripper_action_name", "/right_franka_gripper/move");
    gripper_speed_ = this->declare_parameter<double>("gripper_speed", 0.05);
    command_epsilon_ = this->declare_parameter<double>("gripper_command_epsilon", 1e-3);
    gripper_open_width_ = this->declare_parameter<double>("gripper_open_width", 0.080);
    gripper_close_width_ = this->declare_parameter<double>("gripper_close_width", 0.060);
    min_command_interval_sec_ = this->declare_parameter<double>("min_command_interval_sec", 0.5);
    allow_gripper_without_teleop_ = this->declare_parameter<bool>("allow_gripper_without_teleop", false);
    enable_deprecated_width_topic_ = this->declare_parameter<bool>("enable_deprecated_width_topic", false);

    const auto qos1 = rclcpp::QoS(rclcpp::KeepLast(1));

    gripper_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      gripper_state_command_topic_, qos1,
      std::bind(&GripperManager::gripperStateCallback, this, std::placeholders::_1));

    gripper_inhibit_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      gripper_inhibit_topic_, qos1,
      std::bind(&GripperManager::gripperInhibitCallback, this, std::placeholders::_1));

    teleop_control_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      teleop_control_topic_, qos1,
      std::bind(&GripperManager::teleopControlCallback, this, std::placeholders::_1));

    if (enable_deprecated_width_topic_)
    {
      deprecated_gripper_width_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        gripper_command_topic_, qos1,
        std::bind(&GripperManager::deprecatedGripperWidthCallback, this, std::placeholders::_1));
      RCLCPP_WARN(this->get_logger(), "Deprecated width topic ENABLED: %s", gripper_command_topic_.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Deprecated width topic DISABLED (enable_deprecated_width_topic=false).");
    }

    gripper_client_ = rclcpp_action::create_client<GripperMove>(this, gripper_action_name_);

    RCLCPP_INFO(this->get_logger(), "Gripper manager started.");
    RCLCPP_INFO(this->get_logger(), "Gripper state command topic: %s", gripper_state_command_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Gripper inhibit topic: %s", gripper_inhibit_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Teleop control topic: %s", teleop_control_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Gripper action name: %s", gripper_action_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Gripper speed: %.3f", gripper_speed_);
    RCLCPP_INFO(this->get_logger(), "Gripper command epsilon: %.6f", command_epsilon_);
    RCLCPP_INFO(this->get_logger(), "Gripper widths: open=%.3f close=%.3f", gripper_open_width_, gripper_close_width_);
    RCLCPP_INFO(this->get_logger(), "allow_gripper_without_teleop: %s", allow_gripper_without_teleop_ ? "true" : "false");
  }

private:
  static double clampWidth(double width)
  {
    return std::clamp(width, 0.0, 0.08);
  }

  void gripperInhibitCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool inhibited = msg->data;
    if (inhibited != gripper_inhibited_)
    {
      gripper_inhibited_ = inhibited;
      RCLCPP_INFO(
        this->get_logger(),
        "Gripper inhibit state changed: %s", inhibited ? "INHIBITED" : "RELEASED");
    }
  }

  void teleopControlCallback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    const bool enabled = (msg->data == 1u);
    if (enabled != teleop_enabled_)
    {
      teleop_enabled_ = enabled;
      RCLCPP_INFO(
        this->get_logger(),
        "Teleop control state changed: %s (value=%u)", enabled ? "ENABLED" : "DISABLED", msg->data);
    }
  }

  void gripperStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool closed = msg->data;
    const double requested_width = closed ? gripper_close_width_ : gripper_open_width_;
    const char * state_name = closed ? "closed" : "open";


    const bool sent = sendWidthCommand(requested_width, "state", state_name);
    if (sent)
    {
      has_last_commanded_state_ = true;
      last_commanded_closed_state_ = closed;
    }
  }

  void deprecatedGripperWidthCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Received deprecated width command topic %s. Prefer Bool topic %s.",
      gripper_command_topic_.c_str(),
      gripper_state_command_topic_.c_str());

    const double requested_width = msg->data;
    sendWidthCommand(requested_width, "deprecated_width", "n/a");
  }

  bool sendWidthCommand(double requested_width, const char * source, const char * state_name)
  {
    if (!std::isfinite(requested_width))
    {
      RCLCPP_WARN(this->get_logger(),
        "Dropped gripper command: non-finite width [source=%s state=%s]", source, state_name);
      return false;
    }

    if (gripper_inhibited_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Dropped gripper command: gripper is inhibited [source=%s state=%s width=%.6f]",
        source, state_name, requested_width);
      return false;
    }

    if (!teleop_enabled_ && !allow_gripper_without_teleop_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Dropped gripper command: teleop is disabled [source=%s state=%s width=%.6f]",
        source, state_name, requested_width);
      return false;
    }

    if (goal_active_)
    {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Dropped gripper command: goal already active (drop-while-busy) [source=%s state=%s width=%.6f active_width=%.6f]",
        source, state_name, requested_width, active_goal_width_);
      return false;
    }

    const double clamped_width = clampWidth(requested_width);
    if (std::abs(clamped_width - requested_width) > command_epsilon_)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Clamped gripper width from %.6f to %.6f [source=%s state=%s]",
        requested_width,
        clamped_width,
        source,
        state_name);
    }

    if (has_last_sent_width_ && std::abs(clamped_width - last_sent_width_) <= command_epsilon_)
    {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Dropped gripper command: duplicate width=%.6f last_sent=%.6f eps=%.6f [source=%s state=%s]",
        clamped_width,
        last_sent_width_,
        command_epsilon_,
        source,
        state_name);
      return false;
    }

    const auto now_tp = std::chrono::steady_clock::now();
    if (has_last_goal_send_time_)
    {
      const double dt = std::chrono::duration<double>(now_tp - last_goal_send_time_).count();
      if (dt < min_command_interval_sec_)
      {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "Dropped gripper command: rate-limited [source=%s state=%s width=%.6f dt=%.3fs min=%.3fs]",
          source,
          state_name,
          clamped_width,
          dt,
          min_command_interval_sec_);
        return false;
      }
    }

    if (!gripper_client_->wait_for_action_server(std::chrono::milliseconds(200)))
    {
      RCLCPP_WARN(this->get_logger(),
        "Dropped gripper command: action server not available [source=%s state=%s width=%.6f]",
        source, state_name, clamped_width);
      return false;
    }

    GripperMove::Goal goal;
    goal.width = clamped_width;
    goal.speed = gripper_speed_;

    rclcpp_action::Client<GripperMove>::SendGoalOptions options;
    options.goal_response_callback =
      [this, width = clamped_width, source = std::string(source), state = std::string(state_name)](const GripperMoveGoalHandle::SharedPtr & handle)
      {
        if (!handle)
        {
          goal_active_ = false;
          RCLCPP_WARN(this->get_logger(), "Gripper goal rejected [source=%s state=%s].", source.c_str(), state.c_str());
        }
        else
        {
          last_sent_width_ = width;
          has_last_sent_width_ = true;
          RCLCPP_INFO(
            this->get_logger(),
            "Gripper goal accepted [source=%s state=%s width=%.3f speed=%.3f]",
            source.c_str(),
            state.c_str(),
            width,
            gripper_speed_);
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
    active_goal_width_ = clamped_width;
    last_goal_send_time_ = now_tp;
    has_last_goal_send_time_ = true;
    gripper_client_->async_send_goal(goal, options);
    return true;
  }

  std::string gripper_state_command_topic_;
  std::string gripper_command_topic_;
  std::string gripper_inhibit_topic_;
  std::string teleop_control_topic_;
  std::string gripper_action_name_;
  double gripper_speed_;
  double gripper_open_width_;
  double gripper_close_width_;
  double min_command_interval_sec_;
  double command_epsilon_ = 1e-3;
  bool has_last_sent_width_ = false;
  double last_sent_width_ = 0.0;
  bool goal_active_ = false;
  double active_goal_width_ = 0.0;
  bool has_last_commanded_state_ = false;
  bool last_commanded_closed_state_ = false;
  bool has_last_goal_send_time_ = false;
  std::chrono::steady_clock::time_point last_goal_send_time_;
  bool gripper_inhibited_{true};
  bool teleop_enabled_{false};
  bool allow_gripper_without_teleop_{false};
  bool enable_deprecated_width_topic_{false};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_inhibit_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr teleop_control_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr deprecated_gripper_width_sub_;
  rclcpp_action::Client<GripperMove>::SharedPtr gripper_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperManager>());
  rclcpp::shutdown();
  return 0;
}
