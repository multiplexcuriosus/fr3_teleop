#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "franka_msgs/action/move.hpp"
#include "franka_msgs/action/grasp.hpp"

class GripperManager : public rclcpp::Node
{
public:
  using GripperMove = franka_msgs::action::Move;
  using GripperGrasp = franka_msgs::action::Grasp;
  using GripperMoveGoalHandle = rclcpp_action::ClientGoalHandle<GripperMove>;
  using GripperGraspGoalHandle = rclcpp_action::ClientGoalHandle<GripperGrasp>;

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
    gripper_move_action_name_ = this->declare_parameter<std::string>(
      "gripper_move_action_name", "/right_franka_gripper/move");
    gripper_grasp_action_name_ = this->declare_parameter<std::string>(
      "gripper_grasp_action_name", "/right_franka_gripper/grasp");
    gripper_speed_ = this->declare_parameter<double>("gripper_speed", 0.05);
    command_epsilon_ = this->declare_parameter<double>("gripper_command_epsilon", 1e-3);
    gripper_open_width_ = this->declare_parameter<double>("gripper_open_width", 0.080);
    gripper_close_width_ = this->declare_parameter<double>("gripper_close_width", 0.040);
    use_grasp_for_close_ = this->declare_parameter<bool>("use_grasp_for_close", true);
    gripper_grasp_width_ = this->declare_parameter<double>("gripper_grasp_width", 0.040);
    gripper_grasp_epsilon_inner_ = this->declare_parameter<double>("gripper_grasp_epsilon_inner", 0.020);
    gripper_grasp_epsilon_outer_ = this->declare_parameter<double>("gripper_grasp_epsilon_outer", 0.020);
    gripper_grasp_force_ = this->declare_parameter<double>("gripper_grasp_force", 40.0);
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

    gripper_move_client_ = rclcpp_action::create_client<GripperMove>(this, gripper_move_action_name_);
    gripper_grasp_client_ = rclcpp_action::create_client<GripperGrasp>(this, gripper_grasp_action_name_);
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&GripperManager::onSetParameters, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Gripper manager started.");
    RCLCPP_INFO(this->get_logger(), "Gripper state command topic: %s", gripper_state_command_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Gripper inhibit topic: %s", gripper_inhibit_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Teleop control topic: %s", teleop_control_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Gripper MOVE action: %s", gripper_move_action_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Gripper GRASP action: %s", gripper_grasp_action_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Gripper speed: %.3f", gripper_speed_);
    RCLCPP_INFO(this->get_logger(), "Gripper command epsilon: %.6f", command_epsilon_);
    RCLCPP_INFO(this->get_logger(), "Gripper open width: %.3f", gripper_open_width_);
    RCLCPP_INFO(this->get_logger(), "Gripper close width: %.3f", gripper_close_width_);
    RCLCPP_INFO(this->get_logger(), "Gripper close mode: %s", use_grasp_for_close_ ? "GRASP" : "MOVE");
    RCLCPP_INFO(this->get_logger(), "Gripper grasp width: %.3f  epsilon_inner=%.3f  epsilon_outer=%.3f  force=%.1f N",
      gripper_grasp_width_, gripper_grasp_epsilon_inner_, gripper_grasp_epsilon_outer_, gripper_grasp_force_);
    RCLCPP_INFO(this->get_logger(), "allow_gripper_without_teleop: %s", allow_gripper_without_teleop_ ? "true" : "false");
  }

private:
  static double clampWidth(double width)
  {
    return std::clamp(width, 0.0, 0.08);
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter> & parameters)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    auto get_numeric_param = [&result](const rclcpp::Parameter & param, double & out_value) -> bool
    {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        out_value = param.as_double();
        return true;
      }
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        out_value = static_cast<double>(param.as_int());
        return true;
      }
      result.successful = false;
      result.reason = param.get_name() + " must be numeric";
      return false;
    };

    double new_gripper_speed = gripper_speed_;
    double new_gripper_open_width = gripper_open_width_;
    double new_gripper_close_width = gripper_close_width_;
    bool new_use_grasp_for_close = use_grasp_for_close_;
    double new_gripper_grasp_width = gripper_grasp_width_;
    double new_gripper_grasp_force = gripper_grasp_force_;
    double new_gripper_grasp_epsilon_inner = gripper_grasp_epsilon_inner_;
    double new_gripper_grasp_epsilon_outer = gripper_grasp_epsilon_outer_;
    double new_command_epsilon = command_epsilon_;
    double new_min_command_interval_sec = min_command_interval_sec_;
    bool new_allow_gripper_without_teleop = allow_gripper_without_teleop_;
    bool tuning_changed = false;

    for (const auto & param : parameters)
    {
      const auto & name = param.get_name();

      if (name == "gripper_speed")
      {
        double value = 0.0;
        if (!get_numeric_param(param, value))
        {
          return result;
        }
        if (!std::isfinite(value) || value <= 0.0)
        {
          result.successful = false;
          result.reason = "gripper_speed must be finite and > 0";
          return result;
        }
        new_gripper_speed = value;
        tuning_changed = true;
      }
      else if (name == "gripper_open_width")
      {
        double value = 0.0;
        if (!get_numeric_param(param, value))
        {
          return result;
        }
        if (!std::isfinite(value) || value < 0.0 || value > 0.08)
        {
          result.successful = false;
          result.reason = "gripper_open_width must be finite and in [0.0, 0.08]";
          return result;
        }
        new_gripper_open_width = value;
        tuning_changed = true;
      }
      else if (name == "gripper_grasp_width")
      {
        double value = 0.0;
        if (!get_numeric_param(param, value))
        {
          return result;
        }
        if (!std::isfinite(value) || value < 0.0 || value > 0.08)
        {
          result.successful = false;
          result.reason = "gripper_grasp_width must be finite and in [0.0, 0.08]";
          return result;
        }
        new_gripper_grasp_width = value;
        tuning_changed = true;
      }
      else if (name == "gripper_close_width")
      {
        double value = 0.0;
        if (!get_numeric_param(param, value))
        {
          return result;
        }
        if (!std::isfinite(value) || value < 0.0 || value > 0.08)
        {
          result.successful = false;
          result.reason = "gripper_close_width must be finite and in [0.0, 0.08]";
          return result;
        }
        new_gripper_close_width = value;
        tuning_changed = true;
      }
      else if (name == "use_grasp_for_close")
      {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
        {
          result.successful = false;
          result.reason = "use_grasp_for_close must be bool";
          return result;
        }
        new_use_grasp_for_close = param.as_bool();
        tuning_changed = true;
      }
      else if (name == "gripper_grasp_force")
      {
        double value = 0.0;
        if (!get_numeric_param(param, value))
        {
          return result;
        }
        if (!std::isfinite(value) || value <= 0.0)
        {
          result.successful = false;
          result.reason = "gripper_grasp_force must be finite and > 0";
          return result;
        }
        new_gripper_grasp_force = value;
        tuning_changed = true;
      }
      else if (name == "gripper_grasp_epsilon_inner")
      {
        double value = 0.0;
        if (!get_numeric_param(param, value))
        {
          return result;
        }
        if (!std::isfinite(value) || value < 0.0)
        {
          result.successful = false;
          result.reason = "gripper_grasp_epsilon_inner must be finite and >= 0";
          return result;
        }
        new_gripper_grasp_epsilon_inner = value;
        tuning_changed = true;
      }
      else if (name == "gripper_grasp_epsilon_outer")
      {
        double value = 0.0;
        if (!get_numeric_param(param, value))
        {
          return result;
        }
        if (!std::isfinite(value) || value < 0.0)
        {
          result.successful = false;
          result.reason = "gripper_grasp_epsilon_outer must be finite and >= 0";
          return result;
        }
        new_gripper_grasp_epsilon_outer = value;
        tuning_changed = true;
      }
      else if (name == "gripper_command_epsilon")
      {
        double value = 0.0;
        if (!get_numeric_param(param, value))
        {
          return result;
        }
        if (!std::isfinite(value) || value < 0.0)
        {
          result.successful = false;
          result.reason = "gripper_command_epsilon must be finite and >= 0";
          return result;
        }
        new_command_epsilon = value;
        tuning_changed = true;
      }
      else if (name == "min_command_interval_sec")
      {
        double value = 0.0;
        if (!get_numeric_param(param, value))
        {
          return result;
        }
        if (!std::isfinite(value) || value < 0.0)
        {
          result.successful = false;
          result.reason = "min_command_interval_sec must be finite and >= 0";
          return result;
        }
        new_min_command_interval_sec = value;
        tuning_changed = true;
      }
      else if (name == "allow_gripper_without_teleop")
      {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
        {
          result.successful = false;
          result.reason = "allow_gripper_without_teleop must be bool";
          return result;
        }
        new_allow_gripper_without_teleop = param.as_bool();
        tuning_changed = true;
      }
      else if (name == "gripper_state_command_topic" ||
               name == "gripper_command_topic" ||
               name == "gripper_inhibit_topic" ||
               name == "teleop_control_topic" ||
               name == "gripper_move_action_name" ||
               name == "gripper_grasp_action_name")
      {
        result.successful = false;
        result.reason = "topic/action parameters require node restart";
        return result;
      }
    }

    gripper_speed_ = new_gripper_speed;
    gripper_open_width_ = new_gripper_open_width;
    gripper_close_width_ = new_gripper_close_width;
    use_grasp_for_close_ = new_use_grasp_for_close;
    gripper_grasp_width_ = new_gripper_grasp_width;
    gripper_grasp_force_ = new_gripper_grasp_force;
    gripper_grasp_epsilon_inner_ = new_gripper_grasp_epsilon_inner;
    gripper_grasp_epsilon_outer_ = new_gripper_grasp_epsilon_outer;
    command_epsilon_ = new_command_epsilon;
    min_command_interval_sec_ = new_min_command_interval_sec;
    allow_gripper_without_teleop_ = new_allow_gripper_without_teleop;

    if (tuning_changed)
    {
      RCLCPP_INFO(this->get_logger(),
        "Updated gripper tuning: speed=%.3f open_width=%.3f close_width=%.3f close_mode=%s grasp_width=%.3f force=%.1f eps_inner=%.3f eps_outer=%.3f cmd_eps=%.6f min_interval=%.3f allow_without_teleop=%s",
        gripper_speed_, gripper_open_width_, gripper_close_width_, use_grasp_for_close_ ? "GRASP" : "MOVE", gripper_grasp_width_, gripper_grasp_force_,
        gripper_grasp_epsilon_inner_, gripper_grasp_epsilon_outer_, command_epsilon_,
        min_command_interval_sec_, allow_gripper_without_teleop_ ? "true" : "false");
    }

    return result;
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
    bool sent = false;
    if (closed)
    {
      if (use_grasp_for_close_)
      {
        sent = sendGraspCommand("state", "closed");
      }
      else
      {
        sent = sendMoveCommand(gripper_close_width_, "state", "closed");
      }
    }
    else
    {
      sent = sendMoveCommand(gripper_open_width_, "state", "open");
    }
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

    sendMoveCommand(msg->data, "deprecated_width", "n/a");
  }

  // Returns false (and logs) if any shared guard rejects the command.
  // width_for_log is used only for log messages; pass NaN for grasp commands.
  bool commonCommandGuards(const char * source, const char * state_name, double width_for_log)
  {
    if (gripper_inhibited_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Dropped gripper command: gripper is inhibited [source=%s state=%s]",
        source, state_name);
      return false;
    }

    if (!teleop_enabled_ && !allow_gripper_without_teleop_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Dropped gripper command: teleop is disabled [source=%s state=%s]",
        source, state_name);
      return false;
    }

    if (goal_active_)
    {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Dropped gripper command: goal already active (drop-while-busy) [source=%s state=%s active_width=%.6f]",
        source, state_name, active_goal_width_);
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
          source, state_name, width_for_log, dt, min_command_interval_sec_);
        return false;
      }
    }
    return true;
  }

  bool sendMoveCommand(double requested_width, const char * source, const char * state_name)
  {
    if (!std::isfinite(requested_width))
    {
      RCLCPP_WARN(this->get_logger(),
        "Dropped gripper MOVE command: non-finite width [source=%s state=%s]", source, state_name);
      return false;
    }

    if (!commonCommandGuards(source, state_name, requested_width))
    {
      return false;
    }

    const double clamped_width = clampWidth(requested_width);
    if (std::abs(clamped_width - requested_width) > command_epsilon_)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Clamped gripper MOVE width from %.6f to %.6f [source=%s state=%s]",
        requested_width, clamped_width, source, state_name);
    }

    if (has_last_sent_width_ && std::abs(clamped_width - last_sent_width_) <= command_epsilon_)
    {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Dropped gripper MOVE command: duplicate width=%.6f last_sent=%.6f eps=%.6f [source=%s state=%s]",
        clamped_width, last_sent_width_, command_epsilon_, source, state_name);
      return false;
    }

    if (!gripper_move_client_->wait_for_action_server(std::chrono::milliseconds(200)))
    {
      RCLCPP_WARN(this->get_logger(),
        "Dropped gripper MOVE command: action server not available [source=%s state=%s width=%.6f]",
        source, state_name, clamped_width);
      return false;
    }

    GripperMove::Goal goal;
    goal.width = clamped_width;
    goal.speed = gripper_speed_;

    rclcpp_action::Client<GripperMove>::SendGoalOptions options;
    options.goal_response_callback =
      [this, width = clamped_width, src = std::string(source), state = std::string(state_name)](
        const GripperMoveGoalHandle::SharedPtr & handle)
      {
        if (!handle)
        {
          goal_active_ = false;
          RCLCPP_WARN(this->get_logger(),
            "Gripper MOVE goal rejected [source=%s state=%s].", src.c_str(), state.c_str());
        }
        else
        {
          last_sent_width_ = width;
          has_last_sent_width_ = true;
          // A successful move/open clears grasp-duplicate state so a later close can GRASP again.
          has_last_sent_grasp_ = false;
          RCLCPP_INFO(this->get_logger(),
            "Gripper MOVE goal accepted [source=%s state=%s width=%.3f speed=%.3f]",
            src.c_str(), state.c_str(), width, gripper_speed_);
        }
      };

    options.result_callback =
      [this](const GripperMoveGoalHandle::WrappedResult & result)
      {
        goal_active_ = false;
        switch (result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Gripper MOVE goal succeeded.");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Gripper MOVE goal aborted.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Gripper MOVE goal canceled.");
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "Gripper MOVE goal: unknown result code.");
            break;
        }
      };

    goal_active_ = true;
    active_goal_width_ = clamped_width;
    last_goal_send_time_ = std::chrono::steady_clock::now();
    has_last_goal_send_time_ = true;
    gripper_move_client_->async_send_goal(goal, options);
    return true;
  }

  bool sendGraspCommand(const char * source, const char * state_name)
  {
    if (!std::isfinite(gripper_grasp_width_) ||
        !std::isfinite(gripper_grasp_epsilon_inner_) ||
        !std::isfinite(gripper_grasp_epsilon_outer_) ||
        !std::isfinite(gripper_grasp_force_) ||
        !std::isfinite(gripper_speed_))
    {
      RCLCPP_WARN(this->get_logger(),
        "Dropped gripper GRASP command: non-finite grasp parameter [source=%s state=%s]",
        source, state_name);
      return false;
    }

    if (!commonCommandGuards(source, state_name, gripper_grasp_width_))
    {
      return false;
    }

    // Duplicate grasp filter: skip if last commanded state was already grasp/closed
    if (has_last_sent_grasp_)
    {
      RCLCPP_DEBUG(this->get_logger(),
        "Dropped gripper GRASP command: duplicate [source=%s state=%s]", source, state_name);
      return false;
    }

    if (!gripper_grasp_client_->wait_for_action_server(std::chrono::milliseconds(200)))
    {
      RCLCPP_WARN(this->get_logger(),
        "Dropped gripper GRASP command: action server not available [source=%s state=%s]",
        source, state_name);
      return false;
    }

    GripperGrasp::Goal goal;
    goal.width = clampWidth(gripper_grasp_width_);
    goal.speed = gripper_speed_;
    goal.force = gripper_grasp_force_;
    goal.epsilon.inner = gripper_grasp_epsilon_inner_;
    goal.epsilon.outer = gripper_grasp_epsilon_outer_;

    rclcpp_action::Client<GripperGrasp>::SendGoalOptions options;
    options.goal_response_callback =
      [this, src = std::string(source), state = std::string(state_name)](
        const GripperGraspGoalHandle::SharedPtr & handle)
      {
        if (!handle)
        {
          goal_active_ = false;
          RCLCPP_WARN(this->get_logger(),
            "Gripper GRASP goal rejected [source=%s state=%s].", src.c_str(), state.c_str());
        }
        else
        {
          has_last_sent_grasp_ = true;
          // A grasp invalidates the previous move-width duplicate filter.
          has_last_sent_width_ = false;
          RCLCPP_INFO(this->get_logger(),
            "Gripper GRASP goal accepted [source=%s state=%s width=%.3f speed=%.3f force=%.1f eps_inner=%.3f eps_outer=%.3f]",
            src.c_str(), state.c_str(),
            gripper_grasp_width_, gripper_speed_, gripper_grasp_force_,
            gripper_grasp_epsilon_inner_, gripper_grasp_epsilon_outer_);
        }
      };

    options.result_callback =
      [this](const GripperGraspGoalHandle::WrappedResult & result)
      {
        goal_active_ = false;
        switch (result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Gripper GRASP goal succeeded.");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Gripper GRASP goal aborted.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Gripper GRASP goal canceled.");
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "Gripper GRASP goal: unknown result code.");
            break;
        }
      };

    goal_active_ = true;
    active_goal_width_ = goal.width;
    last_goal_send_time_ = std::chrono::steady_clock::now();
    has_last_goal_send_time_ = true;
    gripper_grasp_client_->async_send_goal(goal, options);
    return true;
  }

  std::string gripper_state_command_topic_;
  std::string gripper_command_topic_; 
  std::string gripper_inhibit_topic_;
  std::string teleop_control_topic_;
  std::string gripper_move_action_name_;
  std::string gripper_grasp_action_name_;
  double gripper_speed_;
  double gripper_open_width_;
  double gripper_close_width_;
  bool use_grasp_for_close_;
  double gripper_grasp_width_;
  double gripper_grasp_epsilon_inner_;
  double gripper_grasp_epsilon_outer_;
  double gripper_grasp_force_;
  double min_command_interval_sec_;
  double command_epsilon_ = 1e-3;
  bool has_last_sent_width_ = false;
  double last_sent_width_ = 0.0;
  bool has_last_sent_grasp_ = false;
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
  rclcpp_action::Client<GripperMove>::SharedPtr gripper_move_client_;
  rclcpp_action::Client<GripperGrasp>::SharedPtr gripper_grasp_client_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperManager>());
  rclcpp::shutdown();
  return 0;
}
