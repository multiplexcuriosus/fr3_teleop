#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "franka_msgs/action/move.hpp"
#include "fr3_husky_msgs/action/omega_haptic.hpp"
#include "fr3_husky_msgs/action/move_to_joint.hpp"

class Ps4InputManager : public rclcpp::Node
{
public:
  using GripperMove = franka_msgs::action::Move;
  using GripperMoveGoalHandle = rclcpp_action::ClientGoalHandle<GripperMove>;

  using TeleopAction = fr3_husky_msgs::action::OmegaHaptic;
  using TeleopGoalHandle = rclcpp_action::ClientGoalHandle<TeleopAction>;

  using HomeAction = fr3_husky_msgs::action::MoveToJoint;
  using HomeGoalHandle = rclcpp_action::ClientGoalHandle<HomeAction>;

  Ps4InputManager() : Node("ps4_input_manager")
  {
    // Button mapping
    button_gripper_open_ = this->declare_parameter<int>("button_gripper_open", 2);    // triangle
    button_gripper_close_ = this->declare_parameter<int>("button_gripper_close", 0);  // cross
    axis_episode_ = this->declare_parameter<int>("axis_episode", 6);
    button_home_ = this->declare_parameter<int>("button_home", 1);                    // circle
    axis_teleop_ = this->declare_parameter<int>("axis_teleop", 7);

    // Teleop mapping moved from old action server
    left_stick_x_idx_ = this->declare_parameter<int>("left_stick_x_idx", 0);
    left_stick_y_idx_ = this->declare_parameter<int>("left_stick_y_idx", 1);
    right_stick_y_idx_ = this->declare_parameter<int>("right_stick_y_idx", 4);

    deadzone_ = this->declare_parameter<double>("deadzone", 0.08);
    max_vx_ = this->declare_parameter<double>("max_vx", 0.08);
    max_vy_ = this->declare_parameter<double>("max_vy", 0.08);
    max_vz_ = this->declare_parameter<double>("max_vz", 0.05);

    haptic_pos_multiplier_ = this->declare_parameter<double>("haptic_pos_multiplier", 2.0);
    haptic_lin_vel_multiplier_ = this->declare_parameter<double>("haptic_lin_vel_multiplier", 1.0);
    haptic_ori_multiplier_ = this->declare_parameter<double>("haptic_ori_multiplier", 1.0);
    haptic_ang_vel_multiplier_ = this->declare_parameter<double>("haptic_ang_vel_multiplier", 1.0);

    twist_topic_name_ = this->declare_parameter<std::string>("twist_topic_name", "/cartesian_cmd/twist");
    twist_frame_id_ = this->declare_parameter<std::string>("twist_frame_id", "base_link");
    teleop_control_topic_ = this->declare_parameter<std::string>("teleop_control_topic", "/teleop/control");
    teleop_action_name_ = this->declare_parameter<std::string>("teleop_action_name", "/cartesian_executor");
    teleop_mode_ = this->declare_parameter<int>("teleop_mode", 0);
    teleop_ee_name_ = this->declare_parameter<std::string>("teleop_ee_name", "right_fr3_hand_tcp");
    teleop_move_orientation_ = this->declare_parameter<bool>("teleop_move_orientation", false);

    // Gripper preset widths
    gripper_open_width_ = this->declare_parameter<double>("gripper_open_width", 0.080);
    gripper_close_width_ = this->declare_parameter<double>("gripper_close_width", 0.06);
    gripper_speed_ = this->declare_parameter<double>("gripper_speed", 0.05);

    // Action names
    gripper_action_name_ = this->declare_parameter<std::string>(
        "gripper_action_name", "/right_franka_gripper/move");
    home_action_name_ = this->declare_parameter<std::string>(
        "home_action_name", "/fr3_move_to_joint");

    // Home configuration
    home_joint_names_ = this->declare_parameter<std::vector<std::string>>(
        "home_joint_names",
        {"right_fr3_joint1", "right_fr3_joint2", "right_fr3_joint3", "right_fr3_joint4",
         "right_fr3_joint5", "right_fr3_joint6", "right_fr3_joint7"});

    home_joint_positions_ = this->declare_parameter<std::vector<double>>(
        "home_joint_positions",
        std::vector<double>{});

    if (home_joint_positions_.empty())
    {
      RCLCPP_FATAL(this->get_logger(), "Parameter 'home_joint_positions' must be provided via config.");
      throw std::runtime_error("Missing required parameter: home_joint_positions");
    }

    home_vel_scale_ = this->declare_parameter<double>("home_vel_scale", 0.1);
    home_acc_scale_ = this->declare_parameter<double>("home_acc_scale", 0.1);

    episode_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/episode/control", 10);
    teleop_pub_ = this->create_publisher<std_msgs::msg::UInt8>(teleop_control_topic_, 10);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_name_, 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&Ps4InputManager::joyCallback, this, std::placeholders::_1));

    gripper_client_ = rclcpp_action::create_client<GripperMove>(this, gripper_action_name_);
    teleop_client_ = rclcpp_action::create_client<TeleopAction>(this, teleop_action_name_);
    home_client_ = rclcpp_action::create_client<HomeAction>(this, home_action_name_);

    RCLCPP_INFO(this->get_logger(), "PS4 input manager started.");
    RCLCPP_INFO(this->get_logger(), "Teleop control topic: %s", teleop_control_topic_.c_str());
  }

private:
  int buttonSafe(const std::vector<int32_t> & buttons, int idx) const
  {
    if (idx < 0 || idx >= static_cast<int>(buttons.size()))
    {
      return 0;
    }
    return buttons[idx];
  }

  float axisSafe(const std::vector<float> & axes, int idx) const
  {
    if (idx < 0 || idx >= static_cast<int>(axes.size()))
    {
      return 0.0f;
    }
    return axes[idx];
  }

  bool risingEdge(const std::vector<int32_t> & current, int idx) const
  {
    const int curr = buttonSafe(current, idx);
    const int prev = buttonSafe(prev_buttons_, idx);
    return (curr == 1 && prev == 0);
  }

  double applyDeadzone(double value) const
  {
    if (std::abs(value) < deadzone_)
    {
      return 0.0;
    }
    return value;
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!has_prev_)
    {
      prev_buttons_ = msg->buttons;
      has_prev_ = true;
    }

    if (risingEdge(msg->buttons, button_gripper_open_))
    {
      if (!teleop_session_enabled_)
      {
        RCLCPP_WARN(this->get_logger(), "Ignoring gripper open: teleop session is not enabled.");
      }
      else
      {
        sendGripperGoal(gripper_open_width_);
      }
    }

    if (risingEdge(msg->buttons, button_gripper_close_))
    {
      if (!teleop_session_enabled_)
      {
        RCLCPP_WARN(this->get_logger(), "Ignoring gripper close: teleop session is not enabled.");
      }
      else
      {
        sendGripperGoal(gripper_close_width_);
      }
    }

    handleEpisodeAxis(msg);

    if (risingEdge(msg->buttons, button_home_))
    {
      handleHomePressed();
    }

    handleTeleopAxis(msg);

    handleTeleopTwist(msg);

    prev_buttons_ = msg->buttons;
  }

  void handleEpisodeAxis(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const float val = axisSafe(msg->axes, axis_episode_);

    int state = 0;
    if (val > 0.9f)
    {
      state = 1;
    }
    else if (val < -0.9f)
    {
      state = -1;
    }

    if (state != prev_episode_state_)
    {
      if (state == 1)
      {
        publishEpisodeCommand(1);
        RCLCPP_INFO(this->get_logger(), "Episode START");
      }
      else if (state == -1)
      {
        publishEpisodeCommand(2);
        RCLCPP_INFO(this->get_logger(), "Episode STOP");
      }
    }

    prev_episode_state_ = state;
  }

  void handleTeleopAxis(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const float val = axisSafe(msg->axes, axis_teleop_);

    int state = 0;
    if (val > 0.9f)
    {
      state = 1;
    }
    else if (val < -0.9f)
    {
      state = -1;
    }

    if (state != prev_teleop_axis_state_)
    {
      if (state == 1)
      {
        if (home_goal_in_progress_)
        {
          RCLCPP_WARN(this->get_logger(), "Cannot enable teleop while home goal is in progress.");
        }
        else if (pending_home_after_teleop_stop_)
        {
          RCLCPP_WARN(this->get_logger(), "Cannot enable teleop while waiting to start homing.");
        }
        else
        {
          enableTeleopSession();
        }
      }
      else if (state == -1)
      {
        disableTeleopSession();
      }
    }

    prev_teleop_axis_state_ = state;
  }

  void handleHomePressed()
  {
    if (home_goal_in_progress_)
    {
      RCLCPP_WARN(this->get_logger(), "Home goal already in progress.");
      return;
    }

    if (pending_home_after_teleop_stop_)
    {
      RCLCPP_WARN(this->get_logger(), "Home goal is already queued until teleop stops.");
      return;
    }

    if (teleop_session_enabled_ || teleop_goal_pending_ || teleop_goal_in_progress_ || teleop_goal_handle_)
    {
      RCLCPP_INFO(this->get_logger(), "Canceling teleop action before sending home goal...");
      pending_home_after_teleop_stop_ = true;
      disableTeleopSession();
      return;
    }

    sendHomeGoalNow();
  }

  void enableTeleopSession()
  {
    if (teleop_goal_pending_ || teleop_goal_in_progress_)
    {
      RCLCPP_INFO(this->get_logger(), "Teleop action already active or pending.");
      return;
    }

    if (!teleop_client_->wait_for_action_server(std::chrono::milliseconds(200)))
    {
      RCLCPP_WARN(this->get_logger(), "Teleop action server not available.");
      return;
    }

    TeleopAction::Goal goal;
    goal.mode = teleop_mode_;
    goal.ee_name = teleop_ee_name_;
    goal.move_orientation = teleop_move_orientation_;
    goal.hapic_pos_multiplier = static_cast<float>(haptic_pos_multiplier_);
    goal.hapic_ori_multiplier = static_cast<float>(haptic_ori_multiplier_);
    goal.hapic_lin_vel_multiplier = static_cast<float>(haptic_lin_vel_multiplier_);
    goal.hapic_ang_vel_multiplier = static_cast<float>(haptic_ang_vel_multiplier_);

    rclcpp_action::Client<TeleopAction>::SendGoalOptions options;
    options.goal_response_callback =
        [this](const TeleopGoalHandle::SharedPtr & handle)
    {
      teleop_goal_pending_ = false;

      if (!handle)
      {
        teleop_goal_handle_.reset();
        teleop_goal_in_progress_ = false;
        teleop_session_enabled_ = false;
        publishTeleopCommand(2);
        RCLCPP_WARN(this->get_logger(), "Teleop action goal rejected.");

        if (pending_home_after_teleop_stop_)
        {
          pending_home_after_teleop_stop_ = false;
          sendHomeGoalNow();
        }
        return;
      }

      teleop_goal_handle_ = handle;
      teleop_goal_in_progress_ = true;

      if (teleop_cancel_requested_)
      {
        requestCancelTeleopGoal();
        return;
      }

      teleop_session_enabled_ = true;
      publishTeleopCommand(1);
      publishZeroTwist();
      RCLCPP_INFO(this->get_logger(), "Teleop session enabled.");
    };

    options.result_callback =
        [this](const TeleopGoalHandle::WrappedResult & result)
    {
      const bool start_home_after_stop = pending_home_after_teleop_stop_;

      teleop_goal_handle_.reset();
      teleop_goal_pending_ = false;
      teleop_goal_in_progress_ = false;
      teleop_cancel_requested_ = false;
      teleop_cancel_in_progress_ = false;
      teleop_session_enabled_ = false;
      pending_home_after_teleop_stop_ = false;

      publishTeleopCommand(2);
      publishZeroTwist();

      switch (result.code)
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Teleop action completed.");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_WARN(this->get_logger(), "Teleop action aborted.");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_INFO(this->get_logger(), "Teleop action canceled.");
          break;
        default:
          RCLCPP_WARN(this->get_logger(), "Unknown teleop action result code.");
          break;
      }

      if (start_home_after_stop)
      {
        sendHomeGoalNow();
      }
    };

    teleop_goal_pending_ = true;
    teleop_cancel_requested_ = false;
    teleop_client_->async_send_goal(goal, options);

    RCLCPP_INFO(this->get_logger(), "Sending teleop action goal...");
  }

  void disableTeleopSession()
  {
    const bool teleop_was_active = teleop_session_enabled_ || teleop_goal_pending_ ||
      teleop_goal_in_progress_ || static_cast<bool>(teleop_goal_handle_);

    teleop_session_enabled_ = false;

    if (teleop_was_active)
    {
      publishTeleopCommand(2);
    }

    publishZeroTwist();

    if (teleop_goal_pending_ || teleop_goal_in_progress_ || teleop_goal_handle_)
    {
      teleop_cancel_requested_ = true;
      requestCancelTeleopGoal();
    }

    RCLCPP_INFO(this->get_logger(), "Teleop session disabled.");
  }

  void requestCancelTeleopGoal()
  {
    if (teleop_cancel_in_progress_)
    {
      return;
    }

    if (teleop_goal_handle_)
    {
      teleop_cancel_in_progress_ = true;
      teleop_client_->async_cancel_goal(
          teleop_goal_handle_,
          [this](auto future)
          {
            teleop_cancel_in_progress_ = false;
            const auto cancel_response = future.get();
            if (cancel_response->goals_canceling.empty())
            {
              RCLCPP_WARN(this->get_logger(), "Teleop action cancel request was not accepted.");
            }
            else
            {
              RCLCPP_INFO(this->get_logger(), "Teleop action cancel requested.");
            }
          });
      return;
    }

    if (teleop_goal_pending_)
    {
      RCLCPP_INFO(this->get_logger(), "Teleop goal is pending; cancel will be requested after acceptance.");
      return;
    }

    teleop_cancel_requested_ = false;
    if (pending_home_after_teleop_stop_)
    {
      pending_home_after_teleop_stop_ = false;
      sendHomeGoalNow();
    }
  }

  void handleTeleopTwist(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!teleop_session_enabled_)
    {
      return;
    }

    // Moved from old teleop/action-server side:
    // map joystick axes -> Cartesian linear velocity
    const double lx = applyDeadzone(axisSafe(msg->axes, left_stick_x_idx_));
    const double ly = applyDeadzone(axisSafe(msg->axes, left_stick_y_idx_));
    const double ry = applyDeadzone(axisSafe(msg->axes, right_stick_y_idx_));

    // This sign convention may need one final flip depending on your robot frame.
    // But structurally this is the right place for the mapping now.
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = twist_frame_id_;

    twist_msg.twist.linear.x = (ly) * max_vx_ * haptic_pos_multiplier_ * haptic_lin_vel_multiplier_;
    twist_msg.twist.linear.y = (-lx) * max_vy_ * haptic_pos_multiplier_ * haptic_lin_vel_multiplier_;
    twist_msg.twist.linear.z = (-ry) * max_vz_ * haptic_pos_multiplier_ * haptic_lin_vel_multiplier_;

    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = 0.0;

    twist_pub_->publish(twist_msg);
  }

  void publishZeroTwist()
  {
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = twist_frame_id_;
    twist_msg.twist.linear.x = 0.0;
    twist_msg.twist.linear.y = 0.0;
    twist_msg.twist.linear.z = 0.0;
    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = 0.0;
    twist_pub_->publish(twist_msg);
  }

  void publishEpisodeCommand(uint8_t value)
  {
    std_msgs::msg::UInt8 msg;
    msg.data = value;
    episode_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published episode command: %u", value);
  }

  void publishTeleopCommand(uint8_t value)
  {
    std_msgs::msg::UInt8 msg;
    msg.data = value;
    teleop_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published teleop command: %u", value);
  }

  void sendGripperGoal(double width)
  {
    if (!gripper_client_->wait_for_action_server(std::chrono::milliseconds(200)))
    {
      RCLCPP_WARN(this->get_logger(), "Gripper action server not available.");
      return;
    }

    GripperMove::Goal goal;
    goal.width = width;
    goal.speed = gripper_speed_;

    rclcpp_action::Client<GripperMove>::SendGoalOptions options;
    options.goal_response_callback =
        [this, width](const GripperMoveGoalHandle::SharedPtr & handle)
    {
      if (!handle)
      {
        RCLCPP_WARN(this->get_logger(), "Gripper goal rejected.");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Gripper goal accepted: width=%.3f", width);
      }
    };

    options.result_callback =
        [this](const GripperMoveGoalHandle::WrappedResult & result)
    {
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

    gripper_client_->async_send_goal(goal, options);
  }

  void sendHomeGoalNow()
  {
    if (!home_client_->wait_for_action_server(std::chrono::milliseconds(200)))
    {
      RCLCPP_WARN(this->get_logger(), "Home action server not available.");
      return;
    }

    if (home_joint_names_.size() != home_joint_positions_.size())
    {
      RCLCPP_ERROR(this->get_logger(), "Home joint name / position size mismatch.");
      return;
    }

    home_goal_in_progress_ = true;

    HomeAction::Goal goal;
    goal.joint_names = home_joint_names_;
    goal.target_positions = home_joint_positions_;
    goal.max_velocity_scaling_factor = home_vel_scale_;
    goal.max_acceleration_scaling_factor = home_acc_scale_;

    rclcpp_action::Client<HomeAction>::SendGoalOptions options;

    options.goal_response_callback =
        [this](const HomeGoalHandle::SharedPtr & handle)
    {
      if (!handle)
      {
        home_goal_in_progress_ = false;
        RCLCPP_WARN(this->get_logger(), "Home goal rejected.");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Home goal accepted.");
      }
    };

    options.result_callback =
        [this](const HomeGoalHandle::WrappedResult & result)
    {
      home_goal_in_progress_ = false;
      switch (result.code)
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Home goal succeeded.");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_WARN(this->get_logger(), "Home goal aborted.");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(this->get_logger(), "Home goal canceled.");
          break;
        default:
          RCLCPP_WARN(this->get_logger(), "Unknown home result code.");
          break;
      }
    };

    RCLCPP_INFO(this->get_logger(), "Sending home goal now...");
    home_client_->async_send_goal(goal, options);
  }

  int button_gripper_open_;
  int button_gripper_close_;
  int axis_episode_;
  int axis_teleop_;
  int button_home_;

  int left_stick_x_idx_;
  int left_stick_y_idx_;
  int right_stick_y_idx_;

  int prev_episode_state_{0};
  int prev_teleop_axis_state_{0};

  double deadzone_;
  double max_vx_;
  double max_vy_;
  double max_vz_;
  double haptic_pos_multiplier_;
  double haptic_ori_multiplier_;
  double haptic_lin_vel_multiplier_;
  double haptic_ang_vel_multiplier_;

  double gripper_open_width_;
  double gripper_close_width_;
  double gripper_speed_;

  std::string twist_topic_name_;
  std::string twist_frame_id_;
  std::string teleop_control_topic_;
  std::string teleop_action_name_;
  int teleop_mode_;
  std::string teleop_ee_name_;
  bool teleop_move_orientation_;
  std::string gripper_action_name_;
  std::string home_action_name_;

  std::vector<std::string> home_joint_names_;
  std::vector<double> home_joint_positions_;
  double home_vel_scale_;
  double home_acc_scale_;

  bool has_prev_{false};
  bool teleop_session_enabled_{false};
  bool home_goal_in_progress_{false};
  bool teleop_goal_pending_{false};
  bool teleop_goal_in_progress_{false};
  bool teleop_cancel_requested_{false};
  bool teleop_cancel_in_progress_{false};
  bool pending_home_after_teleop_stop_{false};

  std::vector<int32_t> prev_buttons_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr episode_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr teleop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

  rclcpp_action::Client<GripperMove>::SharedPtr gripper_client_;
  rclcpp_action::Client<TeleopAction>::SharedPtr teleop_client_;
  TeleopGoalHandle::SharedPtr teleop_goal_handle_;
  rclcpp_action::Client<HomeAction>::SharedPtr home_client_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ps4InputManager>());
  rclcpp::shutdown();
  return 0;
}