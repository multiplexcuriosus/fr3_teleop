#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "fr3_husky_msgs/action/omega_haptic.hpp"
#include "fr3_husky_msgs/action/move_to_joint.hpp"

class Ps4InputManager : public rclcpp::Node
{
public:
  using TeleopAction = fr3_husky_msgs::action::OmegaHaptic;
  using TeleopGoalHandle = rclcpp_action::ClientGoalHandle<TeleopAction>;

  using HomeAction = fr3_husky_msgs::action::MoveToJoint;
  using HomeGoalHandle = rclcpp_action::ClientGoalHandle<HomeAction>;

  Ps4InputManager() : Node("ps4_input_manager")
  {
    // Button mapping
    button_gripper_open_ = this->declare_parameter<int>("button_gripper_open", 2);    // triangle
    button_gripper_close_ = this->declare_parameter<int>("button_gripper_close", 0);  // cross
    button_episode_cancel_ = this->declare_parameter<int>("button_episode_cancel", 6);  // TODO: set to L2 mapping
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
    gripper_state_command_topic_ = this->declare_parameter<std::string>(
      "gripper_state_command_topic", "/teleop/gripper_state_cmd");
    teleop_control_topic_ = this->declare_parameter<std::string>("teleop_control_topic", "/teleop/control");
    teleop_action_name_ = this->declare_parameter<std::string>("teleop_action_name", "/cartesian_executor");
    teleop_mode_ = this->declare_parameter<int>("teleop_mode", 0);
    teleop_ee_name_ = this->declare_parameter<std::string>("teleop_ee_name", "right_fr3_hand_tcp");
    teleop_move_orientation_ = this->declare_parameter<bool>("teleop_move_orientation", false);

    // Action names
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
    delayed_home_delay_ms_ = this->declare_parameter<int>("delayed_home_delay_ms", 750);
    home_settle_delay_ms_ = this->declare_parameter<int>("home_settle_delay_ms", delayed_home_delay_ms_);
    joint_state_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    max_joint_state_age_ms_ = this->declare_parameter<int>("max_joint_state_age_ms", 150);
    home_requires_fresh_joint_state_ =
      this->declare_parameter<bool>("home_requires_fresh_joint_state", true);

    episode_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/episode/control", 10);
    num_valid_episodes_pub_ = this->create_publisher<std_msgs::msg::UInt32>(
      "/data_collection/num_valid_episodes", 10);
    teleop_pub_ = this->create_publisher<std_msgs::msg::UInt8>(teleop_control_topic_, 10);
    gripper_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(gripper_state_command_topic_, 10);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_name_, 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&Ps4InputManager::joyCallback, this, std::placeholders::_1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 10,
      std::bind(&Ps4InputManager::jointStateCallback, this, std::placeholders::_1));

    teleop_client_ = rclcpp_action::create_client<TeleopAction>(this, teleop_action_name_);
    home_client_ = rclcpp_action::create_client<HomeAction>(this, home_action_name_);
    home_block_until_ = this->now();

    RCLCPP_INFO(this->get_logger(), "PS4 input manager started.");
    RCLCPP_INFO(this->get_logger(), "Episode cancel button index: %d", button_episode_cancel_);
    RCLCPP_INFO(this->get_logger(), "Valid episode count topic: /data_collection/num_valid_episodes");
    RCLCPP_INFO(this->get_logger(), "Teleop control topic: %s", teleop_control_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Gripper state command topic: %s", gripper_state_command_topic_.c_str());
    publishNumValidEpisodes();
    publishGripperStateCommand(current_gripper_closed_state_);
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

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_receive_time_ = this->now();
    last_joint_state_stamp_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
    have_joint_state_ = true;
  }

  bool isJointStateFresh() const
  {
    if (!have_joint_state_)
    {
      RCLCPP_WARN(this->get_logger(), "Home rejected: joint state stale, no joint state received yet");
      return false;
    }

    const rclcpp::Time now = this->now();
    rclcpp::Time reference_time = last_joint_state_stamp_;

    if (reference_time.nanoseconds() <= 0 || reference_time > now)
    {
      reference_time = last_joint_state_receive_time_;
    }

    if (reference_time.nanoseconds() <= 0 || reference_time > now)
    {
      RCLCPP_WARN(this->get_logger(), "Home rejected: joint state stale, invalid timestamp. Reference time: %ld, now: %ld", reference_time.nanoseconds(), now.nanoseconds());
      return false;
    }

    const auto age = now - reference_time;
    const auto age_ms = age.nanoseconds() / 1000000;
    if (age_ms > static_cast<int64_t>(max_joint_state_age_ms_))
    {
      RCLCPP_WARN(this->get_logger(), "Home rejected: joint state stale. Reference time=%ld, now=%ld, age=%ld ms",reference_time.nanoseconds(), now.nanoseconds(), age_ms);
      return false;
    }

    return true;
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
        publishGripperStateCommand(false);
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
        publishGripperStateCommand(true);
      }
    }

    bool cancel_edge = false;
    if (risingEdge(msg->buttons, button_episode_cancel_))
    {
      cancel_edge = true;
      if (episode_recording_)
      {
        publishEpisodeCommand(3);
        episode_recording_ = false;
        RCLCPP_INFO(this->get_logger(), "[EPISODE] cancel_current requested");
      }
      else
      {
        publishEpisodeCommand(4);
        if (num_valid_episodes_ > 0)
        {
          --num_valid_episodes_;
        }
        publishNumValidEpisodes();
        episode_recording_ = false;
        RCLCPP_INFO(this->get_logger(), "[EPISODE] cancel_last requested");
      }
    }

    // If cancel was requested this cycle, do not emit start/stop in the same callback.
    if (!cancel_edge)
    {
      handleEpisodeAxis(msg);
    }

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
        episode_recording_ = true;
        RCLCPP_INFO(this->get_logger(), "Episode START");
      }
      else if (state == -1)
      {
        if (episode_recording_)
        {
          publishEpisodeCommand(2);
          episode_recording_ = false;
          ++num_valid_episodes_;
          publishNumValidEpisodes();
          RCLCPP_INFO(this->get_logger(), "Episode STOP");
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "Ignoring episode stop: no active episode is recording.");
        }
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

    if (pending_home_after_teleop_stop_)
    {
      // Keep the edge detector aligned with the current axis position while teleop enable is blocked.
      prev_teleop_axis_state_ = state;
      return;
    }

    if (state != prev_teleop_axis_state_)
    {
      if (state == 1)
      {
        enableTeleopSession();
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
    if (home_active_ || home_goal_in_progress_)
    {
      RCLCPP_WARN(this->get_logger(), "Home goal already in progress.");
      return;
    }

    if (pending_home_after_teleop_stop_)
    {
      RCLCPP_WARN(this->get_logger(), "Home goal is already queued until teleop stops.");
      return;
    }

    if (
      teleop_session_enabled_ || teleop_active_ || teleop_goal_pending_ || teleop_goal_in_progress_ ||
      teleop_goal_handle_ || teleop_cancel_in_progress_)
    {
      pending_home_after_teleop_stop_ = true;
      publishZeroTwist();
      disableTeleopSession();
      RCLCPP_WARN(this->get_logger(), "Home delayed until teleop cancel result and settle delay completed");
      return;
    }

    if (home_requires_fresh_joint_state_ && !isJointStateFresh())
    {
      pending_home_after_teleop_stop_ = false;
      return;
    }

    stopDelayedHomeTimer();
    sendHomeGoalNow();
  }

  void enableTeleopSession()
  {
    if (home_active_ || this->now() < home_block_until_)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring teleop start while home trajectory is active");
      RCLCPP_WARN(this->get_logger(), "Teleop start rejected because home is active");
      return;
    }

    if (teleop_goal_pending_ || teleop_goal_in_progress_)
    {
      RCLCPP_INFO(this->get_logger(), "Teleop action already active or pending.");
      return;
    }

    bool available = false;
    for (int i = 0; i < 5; ++i)
    {
      if (teleop_client_->wait_for_action_server(std::chrono::milliseconds(500)))
      {
        available = true;
        break;
      }
      RCLCPP_WARN(this->get_logger(), "Waiting for teleop action server... (%d/5)", i + 1);
    }

    if (!available)
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
        teleop_active_ = false;
        teleop_session_enabled_ = false;
        publishTeleopCommand(2);
        RCLCPP_WARN(this->get_logger(), "Teleop action goal rejected.");

        if (pending_home_after_teleop_stop_)
        {
          RCLCPP_INFO(this->get_logger(), "Teleop goal was not accepted; treating teleop as stopped.");
          RCLCPP_INFO(this->get_logger(), "Teleop cancel completed");
          startDelayedHomeTimer();
        }
        return;
      }

      teleop_goal_handle_ = handle;
      teleop_goal_in_progress_ = true;
      teleop_active_ = true;

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
      teleop_active_ = false;
      teleop_cancel_requested_ = false;
      teleop_cancel_in_progress_ = false;
      teleop_session_enabled_ = false;

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
        RCLCPP_INFO(this->get_logger(), "Teleop cancel completed");
        startDelayedHomeTimer();
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
      RCLCPP_INFO(this->get_logger(), "Teleop cancel sent");
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
      RCLCPP_INFO(this->get_logger(), "Teleop already stopped; scheduling delayed home goal.");
      startDelayedHomeTimer();
    }
  }

  void stopDelayedHomeTimer()
  {
    if (delayed_home_timer_)
    {
      delayed_home_timer_->cancel();
      delayed_home_timer_.reset();
    }
  }

  void startDelayedHomeTimer()
  {
    stopDelayedHomeTimer();

    const auto delay = std::chrono::milliseconds(home_settle_delay_ms_);
    RCLCPP_INFO(this->get_logger(), "Waiting before home (%d ms)", home_settle_delay_ms_);
    delayed_home_timer_ = this->create_wall_timer(
      delay,
      [this]()
      {
        if (delayed_home_timer_)
        {
          delayed_home_timer_->cancel();
          delayed_home_timer_.reset();
        }

        RCLCPP_INFO(this->get_logger(), "Delayed home timer fired.");

        if (!pending_home_after_teleop_stop_)
        {
          RCLCPP_INFO(this->get_logger(), "Ignoring delayed home trigger: no pending home request.");
          return;
        }

        if (
          teleop_session_enabled_ || teleop_active_ || teleop_goal_pending_ || teleop_goal_in_progress_ ||
          teleop_goal_handle_ || teleop_cancel_in_progress_)
        {
          pending_home_after_teleop_stop_ = false;
          RCLCPP_WARN(this->get_logger(), "Home rejected: teleop still active/canceling");
          return;
        }

        if (home_requires_fresh_joint_state_ && !isJointStateFresh())
        {
          pending_home_after_teleop_stop_ = false;
          return;
        }

        RCLCPP_INFO(this->get_logger(), "Teleop fully stopped and settled; attempting home goal");
        sendHomeGoalNow();
      });
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

  void publishNumValidEpisodes()
  {
    std_msgs::msg::UInt32 msg;
    msg.data = num_valid_episodes_;
    num_valid_episodes_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published valid episode count: %u", num_valid_episodes_);
  }

  void publishTeleopCommand(uint8_t value)
  {
    std_msgs::msg::UInt8 msg;
    msg.data = value;
    teleop_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published teleop command: %u", value);
  }

  void publishGripperStateCommand(bool closed)
  {
    std_msgs::msg::Bool msg;
    msg.data = closed;
    gripper_state_pub_->publish(msg);
    current_gripper_closed_state_ = closed;
    RCLCPP_INFO(this->get_logger(), "Published gripper state command: %s", closed ? "CLOSED" : "OPEN");
  }

  void sendHomeGoalNow()
  {
    stopDelayedHomeTimer();

    if (
      teleop_session_enabled_ || teleop_active_ || teleop_goal_pending_ || teleop_goal_in_progress_ ||
      teleop_goal_handle_ || teleop_cancel_in_progress_)
    {
      pending_home_after_teleop_stop_ = false;
      home_active_ = false;
      home_goal_in_progress_ = false;
      RCLCPP_WARN(this->get_logger(), "Home rejected: teleop still active/canceling");
      return;
    }

    if (home_requires_fresh_joint_state_ && !isJointStateFresh())
    {
      pending_home_after_teleop_stop_ = false;
      home_active_ = false;
      home_goal_in_progress_ = false;
      return;
    }

    if (home_joint_names_.size() != home_joint_positions_.size())
    {
      pending_home_after_teleop_stop_ = false;
      home_active_ = false;
      home_goal_in_progress_ = false;
      RCLCPP_ERROR(this->get_logger(), "Home joint name / position size mismatch.");
      return;
    }

    if (!home_client_->wait_for_action_server(std::chrono::milliseconds(200)))
    {
      pending_home_after_teleop_stop_ = false;
      home_active_ = false;
      home_goal_in_progress_ = false;
      RCLCPP_WARN(this->get_logger(), "Home action server not available.");
      return;
    }

    home_active_ = true;
    home_goal_in_progress_ = true;
    home_block_until_ = this->now() + rclcpp::Duration::from_seconds(3.0);
    pending_home_after_teleop_stop_ = false;
    prev_teleop_axis_state_ = 0;

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
        home_active_ = false;
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
      home_active_ = false;
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

    publishZeroTwist();
    RCLCPP_INFO(this->get_logger(), "Home safety checks passed; sending MoveToJoint goal");
    try
    {
      home_client_->async_send_goal(goal, options);
    }
    catch (const std::exception & e)
    {
      home_active_ = false;
      home_goal_in_progress_ = false;
      pending_home_after_teleop_stop_ = false;
      RCLCPP_ERROR(this->get_logger(), "Failed to send home goal: %s", e.what());
    }
  }

  int button_gripper_open_;
  int button_gripper_close_;
  int button_episode_cancel_;
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

  std::string gripper_state_command_topic_;
  std::string twist_topic_name_;
  std::string twist_frame_id_;
  std::string teleop_control_topic_;
  std::string teleop_action_name_;
  int teleop_mode_;
  std::string teleop_ee_name_;
  bool teleop_move_orientation_;
  std::string home_action_name_;
  std::string joint_state_topic_;

  std::vector<std::string> home_joint_names_;
  std::vector<double> home_joint_positions_;
  double home_vel_scale_;
  double home_acc_scale_;
  int delayed_home_delay_ms_;
  int home_settle_delay_ms_;
  int max_joint_state_age_ms_;
  bool home_requires_fresh_joint_state_;

  bool has_prev_{false};
  bool have_joint_state_{false};
  bool teleop_session_enabled_{false};
  bool teleop_active_ = false;
  bool home_active_ = false;
  bool home_goal_in_progress_{false};
  bool teleop_goal_pending_{false};
  bool teleop_goal_in_progress_{false};
  bool teleop_cancel_requested_{false};
  bool teleop_cancel_in_progress_{false};
  bool pending_home_after_teleop_stop_{false};
  bool current_gripper_closed_state_{false};
  bool episode_recording_{false};
  uint32_t num_valid_episodes_{0};

  std::vector<int32_t> prev_buttons_;
  rclcpp::Time last_joint_state_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_joint_state_receive_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr episode_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr num_valid_episodes_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr teleop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

  rclcpp_action::Client<TeleopAction>::SharedPtr teleop_client_;
  TeleopGoalHandle::SharedPtr teleop_goal_handle_;
  rclcpp_action::Client<HomeAction>::SharedPtr home_client_;
  rclcpp::Time home_block_until_;
  rclcpp::TimerBase::SharedPtr delayed_home_timer_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ps4InputManager>());
  rclcpp::shutdown();
  return 0;
}