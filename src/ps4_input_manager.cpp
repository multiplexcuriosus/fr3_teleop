#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <array>
#include <algorithm>
#include <functional>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "fr3_husky_msgs/action/omega_haptic.hpp"
#include "fr3_husky_msgs/action/move_to_joint.hpp"
#include "fr3_husky_msgs/action/line_trajectory.hpp"
#include "fr3_husky_msgs/srv/capture_line_center.hpp"
#include "std_srvs/srv/trigger.hpp"

class Ps4InputManager : public rclcpp::Node
{
public:
  using TeleopAction = fr3_husky_msgs::action::OmegaHaptic;
  using TeleopGoalHandle = rclcpp_action::ClientGoalHandle<TeleopAction>;

  using HomeAction = fr3_husky_msgs::action::MoveToJoint;
  using HomeGoalHandle = rclcpp_action::ClientGoalHandle<HomeAction>;

  using TrajectoryAction = fr3_husky_msgs::action::LineTrajectory;
  using TrajectoryGoalHandle = rclcpp_action::ClientGoalHandle<TrajectoryAction>;

  using CaptureCenterSrv = fr3_husky_msgs::srv::CaptureLineCenter;
  using TriggerSrv = std_srvs::srv::Trigger;

  Ps4InputManager() : Node("ps4_input_manager")
  {
    // Button mapping
    button_gripper_open_ = this->declare_parameter<int>("button_gripper_open", 2);    // triangle
    button_gripper_close_ = this->declare_parameter<int>("button_gripper_close", 0);  // cross
    button_episode_cancel_ = this->declare_parameter<int>("button_episode_cancel", 6);
    axis_episode_ = this->declare_parameter<int>("axis_episode", 6);
    button_home_ = this->declare_parameter<int>("button_home", 1);                    // circle
    axis_teleop_ = this->declare_parameter<int>("axis_teleop", 7);
    button_trajectory_phase_test_ = this->declare_parameter<int>("button_trajectory_phase_test", 3);  // square
    button_capture_line_center_ = this->declare_parameter<int>("button_capture_line_center", 5);      // R1

    interception_arm_button_ = this->declare_parameter<int>("interception_arm_button", 4);            // L1

    // Teleop mapping moved from old action server
    left_stick_x_idx_ = this->declare_parameter<int>("left_stick_x_idx", 0);
    left_stick_y_idx_ = this->declare_parameter<int>("left_stick_y_idx", 1);
    right_stick_y_idx_ = this->declare_parameter<int>("right_stick_y_idx", 4);

    deadzone_ = this->declare_parameter<double>("deadzone", 0.08);
    max_vx_ = this->declare_parameter<double>("max_vx", 0.08);
    max_vy_ = this->declare_parameter<double>("max_vy", 0.08);
    max_vz_ = this->declare_parameter<double>("max_vz", 0.05);

    spacemouse_deadzone_ = this->declare_parameter<double>("spacemouse_deadzone", 0.08);
    spacemouse_timeout_ms_ = this->declare_parameter<int>("spacemouse_timeout_ms", 200);
    ps4_override_hold_ms_ = this->declare_parameter<int>("ps4_override_hold_ms", 300);
    spacemouse_max_vx_ = this->declare_parameter<double>("spacemouse_max_vx", max_vx_);
    spacemouse_max_vy_ = this->declare_parameter<double>("spacemouse_max_vy", max_vy_);
    spacemouse_max_vz_ = this->declare_parameter<double>("spacemouse_max_vz", max_vz_);
    spacemouse_enable_rotation_ = this->declare_parameter<bool>("spacemouse_enable_rotation", false);
    spacemouse_max_wx_ = this->declare_parameter<double>("spacemouse_max_wx", 0.3);
    spacemouse_max_wy_ = this->declare_parameter<double>("spacemouse_max_wy", 0.3);
    spacemouse_max_wz_ = this->declare_parameter<double>("spacemouse_max_wz", 0.3);
    spacemouse_sign_x_ = this->declare_parameter<double>("spacemouse_sign_x", 1.0);
    spacemouse_sign_y_ = this->declare_parameter<double>("spacemouse_sign_y", 1.0);
    spacemouse_sign_z_ = this->declare_parameter<double>("spacemouse_sign_z", 1.0);
    spacemouse_sign_roll_ = this->declare_parameter<double>("spacemouse_sign_roll", 1.0);
    spacemouse_sign_pitch_ = this->declare_parameter<double>("spacemouse_sign_pitch", 1.0);
    spacemouse_sign_yaw_ = this->declare_parameter<double>("spacemouse_sign_yaw", 1.0);

    haptic_pos_multiplier_ = this->declare_parameter<double>("haptic_pos_multiplier", 2.0);
    haptic_lin_vel_multiplier_ = this->declare_parameter<double>("haptic_lin_vel_multiplier", 1.0);
    haptic_ori_multiplier_ = this->declare_parameter<double>("haptic_ori_multiplier", 1.0);
    haptic_ang_vel_multiplier_ = this->declare_parameter<double>("haptic_ang_vel_multiplier", 1.0);

    twist_topic_name_ = this->declare_parameter<std::string>("twist_topic_name", "/cartesian_cmd/twist");
    twist_frame_id_ = this->declare_parameter<std::string>("twist_frame_id", "base_link");
    gripper_state_command_topic_ = this->declare_parameter<std::string>(
      "gripper_state_command_topic", "/teleop/gripper_state_cmd");
    gripper_inhibit_topic_ = this->declare_parameter<std::string>(
      "gripper_inhibit_topic", "/teleop/gripper_inhibit");
    teleop_control_topic_ = this->declare_parameter<std::string>("teleop_control_topic", "/teleop/control");
    teleop_action_name_ = this->declare_parameter<std::string>("teleop_action_name", "/cartesian_executor");
    teleop_mode_ = 0;
    teleop_ee_name_ = this->declare_parameter<std::string>("teleop_ee_name", "right_fr3_hand_tcp");
    teleop_move_orientation_ = this->declare_parameter<bool>("teleop_move_orientation", false);
    spacemouse_topic_name_ = this->declare_parameter<std::string>("spacemouse_topic_name", "/spacemouse_cmd");

    trajectory_action_name_ = this->declare_parameter<std::string>("trajectory_action_name", "/trajectory_executor");
    trajectory_capture_center_service_ = this->declare_parameter<std::string>(
      "trajectory_capture_center_service", "/trajectory_executor/capture_line_center");
    reset_episode_counter_service_ = this->declare_parameter<std::string>(
      "reset_episode_counter_service", "/data_collection/reset_episode_counter");
    interception_arm_service_ = this->declare_parameter<std::string>(
      "interception_arm_service", "/interception_controller/arm");
    interception_disarm_service_ = this->declare_parameter<std::string>(
      "interception_disarm_service", "/interception_controller/disarm");
    initial_motion_interface_ = this->declare_parameter<std::string>("initial_motion_interface", "trajectory");
    trajectory_armed_by_default_ = this->declare_parameter<bool>("trajectory_armed_by_default", true);
    trajectory_profile_name_ = this->declare_parameter<std::string>("trajectory_profile_name", "phase_test_slow");
    trajectory_v_max_ = this->declare_parameter<double>("trajectory_v_max", 0.10);
    trajectory_a_max_ = this->declare_parameter<double>("trajectory_a_max", 0.30);
    trajectory_j_max_ = this->declare_parameter<double>("trajectory_j_max", 10.0);
    trajectory_hold_before_sec_ = this->declare_parameter<double>("trajectory_hold_before_sec", 0.5);
    trajectory_hold_after_sec_ = this->declare_parameter<double>("trajectory_hold_after_sec", 0.5);
    trajectory_repetitions_ = this->declare_parameter<int>("trajectory_repetitions", 1);
    trajectory_settle_delay_ms_ = this->declare_parameter<int>("trajectory_settle_delay_ms", 750);

    // SpaceMouse button handling
    enable_spacemouse_buttons_ = this->declare_parameter<bool>("enable_spacemouse_buttons", true);
    spacemouse_button_topic_name_ = this->declare_parameter<std::string>(
        "spacemouse_button_topic_name", "/spacemouse_buttons");
    spacemouse_button_gripper_open_ = this->declare_parameter<int>("spacemouse_button_gripper_open", 0);
    spacemouse_button_gripper_close_ = this->declare_parameter<int>("spacemouse_button_gripper_close", 1);

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

    home_joint_positions_ = reorderHomeJointPositionsAssumedInputOrder(home_joint_positions_);

    home_vel_scale_ = this->declare_parameter<double>("home_vel_scale", 0.1);
    home_acc_scale_ = this->declare_parameter<double>("home_acc_scale", 0.1);
    delayed_home_delay_ms_ = this->declare_parameter<int>("delayed_home_delay_ms", 750);
    home_settle_delay_ms_ = this->declare_parameter<int>("home_settle_delay_ms", delayed_home_delay_ms_);
    joint_state_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/right_fr3/joint_states");
    max_joint_state_age_ms_ = this->declare_parameter<int>("max_joint_state_age_ms", 250);
    home_requires_fresh_joint_state_ =
      this->declare_parameter<bool>("home_requires_fresh_joint_state", true);

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1));

    episode_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/episode/control", 10);
    num_valid_episodes_pub_ = this->create_publisher<std_msgs::msg::UInt32>(
      "/data_collection/num_valid_episodes", 10);
    teleop_pub_ = this->create_publisher<std_msgs::msg::UInt8>(teleop_control_topic_, command_qos);
    gripper_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(gripper_state_command_topic_, command_qos);
    gripper_inhibit_pub_ = this->create_publisher<std_msgs::msg::Bool>(gripper_inhibit_topic_, command_qos);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_name_, command_qos);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&Ps4InputManager::joyCallback, this, std::placeholders::_1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 10,
      std::bind(&Ps4InputManager::jointStateCallback, this, std::placeholders::_1));
    spacemouse_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      spacemouse_topic_name_, 10,
      std::bind(&Ps4InputManager::spacemouseCallback, this, std::placeholders::_1));
    
    // SpaceMouse button subscription (if enabled)
    if (enable_spacemouse_buttons_)
    {
      spacemouse_buttons_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
          spacemouse_button_topic_name_, command_qos,
          std::bind(&Ps4InputManager::spacemouseButtonsCallback, this, std::placeholders::_1));
    }

    teleop_client_ = rclcpp_action::create_client<TeleopAction>(this, teleop_action_name_);
    home_client_ = rclcpp_action::create_client<HomeAction>(this, home_action_name_);
    trajectory_client_ = rclcpp_action::create_client<TrajectoryAction>(this, trajectory_action_name_);
    capture_center_client_ = this->create_client<CaptureCenterSrv>(trajectory_capture_center_service_);
    interception_arm_client_ = this->create_client<TriggerSrv>(interception_arm_service_);
    interception_disarm_client_ = this->create_client<TriggerSrv>(interception_disarm_service_);
    reset_episode_counter_service_server_ = this->create_service<TriggerSrv>(
      reset_episode_counter_service_,
      std::bind(
        &Ps4InputManager::handleResetEpisodeCounter,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    if (initial_motion_interface_ == "trajectory")
    {
      trajectory_armed_ = true;
    }
    else if (initial_motion_interface_ == "none")
    {
      trajectory_armed_ = trajectory_armed_by_default_;
    }
    else if (initial_motion_interface_ == "teleop")
    {
      trajectory_armed_ = trajectory_armed_by_default_;
    }
    else
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Unknown initial_motion_interface '%s'; using trajectory_armed_by_default=%s",
        initial_motion_interface_.c_str(), trajectory_armed_by_default_ ? "true" : "false");
      trajectory_armed_ = trajectory_armed_by_default_;
    }

    home_block_until_ = this->now();

    RCLCPP_INFO(this->get_logger(), "PS4 input manager started.");
    RCLCPP_INFO(this->get_logger(), "Episode cancel button index: %d", button_episode_cancel_);
    RCLCPP_INFO(this->get_logger(), "Valid episode count topic: /data_collection/num_valid_episodes");
    RCLCPP_INFO(this->get_logger(), "Teleop control topic: %s", teleop_control_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Gripper state command topic: %s", gripper_state_command_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Gripper inhibit topic: %s", gripper_inhibit_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "SpaceMouse topic: %s", spacemouse_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Trajectory action: %s", trajectory_action_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Trajectory capture service: %s", trajectory_capture_center_service_.c_str());
    RCLCPP_INFO(this->get_logger(), "Reset episode counter service: %s", reset_episode_counter_service_.c_str());
    RCLCPP_INFO(this->get_logger(), "Interception arm service: %s", interception_arm_service_.c_str());
    RCLCPP_INFO(this->get_logger(), "Interception disarm service: %s", interception_disarm_service_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "PS4 mapping: L1: Arm interception (button index %d)",
      interception_arm_button_);
    RCLCPP_INFO(this->get_logger(), "Trajectory armed on startup: %s", trajectory_armed_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "SpaceMouse timeout (ms): %d", spacemouse_timeout_ms_);
    RCLCPP_INFO(this->get_logger(), "SpaceMouse rotation enabled: %s", spacemouse_enable_rotation_ ? "true" : "false");
    if (enable_spacemouse_buttons_)
    {
      RCLCPP_INFO(this->get_logger(), "SpaceMouse buttons enabled on topic: %s", spacemouse_button_topic_name_.c_str());
      RCLCPP_INFO(this->get_logger(), "  Button %d = gripper TOGGLE(open/close), Button %d = unused",
                  spacemouse_button_gripper_open_, spacemouse_button_gripper_close_);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "SpaceMouse buttons are disabled");
    }
    publishNumValidEpisodes();
    publishGripperInhibit(true);
  }

private:
  std::vector<double> reorderHomeJointPositionsAssumedInputOrder(
    const std::vector<double> & input_positions) const
  {
    // Expected input order: joint1, joint2, joint3, joint6, joint5, joint7, joint4
    // Output order:         joint1, joint2, joint3, joint4, joint5, joint6, joint7
    if (input_positions.size() != 7)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "home_joint_positions size is %zu (expected 7). Skipping remap.",
        input_positions.size());
      return input_positions;
    }

    static constexpr std::array<size_t, 7> kInputToAscendingIndex = {0, 1, 2, 6, 4, 3, 5};
    std::vector<double> ordered_positions(7, 0.0);
    for (size_t out_idx = 0; out_idx < kInputToAscendingIndex.size(); ++out_idx)
    {
      ordered_positions[out_idx] = input_positions[kInputToAscendingIndex[out_idx]];
    }
    return ordered_positions;
  }

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

  bool isButtonPressed(const std::vector<int32_t> & buttons, int idx) const
  {
    return buttonSafe(buttons, idx) == 1;
  }

  void requestInterceptionArm()
  {
    if (interception_arm_request_pending_)
    {
      RCLCPP_INFO(this->get_logger(), "Ignoring interception ARM press: previous ARM request is still pending.");
      return;
    }

    if (!interception_arm_client_->service_is_ready())
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Interception ARM request not sent: service unavailable (%s).",
        interception_arm_service_.c_str());
      return;
    }

    interception_arm_request_pending_ = true;
    auto req = std::make_shared<TriggerSrv::Request>();
    interception_arm_client_->async_send_request(
      req,
      [this](rclcpp::Client<TriggerSrv>::SharedFuture future)
      {
        interception_arm_request_pending_ = false;
        try
        {
          const auto response = future.get();
          if (response->success)
          {
            RCLCPP_INFO(this->get_logger(), "Interception ARM succeeded: %s", response->message.c_str());
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "Interception ARM failed: %s", response->message.c_str());
          }
        }
        catch (const std::exception & e)
        {
          RCLCPP_ERROR(this->get_logger(), "Interception ARM call failed: %s", e.what());
        }
      });

    RCLCPP_INFO(this->get_logger(), "L1 requested interception ARM.");
  }

  void requestInterceptionDisarm()
  {
    if (interception_disarm_request_pending_)
    {
      RCLCPP_INFO(this->get_logger(), "Ignoring interception DISARM press: previous DISARM request is still pending.");
      return;
    }

    if (!interception_disarm_client_->service_is_ready())
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Interception DISARM request not sent: service unavailable (%s).",
        interception_disarm_service_.c_str());
      return;
    }

    interception_disarm_request_pending_ = true;
    auto req = std::make_shared<TriggerSrv::Request>();
    interception_disarm_client_->async_send_request(
      req,
      [this](rclcpp::Client<TriggerSrv>::SharedFuture future)
      {
        interception_disarm_request_pending_ = false;
        try
        {
          const auto response = future.get();
          if (response->success)
          {
            RCLCPP_INFO(this->get_logger(), "Interception DISARM succeeded: %s", response->message.c_str());
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "Interception DISARM failed: %s", response->message.c_str());
          }
        }
        catch (const std::exception & e)
        {
          RCLCPP_ERROR(this->get_logger(), "Interception DISARM call failed: %s", e.what());
        }
      });

    RCLCPP_INFO(this->get_logger(), "Interception DISARM request sent.");
  }

  bool gripperBlockedByTransition() const
  {
    return home_active_ || home_goal_in_progress_ || pending_home_after_teleop_stop_ ||
      pending_home_after_trajectory_stop_ || teleop_cancel_in_progress_ || teleop_cancel_requested_ ||
      trajectory_active_ || trajectory_goal_pending_ || trajectory_goal_in_progress_ ||
      trajectory_cancel_in_progress_ || trajectory_cancel_requested_ || pending_trajectory_after_teleop_stop_ ||
      this->now() < home_block_until_;
  }

  bool isTrajectoryBlocked() const
  {
    return trajectory_active_ || trajectory_goal_pending_ || trajectory_goal_in_progress_ ||
      trajectory_cancel_in_progress_ || trajectory_cancel_requested_ || trajectory_goal_handle_ ||
      pending_trajectory_after_teleop_stop_;
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

  void spacemouseCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_spacemouse_msg_ = *msg;
    last_spacemouse_time_ = this->now();
    have_spacemouse_msg_ = true;
  }

  bool spacemouseButtonRisingEdge(uint8_t current_mask, int button_idx) const
  {
    const uint8_t current_bit = (current_mask >> button_idx) & 0x01;
    const uint8_t prev_bit = (prev_spacemouse_button_mask_ >> button_idx) & 0x01;
    return (current_bit == 1 && prev_bit == 0);
  }

  void spacemouseButtonsCallback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    const uint8_t current_mask = msg->data;

    // SpaceMouse gripper-open button now toggles open/close on each rising edge.
    if (spacemouseButtonRisingEdge(current_mask, spacemouse_button_gripper_open_))
    {
      if (!teleop_session_enabled_)
      {
        RCLCPP_WARN(this->get_logger(), "Ignoring SpaceMouse gripper toggle: teleop session is not enabled.");
      }
      else if (gripperBlockedByTransition())
      {
        RCLCPP_WARN(this->get_logger(), "Ignoring SpaceMouse gripper toggle: teleop/home transition is active.");
      }
      else
      {
        const bool next_closed_state = !current_gripper_closed_state_;
        publishGripperStateCommand(next_closed_state);
      }
    }

    prev_spacemouse_button_mask_ = current_mask;
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

  bool isSpaceMouseFresh() const
  {
    if (!have_spacemouse_msg_)
    {
      return false;
    }

    const rclcpp::Time now = this->now();
    if (last_spacemouse_time_.nanoseconds() <= 0 || last_spacemouse_time_ > now)
    {
      return false;
    }

    const auto age_ms = (now - last_spacemouse_time_).nanoseconds() / 1000000;
    return age_ms <= static_cast<int64_t>(spacemouse_timeout_ms_);
  }

  double applySpaceMouseDeadzone(double value) const
  {
    if (std::abs(value) < spacemouse_deadzone_)
    {
      return 0.0;
    }
    return value;
  }

  double scaleSpaceMouseAxis(double value, double sign, double max_value) const
  {
    const double input = std::clamp(applySpaceMouseDeadzone(value), -1.0, 1.0);
    return input * sign * max_value;
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!has_prev_)
    {
      prev_buttons_ = msg->buttons;
      has_prev_ = true;
    }

    if (risingEdge(msg->buttons, interception_arm_button_))
    {
      requestInterceptionArm();
    }

    const bool home_edge = risingEdge(msg->buttons, button_home_);
    if (home_edge)
    {
      publishZeroTwist();
      publishGripperInhibit(true);
      handleHomePressed();
      prev_buttons_ = msg->buttons;
      return;
    }

    if (risingEdge(msg->buttons, button_trajectory_phase_test_))
    {
      sendTrajectoryPhaseTestGoal();
      prev_buttons_ = msg->buttons;
      return;
    }

    if (risingEdge(msg->buttons, button_capture_line_center_))
    {
      handleCaptureLineCenterPressed();
    }

    if (risingEdge(msg->buttons, button_episode_cancel_) && isTrajectoryBlocked())
    {
      publishZeroTwist();
      publishGripperInhibit(true);
      trajectory_cancel_requested_ = true;
      requestCancelTrajectoryGoal();
      prev_buttons_ = msg->buttons;
      RCLCPP_INFO(this->get_logger(), "Episode cancel deferred: trajectory stop requested first.");
      return;
    }

    if (risingEdge(msg->buttons, button_gripper_open_))
    {
      if (!teleop_session_enabled_)
      {
        RCLCPP_WARN(this->get_logger(), "Ignoring gripper open: teleop session is not enabled.");
      }
      else if (gripperBlockedByTransition())
      {
        RCLCPP_WARN(this->get_logger(), "Ignoring gripper open: teleop/home transition is active.");
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
      else if (gripperBlockedByTransition())
      {
        RCLCPP_WARN(this->get_logger(), "Ignoring gripper close: teleop/home transition is active.");
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

    if (pending_home_after_teleop_stop_ || pending_home_after_trajectory_stop_)
    {
      // Keep the edge detector aligned with the current axis position while teleop enable is blocked.
      prev_teleop_axis_state_ = state;
      return;
    }

    if (state != prev_teleop_axis_state_)
    {
      if (state == 1)
      {
        if (!enableTeleopSession())
        {
          prev_teleop_axis_state_ = state;
          return;
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

    if (trajectory_active_ || trajectory_goal_pending_ || trajectory_goal_in_progress_ ||
      trajectory_cancel_in_progress_ || trajectory_goal_handle_)
    {
      pending_home_after_trajectory_stop_ = true;
      trajectory_cancel_requested_ = true;
      publishZeroTwist();
      publishGripperInhibit(true);
      requestCancelTrajectoryGoal();
      RCLCPP_INFO(this->get_logger(), "Home delayed until trajectory stop is complete");
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

  bool enableTeleopSession()
  {
    if (home_active_ || this->now() < home_block_until_)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring teleop start while home trajectory is active");
      RCLCPP_WARN(this->get_logger(), "Teleop start rejected because home is active");
      publishZeroTwist();
      return false;
    }

    if (isTrajectoryBlocked())
    {
      RCLCPP_WARN(this->get_logger(), "Teleop start rejected because trajectory is active or pending.");
      publishZeroTwist();
      return false;
    }

    if (teleop_goal_pending_ || teleop_goal_in_progress_)
    {
      RCLCPP_INFO(this->get_logger(), "Teleop action already active or pending.");
      return true;
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
      publishZeroTwist();
      return false;
    }

    RCLCPP_WARN(this->get_logger(), "Teleop mode: %d", teleop_mode_);


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

        if (pending_trajectory_after_teleop_stop_)
        {
          startTrajectorySettleTimer();
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
      publishGripperInhibit(false);
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
      publishGripperInhibit(true);
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

      if (pending_trajectory_after_teleop_stop_)
      {
        startTrajectorySettleTimer();
      }
    };

    teleop_goal_pending_ = true;
    teleop_cancel_requested_ = false;
    teleop_client_->async_send_goal(goal, options);

    RCLCPP_INFO(this->get_logger(), "Sending teleop action goal...");
    return true;
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

    publishGripperInhibit(true);
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
      scheduleHomeAfterMotionStop();
    }
  }

  void handleCaptureLineCenterPressed()
  {
    if (home_active_ || home_goal_in_progress_ || pending_home_after_teleop_stop_ ||
      pending_home_after_trajectory_stop_ || this->now() < home_block_until_)
    {
      RCLCPP_WARN(this->get_logger(), "Capture line center rejected: home/recovery is active or pending.");
      return;
    }

    if (isTrajectoryBlocked())
    {
      RCLCPP_WARN(this->get_logger(), "Capture line center rejected: trajectory is active, pending, or canceling.");
      return;
    }

    if (!capture_center_client_->wait_for_service(std::chrono::milliseconds(300)))
    {
      RCLCPP_WARN(this->get_logger(), "Capture line center service is not available.");
      return;
    }

    auto req = std::make_shared<CaptureCenterSrv::Request>();
    req->ee_name = teleop_ee_name_;
    req->keep_current_orientation = true;

    capture_center_client_->async_send_request(
      req,
      [this](const rclcpp::Client<CaptureCenterSrv>::SharedFuture future)
      {
        const auto response = future.get();
        if (response->success)
        {
          RCLCPP_INFO(this->get_logger(), "Capture line center succeeded: %s", response->message.c_str());
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "Capture line center failed: %s", response->message.c_str());
        }
      });

    RCLCPP_INFO(this->get_logger(), "Requested capture of the current line center.");
  }

  void stopTrajectorySettleTimer()
  {
    if (trajectory_settle_timer_)
    {
      trajectory_settle_timer_->cancel();
      trajectory_settle_timer_.reset();
    }
  }

  void startTrajectorySettleTimer()
  {
    stopTrajectorySettleTimer();

    const auto delay = std::chrono::milliseconds(trajectory_settle_delay_ms_);
    RCLCPP_INFO(this->get_logger(), "Waiting before trajectory goal (%d ms)", trajectory_settle_delay_ms_);
    trajectory_settle_timer_ = this->create_wall_timer(
      delay,
      [this]()
      {
        stopTrajectorySettleTimer();

        if (!pending_trajectory_after_teleop_stop_)
        {
          RCLCPP_INFO(this->get_logger(), "Ignoring trajectory settle trigger: no pending trajectory request.");
          return;
        }

        if (teleop_session_enabled_ || teleop_active_ || teleop_goal_pending_ || teleop_goal_in_progress_ ||
          teleop_goal_handle_ || teleop_cancel_in_progress_)
        {
          RCLCPP_WARN(this->get_logger(), "Teleop still active when trajectory settle timer fired; retrying.");
          startTrajectorySettleTimer();
          return;
        }

        pending_trajectory_after_teleop_stop_ = false;
        sendTrajectoryPhaseTestGoal();
      });
  }

  void requestCancelTrajectoryGoal()
  {
    if (trajectory_cancel_in_progress_)
    {
      return;
    }

    if (trajectory_goal_handle_)
    {
      trajectory_cancel_in_progress_ = true;
      RCLCPP_INFO(this->get_logger(), "Trajectory cancel sent");
      trajectory_client_->async_cancel_goal(
        trajectory_goal_handle_,
        [this](auto future)
        {
          trajectory_cancel_in_progress_ = false;
          const auto cancel_response = future.get();
          if (cancel_response->goals_canceling.empty())
          {
            RCLCPP_WARN(this->get_logger(), "Trajectory action cancel request was not accepted.");
          }
          else
          {
            RCLCPP_INFO(this->get_logger(), "Trajectory action cancel requested.");
          }
        });
      return;
    }

    if (trajectory_goal_pending_)
    {
      trajectory_cancel_requested_ = true;
      RCLCPP_INFO(this->get_logger(), "Trajectory goal is pending; cancel will be requested after acceptance.");
      return;
    }

    trajectory_cancel_requested_ = false;
    if (pending_home_after_trajectory_stop_)
    {
      RCLCPP_INFO(this->get_logger(), "Trajectory already stopped; scheduling delayed home goal.");
      scheduleHomeAfterMotionStop();
    }
  }

  void sendTrajectoryPhaseTestGoal()
  {
    if (!trajectory_armed_)
    {
      RCLCPP_WARN(this->get_logger(), "Trajectory phase test rejected: trajectory is not armed.");
      return;
    }

    if (home_active_ || home_goal_in_progress_)
    {
      RCLCPP_WARN(this->get_logger(), "Trajectory phase test rejected: home is active.");
      return;
    }

    if (trajectory_active_ || trajectory_goal_pending_ || trajectory_goal_in_progress_ ||
      trajectory_cancel_in_progress_ || trajectory_goal_handle_)
    {
      RCLCPP_WARN(this->get_logger(), "Trajectory phase test rejected: trajectory is already active or pending.");
      return;
    }

    if (teleop_session_enabled_ || teleop_active_ || teleop_goal_pending_ || teleop_goal_in_progress_ ||
      teleop_goal_handle_ || teleop_cancel_in_progress_)
    {
      pending_trajectory_after_teleop_stop_ = true;
      publishZeroTwist();
      publishGripperInhibit(true);
      disableTeleopSession();
      RCLCPP_INFO(this->get_logger(), "Trajectory queued until teleop fully stops");
      return;
    }

    if (!trajectory_client_->wait_for_action_server(std::chrono::milliseconds(300)))
    {
      RCLCPP_WARN(this->get_logger(), "Trajectory action server not available.");
      if (pending_home_after_trajectory_stop_)
      {
        scheduleHomeAfterMotionStop();
      }
      return;
    }

    TrajectoryAction::Goal goal;
    goal.command = TrajectoryAction::Goal::CMD_PHASE_TEST;
    goal.ee_name = teleop_ee_name_;
    goal.profile_name = trajectory_profile_name_;
    goal.v_max = trajectory_v_max_;
    goal.a_max = trajectory_a_max_;
    goal.j_max = trajectory_j_max_;
    goal.hold_before_sec = trajectory_hold_before_sec_;
    goal.hold_after_sec = trajectory_hold_after_sec_;
    goal.repetitions = static_cast<uint32_t>(std::max(1, trajectory_repetitions_));

    rclcpp_action::Client<TrajectoryAction>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const TrajectoryGoalHandle::SharedPtr & handle)
      {
        trajectory_goal_pending_ = false;

        if (!handle)
        {
          trajectory_goal_handle_.reset();
          trajectory_goal_in_progress_ = false;
          trajectory_active_ = false;
          trajectory_cancel_requested_ = false;
          trajectory_cancel_in_progress_ = false;
          publishZeroTwist();
          RCLCPP_WARN(this->get_logger(), "Trajectory action goal rejected.");

          if (pending_home_after_trajectory_stop_)
          {
            scheduleHomeAfterMotionStop();
          }
          return;
        }

        trajectory_goal_handle_ = handle;
        trajectory_goal_in_progress_ = true;
        trajectory_active_ = true;
        publishZeroTwist();
        publishGripperInhibit(true);
        RCLCPP_INFO(this->get_logger(), "Trajectory phase test goal accepted.");

        if (trajectory_cancel_requested_)
        {
          requestCancelTrajectoryGoal();
        }
      };

    options.result_callback =
      [this](const TrajectoryGoalHandle::WrappedResult & result)
      {
        trajectory_goal_handle_.reset();
        trajectory_goal_pending_ = false;
        trajectory_goal_in_progress_ = false;
        trajectory_active_ = false;
        trajectory_cancel_requested_ = false;
        trajectory_cancel_in_progress_ = false;

        publishZeroTwist();
        publishGripperInhibit(true);

        if (result.result)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "Trajectory result message: '%s', final_state=%u",
            result.result->message.c_str(), result.result->final_state);
        }

        switch (result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Trajectory action succeeded.");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Trajectory action aborted.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Trajectory action canceled.");
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "Unknown trajectory action result code.");
            break;
        }

        if (pending_home_after_trajectory_stop_ || pending_home_after_teleop_stop_)
        {
          scheduleHomeAfterMotionStop();
        }
      };

    trajectory_goal_pending_ = true;
    trajectory_cancel_requested_ = false;
    try
    {
      trajectory_client_->async_send_goal(goal, options);
      publishZeroTwist();
      publishGripperInhibit(true);
      RCLCPP_INFO(this->get_logger(), "Sending trajectory phase test goal...");
    }
    catch (const std::exception & e)
    {
      trajectory_goal_pending_ = false;
      trajectory_goal_in_progress_ = false;
      trajectory_active_ = false;
      trajectory_cancel_requested_ = false;
      trajectory_cancel_in_progress_ = false;
      RCLCPP_ERROR(this->get_logger(), "Failed to send trajectory goal: %s", e.what());
      if (pending_home_after_trajectory_stop_)
      {
        scheduleHomeAfterMotionStop();
      }
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
    scheduleHomeAfterMotionStop();
  }

  void scheduleHomeAfterMotionStop()
  {
    if (!pending_home_after_teleop_stop_ && !pending_home_after_trajectory_stop_)
    {
      stopDelayedHomeTimer();
      return;
    }

    stopDelayedHomeTimer();

    const auto delay = std::chrono::milliseconds(home_settle_delay_ms_);
    RCLCPP_INFO(this->get_logger(), "Waiting before home (%d ms)", home_settle_delay_ms_);
    delayed_home_timer_ = this->create_wall_timer(
      delay,
      [this]()
      {
        stopDelayedHomeTimer();

        RCLCPP_INFO(this->get_logger(), "Delayed home timer fired.");

        if (!pending_home_after_teleop_stop_ && !pending_home_after_trajectory_stop_)
        {
          RCLCPP_INFO(this->get_logger(), "Ignoring delayed home trigger: no pending home request.");
          return;
        }

        if (
          teleop_session_enabled_ || teleop_active_ || teleop_goal_pending_ || teleop_goal_in_progress_ ||
          teleop_goal_handle_ || teleop_cancel_in_progress_ || trajectory_active_ || trajectory_goal_pending_ ||
          trajectory_goal_in_progress_ || trajectory_goal_handle_ || trajectory_cancel_in_progress_)
        {
          RCLCPP_WARN(this->get_logger(), "Motion still active when delayed home timer fired; retrying.");
          scheduleHomeAfterMotionStop();
          return;
        }

        if (home_requires_fresh_joint_state_ && !isJointStateFresh())
        {
          pending_home_after_teleop_stop_ = false;
          pending_home_after_trajectory_stop_ = false;
          return;
        }

        RCLCPP_INFO(this->get_logger(), "Motion fully stopped and settled; attempting home goal");
        sendHomeGoalNow();
      });
  }

  void handleTeleopTwist(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!teleop_session_enabled_)
    {
      return;
    }

    if (isTrajectoryBlocked())
    {
      publishZeroTwist();
      return;
    }

    // Moved from old teleop/action-server side:
    // map joystick axes -> Cartesian linear velocity
    const double lx = applyDeadzone(axisSafe(msg->axes, left_stick_x_idx_));
    const double ly = applyDeadzone(axisSafe(msg->axes, left_stick_y_idx_));
    const double ry = applyDeadzone(axisSafe(msg->axes, right_stick_y_idx_));

    const bool ps4_active = (lx != 0.0) || (ly != 0.0) || (ry != 0.0);
    const rclcpp::Time now = this->now();

    if (ps4_active)
    {
      last_ps4_motion_time_ = now;
      publishPs4Twist(lx, ly, ry);
      return;
    }

    if (last_ps4_motion_time_.nanoseconds() > 0)
    {
      const auto hold_ms = (now - last_ps4_motion_time_).nanoseconds() / 1000000;
      if (hold_ms >= 0 && hold_ms <= static_cast<int64_t>(ps4_override_hold_ms_))
      {
        publishZeroTwist();
        return;
      }
    }

    if (isSpaceMouseFresh())
    {
      publishSpaceMouseTwist(last_spacemouse_msg_);
      return;
    }

    publishZeroTwist();
  }

  void publishPs4Twist(double lx, double ly, double ry)
  {
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = twist_frame_id_;

    twist_msg.twist.linear.x = ly * max_vx_ * haptic_pos_multiplier_ * haptic_lin_vel_multiplier_;
    twist_msg.twist.linear.y = lx * max_vy_ * haptic_pos_multiplier_ * haptic_lin_vel_multiplier_;
    twist_msg.twist.linear.z = ry * max_vz_ * haptic_pos_multiplier_ * haptic_lin_vel_multiplier_;

    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = 0.0;

    twist_pub_->publish(twist_msg);
  }

  void publishSpaceMouseTwist(const geometry_msgs::msg::Twist & msg)
  {
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = twist_frame_id_;

    twist_msg.twist.linear.x =
      scaleSpaceMouseAxis(msg.linear.x, spacemouse_sign_x_, spacemouse_max_vx_) *
      haptic_pos_multiplier_ * haptic_lin_vel_multiplier_;
    twist_msg.twist.linear.y =
      scaleSpaceMouseAxis(msg.linear.y, spacemouse_sign_y_, spacemouse_max_vy_) *
      haptic_pos_multiplier_ * haptic_lin_vel_multiplier_;
    twist_msg.twist.linear.z =
      scaleSpaceMouseAxis(msg.linear.z, spacemouse_sign_z_, spacemouse_max_vz_) *
      haptic_pos_multiplier_ * haptic_lin_vel_multiplier_;

    if (spacemouse_enable_rotation_)
    {
      twist_msg.twist.angular.x =
        scaleSpaceMouseAxis(msg.angular.x, spacemouse_sign_roll_, spacemouse_max_wx_) *
        haptic_ori_multiplier_ * haptic_ang_vel_multiplier_;
      twist_msg.twist.angular.y =
        scaleSpaceMouseAxis(msg.angular.y, spacemouse_sign_pitch_, spacemouse_max_wy_) *
        haptic_ori_multiplier_ * haptic_ang_vel_multiplier_;
      twist_msg.twist.angular.z =
        scaleSpaceMouseAxis(msg.angular.z, spacemouse_sign_yaw_, spacemouse_max_wz_) *
        haptic_ori_multiplier_ * haptic_ang_vel_multiplier_;
    }
    else
    {
      twist_msg.twist.angular.x = 0.0;
      twist_msg.twist.angular.y = 0.0;
      twist_msg.twist.angular.z = 0.0;
    }

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

  void handleResetEpisodeCounter(
    const std::shared_ptr<TriggerSrv::Request> request,
    std::shared_ptr<TriggerSrv::Response> response)
  {
    (void)request;

    // This reset only affects completed successful episodes and does not alter an active episode.
    num_valid_episodes_ = 0;
    publishNumValidEpisodes();

    response->success = true;
    response->message = "Successful episode counter reset to 0.";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
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

  void publishGripperInhibit(bool inhibited)
  {
    if (has_published_gripper_inhibit_ && current_gripper_inhibited_ == inhibited)
    {
      return;
    }

    std_msgs::msg::Bool msg;
    msg.data = inhibited;
    gripper_inhibit_pub_->publish(msg);
    current_gripper_inhibited_ = inhibited;
    has_published_gripper_inhibit_ = true;
    RCLCPP_INFO(this->get_logger(), "Published gripper inhibit: %s", inhibited ? "true" : "false");
  }

  void sendHomeGoalNow()
  {
    stopDelayedHomeTimer();

    if (
      teleop_session_enabled_ || teleop_active_ || teleop_goal_pending_ || teleop_goal_in_progress_ ||
      teleop_goal_handle_ || teleop_cancel_in_progress_ || trajectory_active_ || trajectory_goal_pending_ ||
      trajectory_goal_in_progress_ || trajectory_goal_handle_ || trajectory_cancel_in_progress_)
    {
      pending_home_after_teleop_stop_ = false;
      pending_home_after_trajectory_stop_ = false;
      home_active_ = false;
      home_goal_in_progress_ = false;
      RCLCPP_WARN(this->get_logger(), "Home rejected: motion still active/canceling");
      return;
    }

    if (home_requires_fresh_joint_state_ && !isJointStateFresh())
    {
      pending_home_after_teleop_stop_ = false;
      pending_home_after_trajectory_stop_ = false;
      home_active_ = false;
      home_goal_in_progress_ = false;
      return;
    }

    if (home_joint_names_.size() != home_joint_positions_.size())
    {
      pending_home_after_teleop_stop_ = false;
      pending_home_after_trajectory_stop_ = false;
      home_active_ = false;
      home_goal_in_progress_ = false;
      RCLCPP_ERROR(this->get_logger(), "Home joint name / position size mismatch.");
      return;
    }

    if (!home_client_->wait_for_action_server(std::chrono::milliseconds(200)))
    {
      pending_home_after_teleop_stop_ = false;
      pending_home_after_trajectory_stop_ = false;
      home_active_ = false;
      home_goal_in_progress_ = false;
      RCLCPP_WARN(this->get_logger(), "Home action server not available.");
      return;
    }

    home_active_ = true;
    home_goal_in_progress_ = true;
    home_block_until_ = this->now() + rclcpp::Duration::from_seconds(3.0);
    pending_home_after_teleop_stop_ = false;
    pending_home_after_trajectory_stop_ = false;
    prev_teleop_axis_state_ = 0;

    HomeAction::Goal goal;
    goal.joint_names = home_joint_names_;
    goal.target_positions = home_joint_positions_;
    goal.max_velocity_scaling_factor = home_vel_scale_;
    goal.max_acceleration_scaling_factor = home_acc_scale_;
    
    RCLCPP_WARN(this->get_logger(), "Home joint names : %zu", goal.joint_names.size());
    RCLCPP_WARN(this->get_logger(), "Home target positions size: %zu", goal.target_positions.size());
    RCLCPP_WARN(this->get_logger(), "Home max velocity scaling factor: %f", goal.max_velocity_scaling_factor);
    RCLCPP_WARN(this->get_logger(), "Home max acceleration scaling factor: %f", goal.max_acceleration_scaling_factor);

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
      pending_home_after_trajectory_stop_ = false;
      RCLCPP_ERROR(this->get_logger(), "Failed to send home goal: %s", e.what());
    }
  }

  int button_gripper_open_;
  int button_gripper_close_;
  int button_episode_cancel_;
  int axis_episode_;
  int axis_teleop_;
  int button_home_;
  int button_trajectory_phase_test_;
  int button_capture_line_center_;
  int interception_arm_button_;

  int left_stick_x_idx_;
  int left_stick_y_idx_;
  int right_stick_y_idx_;

  int prev_episode_state_{0};
  int prev_teleop_axis_state_{0};

  double deadzone_;
  double max_vx_;
  double max_vy_;
  double max_vz_;
  double spacemouse_deadzone_;
  int spacemouse_timeout_ms_;
  int ps4_override_hold_ms_;
  double spacemouse_max_vx_;
  double spacemouse_max_vy_;
  double spacemouse_max_vz_;
  bool spacemouse_enable_rotation_;
  double spacemouse_max_wx_;
  double spacemouse_max_wy_;
  double spacemouse_max_wz_;
  double spacemouse_sign_x_;
  double spacemouse_sign_y_;
  double spacemouse_sign_z_;
  double spacemouse_sign_roll_;
  double spacemouse_sign_pitch_;
  double spacemouse_sign_yaw_;
  double haptic_pos_multiplier_;
  double haptic_ori_multiplier_;
  double haptic_lin_vel_multiplier_;
  double haptic_ang_vel_multiplier_;

  std::string gripper_state_command_topic_;
  std::string gripper_inhibit_topic_;
  std::string twist_topic_name_;
  std::string twist_frame_id_;
  std::string teleop_control_topic_;
  std::string teleop_action_name_;
  std::string spacemouse_topic_name_;
  int teleop_mode_;
  std::string teleop_ee_name_;
  bool teleop_move_orientation_;
  std::string home_action_name_;
  std::string joint_state_topic_;
  std::string trajectory_action_name_;
  std::string trajectory_capture_center_service_;
  std::string reset_episode_counter_service_;
  std::string interception_arm_service_;
  std::string interception_disarm_service_;
  std::string initial_motion_interface_;
  bool trajectory_armed_by_default_{true};
  std::string trajectory_profile_name_;
  double trajectory_v_max_{0.0};
  double trajectory_a_max_{0.0};
  double trajectory_j_max_{0.0};
  double trajectory_hold_before_sec_{0.0};
  double trajectory_hold_after_sec_{0.0};
  int trajectory_repetitions_{1};
  int trajectory_settle_delay_ms_{750};

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
  bool trajectory_armed_{false};
  bool trajectory_goal_pending_{false};
  bool trajectory_goal_in_progress_{false};
  bool trajectory_active_{false};
  bool trajectory_cancel_requested_{false};
  bool trajectory_cancel_in_progress_{false};
  bool pending_trajectory_after_teleop_stop_{false};
  bool pending_home_after_trajectory_stop_{false};
  bool current_gripper_closed_state_{false};
  bool current_gripper_inhibited_{true};
  bool has_published_gripper_inhibit_{false};
  bool episode_recording_{false};
  bool interception_arm_request_pending_{false};
  bool interception_disarm_request_pending_{false};
  uint32_t num_valid_episodes_{0};

  std::vector<int32_t> prev_buttons_;
  geometry_msgs::msg::Twist last_spacemouse_msg_;
  rclcpp::Time last_joint_state_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_joint_state_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_spacemouse_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ps4_motion_time_{0, 0, RCL_ROS_TIME};
  bool have_spacemouse_msg_{false};

  // SpaceMouse button handling
  bool enable_spacemouse_buttons_{true};
  std::string spacemouse_button_topic_name_;
  int spacemouse_button_gripper_open_{0};
  int spacemouse_button_gripper_close_{1};
  uint8_t prev_spacemouse_button_mask_{0};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr spacemouse_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr spacemouse_buttons_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr episode_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr num_valid_episodes_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr teleop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_inhibit_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

  rclcpp_action::Client<TeleopAction>::SharedPtr teleop_client_;
  TeleopGoalHandle::SharedPtr teleop_goal_handle_;
  rclcpp_action::Client<HomeAction>::SharedPtr home_client_;
  rclcpp_action::Client<TrajectoryAction>::SharedPtr trajectory_client_;
  TrajectoryGoalHandle::SharedPtr trajectory_goal_handle_;
  rclcpp::Client<CaptureCenterSrv>::SharedPtr capture_center_client_;
  rclcpp::Client<TriggerSrv>::SharedPtr interception_arm_client_;
  rclcpp::Client<TriggerSrv>::SharedPtr interception_disarm_client_;
  rclcpp::Service<TriggerSrv>::SharedPtr reset_episode_counter_service_server_;
  rclcpp::Time home_block_until_;
  rclcpp::TimerBase::SharedPtr delayed_home_timer_;
  rclcpp::TimerBase::SharedPtr trajectory_settle_timer_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ps4InputManager>());
  rclcpp::shutdown();
  return 0;
}