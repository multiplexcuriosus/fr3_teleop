#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "franka_msgs/action/move.hpp"
#include "fr3_husky_msgs/action/move_to_joint.hpp"
#include "fr3_husky_msgs/action/omega_haptic.hpp"

class Ps4InputManager : public rclcpp::Node {
public:
  using GripperMove = franka_msgs::action::Move;
  using GripperMoveGoalHandle = rclcpp_action::ClientGoalHandle<GripperMove>;

  using HomeAction = fr3_husky_msgs::action::MoveToJoint;
  using HomeGoalHandle = rclcpp_action::ClientGoalHandle<HomeAction>;

  using HapticAction = fr3_husky_msgs::action::OmegaHaptic;
  using HapticGoalHandle = rclcpp_action::ClientGoalHandle<HapticAction>;

  Ps4InputManager() : Node("ps4_input_manager") {
    // Button mapping
    button_gripper_open_  = this->declare_parameter<int>("button_gripper_open", 2);   // triangle
    button_gripper_close_ = this->declare_parameter<int>("button_gripper_close", 0);  // cross
    button_episode_start_ = this->declare_parameter<int>("button_episode_start", 9);  // options
    button_episode_stop_  = this->declare_parameter<int>("button_episode_stop", 8);   // share
    button_home_          = this->declare_parameter<int>("button_home", 1);           // circle

    // Gripper preset widths
    gripper_open_width_  = this->declare_parameter<double>("gripper_open_width", 0.080);
    gripper_close_width_ = this->declare_parameter<double>("gripper_close_width", 0.06);
    gripper_speed_       = this->declare_parameter<double>("gripper_speed", 0.05);

    // Action names
    gripper_action_name_ = this->declare_parameter<std::string>(
      "gripper_action_name", "/right_franka_gripper/move");
    home_action_name_ = this->declare_parameter<std::string>(
      "home_action_name", "/fr3_move_to_joint");

    button_haptic_start_ = this->declare_parameter<int>("button_haptic_start", 7);  // example only
    haptic_action_name_ = this->declare_parameter<std::string>(
      "haptic_action_name", "/ps4_haptic");  

    // Home configuration
    home_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "home_joint_names",
      {"right_fr3_joint1", "right_fr3_joint2", "right_fr3_joint3", "right_fr3_joint4",
       "right_fr3_joint5", "right_fr3_joint6", "right_fr3_joint7"});

    home_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "home_joint_positions",
      {0.044, -2.72, 0.016, 0.834, -0.012, 0.013, 2.711});

    home_vel_scale_ = this->declare_parameter<double>("home_vel_scale", 0.1);
    home_acc_scale_ = this->declare_parameter<double>("home_acc_scale", 0.1);

    episode_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/episode/control", 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&Ps4InputManager::joyCallback, this, std::placeholders::_1));

    gripper_client_ = rclcpp_action::create_client<GripperMove>(this, gripper_action_name_);
    home_client_ = rclcpp_action::create_client<HomeAction>(this, home_action_name_);
    haptic_client_ = rclcpp_action::create_client<HapticAction>(this, haptic_action_name_);

    RCLCPP_INFO(this->get_logger(), "PS4 input manager started.");
  }

private:

  enum EpisodeCommand : uint8_t {
    STOP = 0,
    START = 1
  };

  int buttonSafe(const std::vector<int32_t>& buttons, int idx) const {
    if (idx < 0 || idx >= static_cast<int>(buttons.size())) {
      return 0;
    }
    return buttons[idx];
  }

  bool risingEdge(const std::vector<int32_t>& current, int idx) const {
    int curr = buttonSafe(current, idx);
    int prev = buttonSafe(prev_buttons_, idx);
    return (curr == 1 && prev == 0);
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (!has_prev_) {
      prev_buttons_ = msg->buttons;
      has_prev_ = true;
      return;
    }

    if (risingEdge(msg->buttons, button_gripper_open_)) {
      sendGripperGoal(gripper_open_width_);
    }

    if (risingEdge(msg->buttons, button_gripper_close_)) {
      sendGripperGoal(gripper_close_width_);
    }

    if (risingEdge(msg->buttons, button_episode_start_)) {
      publishEpisodeCommand(START);
    }

    if (risingEdge(msg->buttons, button_episode_stop_)) {
      publishEpisodeCommand(STOP);
    }

    if (risingEdge(msg->buttons, button_home_)) {
      sendHomeGoal();
    }

    if (risingEdge(msg->buttons, button_haptic_start_)) {
      sendHapticGoal();
    }

    prev_buttons_ = msg->buttons;
  }

  void publishEpisodeCommand(uint8_t value) {
    std_msgs::msg::UInt8 msg;
    msg.data = value;
    episode_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published episode command: %u", value);
  }

  void sendHapticGoal() {
    if (!haptic_client_->wait_for_action_server(std::chrono::milliseconds(200))) {
      RCLCPP_WARN(this->get_logger(), "Haptic action server not available.");
      return;
    }

    HapticAction::Goal goal;
    goal.mode = 0;
    goal.ee_name = "right_fr3_hand_tcp";
    goal.move_orientation = false;
    goal.hapic_pos_multiplier = 2.0;
    goal.hapic_ori_multiplier = 1.0;
    goal.hapic_lin_vel_multiplier = 1.0;
    goal.hapic_ang_vel_multiplier = 1.0;

    rclcpp_action::Client<HapticAction>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const HapticGoalHandle::SharedPtr& handle) {
        if (!handle) {
          RCLCPP_WARN(this->get_logger(), "Haptic goal rejected.");
        } else {
          RCLCPP_INFO(this->get_logger(), "Haptic goal accepted.");
        }
      };

    options.result_callback =
      [this](const HapticGoalHandle::WrappedResult& result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Haptic goal succeeded.");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Haptic goal aborted.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Haptic goal canceled.");
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "Unknown haptic result code.");
            break;
        }
      };

    haptic_client_->async_send_goal(goal, options);
  }

  void sendGripperGoal(double width) {
    if (!gripper_client_->wait_for_action_server(std::chrono::milliseconds(200))) {
      RCLCPP_WARN(this->get_logger(), "Gripper action server not available.");
      return;
    }

    GripperMove::Goal goal;
    goal.width = width;
    goal.speed = gripper_speed_;

    rclcpp_action::Client<GripperMove>::SendGoalOptions options;
    options.goal_response_callback =
      [this, width](const GripperMoveGoalHandle::SharedPtr& handle) {
        if (!handle) {
          RCLCPP_WARN(this->get_logger(), "Gripper goal rejected.");
        } else {
          RCLCPP_INFO(this->get_logger(), "Gripper goal accepted: width=%.3f", width);
        }
      };

    options.result_callback =
      [this](const GripperMoveGoalHandle::WrappedResult& result) {
        switch (result.code) {
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

  void sendHomeGoal() {
    if (!home_client_->wait_for_action_server(std::chrono::milliseconds(200))) {
      RCLCPP_WARN(this->get_logger(), "Home action server not available.");
      return;
    }

    if (home_joint_names_.size() != home_joint_positions_.size()) {
      RCLCPP_ERROR(this->get_logger(), "Home joint name / position size mismatch.");
      return;
    }

    HomeAction::Goal goal;
    goal.joint_names = home_joint_names_;
    goal.target_positions = home_joint_positions_;
    goal.max_velocity_scaling_factor = home_vel_scale_;
    goal.max_acceleration_scaling_factor = home_acc_scale_;

    rclcpp_action::Client<HomeAction>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const HomeGoalHandle::SharedPtr& handle) {
        if (!handle) {
          RCLCPP_WARN(this->get_logger(), "Home goal rejected.");
        } else {
          RCLCPP_INFO(this->get_logger(), "Home goal accepted.");
        }
      };

    options.result_callback =
      [this](const HomeGoalHandle::WrappedResult& result) {
        switch (result.code) {
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

    home_client_->async_send_goal(goal, options);
  }

  int button_gripper_open_;
  int button_gripper_close_;
  int button_episode_start_;
  int button_episode_stop_;
  int button_home_;
  int button_haptic_start_;

  double gripper_open_width_;
  double gripper_close_width_;
  double gripper_speed_;

  std::string gripper_action_name_;
  std::string home_action_name_;

  std::vector<std::string> home_joint_names_;
  std::vector<double> home_joint_positions_;
  double home_vel_scale_;
  double home_acc_scale_;

  bool has_prev_{false};
  std::vector<int32_t> prev_buttons_;

  std::string haptic_action_name_;
  rclcpp_action::Client<HapticAction>::SharedPtr haptic_client_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr episode_pub_;

  rclcpp_action::Client<GripperMove>::SharedPtr gripper_client_;
  rclcpp_action::Client<HomeAction>::SharedPtr home_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ps4InputManager>());
  rclcpp::shutdown();
  return 0;
}