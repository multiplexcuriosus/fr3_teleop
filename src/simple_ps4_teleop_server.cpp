#include <memory>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

class SimplePs4TeleopServer : public rclcpp::Node {
public:
  SimplePs4TeleopServer() : Node("simple_ps4_teleop_server") {
    deadman_button_ = this->declare_parameter<int>("deadman_button", 4);  // e.g. L1
    axis_linear_x_  = this->declare_parameter<int>("axis_linear_x", 1);   // left stick up/down
    axis_linear_y_  = this->declare_parameter<int>("axis_linear_y", 0);   // left stick left/right
    axis_linear_z_  = this->declare_parameter<int>("axis_linear_z", 4);   // right stick up/down
    axis_angular_z_ = this->declare_parameter<int>("axis_angular_z", 3);  // right stick left/right

    linear_scale_   = this->declare_parameter<double>("linear_scale", 0.10);
    angular_scale_  = this->declare_parameter<double>("angular_scale", 0.60);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 100.0);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&SimplePs4TeleopServer::joyCallback, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/ps4/teleop_twist_cmd", 10);

    enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/ps4/teleop_enable", 10);

    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&SimplePs4TeleopServer::publishLoop, this));

    RCLCPP_INFO(this->get_logger(), "Simple PS4 teleop server started.");
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_joy_ = *msg;
    has_joy_ = true;
  }

  double getAxisSafe(const std::vector<float>& axes, int idx) const {
    if (idx < 0 || idx >= static_cast<int>(axes.size())) {
      return 0.0;
    }
    return static_cast<double>(axes[idx]);
  }

  int getButtonSafe(const std::vector<int32_t>& buttons, int idx) const {
    if (idx < 0 || idx >= static_cast<int>(buttons.size())) {
      return 0;
    }
    return buttons[idx];
  }

  void publishLoop() {
    sensor_msgs::msg::Joy joy_copy;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_joy_) {
        return;
      }
      joy_copy = latest_joy_;
    }

    bool enabled = getButtonSafe(joy_copy.buttons, deadman_button_) != 0;

    std_msgs::msg::Bool enable_msg;
    enable_msg.data = enabled;
    enable_pub_->publish(enable_msg);

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base";

    if (enabled) {
      twist_msg.twist.linear.x  = linear_scale_  * getAxisSafe(joy_copy.axes, axis_linear_x_);
      twist_msg.twist.linear.y  = linear_scale_  * getAxisSafe(joy_copy.axes, axis_linear_y_);
      twist_msg.twist.linear.z  = linear_scale_  * getAxisSafe(joy_copy.axes, axis_linear_z_);
      twist_msg.twist.angular.z = angular_scale_ * getAxisSafe(joy_copy.axes, axis_angular_z_);
    } else {
      twist_msg.twist.linear.x = 0.0;
      twist_msg.twist.linear.y = 0.0;
      twist_msg.twist.linear.z = 0.0;
      twist_msg.twist.angular.x = 0.0;
      twist_msg.twist.angular.y = 0.0;
      twist_msg.twist.angular.z = 0.0;
    }

    twist_pub_->publish(twist_msg);
  }

  std::mutex mutex_;
  sensor_msgs::msg::Joy latest_joy_;
  bool has_joy_{false};

  int deadman_button_;
  int axis_linear_x_;
  int axis_linear_y_;
  int axis_linear_z_;
  int axis_angular_z_;

  double linear_scale_;
  double angular_scale_;
  double publish_rate_hz_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePs4TeleopServer>());
  rclcpp::shutdown();
  return 0;
}