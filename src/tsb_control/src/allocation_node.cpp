// Copyright 2026 Joao Roxo
/// @file allocation_node.cpp
/// @brief Maps the surge and yaw commands onto the two wheels.
///
/// Subscribes: tau_u, tau_r (std_msgs/Float64)
/// Publishes:  current_left, current_right (std_msgs/Float64),
///             cmd_vel (geometry_msgs/Twist),
///             allocation/feedback (tsb_control_msgs/AllocationFeedback)
///
/// current_left and current_right keep the names from Figure 1. The TurtleBot has
/// velocity-controlled wheels rather than current-controlled motors, so they carry
/// wheel-speed setpoints in rad/s. cmd_vel is rebuilt from the saturated wheel
/// speeds, so the simulator only ever receives commands the wheels can deliver.

#include <chrono>
#include <memory>
#include <optional>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tsb_control/allocation.hpp"
#include "tsb_control_msgs/msg/allocation_feedback.hpp"

namespace tsb_control
{

using geometry_msgs::msg::Twist;
using std_msgs::msg::Float64;
using tsb_control_msgs::msg::AllocationFeedback;

/// ROS wrapper around allocate().
class AllocationNode : public rclcpp::Node
{
public:
  AllocationNode()
  : Node("allocation_node")
  {
    geometry_.wheel_radius = declare_parameter("wheel_radius", 0.033);
    geometry_.wheel_separation = declare_parameter("wheel_separation", 0.160);
    geometry_.max_wheel_speed = declare_parameter("max_wheel_speed", 6.67);
    command_timeout_ = declare_parameter("command_timeout", 0.5);
    const double rate = declare_parameter("rate", 50.0);

    tau_u_sub_ = create_subscription<Float64>(
      "tau_u", rclcpp::QoS(10),
      [this](Float64::ConstSharedPtr msg) {
        tau_u_ = msg->data;
        last_command_ = now();
      });
    tau_r_sub_ = create_subscription<Float64>(
      "tau_r", rclcpp::QoS(10),
      [this](Float64::ConstSharedPtr msg) {
        tau_r_ = msg->data;
        last_command_ = now();
      });

    left_pub_ = create_publisher<Float64>("current_left", rclcpp::QoS(10));
    right_pub_ = create_publisher<Float64>("current_right", rclcpp::QoS(10));
    cmd_vel_pub_ = create_publisher<Twist>("cmd_vel", rclcpp::QoS(10));
    feedback_pub_ = create_publisher<AllocationFeedback>("allocation/feedback", rclcpp::QoS(10));

    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / rate), [this]() {tick();});
  }

private:
  void tick()
  {
    const auto t = now();
    // Watchdog: if the controller dies, the robot must stop rather than keep
    // running on its last command.
    const bool fresh = last_command_ && (t - *last_command_).seconds() < command_timeout_;
    const auto result = fresh ? allocate(tau_u_, tau_r_, geometry_) : allocate(0.0, 0.0, geometry_);

    Float64 left;
    left.data = result.left;
    left_pub_->publish(left);
    Float64 right;
    right.data = result.right;
    right_pub_->publish(right);

    Twist cmd_vel;
    cmd_vel.linear.x = result.surge;
    cmd_vel.angular.z = result.yaw;
    cmd_vel_pub_->publish(cmd_vel);

    AllocationFeedback feedback;
    feedback.header.stamp = t;
    feedback.surge_ratio = result.surge_ratio;
    feedback.yaw_ratio = result.yaw_ratio;
    feedback.saturated = result.saturated;
    feedback_pub_->publish(feedback);
  }

  DiffDriveGeometry geometry_;
  double command_timeout_{0.5};
  double tau_u_{0.0};
  double tau_r_{0.0};
  std::optional<rclcpp::Time> last_command_;

  rclcpp::Subscription<Float64>::SharedPtr tau_u_sub_, tau_r_sub_;
  rclcpp::Publisher<Float64>::SharedPtr left_pub_, right_pub_;
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<AllocationFeedback>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tsb_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tsb_control::AllocationNode>());
  rclcpp::shutdown();
  return 0;
}
