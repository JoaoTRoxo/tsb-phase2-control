// Copyright 2026 Joao Roxo
/// @file sensor_processing_node.cpp
/// @brief Turns raw odometry and IMU data into the state the controllers use.
///
/// Subscribes: odom (nav_msgs/Odometry), imu (sensor_msgs/Imu)
/// Publishes:  state (tsb_control_msgs/VesselState) at a fixed rate
///
/// Surge comes from the odometry twist through a low-pass filter. Yaw rate comes
/// straight from the gyroscope. Heading fuses both sensors with a complementary
/// filter. If either sensor goes quiet the state is still published, with
/// valid = false, so that consumers can react instead of acting on old data.

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tsb_control/angles.hpp"
#include "tsb_control/filters.hpp"
#include "tsb_control_msgs/msg/vessel_state.hpp"

namespace tsb_control
{

using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;
using tsb_control_msgs::msg::VesselState;

/// ROS wrapper around LowPassFilter and HeadingFilter.
class SensorProcessingNode : public rclcpp::Node
{
public:
  SensorProcessingNode()
  : Node("sensor_processing_node"),
    surge_filter_(declare_parameter("surge_filter_tau", 0.05)),
    heading_filter_(declare_parameter("heading_filter_tau", 1.0))
  {
    const double rate = declare_parameter("rate", 50.0);
    sensor_timeout_ = declare_parameter("sensor_timeout", 0.5);

    odom_sub_ = create_subscription<Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      [this](Odometry::ConstSharedPtr msg) {on_odometry(*msg);});
    imu_sub_ = create_subscription<Imu>(
      "imu", rclcpp::SensorDataQoS(),
      [this](Imu::ConstSharedPtr msg) {on_imu(*msg);});
    state_pub_ = create_publisher<VesselState>("state", rclcpp::QoS(10));

    // rclcpp::create_timer follows the node clock, so the loop also runs correctly
    // on simulated time from Gazebo or a rosbag.
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / rate),
      [this]() {publish_state();});
  }

private:
  void on_odometry(const Odometry & msg)
  {
    const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
    const double dt = last_odom_stamp_ ? (stamp - *last_odom_stamp_).seconds() : 0.0;
    last_odom_stamp_ = stamp;
    last_odom_arrival_ = now();

    state_.header.frame_id = msg.header.frame_id;
    state_.x = msg.pose.pose.position.x;
    state_.y = msg.pose.pose.position.y;
    state_.surge = surge_filter_.update(msg.twist.twist.linear.x, dt);

    const auto & q = msg.pose.pose.orientation;
    heading_filter_.correct(yaw_from_quaternion(q.x, q.y, q.z, q.w), dt);
  }

  void on_imu(const Imu & msg)
  {
    const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
    const double dt = last_imu_stamp_ ? (stamp - *last_imu_stamp_).seconds() : 0.0;
    last_imu_stamp_ = stamp;
    last_imu_arrival_ = now();

    state_.yaw_rate = msg.angular_velocity.z;
    heading_filter_.predict(msg.angular_velocity.z, dt);
  }

  /// True when @p arrival exists and is younger than the sensor timeout.
  bool fresh(const std::optional<rclcpp::Time> & arrival, const rclcpp::Time & t) const
  {
    return arrival && (t - *arrival).seconds() < sensor_timeout_;
  }

  void publish_state()
  {
    const auto t = now();
    state_.header.stamp = t;
    state_.heading = heading_filter_.heading();
    state_.valid = fresh(last_odom_arrival_, t) && fresh(last_imu_arrival_, t);
    if (!state_.valid && (last_odom_arrival_ || last_imu_arrival_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "odometry or IMU is stale, state marked invalid");
    }
    state_pub_->publish(state_);
  }

  LowPassFilter surge_filter_;
  HeadingFilter heading_filter_;
  double sensor_timeout_{0.5};

  VesselState state_;
  std::optional<rclcpp::Time> last_odom_stamp_, last_imu_stamp_;
  std::optional<rclcpp::Time> last_odom_arrival_, last_imu_arrival_;

  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<VesselState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tsb_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tsb_control::SensorProcessingNode>());
  rclcpp::shutdown();
  return 0;
}
