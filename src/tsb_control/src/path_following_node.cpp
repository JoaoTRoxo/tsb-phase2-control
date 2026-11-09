// Copyright 2026 Joao Roxo
/// @file path_following_node.cpp
/// @brief Line-of-sight guidance: turns a list of waypoints into [surge, heading] references.
///
/// Subscribes: state (tsb_control_msgs/VesselState), waypoints (nav_msgs/Path)
/// Publishes:  desired (tsb_control_msgs/Reference),
///             path_following/status (tsb_control_msgs/PathStatus),
///             path_following/path (nav_msgs/Path, latched, for RViz)
///
/// A path can come from the waypoints_x / waypoints_y parameters or at any time
/// from the waypoints topic. By default the current position is added as the start
/// of the path, so the first waypoint does not need to be where the robot is.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tsb_control/angles.hpp"
#include "tsb_control/los.hpp"
#include "tsb_control_msgs/msg/path_status.hpp"
#include "tsb_control_msgs/msg/reference.hpp"
#include "tsb_control_msgs/msg/vessel_state.hpp"

namespace tsb_control
{

using nav_msgs::msg::Path;
using tsb_control_msgs::msg::PathStatus;
using tsb_control_msgs::msg::Reference;
using tsb_control_msgs::msg::VesselState;

/// ROS wrapper around PathTracker, plus the speed law.
class PathFollowingNode : public rclcpp::Node
{
public:
  PathFollowingNode()
  : Node("path_following_node")
  {
    rate_ = declare_parameter("rate", 20.0);
    cruise_speed_ = declare_parameter("cruise_speed", 0.15);
    min_speed_ = declare_parameter("min_speed", 0.03);
    arrival_gain_ = declare_parameter("arrival_gain", 0.5);
    turn_slowdown_ = declare_parameter("turn_slowdown", 2.0);
    lookahead_ = declare_parameter("lookahead", 0.4);
    sigma_ = declare_parameter("sigma", 0.0);
    acceptance_radius_ = declare_parameter("acceptance_radius", 0.08);
    start_from_current_ = declare_parameter("start_from_current", true);

    const auto xs = declare_parameter("waypoints_x", std::vector<double>{});
    const auto ys = declare_parameter("waypoints_y", std::vector<double>{});
    if (xs.size() != ys.size()) {
      throw std::invalid_argument("waypoints_x and waypoints_y must have the same length");
    }
    for (std::size_t i = 0; i < xs.size(); ++i) {
      pending_.push_back(Waypoint{xs[i], ys[i]});
    }

    state_sub_ = create_subscription<VesselState>(
      "state", rclcpp::QoS(10),
      [this](VesselState::ConstSharedPtr msg) {state_ = *msg;});
    // Transient-local, so a path published before this node started is still received.
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    path_sub_ = create_subscription<Path>(
      "waypoints", latched, [this](Path::ConstSharedPtr msg) {on_path(*msg);});

    desired_pub_ = create_publisher<Reference>("desired", rclcpp::QoS(10));
    status_pub_ = create_publisher<PathStatus>("path_following/status", rclcpp::QoS(10));
    path_pub_ = create_publisher<Path>("path_following/path", latched);

    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / rate_), [this]() {tick();});
  }

private:
  void on_path(const Path & msg)
  {
    pending_.clear();
    for (const auto & pose : msg.poses) {
      pending_.push_back(Waypoint{pose.pose.position.x, pose.pose.position.y});
    }
    RCLCPP_INFO(get_logger(), "received a path with %zu waypoints", pending_.size());
    tracker_.reset();
  }

  /// Build the tracker from the pending waypoints once the position is known.
  void start_tracking()
  {
    auto waypoints = pending_;
    pending_.clear();
    if (start_from_current_) {
      waypoints.insert(waypoints.begin(), Waypoint{state_->x, state_->y});
    }
    if (waypoints.size() < 2) {
      RCLCPP_ERROR(get_logger(), "a path needs at least two points, ignoring it");
      return;
    }
    tracker_.emplace(std::move(waypoints), lookahead_, sigma_, acceptance_radius_);
    announced_finish_ = false;
    publish_path();
  }

  void publish_path()
  {
    Path path;
    path.header.stamp = now();
    path.header.frame_id = state_->header.frame_id;
    for (const auto & waypoint : tracker_->waypoints()) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = waypoint.x;
      pose.pose.position.y = waypoint.y;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  /// Slow down for large heading errors and on the way into the last waypoint.
  double speed_for(const PathCommand & command) const
  {
    if (command.finished) {
      return 0.0;
    }
    const double heading_error = wrap_pi(command.heading - state_->heading);
    const double alignment = std::max(0.0, std::cos(heading_error));
    const double turning = cruise_speed_ * std::pow(alignment, turn_slowdown_);
    const double arriving = std::max(min_speed_, arrival_gain_ * command.distance_to_goal);
    return std::min(turning, arriving);
  }

  void tick()
  {
    if (!state_ || !state_->valid) {
      return;
    }
    if (!pending_.empty()) {
      start_tracking();
    }
    if (!tracker_) {
      return;
    }

    const auto command = tracker_->update(state_->x, state_->y, 1.0 / rate_);
    if (command.finished && !announced_finish_) {
      announced_finish_ = true;
      RCLCPP_INFO(get_logger(), "final waypoint reached");
    }

    Reference desired;
    desired.header.stamp = now();
    desired.surge = speed_for(command);
    desired.heading = command.heading;
    desired_pub_->publish(desired);

    PathStatus status;
    status.header.stamp = desired.header.stamp;
    status.active_segment = static_cast<std::uint32_t>(command.segment);
    status.segment_count = static_cast<std::uint32_t>(tracker_->segment_count());
    status.cross_track_error = command.cross_track;
    status.distance_to_waypoint = command.distance_to_waypoint;
    status.desired_heading = desired.heading;
    status.desired_surge = desired.surge;
    status.finished = command.finished;
    status_pub_->publish(status);
  }

  double rate_{20.0};
  double cruise_speed_{0.15};
  double min_speed_{0.03};
  double arrival_gain_{0.5};
  double turn_slowdown_{2.0};
  double lookahead_{0.4};
  double sigma_{0.0};
  double acceptance_radius_{0.08};
  bool start_from_current_{true};
  bool announced_finish_{false};

  std::vector<Waypoint> pending_;
  std::optional<PathTracker> tracker_;
  std::optional<VesselState> state_;

  rclcpp::Subscription<VesselState>::SharedPtr state_sub_;
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  rclcpp::Publisher<Reference>::SharedPtr desired_pub_;
  rclcpp::Publisher<PathStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tsb_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tsb_control::PathFollowingNode>());
  rclcpp::shutdown();
  return 0;
}
