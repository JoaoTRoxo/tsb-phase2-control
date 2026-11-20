// Copyright 2026 Joao Roxo
/// @file pid_node.cpp
/// @brief Surge and heading PID loops.
///
/// Subscribes: state (tsb_control_msgs/VesselState), desired (tsb_control_msgs/Reference),
///             allocation/feedback (tsb_control_msgs/AllocationFeedback, optional)
/// Publishes:  tau_u, tau_r (std_msgs/Float64), pid/debug (tsb_control_msgs/PidDebug)
///
/// On the TurtleBot the plant accepts velocity commands, so tau_u is a surge
/// command in m/s and tau_r a yaw-rate command in rad/s. The names follow Figure 1
/// of the statement, where on the boat they would be a force and a moment. On the
/// boat the gains, limits and feedforward law would change: thrust must also cover drag.
///
/// Advanced features: surge feedforward, heading gain scheduling against speed,
/// selectable anti-windup with saturation feedback from the allocation stage, and
/// live retuning through `ros2 param set`.

#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tsb_control/angles.hpp"
#include "tsb_control/gain_schedule.hpp"
#include "tsb_control/pid.hpp"
#include "tsb_control_msgs/msg/allocation_feedback.hpp"
#include "tsb_control_msgs/msg/pid_debug.hpp"
#include "tsb_control_msgs/msg/reference.hpp"
#include "tsb_control_msgs/msg/vessel_state.hpp"

namespace tsb_control
{

using std_msgs::msg::Float64;
using tsb_control_msgs::msg::AllocationFeedback;
using tsb_control_msgs::msg::PidDebug;
using tsb_control_msgs::msg::PidTerms;
using tsb_control_msgs::msg::Reference;
using tsb_control_msgs::msg::VesselState;

namespace
{

AntiWindup parse_anti_windup(const std::string & name)
{
  if (name == "none") {return AntiWindup::kNone;}
  if (name == "clamp") {return AntiWindup::kClamp;}
  if (name == "back_calculation") {return AntiWindup::kBackCalculation;}
  throw std::invalid_argument("anti_windup must be none, clamp or back_calculation");
}

bool ends_with(const std::string & text, const std::string & suffix)
{
  return text.size() >= suffix.size() &&
         text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

/// Copy one loop's update into its debug message.
void fill_terms(
  PidTerms & terms, double reference, double measurement, double error,
  const PidGains & gains, const PidResult & result)
{
  terms.reference = reference;
  terms.measurement = measurement;
  terms.error = error;
  terms.kp = gains.kp;
  terms.ki = gains.ki;
  terms.kd = gains.kd;
  terms.p = result.p;
  terms.i = result.i;
  terms.d = result.d;
  terms.feedforward = result.feedforward;
  terms.unsaturated = result.unsaturated;
  terms.output = result.output;
  terms.saturated = result.saturated;
}

}  // namespace

/// ROS wrapper around the two Pid instances.
class PidNode : public rclcpp::Node
{
public:
  PidNode()
  : Node("pid_node")
  {
    declare_parameters();
    apply_parameters();

    // Changes are validated here and applied on the next control tick, so the
    // gains never change in the middle of an update.
    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return validate_parameters(parameters);
      });

    state_sub_ = create_subscription<VesselState>(
      "state", rclcpp::QoS(10),
      [this](VesselState::ConstSharedPtr msg) {
        state_ = *msg;
        state_arrival_ = now();
      });
    reference_sub_ = create_subscription<Reference>(
      "desired", rclcpp::QoS(10),
      [this](Reference::ConstSharedPtr msg) {
        reference_ = *msg;
        reference_arrival_ = now();
      });
    feedback_sub_ = create_subscription<AllocationFeedback>(
      "allocation/feedback", rclcpp::QoS(10),
      [this](AllocationFeedback::ConstSharedPtr msg) {
        feedback_ = *msg;
        feedback_arrival_ = now();
      });

    tau_u_pub_ = create_publisher<Float64>("tau_u", rclcpp::QoS(10));
    tau_r_pub_ = create_publisher<Float64>("tau_r", rclcpp::QoS(10));
    debug_pub_ = create_publisher<PidDebug>("pid/debug", rclcpp::QoS(10));

    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / rate_), [this]() {tick();});
  }

private:
  void declare_parameters()
  {
    rate_ = declare_parameter("rate", 50.0);

    declare_parameter("state_timeout", 0.5);
    declare_parameter("reference_timeout", 0.0);
    declare_parameter("anti_windup", std::string("back_calculation"));
    declare_parameter("tracking_gain", 0.0);

    declare_parameter("surge.kp", 1.0);
    declare_parameter("surge.ki", 0.5);
    declare_parameter("surge.kd", 0.0);
    declare_parameter("surge.feedforward", 1.0);
    declare_parameter("surge.output_limit", 0.22);
    declare_parameter("surge.derivative_filter_tau", 0.1);

    declare_parameter("heading.kp", 2.0);
    declare_parameter("heading.ki", 0.05);
    declare_parameter("heading.kd", 0.1);
    declare_parameter("heading.output_limit", 2.75);
    declare_parameter("heading.integral_zone", 0.35);

    declare_parameter("gain_schedule.enabled", false);
    declare_parameter("gain_schedule.speeds", std::vector<double>{0.0, 0.22});
    declare_parameter("gain_schedule.heading_kp", std::vector<double>{2.0, 2.0});
    declare_parameter("gain_schedule.heading_ki", std::vector<double>{0.05, 0.05});
    declare_parameter("gain_schedule.heading_kd", std::vector<double>{0.1, 0.1});
  }

  /// Read every parameter into the controllers, keeping their integrator state.
  void apply_parameters()
  {
    state_timeout_ = get_parameter("state_timeout").as_double();
    reference_timeout_ = get_parameter("reference_timeout").as_double();
    surge_feedforward_ = get_parameter("surge.feedforward").as_double();

    const auto anti_windup = parse_anti_windup(get_parameter("anti_windup").as_string());
    const double tracking_gain = get_parameter("tracking_gain").as_double();

    PidConfig surge;
    surge.gains = {
      get_parameter("surge.kp").as_double(),
      get_parameter("surge.ki").as_double(),
      get_parameter("surge.kd").as_double()};
    surge.output_max = get_parameter("surge.output_limit").as_double();
    surge.output_min = -surge.output_max;
    surge.anti_windup = anti_windup;
    surge.tracking_gain = tracking_gain;
    surge.derivative_filter_tau = get_parameter("surge.derivative_filter_tau").as_double();
    surge_pid_.configure(surge);

    PidConfig heading;
    heading.gains = {
      get_parameter("heading.kp").as_double(),
      get_parameter("heading.ki").as_double(),
      get_parameter("heading.kd").as_double()};
    heading.output_max = get_parameter("heading.output_limit").as_double();
    heading.output_min = -heading.output_max;
    heading.anti_windup = anti_windup;
    heading.integral_zone = get_parameter("heading.integral_zone").as_double();
    heading.tracking_gain = tracking_gain;
    heading_pid_.configure(heading);

    heading_schedule_.reset();
    if (get_parameter("gain_schedule.enabled").as_bool()) {
      heading_schedule_ = make_schedule();
    }
  }

  GainSchedule make_schedule() const
  {
    const auto speeds = get_parameter("gain_schedule.speeds").as_double_array();
    const auto kp = get_parameter("gain_schedule.heading_kp").as_double_array();
    const auto ki = get_parameter("gain_schedule.heading_ki").as_double_array();
    const auto kd = get_parameter("gain_schedule.heading_kd").as_double_array();
    if (kp.size() != speeds.size() || ki.size() != speeds.size() || kd.size() != speeds.size()) {
      throw std::invalid_argument("gain_schedule arrays must all have the same length");
    }
    std::vector<PidGains> gains(speeds.size());
    for (std::size_t i = 0; i < speeds.size(); ++i) {
      gains[i] = PidGains{kp[i], ki[i], kd[i]};
    }
    return GainSchedule(speeds, gains);
  }

  rcl_interfaces::msg::SetParametersResult validate_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "anti_windup") {
        try {
          parse_anti_windup(parameter.as_string());
        } catch (const std::invalid_argument & error) {
          result.successful = false;
          result.reason = error.what();
        }
        continue;
      }
      const bool is_gain = ends_with(name, ".kp") || ends_with(name, ".ki") ||
        ends_with(name, ".kd");
      if (is_gain && parameter.as_double() < 0.0) {
        result.successful = false;
        result.reason = name + " must not be negative";
      }
    }
    parameters_changed_ = result.successful;
    return result;
  }

  /// True when @p arrival exists and is younger than @p timeout. A timeout of 0 disables the check.
  bool fresh(
    const std::optional<rclcpp::Time> & arrival, double timeout, const rclcpp::Time & t) const
  {
    return arrival && (timeout <= 0.0 || (t - *arrival).seconds() < timeout);
  }

  /// Disable both loops: zero command, and an empty integrator for the next start.
  void hold()
  {
    surge_pid_.reset();
    heading_pid_.reset();
    Float64 zero;
    zero.data = 0.0;
    tau_u_pub_->publish(zero);
    tau_r_pub_->publish(zero);
  }

  void tick()
  {
    const auto t = now();
    const double dt = last_tick_ ? (t - *last_tick_).seconds() : 0.0;
    last_tick_ = t;

    if (parameters_changed_) {
      parameters_changed_ = false;
      apply_parameters();
    }

    const bool state_ok = state_ && state_->valid && fresh(state_arrival_, state_timeout_, t);
    if (!state_ok || !fresh(reference_arrival_, reference_timeout_, t)) {
      if (reference_arrival_ && !state_ok) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "state is invalid or stale, commanding zero");
      }
      hold();
      return;
    }
    // Simulated time can stand still or jump (Gazebo paused, bag restarted).
    if (dt <= 0.0 || dt > 1.0) {
      return;
    }

    const bool feedback_ok = feedback_ && fresh(feedback_arrival_, state_timeout_, t);
    const double surge_ratio = feedback_ok ? feedback_->surge_ratio : 1.0;
    const double yaw_ratio = feedback_ok ? feedback_->yaw_ratio : 1.0;

    // Surge loop. The plant already follows velocity commands, so the feedforward
    // does most of the work and the PI only trims the remaining error.
    const double surge_error = reference_->surge - state_->surge;
    const auto surge_result = surge_pid_.update(
      surge_error, dt, surge_feedforward_ * reference_->surge, std::nullopt, surge_ratio);

    // Heading loop. The error is wrapped so the robot takes the shortest turn. The
    // gyro gives the derivative directly (derivative on measurement, de/dt = -r),
    // with no noise amplification and no kick when the setpoint steps.
    if (heading_schedule_) {
      heading_pid_.set_gains(heading_schedule_->at(std::abs(state_->surge)));
    }
    const double heading_error = wrap_pi(reference_->heading - state_->heading);
    const auto heading_result = heading_pid_.update(
      heading_error, dt, 0.0, -state_->yaw_rate, yaw_ratio);

    Float64 tau_u;
    tau_u.data = surge_result.output;
    tau_u_pub_->publish(tau_u);
    Float64 tau_r;
    tau_r.data = heading_result.output;
    tau_r_pub_->publish(tau_r);

    PidDebug debug;
    debug.header.stamp = t;
    fill_terms(
      debug.surge, reference_->surge, state_->surge, surge_error,
      surge_pid_.config().gains, surge_result);
    fill_terms(
      debug.heading, reference_->heading, state_->heading, heading_error,
      heading_pid_.config().gains, heading_result);
    debug_pub_->publish(debug);
  }

  Pid surge_pid_, heading_pid_;
  std::optional<GainSchedule> heading_schedule_;
  double rate_{50.0};
  double surge_feedforward_{1.0};
  double state_timeout_{0.5};
  double reference_timeout_{0.0};
  bool parameters_changed_{false};

  std::optional<VesselState> state_;
  std::optional<Reference> reference_;
  std::optional<AllocationFeedback> feedback_;
  std::optional<rclcpp::Time> state_arrival_, reference_arrival_, feedback_arrival_, last_tick_;

  rclcpp::Subscription<VesselState>::SharedPtr state_sub_;
  rclcpp::Subscription<Reference>::SharedPtr reference_sub_;
  rclcpp::Subscription<AllocationFeedback>::SharedPtr feedback_sub_;
  rclcpp::Publisher<Float64>::SharedPtr tau_u_pub_, tau_r_pub_;
  rclcpp::Publisher<PidDebug>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

}  // namespace tsb_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tsb_control::PidNode>());
  rclcpp::shutdown();
  return 0;
}
