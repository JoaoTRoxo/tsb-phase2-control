# tsb_control

The control nodes, in C++. Algorithms live in headers under `include/tsb_control/` with no
ROS dependency, and each node is a thin wrapper around them, so the logic can be unit tested
without a running system. Every length is in metres, angles are in radians, and frames follow
REP-103 (x forward, y left, heading counter-clockwise).

## sensor_processing_node

Turns raw sensors into the state the controllers use.

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| Sub | `odom` | `nav_msgs/Odometry` | Position, surge, absolute yaw |
| Sub | `imu` | `sensor_msgs/Imu` | Gyroscope |
| Pub | `state` | `tsb_control_msgs/VesselState` | At a fixed rate; `valid` is false when a sensor is stale |

| Parameter | Default | Meaning |
| --- | --- | --- |
| `rate` | 50.0 | Publication rate [Hz] |
| `surge_filter_tau` | 0.05 | Low-pass on surge [s] |
| `heading_filter_tau` | 1.0 | Complementary filter crossover [s]; gyro below, odometry above |
| `sensor_timeout` | 0.5 | Age after which a sensor counts as stale [s] |

## pid_node

Surge and heading PID loops. See the top-level README for the design.

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| Sub | `state` | `tsb_control_msgs/VesselState` | Measurements |
| Sub | `desired` | `tsb_control_msgs/Reference` | Desired surge and heading |
| Sub | `allocation/feedback` | `tsb_control_msgs/AllocationFeedback` | Optional, for anti-windup |
| Pub | `tau_u` | `std_msgs/Float64` | Surge command [m/s] |
| Pub | `tau_r` | `std_msgs/Float64` | Yaw-rate command [rad/s] |
| Pub | `pid/debug` | `tsb_control_msgs/PidDebug` | Every internal term of both loops |

| Parameter | Default | Meaning |
| --- | --- | --- |
| `rate` | 50.0 | Loop rate [Hz] |
| `state_timeout` | 0.5 | Older state means zero command and cleared integrators [s] |
| `reference_timeout` | 0.0 | The same for references; 0 holds the last reference [s] |
| `anti_windup` | `back_calculation` | `none`, `clamp` or `back_calculation` |
| `tracking_gain` | 0.0 | Back-calculation `Kt` [1/s]; 0 uses `Ki / Kp` for each loop |
| `surge.kp`, `.ki`, `.kd` | 1.0, 0.5, 0.0 | Surge gains |
| `surge.feedforward` | 1.0 | Fraction of `v_d` commanded directly |
| `surge.output_limit` | 0.22 | Command limit [m/s] |
| `surge.derivative_filter_tau` | 0.1 | Low-pass on the surge derivative [s] |
| `heading.kp`, `.ki`, `.kd` | 2.0, 0.05, 0.1 | Heading gains when scheduling is off |
| `heading.output_limit` | 2.75 | Command limit [rad/s] |
| `heading.integral_zone` | 0.35 | Integrate only below this error [rad]; 0 always integrates |
| `gain_schedule.enabled` | see YAML | Schedule heading gains against surge speed |
| `gain_schedule.speeds` | see YAML | Breakpoints [m/s], strictly increasing |
| `gain_schedule.heading_kp`, `_ki`, `_kd` | see YAML | Gains at each breakpoint |

Every parameter except `rate` can be changed at runtime with `ros2 param set`. Negative gains
and unknown anti-windup modes are rejected.

## allocation_node

Maps the two commands onto the wheels, serving yaw first when both do not fit.

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| Sub | `tau_u`, `tau_r` | `std_msgs/Float64` | Commands from the PID |
| Pub | `current_left`, `current_right` | `std_msgs/Float64` | Wheel-speed setpoints [rad/s] |
| Pub | `cmd_vel` | `geometry_msgs/Twist` | Rebuilt from the saturated wheel speeds |
| Pub | `allocation/feedback` | `tsb_control_msgs/AllocationFeedback` | Delivered / requested ratios |

| Parameter | Default | Meaning |
| --- | --- | --- |
| `rate` | 50.0 | Loop rate [Hz] |
| `wheel_radius` | 0.033 | [m] |
| `wheel_separation` | 0.160 | [m] |
| `max_wheel_speed` | 6.67 | [rad/s] |
| `command_timeout` | 0.5 | Watchdog: no command for this long stops the wheels [s] |

## path_following_node

Line-of-sight guidance over a list of waypoints.

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| Sub | `state` | `tsb_control_msgs/VesselState` | Position and heading |
| Sub | `waypoints` | `nav_msgs/Path` | Latched; replaces the current path |
| Pub | `desired` | `tsb_control_msgs/Reference` | To the PID |
| Pub | `path_following/status` | `tsb_control_msgs/PathStatus` | Progress and errors |
| Pub | `path_following/path` | `nav_msgs/Path` | Latched, for RViz |

| Parameter | Default | Meaning |
| --- | --- | --- |
| `rate` | 20.0 | Loop rate [Hz] |
| `cruise_speed` | 0.15 | [m/s] |
| `min_speed` | 0.03 | Floor of the arrival slowdown [m/s] |
| `arrival_gain` | 0.5 | Speed per metre remaining near the goal [1/s] |
| `turn_slowdown` | 2.0 | Speed is scaled by `cos(heading error)^this` |
| `lookahead` | 0.4 | LOS lookahead distance [m] |
| `sigma` | 0.0 | Integral LOS gain; 0 is classic LOS |
| `acceptance_radius` | 0.05 | [m] |
| `start_from_current` | true | Add the robot's position as the start of the path |
| `waypoints_x`, `waypoints_y` | empty | Path from parameters |

## Scripts

| Script | Use |
| --- | --- |
| `reference_sequence.py` | Publishes timed `[duration, surge, heading]` steps on `desired` |
| `send_waypoints.py` | `ros2 run tsb_control send_waypoints.py 1,0 1,1 0,1` |
| `plot_run.py` | Turns a rosbag into a trajectory map, response plots and metrics |
