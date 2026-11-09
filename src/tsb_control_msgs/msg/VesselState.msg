# Best estimate of the vehicle state, published by sensor_processing_node on /state.
# Follows REP-103: body x forward, y left; heading counter-clockwise from +x of frame_id.

std_msgs/Header header  # stamp of the estimate; frame_id is the odometry frame

float64 x         # [m] position in frame_id
float64 y         # [m] position in frame_id
float64 heading   # [rad] wrapped to [-pi, pi]
float64 surge     # [m/s] forward velocity in the body frame, low-pass filtered
float64 yaw_rate  # [rad/s] from the gyroscope, counter-clockwise positive

bool valid        # false while odometry or IMU are missing or stale
