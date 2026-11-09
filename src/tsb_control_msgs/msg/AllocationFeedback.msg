# How much of the requested command the wheels could actually deliver.
# pid_node uses the ratios for anti-windup: surge and yaw share the same two wheels,
# so a loop can be saturated even when its own command is inside its own limits.

std_msgs/Header header

float64 surge_ratio  # delivered / requested surge, 1.0 when nothing was cut
float64 yaw_ratio    # delivered / requested yaw rate, 1.0 when nothing was cut
bool saturated       # at least one ratio is below 1.0
