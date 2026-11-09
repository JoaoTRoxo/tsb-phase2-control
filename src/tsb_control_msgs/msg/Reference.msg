# Setpoint for the inner loops: the [desired surge, desired heading] vector from the
# project statement, with named fields so the units are explicit.

std_msgs/Header header

float64 surge    # [m/s] desired forward velocity
float64 heading  # [rad] desired heading; any value, the controller takes the shortest turn
