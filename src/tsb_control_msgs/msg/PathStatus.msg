# Progress along the path, published by path_following_node on /path_following/status.

std_msgs/Header header

uint32 active_segment        # index of the segment being followed
uint32 segment_count         # number of segments in the path
float64 cross_track_error    # [m] positive when the vehicle is left of the path
float64 distance_to_waypoint # [m] straight-line distance to the end of the segment
float64 desired_heading      # [rad] output of the line-of-sight law
float64 desired_surge        # [m/s] output of the speed law
bool finished                # the last waypoint has been reached
