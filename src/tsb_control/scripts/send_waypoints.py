#!/usr/bin/env python3
"""
Send a path to path_following_node from the command line.

Usage: ros2 run tsb_control send_waypoints.py 1,0 1,1 0,1
"""

import sys
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    """Publish the waypoints once, on a latched topic."""
    points = [tuple(float(value) for value in arg.split(',')) for arg in sys.argv[1:]]
    if not points or any(len(point) != 2 for point in points):
        sys.exit(__doc__)

    rclpy.init()
    node = rclpy.create_node('send_waypoints')
    latched = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(Path, 'waypoints', latched)

    path = Path()
    path.header.frame_id = 'odom'
    for x, y in points:
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    publisher.publish(path)

    # Stay alive briefly so discovery finishes and the subscriber gets the sample.
    time.sleep(2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
