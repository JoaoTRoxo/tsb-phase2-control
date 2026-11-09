"""
Integration test: the three inner nodes, wired together, fed fake sensors.

Checks the ROS plumbing the unit tests cannot: topic names, QoS compatibility,
parameter loading from the YAML file, and the safety behaviour when sensors stop.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tsb_control_msgs.msg import Reference, VesselState


@pytest.mark.launch_test
def generate_test_description():
    """Start the pipeline from the installed configuration."""
    config = os.path.join(get_package_share_directory('tsb_control'), 'config', 'control.yaml')
    nodes = [
        launch_ros.actions.Node(package='tsb_control', executable=name, parameters=[config])
        for name in ('sensor_processing_node', 'pid_node', 'allocation_node')
    ]
    return launch.LaunchDescription([*nodes, launch_testing.actions.ReadyToTest()])


class TestPipeline(unittest.TestCase):
    """Drive the pipeline from the outside, like the simulator would."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_pipeline')
        cls.odom = cls.node.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        cls.imu = cls.node.create_publisher(Imu, 'imu', qos_profile_sensor_data)
        cls.desired = cls.node.create_publisher(Reference, 'desired', 10)
        cls.latest = {}
        cls.node.create_subscription(
            Twist, 'cmd_vel', lambda m: cls.latest.__setitem__('cmd_vel', m), 10)
        cls.node.create_subscription(
            VesselState, 'state', lambda m: cls.latest.__setitem__('state', m), 10)
        cls.node.create_subscription(
            Float64, 'current_left', lambda m: cls.latest.__setitem__('left', m.data), 10)
        cls.node.create_subscription(
            Float64, 'current_right', lambda m: cls.latest.__setitem__('right', m.data), 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def run_for(self, seconds, sensors=True, surge=0.1, heading=0.0):
        """Spin for a while, optionally publishing sensors and a reference."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            stamp = self.node.get_clock().now().to_msg()
            if sensors:
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = 'odom'
                odom.pose.pose.orientation.w = 1.0
                self.odom.publish(odom)
                imu = Imu()
                imu.header.stamp = stamp
                self.imu.publish(imu)
            reference = Reference()
            reference.surge = surge
            reference.heading = heading
            self.desired.publish(reference)
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def test_full_chain(self):
        """Sensors in, wheel commands out, and a stop when the sensors go quiet."""
        # Standing still with surge error and a heading to the left.
        self.run_for(3.0, heading=0.5)
        self.assertTrue(self.latest['state'].valid)
        self.assertGreater(self.latest['cmd_vel'].linear.x, 0.05)
        self.assertGreater(self.latest['cmd_vel'].angular.z, 0.1)
        self.assertGreater(self.latest['right'], self.latest['left'])

        # Sensors stop: state invalid, so the commands must drop to zero.
        self.run_for(2.0, sensors=False, heading=0.5)
        self.assertFalse(self.latest['state'].valid)
        self.assertAlmostEqual(self.latest['cmd_vel'].linear.x, 0.0)
        self.assertAlmostEqual(self.latest['cmd_vel'].angular.z, 0.0)
