#!/usr/bin/env python3
"""Publish a timed sequence of [surge, heading] references, for step-response tests."""

import rclpy
from rclpy.node import Node
from tsb_control_msgs.msg import Reference


class ReferenceSequence(Node):
    """Walks through the `steps` parameter: flat [duration, surge, heading] triples."""

    def __init__(self):
        super().__init__('reference_sequence')
        flat = self.declare_parameter('steps', [0.0]).value
        if len(flat) % 3 != 0:
            raise ValueError('steps must hold [duration, surge, heading] triples')
        self.steps = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]
        self.publisher = self.create_publisher(Reference, 'desired', 10)
        self.start = None
        self.finished = False
        self.timer = self.create_timer(0.05, self.tick)

    def tick(self):
        """Publish the reference for the current time, or finish."""
        now = self.get_clock().now()
        # Under simulated time the clock reads zero until Gazebo starts publishing it.
        if now.nanoseconds == 0:
            return
        if self.start is None:
            self.start = now
        elapsed = (now - self.start).nanoseconds * 1e-9
        for duration, surge, heading in self.steps:
            if elapsed < duration:
                message = Reference()
                message.header.stamp = now.to_msg()
                message.surge = surge
                message.heading = heading
                self.publisher.publish(message)
                return
            elapsed -= duration
        self.finished = True


def main():
    """Run until the sequence is over."""
    rclpy.init()
    node = ReferenceSequence()
    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
