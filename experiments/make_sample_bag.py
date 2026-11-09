#!/usr/bin/env python3
"""
Cut a short sensors-only sample out of a recorded run.

Usage: make_sample_bag.py <input bag> <output bag> [start s] [duration s]

The result is small enough to keep in the repository, and is what
bag_replay.launch.py needs: raw sensors and references, with no controller output.
"""

import sys

import rosbag2_py

KEEP = {'/odom', '/imu', '/desired', '/clock'}


def main():
    """Copy the selected topics and time window."""
    source, target = sys.argv[1:3]
    start = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    duration = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=source, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=target, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for topic in reader.get_all_topics_and_types():
        if topic.name in KEEP:
            writer.create_topic(topic)

    first = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        first = stamp if first is None else first
        elapsed = (stamp - first) * 1e-9
        if topic in KEEP and start <= elapsed <= start + duration:
            writer.write(topic, data, stamp)


if __name__ == '__main__':
    main()
