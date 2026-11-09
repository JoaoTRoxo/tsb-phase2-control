"""Start the inner pipeline: sensor processing, PID and allocation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_nodes(context):
    """Build the node actions once the launch arguments are known."""
    share = get_package_share_directory('tsb_control')
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    parameters = [os.path.join(share, 'config', 'control.yaml')]
    if get('variant'):
        parameters.append(os.path.join(share, 'config', 'variants', get('variant') + '.yaml'))

    overrides = {'use_sim_time': get('use_sim_time').lower() == 'true'}
    pid_overrides = dict(overrides, reference_timeout=float(get('reference_timeout')))

    return [
        Node(
            package='tsb_control', executable='sensor_processing_node', output='screen',
            parameters=[*parameters, overrides]),
        Node(
            package='tsb_control', executable='pid_node', output='screen',
            parameters=[*parameters, pid_overrides]),
        Node(
            package='tsb_control', executable='allocation_node', output='screen',
            parameters=[*parameters, overrides]),
    ]


def generate_launch_description():
    """Return the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'variant', default_value='',
            description='optional config/variants/<name>.yaml layered over control.yaml'),
        DeclareLaunchArgument(
            'reference_timeout', default_value='0.0',
            description='seconds before an old reference stops the robot; 0 disables it'),
        OpaqueFunction(function=make_nodes),
    ])
