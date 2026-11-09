"""Start the inner pipeline plus line-of-sight path following."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_actions(context):
    """Build the actions once the launch arguments are known."""
    share = get_package_share_directory('tsb_control')
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    parameters = [os.path.join(share, 'config', 'path_following.yaml')]
    if get('mission'):
        parameters.append(os.path.join(share, 'config', 'missions', get('mission') + '.yaml'))
    parameters.append({'use_sim_time': get('use_sim_time').lower() == 'true'})

    return [
        # If the follower dies, a one-second reference timeout stops the robot.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'control.launch.py')),
            launch_arguments={
                'use_sim_time': get('use_sim_time'),
                'variant': get('variant'),
                'reference_timeout': '1.0',
            }.items()),
        Node(
            package='tsb_control', executable='path_following_node', output='screen',
            parameters=parameters),
    ]


def generate_launch_description():
    """Return the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('variant', default_value=''),
        DeclareLaunchArgument(
            'mission', default_value='',
            description='config/missions/<name>.yaml; empty waits for a path on /waypoints'),
        OpaqueFunction(function=make_actions),
    ])
