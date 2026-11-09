"""
Run the whole system in Gazebo.

For example, with the Gazebo window open:
  ros2 launch tsb_control sim.launch.py mission:=square gui:=true
Or headless, running a step test or recording a rosbag:
  ros2 launch tsb_control sim.launch.py scenario:=steps
  ros2 launch tsb_control sim.launch.py mission:=square bag:=/tmp/square_run
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RECORDED_TOPICS = [
    '/odom', '/imu', '/state', '/desired', '/tau_u', '/tau_r', '/current_left', '/current_right',
    '/cmd_vel', '/pid/debug', '/allocation/feedback', '/path_following/status',
    '/path_following/path',
]


def make_actions(context):
    """Build the actions once the launch arguments are known."""
    share = get_package_share_directory('tsb_control')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    turtlebot = get_package_share_directory('turtlebot3_gazebo')
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    def include(path, **arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path), launch_arguments=arguments.items())

    world = get('world') or os.path.join(share, 'worlds', 'flat.world')
    actions = [
        include(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'),
            world=world, params_file=os.path.join(share, 'config', 'gazebo.yaml')),
        include(
            os.path.join(turtlebot, 'launch', 'robot_state_publisher.launch.py'),
            use_sim_time='true'),
        include(
            os.path.join(turtlebot, 'launch', 'spawn_turtlebot3.launch.py'),
            x_pose=get('x_pose'), y_pose=get('y_pose')),
    ]
    if get('gui').lower() == 'true':
        actions.append(include(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')))

    if get('mission') or get('follow').lower() == 'true':
        actions.append(include(
            os.path.join(share, 'launch', 'path_following.launch.py'),
            use_sim_time='true', variant=get('variant'), mission=get('mission')))
    else:
        actions.append(include(
            os.path.join(share, 'launch', 'control.launch.py'),
            use_sim_time='true', variant=get('variant')))

    if get('scenario'):
        actions.append(Node(
            package='tsb_control', executable='reference_sequence.py', output='screen',
            parameters=[
                os.path.join(share, 'config', 'scenarios', get('scenario') + '.yaml'),
                {'use_sim_time': True}]))

    if get('bag'):
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--use-sim-time', '-o', get('bag'), *RECORDED_TOPICS],
            output='log'))
    return actions


def generate_launch_description():
    """Return the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false', description='open the Gazebo window'),
        DeclareLaunchArgument(
            'world', default_value='', description='path of a world file; empty uses flat.world'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('variant', default_value=''),
        DeclareLaunchArgument('mission', default_value='', description='config/missions/<name>'),
        DeclareLaunchArgument(
            'follow', default_value='false',
            description='start path following with no mission, waiting for /waypoints'),
        DeclareLaunchArgument('scenario', default_value='', description='config/scenarios/<name>'),
        DeclareLaunchArgument('bag', default_value='', description='record to this rosbag path'),
        OpaqueFunction(function=make_actions),
    ])
