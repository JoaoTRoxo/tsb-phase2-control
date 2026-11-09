"""
Feed a recorded rosbag into the pipeline instead of the simulator (Figure 1).

Only the raw sensors are replayed. The nodes compute their own /state, /tau_u,
/tau_r and wheel commands from them, on the bag's clock:
  ros2 launch tsb_control bag_replay.launch.py bag:=bags/square
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Return the launch description."""
    share = get_package_share_directory('tsb_control')
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='path of the rosbag to replay'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'control.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'), '--clock', '200',
                 '--topics', '/odom', '/imu', '/desired'],
            output='screen'),
    ])
