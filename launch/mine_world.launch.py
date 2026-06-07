#!/usr/bin/env python3
"""
Launch the underground mine world with TurtleBot3.

Reuses the standard TurtleBot3 sub-launches (robot_state_publisher + spawn),
so the full TF tree and /scan, /odom, /cmd_vel all come up exactly like the
default turtlebot3_world launch -- just with the mine world instead.

Edit WORLD_PATH below if you put mine.world somewhere else.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# ---- Edit this path if needed ----
WORLD_PATH = os.path.expanduser('~/ros2_ws/src/my_bot/worlds/mine.world')

# Robot spawn position (a clear corner of the mine)
X_POSE = '-6.0'
Y_POSE = '0.0'


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': WORLD_PATH}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': X_POSE, 'y_pose': Y_POSE}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    return ld
