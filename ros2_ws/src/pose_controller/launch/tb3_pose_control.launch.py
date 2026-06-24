"""Bring up the TurtleBot3 Gazebo simulation and the pose controller together.

Requires ros-<distro>-turtlebot3-gazebo and TURTLEBOT3_MODEL (e.g. burger).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("pose_controller")
    default_params = os.path.join(pkg_share, "config", "pose_controller.yaml")

    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    world_launch = os.path.join(
        tb3_gazebo_share, "launch", "empty_world.launch.py"
    )

    params_arg = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="Full path to the controller parameter YAML file.",
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
    )

    controller = Node(
        package="pose_controller",
        executable="pose_controller_node",
        name="pose_controller",
        output="screen",
        # TurtleBot3's Gazebo bridge expects TwistStamped on /cmd_vel.
        parameters=[LaunchConfiguration("params_file"),
                    {"use_stamped_twist": True, "use_sim_time": True}],
    )

    return LaunchDescription([params_arg, simulation, controller])
