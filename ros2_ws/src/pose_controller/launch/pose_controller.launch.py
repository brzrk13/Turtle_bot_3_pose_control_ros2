"""Launch the pose_controller node on its own (sim or robot already running)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("pose_controller")
    default_params = os.path.join(pkg_share, "config", "pose_controller.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="Full path to the controller parameter YAML file.",
    )

    controller = Node(
        package="pose_controller",
        executable="pose_controller_node",
        name="pose_controller",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_arg, controller])
