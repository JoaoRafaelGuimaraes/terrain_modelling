"""Bring up the terrain model against an already-running ground segmenter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("terrain_modelling"),
        "config",
        "terrain_modelling.yaml",
    )

    args = [
        DeclareLaunchArgument("config_yaml", default_value=default_config),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]

    node = Node(
        package="terrain_modelling",
        executable="terrain_modelling_node",
        name="terrain_modelling_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription(args + [node])
