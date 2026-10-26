"""Bring up the terrain model against an already-running ground segmenter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    parameters = [
        LaunchConfiguration("config_yaml"),
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]
    # Empty keeps the ground_topic from the yaml; the start.sh sets it per segmenter.
    ground_topic = LaunchConfiguration("ground_topic").perform(context)
    if ground_topic:
        parameters.append({"ground_topic": ground_topic})

    node = Node(
        package="terrain_modelling",
        executable="terrain_modelling_node",
        name="terrain_modelling_node",
        output="screen",
        parameters=parameters,
    )
    return [node]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("terrain_modelling"),
        "config",
        "terrain_modelling.yaml",
    )

    args = [
        DeclareLaunchArgument("config_yaml", default_value=default_config),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "ground_topic",
            default_value="",
            description="Ground points topic to read; empty uses the one in config_yaml",
        ),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
