#!/usr/bin/env python3
#
# Authors:
#     Enrico Saccon     enrico.saccon [at] unitn.it
#     Placido Falqueto  placido.falqueto [at] unitn.it
#     Christian Johansen christian.johansen [at] unitn.it

import logging
import os
from pathlib import Path
import yaml

import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def print_env(context):
    print(__file__)
    for key in context.launch_configurations.keys():
        print("\t", key, context.launch_configurations[key])
    return


def generate_launch_description():
    # launch.logging.launch_config.level = logging.DEBUG

    planning_pkg = get_package_share_directory("shelfino_planning")

    planning_params_file_path = os.path.join(planning_pkg, "config", "planning_config.yaml")
    if not os.path.exists(planning_params_file_path):
        raise Exception(
            "[{}] Planning config file `{}` does not exist".format(
                __file__, planning_params_file_path
            )
        )

    # General arguments
    planning_params_file = LaunchConfiguration('planning_params_file', default=planning_params_file_path)
    planning_config = {}
    with open(planning_params_file_path, "r") as f:
        planning_config = yaml.safe_load(f)

    # Declare LaunchArguments for exposing launching arguments
    launch_args = [
        DeclareLaunchArgument(
            "planning_params_file",
            default_value=planning_params_file_path,
            description="Full path to the planning params file to use",
        ),
    ]

    # List of nodes to launch
    nodes = [
        Node (
            package='shelfino_planning',
            executable='roadmap_harness',
            name='roadmap_harness',
            output='screen',
            parameters=[planning_config['roadmap_harness']]
        ),
        Node (
            package='shelfino_planning',
            executable='graph_search',
            name='graph_search',
            output='screen',
        ),
        Node (
            package='shelfino_planning',
            executable='dubins_node',
            name='dubins_node',
            output='screen',
        ),
    ]

    # Launch configured roadmap services
    logging.info("Launching config: {}".format(planning_config))
    for node in planning_config['roadmap_harness']['roadmap_services']:
        nodes.append(Node(
            package='shelfino_planning',
            executable=node,
            name='roadmap_harness_' + node,
            output='screen',
        ))

    # LaunchDescription with the additional launch files
    ld = LaunchDescription()

    for launch_arg in launch_args:
        ld.add_action(launch_arg)

    ld.add_action(OpaqueFunction(function=print_env))

    for node in nodes:
        ld.add_action(node)

    return ld
