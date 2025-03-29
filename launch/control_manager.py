#!/usr/bin/env python3

import launch
import os
import sys

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    IfElseSubstitution,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_uav_managers"

    this_pkg_path = get_package_share_directory(pkg_name)
    namespace='control_manager'

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
    )

    # #} end of custom_config

    # #{ platform_config

    platform_config = LaunchConfiguration('platform_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'platform_config',
        default_value="",
        description="Path to the platform configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    ))

    # behaviour:
    #     platform_config == "" => platform_config: ""
    #     platform_config == "/<path>" => platform_config: "/<path>"
    #     platform_config == "<path>" => platform_config: "$(pwd)/<path>"
    platform_config = IfElseSubstitution(
            condition=PythonExpression(['"', platform_config, '" != "" and ', 'not "', platform_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), platform_config]),
            else_value=platform_config
    )

    # #} end of platform_config

    # #{ world_config

    world_config = LaunchConfiguration('world_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'world_config',
        default_value="",
        description="Path to the world configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    ))

    # behaviour:
    #     world_config == "" => world_config: ""
    #     world_config == "/<path>" => world_config: "/<path>"
    #     world_config == "<path>" => world_config: "$(pwd)/<path>"
    world_config = IfElseSubstitution(
            condition=PythonExpression(['"', world_config, '" != "" and ', 'not "', world_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), world_config]),
            else_value=world_config
    )

    # #} end of world_config

    # #{ network_config

    network_config = LaunchConfiguration('network_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'network_config',
        default_value="",
        description="Path to the network configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    ))

    # behaviour:
    #     network_config == "" => network_config: ""
    #     network_config == "/<path>" => network_config: "/<path>"
    #     network_config == "<path>" => network_config: "$(pwd)/<path>"
    network_config = IfElseSubstitution(
            condition=PythonExpression(['"', network_config, '" != "" and ', 'not "', network_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), network_config]),
            else_value=network_config
    )

    # #} end of network_config

    # #{ env-based params

    uav_name=os.getenv('UAV_NAME', "uav1")

    # #} end of env-based params

    ld.add_action(ComposableNodeContainer(

        namespace=uav_name,
        name=namespace+'_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output="screen",

        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],

        composable_node_descriptions=[

            ComposableNode(

                package=pkg_name,
                plugin='mrs_uav_managers::control_manager::ControlManager',
                namespace=uav_name,
                name='control_manager',
                parameters=[
                    {"uav_name": uav_name},
                    {"uav_mass": 2.0},
                    {"topic_prefix": "/" + uav_name},
                    {"enable_profiler": False},
                    {"g": 9.81},
                    {"body_frame": "fcu"},
                    {"run_type": "simulation"},
                    {"body_disturbance_x": 0.0},
                    {"body_disturbance_y": 0.0},
                    {'private_config': this_pkg_path + '/config/private/control_manager.yaml'},
                    {'public_config': this_pkg_path + '/config/public/control_manager.yaml'},
                    {'private_trackers': this_pkg_path + '/config/private/trackers.yaml'},
                    {'private_controllers': this_pkg_path + '/config/private/controllers.yaml'},
                    {'public_controllers': this_pkg_path + '/config/public/controllers.yaml'},
                    {'uav_manager_config': this_pkg_path + '/config/public/uav_manager.yaml'},
                    {'platform_config': platform_config},
                    {'custom_config': custom_config},
                    {'world_config': world_config},
                    {'network_config': network_config},
                ],

                remappings=[
                    # subscribers
                    ("~/hw_api_capabilities_in", "hw_api/capabilities"),
                    ("~/hw_api_status_in", "hw_api/status"),
                ],
            )

        ],

    ))

    return ld
