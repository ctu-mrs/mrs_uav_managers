#!/usr/bin/env python3

import launch
import os
import sys

import launch_ros
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
    namespace='transform_manager'

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

    # #{ env-based params

    uav_name=os.getenv('UAV_NAME', "uav1")
    use_sim_time=os.getenv('USE_SIM_TIME', "false") == "true"

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
                plugin='mrs_uav_managers::transform_manager::TransformManager',
                namespace=uav_name,
                name='transform_manager',
                parameters=[
                    {"uav_name": uav_name},
                    {"enable_profiler": False},
                    {"use_sim_time": use_sim_time},
                    {'private_config': this_pkg_path + '/config/private/transform_manager/transform_manager.yaml'},
                    {'public_config': this_pkg_path + '/config/public/transform_manager/transform_manager.yaml'},
                    {'estimators_config': this_pkg_path + '/config/public/active_estimators.yaml'},
                    {'platform_config': platform_config},
                    {'custom_config': custom_config},
                    {'world_config': world_config},
                ],

                remappings=[
                    # subscribers
                    ("~/uav_state_in", "estimation_manager/uav_state"),
                    ("~/height_agl_in", "estimation_manager/height_agl"),
                    ("~/orientation_in", "hw_api/orientation"),
                    ("~/gnss_in", "hw_api/gnss"),
                    ("~/rtk_gps_in", "hw_api/rtk"),
                    ("~/altitude_amsl_in", "hw_api/altitude"),
                    # publishers
                    ("~/profiler", "profiler"),
                    ("~/map_delay_out", "~/map_delay"),
                ],
            )

        ],

    ))

    ld.add_action(
        # Nodes under test
        launch_ros.actions.Node(
            package='tf2_ros',
            namespace='',
            executable='static_transform_publisher',
            name='fcu_to_rtk_antenna',
            arguments=["0.0", "0.0", "0.20", "0", "0", "0", uav_name+"/fcu", uav_name+"/rtk_antenna"],
        )
    )

    return ld
