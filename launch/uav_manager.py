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
    namespace='uav_manager'

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

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    # #} end of log_level

    ld.add_action(ComposableNodeContainer(

        namespace=uav_name,
        name=namespace+'_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output="screen",
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],

        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],

        composable_node_descriptions=[

            ComposableNode(

                package=pkg_name,
                plugin='mrs_uav_managers::uav_manager::UavManager',
                namespace=uav_name,
                name='uav_manager',

                parameters=[
                    {"uav_name": uav_name},
                    {"uav_mass": 2.0},
                    {"enable_profiler": False},
                    {"use_sim_time": use_sim_time},
                    {"g": 9.81},
                    {"run_type": "simulation"},
                    {'private_config': this_pkg_path + '/config/private/uav_manager.yaml'},
                    {'public_config': this_pkg_path + '/config/public/uav_manager.yaml'},
                    {'platform_config': platform_config},
                    {'custom_config': custom_config},
                    {'world_config': world_config}
                    ],

                remappings=[
                    # subscribers
                    ("~/controller_diagnostics_in", "control_manager/controller_diagnostics"),
                    ("~/odometry_in", "estimation_manager/odom_main"),
                    ("~/odometry_diagnostics_in", "estimation_manager/diagnostics"),
                    ("~/control_manager_diagnostics_in", "control_manager/diagnostics"),
                    ("~/mass_estimate_in", "control_manager/mass_estimate"),
                    ("~/throttle_in", "control_manager/throttle"),
                    ("~/tracker_cmd_in", "control_manager/tracker_cmd"),
                    ("~/hw_api_status_in", "hw_api/status"),
                    ("~/hw_api_gnss_in", "hw_api/gnss"),
                    ("~/max_height_in", "estimation_manager/max_flight_z_agl"),
                    ("~/height_in", "estimation_manager/height_agl"),
                    ("~/gain_manager_diagnostics_in", "gain_manager/diagnostics"),
                    ("~/constraint_manager_diagnostics_in", "constraint_manager/diagnostics"),
                    ("~/hw_api_capabilities_in", "hw_api/capabilities"),
                    # publishers
                    ("~/diagnostics_out", "~/diagnostics"),
                    ("~/profiler", "profiler"),
                    # services in
                    ("~/takeoff_in", "~/takeoff"),
                    ("~/land_in", "~/land"),
                    ("~/land_home_in", "~/land_home"),
                    ("~/land_there_in", "~/land_there"),
                    ("~/midair_activation_in", "~/midair_activation"),
                    ("~/enable_min_height_check_in", "~/enable_min_height_check"),
                    # services out
                    ("~/takeoff_out", "control_manager/landoff_tracker/takeoff"),
                    ("~/land_out", "control_manager/landoff_tracker/land"),
                    ("~/eland_out", "control_manager/eland"),
                    ("~/ehover_out", "control_manager/ehover"),
                    ("~/switch_tracker_out", "control_manager/switch_tracker"),
                    ("~/switch_controller_out", "control_manager/switch_controller"),
                    ("~/toggle_control_output_out", "control_manager/toggle_output"),
                    ("~/emergency_reference_out", "control_manager/emergency_reference"),
                    ("~/enable_callbacks_out", "control_manager/enable_callbacks"),
                    ("~/arm_out", "control_manager/arm"),
                    ("~/pirouette_out", "control_manager/pirouette"),
                    ("~/set_odometry_callbacks_out", "estimation_manager/toggle_service_callbacks"),
                    ("~/ungrip_out", "gripper/ungrip"),
                    ("~/offboard_out", "hw_api/offboard"),
                ],
            )

        ],

    ))

    return ld
