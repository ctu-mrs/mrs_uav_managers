#!/usr/bin/env python3

import launch
import os
import sys

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
    namespace='estimation_manager'

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    # #} end of custom_config

    # #{ standalone

    standalone = LaunchConfiguration('standalone')

    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='true',
        description='Whether to start a as a standalone or load into an existing container.'
    )

    ld.add_action(declare_standalone)

    # #} end of standalone

    # #{ container_name

    container_name = LaunchConfiguration('container_name')

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    )

    ld.add_action(declare_container_name)

    # #} end of container_name

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

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', "false"),
        description="Should the node subscribe to sim time?",
    ))

    # #} end of custom_config

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    # #} end of log_level

    # #{ estimation manager node

    estimation_manager_node = ComposableNode(

        package=pkg_name,
        plugin='mrs_uav_managers::estimation_manager::EstimationManager',
        namespace=uav_name,
        name='estimation_manager',
        parameters=[
            {"uav_name": uav_name},
            {"topic_prefix": ["/", uav_name]},
            {"enable_profiler": False},
            {"use_sim_time": use_sim_time},
            {'private_config': this_pkg_path + '/config/private/estimation_manager/estimation_manager.yaml'},
            {'public_config': this_pkg_path + '/config/public/estimation_manager/estimation_manager.yaml'},
            {'uav_manager_config': this_pkg_path + '/config/public/uav_manager.yaml'},
            {'estimators_config': this_pkg_path + '/config/private/estimators.yaml'},
            {'active_estimators_config': this_pkg_path + '/config/public/active_estimators.yaml'},
            {'platform_config': platform_config},
            {'custom_config': custom_config},
            {'world_config': world_config},
        ],

        remappings=[
            # subscribers
            ("~/control_input_in", "control_manager/estimator_input"),
            ("~/imu_in", "hw_api/imu"),
            ("~/hw_api_capabilities_in", "hw_api/capabilities"),
            ("~/control_manager_diagnostics_in", "control_manager/diagnostics"),
            ("~/control_reference_in", "control_manager/control_reference"),
            ("~/controller_diagnostics_in", "control_manager/controller_diagnostics"),
            # publishers
            ("~/odom_main_out", "~/odom_main"),
            ("~/innovation_out", "~/innovation"),
            ("~/uav_state_out", "~/uav_state"),
            ("~/diagnostics_out", "~/diagnostics"),
            ("~/max_flight_z_agl_out", "~/max_flight_z_agl"),
            ("~/height_agl_out", "~/height_agl"),
            # services in
            ("~/change_estimator_in", "~/change_estimator"),
            ("~/set_world_origin_in", "~/set_world_origin"),
            ("~/reset_estimator_in", "~/reset_estimator"),
            ("~/toggle_service_callbacks_in", "~/toggle_service_callbacks"),
            # services out
            ("~/failsafe_out", "control_manager/failsafe"),
            ("~/set_world_origin_out", "transform_manager/set_world_origin"),
            ("~/update_world_origin_out", "safety_area_manager/update_world_origin"),
        ],
    )

    load_into_existing = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[estimation_manager_node],
        condition=UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # #} end of estimation manager node

    # #{ standalone container

    standalone_container = ComposableNodeContainer(
        namespace=uav_name,
        name=namespace+'_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output="screen",
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],
        # prefix=["valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --log-file=/tmp/valgrind-out.txt"],
        composable_node_descriptions=[estimation_manager_node],
        parameters=[
            {'use_intra_process_comms': True},
            {'thread_num': os.cpu_count()},
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(standalone)
    )

    ld.add_action(standalone_container)

    # #} end of own container

    return ld
