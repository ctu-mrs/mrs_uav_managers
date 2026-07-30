#!/usr/bin/env python3

import launch
import os

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
    namespace = 'diagnostics_manager'

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    # #} end of uav_name

    # #{ uav_type

    uav_type = LaunchConfiguration('uav_type')
    ld.add_action(DeclareLaunchArgument(
        'uav_type',
        default_value=os.getenv('UAV_TYPE', "x500"),
        description="The uav type used for selecting platform configuration.",
    ))
    # #} end of uav_type

    # #{ robot_type

    robot_type = LaunchConfiguration('robot_type')
    ld.add_action(DeclareLaunchArgument(
        'robot_type',
        default_value=os.getenv('ROBOT_TYPE', "multirotor"),
        description="The robot type used for selecting platform configuration.",
    ))

    # #} end of robot_type

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

    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
            )

    # #} end of custom_config

    # #{ platform_config

    # Declared for argument-compatibility with core.launch.py's uniform pass-through to every
    # manager launch file. Unlike its sibling managers, DiagnosticsManager does not read this
    # parameter (state_monitor.launch.py never had it either) -- it is computed here but not
    # forwarded to the node below.
    platform_config = LaunchConfiguration('platform_config')

    ld.add_action(DeclareLaunchArgument(
        'platform_config',
        default_value="",
        description="Path to the platform configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    platform_config = IfElseSubstitution(
            condition=PythonExpression(['"', platform_config, '" != "" and ', 'not "', platform_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), platform_config]),
            else_value=platform_config
            )

    # #} end of platform_config

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', "false"),
        description="Should the node subscribe to sim time?",
    ))

    # #} end of use_sim_time

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    # #} end of log_level

    # #{ diagnostics manager node

    diagnostics_manager_node = ComposableNode(

        package=pkg_name,
        plugin='mrs_uav_managers::diagnostics_manager::DiagnosticsManager',
        namespace=uav_name,
        name='diagnostics_manager',

        parameters=[
            {"custom_config": custom_config},
            {"robot_name": uav_name},
            {"robot_type": robot_type},
            {"uav_type": uav_type},
            {"use_sim_time": use_sim_time},
            {'private_config': this_pkg_path + '/config/private/diagnostics_manager/diagnostics_manager.yaml'},
            {'public_config': this_pkg_path + '/config/public/diagnostics_manager/diagnostics_manager.yaml'},
            {'private_sensor_handlers': this_pkg_path + '/config/private/diagnostics_sensor_handlers.yaml'},
            {'public_sensor_handlers': this_pkg_path + '/config/public/diagnostics_manager/diagnostics_sensor_handlers.yaml'},
            {'preflight_check_config': this_pkg_path + '/config/public/diagnostics_manager/preflight_check.yaml'},
        ],

        remappings=[
            # publishers
            ("~/collision_avoidance_info_out", "~/collision_avoidance_info"),
            ("~/control_info_out", "~/control_info"),
            ("~/general_robot_info_out", "~/general_robot_info"),
            ("~/state_estimation_info_out", "~/state_estimation_info"),
            ("~/system_health_info_out", "~/system_health_info"),
            ("~/uav_info_out", "~/uav_info"),
            ("~/uav_state_out", "~/uav_state"),

            # subscribers
            ("~/battery_state_in", "hw_api/battery_state"),
            ("~/control_manager_diagnostics_in", "control_manager/diagnostics"),
            ("~/control_manager_heading_in", "control_manager/heading"),
            ("~/control_manager_thrust_in", "control_manager/thrust"),
            ("~/constraint_manager_diagnostics_in", "constraint_manager/diagnostics"),
            ("~/estimation_diagnostics_in", "estimation_manager/diagnostics"),
            ("~/gain_manager_diagnostics_in", "gain_manager/diagnostics"),
            ("~/hw_api_gnss_in", "hw_api/gnss"),
            ("~/hw_api_mag_heading_in", "hw_api/mag_heading"),
            ("~/hw_api_rc_rssi_in", "hw_api/rc_rssi"),
            ("~/hw_api_status_in", "hw_api/status"),
            ("~/hw_api_odometry_in", "hw_api/odometry"),
            ("~/estimator_uav_state_in", "estimation_manager/uav_state"),
            ("~/mass_estimate_in", "control_manager/mass_estimate"),
            ("~/mass_nominal_in", "control_manager/mass_nominal"),
            ("~/mpc_tracker_diagnostics_in", "control_manager/mpc_tracker/diagnostics"),
            ("~/tracker_cmd_in", "control_manager/tracker_cmd"),
            ("~/hw_api_capabilities_in", "hw_api/capabilities"),
            ("~/hw_api_distance_sensor_in", "hw_api/distance_sensor"),
            ("~/hw_api_imu_in", "hw_api/imu"),
            ("~/safety_area_manager_diagnostics_in", "safety_area_manager/diagnostics"),

            # errorgraph
            ("~/errors_in", "errors"),
            ("~/errors_out", "root_errors"),
            ("~/errors", "errors"),
        ],

        extra_arguments=[
            {'use_intra_process_comms': True}
        ],
    )

    load_into_existing = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[diagnostics_manager_node],
        condition=UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # #} end of diagnostics manager node

    # #{ standalone container

    standalone_container = ComposableNodeContainer(
        namespace=uav_name,
        name=namespace+'_container',
        package='rclcpp_components',
        executable='component_container_events_cbg',
        output="screen",
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[diagnostics_manager_node],
        parameters=[
            {'thread_num': os.cpu_count()},
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(standalone)
    )

    ld.add_action(standalone_container)

    # #} end of own container

    return ld
