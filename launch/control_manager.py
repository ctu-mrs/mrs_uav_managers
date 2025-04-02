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
                    ("~/odometry_in", "estimation_manager/odom_main"),
                    ("~/uav_state_in", "estimation_manager/uav_state"),
                    ("~/odometry_innovation_in", "estimation_manager/innovation"),
                    ("~/local_odometry_in", "hw_api/odometry"),
                    ("~/gnss_in", "hw_api/gnss"),
                    ("~/max_z_in", "estimation_manager/max_flight_z_agl"),
                    ("~/joystick_in", "joy"),
                    ("~/bumper_sectors_in", "bumper/obstacle_sectors"),
                    ("~/hw_api_rc_in", "hw_api/rc_channels"),
                    ("~/reference_in", "~/reference"),
                    ("~/velocity_reference_in", "~/velocity_reference"),
                    ("~/trajectory_reference_in", "~/trajectory_reference"),
                    ("~/goto_in", "~/goto"),
                    ("~/goto_fcu_in", "~/goto_fcu"),
                    ("~/goto_relative_in", "~/goto_relative"),
                    ("~/goto_altitude_in", "~/goto_altitude"),
                    ("~/set_heading_in", "~/set_heading"),
                    ("~/set_heading_relative_in", "~/set_heading_relative"),
                    ("~/transform_reference_in", "~/transform_reference"),
                    ("~/transform_reference_array_in", "~/transform_reference_array"),
                    ("~/transform_pose_in", "~/transform_pose"),
                    ("~/transform_vector3_in", "~/transform_vector3"),
                    ("~/validate_reference_in", "~/validate_reference"),
                    ("~/validate_reference_2d_in", "~/validate_reference_2d"),
                    ("~/validate_reference_array_in", "~/validate_reference_array"),
                    ("~/set_min_z_in", "~/set_min_z"),
                    # publishers
                    ("~/hw_api_actuator_cmd_out", "hw_api/actuator_cmd"),
                    ("~/hw_api_control_group_cmd_out", "hw_api/control_group_cmd"),
                    ("~/hw_api_attitude_rate_cmd_out", "hw_api/attitude_rate_cmd"),
                    ("~/hw_api_attitude_cmd_out", "hw_api/attitude_cmd"),
                    ("~/hw_api_acceleration_hdg_rate_cmd_out", "hw_api/acceleration_hdg_rate_cmd"),
                    ("~/hw_api_acceleration_hdg_cmd_out", "hw_api/acceleration_hdg_cmd"),
                    ("~/hw_api_velocity_hdg_rate_cmd_out", "hw_api/velocity_hdg_rate_cmd"),
                    ("~/hw_api_velocity_hdg_cmd_out", "hw_api/velocity_hdg_cmd"),
                    ("~/hw_api_position_cmd_out", "hw_api/position_cmd"),
                    ("~/control_reference_out", "~/control_reference"),
                    ("~/tracker_cmd_out", "~/tracker_cmd"),
                    ("~/estimator_input_out", "~/estimator_input"),
                    ("~/control_error_out", "~/control_error"),
                    ("~/tilt_error_out", "~/tilt_error"),
                    ("~/thrust_force_out", "~/thrust_force"),
                    ("~/throttle_out", "~/throttle"),
                    ("~/thrust_out", "~/thrust"),
                    ("~/offboard_on_out", "~/offboard_on"),
                    ("~/mass_estimate_out", "~/mass_estimate"),
                    ("~/mass_nominal_out", "~/mass_nominal"),
                    ("~/safety_area_markers_out", "~/safety_area_markers"),
                    ("~/safety_area_coordinates_markers_out", "~/safety_area_coordinates_markers"),
                    ("~/disturbances_markers_out", "~/disturbances_markers"),
                    ("~/diagnostics_out", "~/diagnostics"),
                    ("~/bumper_status_out", "~/bumper_status"),
                    ("~/current_constraints_out", "~/current_constraints"),
                    ("~/heading_out", "~/heading"),
                    ("~/speed_out", "~/speed"),
                    ("~/trajectory_original/poses_out", "~/trajectory_original/poses"),
                    ("~/trajectory_original/markers_out", "~/trajectory_original/markers"),
                    ("~/controller_diagnostics_out", "~/controller_diagnostics"),
                    ("~/profiler", "profiler"),
                    # services
                    ("~/switch_tracker_in", "~/switch_tracker"),
                    ("~/switch_controller_in", "~/switch_controller"),
                    ("~/tracker_reset_static_in", "~/tracker_reset_static"),
                    ("~/hover_in", "~/hover"),
                    ("~/ehover_in", "~/ehover"),
                    ("~/start_trajectory_tracking_in", "~/start_trajectory_tracking"),
                    ("~/stop_trajectory_tracking_in", "~/stop_trajectory_tracking"),
                    ("~/resume_trajectory_tracking_in", "~/resume_trajectory_tracking"),
                    ("~/goto_trajectory_start_in", "~/goto_trajectory_start"),
                    ("~/toggle_output_in", "~/toggle_output"),
                    ("~/emergency_reference_in", "~/emergency_reference"),
                    ("~/enable_callbacks_in", "~/enable_callbacks"),
                    ("~/set_gains_out", "gain_manager/set_gains"),
                    ("~/set_constraints_in", "~/set_constraints"),
                    ("~/use_joystick_in", "~/use_joystick"),
                    ("~/hw_api_arming_out", "hw_api/arming"),
                    ("~/arm_in", "~/arm"),
                    ("~/eland_in", "~/eland"),
                    ("~/failsafe_in", "~/failsafe"),
                    ("~/failsafe_escalating_in", "~/failsafe_escalating"),
                    ("~/eland_out", "~/landoff_tracker/eland"),
                    ("~/land_out", "~/landoff_tracker/land"),
                    ("~/pirouette_in", "~/pirouette"),
                    ("~/bumper_in", "~/bumper"),
                    ("~/bumper_repulsion_in", "~/bumper_repulsion"),
                    ("~/bumper_set_params_in", "~/bumper_set_params"),
                    ("~/get_min_z_in", "~/get_min_z"),
                    ("~/set_odometry_callbacks_out", "estimation_manager/toggle_service_callbacks"),
                    ("~/ungrip_out", "gripper/ungrip"),
                    ("~/use_safety_area_in", "~/use_safety_area"),
                    ("~/parachute_out", "parachute_driver/fire_parachute"),
                    ("~/parachute_in", "~/parachute"),
                ],
            )

        ],

    ))

    return ld
