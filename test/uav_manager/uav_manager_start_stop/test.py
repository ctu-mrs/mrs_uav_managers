import time
import unittest
import os
import sys

import launch
import launch_ros
import launch_testing.actions
import launch_testing.asserts
from launch.actions import IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
import rclpy
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool

def generate_test_description():

    SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')

    ld = launch.LaunchDescription()

    launch_file_path = os.path.abspath(__file__)
    launch_dir = os.path.dirname(launch_file_path)

    test_name = os.path.basename(launch_dir)

    uav_name="uav1"

    world_config=get_package_share_directory("mrs_uav_testing")+"/config/default_world_config.yaml",
    platform_config=get_package_share_directory("mrs_multirotor_simulator")+"/config/mrs_uav_system/x500.yaml",

    # ld.add_action(
    #         launch_ros.actions.Node(
    #             package='rmw_zenoh_cpp',
    #             namespace='',
    #             executable='rmw_zenohd',
    #             name='zenoh_router',
    #         )
    #     )

    ld.add_action(
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('mrs_multirotor_simulator'),
                            'launch',
                            'hw_api.launch.py'
                        ])
                    ]),
                )
            ]
        )
    )

    ld.add_action(
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('mrs_uav_managers'),
                            'launch',
                            'uav_manager.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'uav_name': uav_name,
                        'platform_config': platform_config,
                        'world_config': world_config,
                    }.items()
                )
            ]
        )
    )

    # starts the integration interactor
    ld.add_action(
            # Nodes under test
            launch_ros.actions.Node(
                package='mrs_uav_managers',
                namespace='',
                executable='test_'+test_name,
                name='test_'+test_name,
                output="screen",
                parameters=[
                        {'test_name': test_name},
                ],
            )
        )

    # starts the python test part down below
    ld.add_action(
        launch.actions.TimerAction(
            period=1.0, actions=[launch_testing.actions.ReadyToTest()]),
        )

    return ld

# #{ class PublisherHandlerTest(unittest.TestCase)

class PublisherHandlerTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('integration_test_handler')

    def tearDown(self):
        self.node.destroy_node()

    def test_interactor(self, proc_output, timeout=120):

        """Check whether pose messages published"""

        test_result = []

        sub = self.node.create_subscription(
                Bool, '/test_result',
                lambda msg: test_result.append(msg), 100)
        try:

            end_time = time.time() + timeout

            while time.time() < end_time:

                if len(test_result) > 0:
                    break

                rclpy.spin_once(self.node, timeout_sec=1)

            time.sleep(2.0)

            # check if we have the result
            self.assertTrue(len(test_result) > 0)

            # check if the result is true
            self.assertTrue(test_result[0].data)

        finally:
            self.node.destroy_subscription(sub)

# #} end of 

# #{ Post-shutdown tests

# @launch_testing.post_shutdown_test()
# class PublisherHandlerTestShutdown(unittest.TestCase):
#     def test_exit_codes(self, proc_info):
#         """Check if the processes exited normally."""
#         launch_testing.asserts.assertExitCodes(proc_info)

# #} end of Post-shutdown tests
