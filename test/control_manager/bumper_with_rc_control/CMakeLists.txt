get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

add_executable(test_${TEST_NAME}
  test.cpp
  )

ament_target_dependencies(test_${TEST_NAME}
  rclcpp
  mrs_lib
  mrs_msgs
  mrs_uav_testing
  backward_ros
  )

install(TARGETS
  test_${TEST_NAME}
  DESTINATION lib/${PROJECT_NAME}
  )

target_compile_definitions(test_${TEST_NAME} PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

add_ros_isolated_launch_test(test.py TIMEOUT 300)
