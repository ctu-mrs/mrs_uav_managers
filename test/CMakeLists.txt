find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

function(add_ros_isolated_launch_test path)
  set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
  add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
endfunction()

set(MRS_UAV_MANAGERS_INTERNAL_TEST_INCLUDE_DIRS
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

function(add_mrs_uav_managers_test_target test_name)
  set(target_name "test_${test_name}")
  if(TARGET "${target_name}")
    message(FATAL_ERROR "Target for test name '${test_name}' already exists!")
  endif()

  add_executable(${target_name}
    test.cpp
  )

  target_include_directories(${target_name}
    PRIVATE
      ${MRS_UAV_MANAGERS_INTERNAL_TEST_INCLUDE_DIRS}
  )

  target_link_libraries(${target_name}
    PRIVATE
      rclcpp::rclcpp
      mrs_lib::mrs_lib
      mrs_uav_testing::mrs_uav_testing
      ${mrs_msgs_TARGETS}
  )

  install(
    TARGETS ${target_name}
    DESTINATION lib/${PROJECT_NAME}
  )

  target_compile_definitions(${target_name} PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

  add_ros_isolated_launch_test(test.py TIMEOUT 300)
endfunction()

add_subdirectory(constraint_manager)
add_subdirectory(control_manager)
add_subdirectory(estimation_manager)
add_subdirectory(gain_manager)
add_subdirectory(uav_manager)
