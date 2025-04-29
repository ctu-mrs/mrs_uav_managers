find_package(rostest REQUIRED)
find_package(mrs_uav_testing REQUIRED)

add_subdirectory(control_manager)

add_subdirectory(uav_manager)

add_subdirectory(estimation_manager)

add_subdirectory(gain_manager)

add_subdirectory(constraint_manager)
