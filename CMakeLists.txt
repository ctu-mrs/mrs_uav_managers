cmake_minimum_required(VERSION 3.5)
project(mrs_uav_managers)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_managers]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(MRS_ENABLE_TESTING)
  message(WARNING "Setting optimizaton levels for tests")
  add_definitions("-O0")
  add_definitions("-g")
endif()

if(COVERAGE)
  message(WARNING "building with --coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  mrs_msgs
  mrs_lib
  pluginlib
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  mrs_uav_hw_api
  visualization_msgs
  tf2_geometry_msgs
  Eigen3
  )

set(LIBRARIES
  MrsUavManagers_EstimationManager
  MrsUavManagers_TransformManager
  MrsUavManagers_ControlManager
  MrsUavManagers_NullTracker
  MrsUavManagers_UavManager
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  )

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

# Estimation Manager

add_library(MrsUavManagers_EstimationManager SHARED
  src/estimation_manager/estimation_manager.cpp
  src/estimation_manager/estimator.cpp
  src/estimation_manager/estimators/state_estimator.cpp
  src/estimation_manager/estimators/agl_estimator.cpp
  )

ament_target_dependencies(MrsUavManagers_EstimationManager
  ${DEPENDENCIES}
  )

rclcpp_components_register_nodes(MrsUavManagers_EstimationManager PLUGIN "mrs_uav_managers::estimation_manager::EstimationManager" EXECUTABLE MrsUavManagers_EstimationManager)

target_compile_definitions(MrsUavManagers_EstimationManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Transform Manager

add_library(MrsUavManagers_TransformManager SHARED
  src/transform_manager/transform_manager.cpp
  )

ament_target_dependencies(MrsUavManagers_TransformManager
  ${DEPENDENCIES}
  )

rclcpp_components_register_nodes(MrsUavManagers_TransformManager PLUGIN "mrs_uav_managers::transform_manager::TransformManager" EXECUTABLE MrsUavManagers_TransformManager)

target_compile_definitions(MrsUavManagers_TransformManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Control Manager

add_library(MrsUavManagers_ControlManager SHARED
  src/control_manager/control_manager.cpp
  src/control_manager/output_publisher.cpp
  src/control_manager/common/common.cpp
  )

ament_target_dependencies(MrsUavManagers_ControlManager
  ${DEPENDENCIES}
  )

rclcpp_components_register_nodes(MrsUavManagers_ControlManager PLUGIN "mrs_uav_managers::control_manager::ControlManager" EXECUTABLE MrsUavManagers_ControlManager)

target_compile_definitions(MrsUavManagers_ControlManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# UAV Manager

add_library(MrsUavManagers_UavManager SHARED
  src/uav_manager.cpp
  )

ament_target_dependencies(MrsUavManagers_UavManager
  ${DEPENDENCIES}
  )

rclcpp_components_register_nodes(MrsUavManagers_UavManager PLUGIN "mrs_uav_managers::uav_manager::UavManager" EXECUTABLE MrsUavManagers_UavManager)

target_compile_definitions(MrsUavManagers_UavManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Null Tracker

add_library(MrsUavManagers_NullTracker SHARED
  src/null_tracker.cpp
  )

ament_target_dependencies(MrsUavManagers_NullTracker
  ${DEPENDENCIES}
  )

target_link_libraries(MrsUavManagers_NullTracker
  ${MPC_CONTROLLER_SOLVER_BIN}
  )

target_compile_definitions(MrsUavManagers_NullTracker PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                       export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(mrs_uav_managers plugins.xml)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
  )

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(
  DIRECTORY include
  DESTINATION .
  )

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
  )

ament_export_include_directories(
  include
  )

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
  )

ament_export_dependencies(
  ${DEPENDENCIES}
  )

ament_package()
