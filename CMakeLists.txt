cmake_minimum_required(VERSION 3.5)
project(mrs_uav_managers)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(MRS_ENABLE_TESTING)
  message(WARNING "Setting optimizaton levels for tests")
  add_definitions("-O0")
  add_definitions("-g")
else()
  add_definitions("-O2")
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

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

set(LIBRARIES
  MrsUavManagers_EstimationManager
  )

# ROS Wrapper for the Multirotor UAV System + controllers

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

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include
  DESTINATION .
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()
