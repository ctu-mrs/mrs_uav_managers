cmake_minimum_required(VERSION 3.1.2)
project(mrs_uav_managers)

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  rospy
  nodelet
  geometry_msgs
  std_msgs
  nav_msgs
  mrs_msgs
  visualization_msgs
  mrs_lib
  tf2
  tf2_ros
  tf2_geometry_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(LIBRARIES
  GainManager ConstraintManager ControlManager UavManager TfManager EstimationManager TransformManager NullTracker
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

# ControlManager

add_library(ControlManager
  src/control_manager/control_manager.cpp
  src/control_manager/output_publisher.cpp
  src/control_manager/common/common.cpp
  )

add_dependencies(ControlManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(ControlManager
  ${catkin_LIBRARIES}
  )

# NullTracker

add_library(NullTracker
  src/null_tracker.cpp
  )

add_dependencies(NullTracker
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(NullTracker
  ${catkin_LIBRARIES}
  )

# UavManager

add_library(UavManager
  src/uav_manager.cpp
  )

add_dependencies(UavManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UavManager
  ${catkin_LIBRARIES}
  )

# GainManager

add_library(GainManager
  src/gain_manager.cpp
  )

add_dependencies(GainManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(GainManager
  ${catkin_LIBRARIES}
  )

# ConstraintManager

add_library(ConstraintManager
  src/constraint_manager.cpp
  )

add_dependencies(ConstraintManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(ConstraintManager
  ${catkin_LIBRARIES}
  )

# TfManager
add_library(TfManager
  src/tf_manager.cpp
  )

add_dependencies(TfManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(TfManager
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# EstimationManager
add_library(EstimationManager
  src/estimation_manager/estimation_manager.cpp
  src/estimation_manager/estimator.cpp
  src/estimation_manager/estimators/state_estimator.cpp
  src/estimation_manager/estimators/agl_estimator.cpp
  )

add_dependencies(EstimationManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(EstimationManager
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# TransformManager
add_library(TransformManager
  src/transform_manager/transform_manager.cpp
  )

add_dependencies(TransformManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(TransformManager
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/mrs_uav_managers/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES plugins.xml null_tracker.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
