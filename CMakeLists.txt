cmake_minimum_required(VERSION 3.1.2)
project(mrs_uav_managers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  mrs_msgs
  mrs_lib
  cmake_modules
  nodelet
  tf2_geometry_msgs
  tf2
  tf2_ros
  )

# find_package(mavros_msgs 1.4.0 EXACT REQUIRED)
find_package(mavros_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(LIBRARIES
  GainManager ConstraintManager ControlManager UavManager TfManager NullTracker
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs mrs_msgs mrs_lib tf2 tf2_ros tf2_geometry_msgs
  DEPENDS mavros_msgs
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
  )

# ControlManager

add_library(ControlManager
  src/control_manager.cpp
  )

add_dependencies(ControlManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(ControlManager
  ${catkin_LIBRARIES}
  ${mavros_msgs_LIBRARIES}
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
  ${mavros_msgs_LIBRARIES}
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
  ${mavros_msgs_LIBRARIES}
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
  ${mavros_msgs_LIBRARIES}
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
  ${mavros_msgs_LIBRARIES}
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

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
