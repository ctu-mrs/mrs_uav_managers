cmake_minimum_required(VERSION 2.8.3)
project(mrs_mav_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  nav_msgs
  mrs_msgs
  mrs_lib
  tf
  cmake_modules
  nodelet
  )

set(CMAKE_CXX_FLAGS "-std=c++11 -fno-diagnostics-color")

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs mrs_msgs mrs_lib
  LIBRARIES ControlManager MavManager
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

# ControlManager

add_library(ControlManager src/control_manager.cpp)

add_dependencies(ControlManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(ControlManager
  ${catkin_LIBRARIES}
  )

# NullTracker

add_library(NullTracker src/null_tracker.cpp)

add_dependencies(NullTracker
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(NullTracker ${catkin_LIBRARIES})

# MavManager

add_library(MavManager src/mav_manager.cpp)

add_dependencies(MavManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MavManager
  ${catkin_LIBRARIES}
  )
