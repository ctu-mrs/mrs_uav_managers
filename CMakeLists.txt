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
  tf
  cmake_modules
  nodelet
  mav_manager
  )

set(CMAKE_CXX_FLAGS "-std=c++11 -fno-diagnostics-color")

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs mrs_msgs mav_manager
  LIBRARIES TrackersManager
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

# Declare a C++ executable
add_library(TrackersManager src/trackers_manager.cpp)

add_dependencies(TrackersManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(TrackersManager
  ${catkin_LIBRARIES}
  )

# Null Tracker

add_library(NullTracker src/null_tracker.cpp)
add_dependencies(NullTracker
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
target_link_libraries(NullTracker ${catkin_LIBRARIES})
