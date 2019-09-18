cmake_minimum_required(VERSION 2.8.3)
project(mrs_uav_manager)

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
  cmake_modules
  nodelet
  )

find_package(mavros_msgs 0.30.0 EXACT REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES GainManager ConstraintManager ControlManager UavManager
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs mrs_msgs mrs_lib 
  DEPENDS mavros_msgs
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
  ${mavros_msgs_INCLUDE_DIRS}
  )

# ControlManager

add_library(ControlManager src/control_manager.cpp)

add_dependencies(ControlManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(ControlManager
  ${catkin_LIBRARIES}
  ${mavros_msgs_LIBRARIES}
  )

# NullTracker

add_library(NullTracker src/null_tracker.cpp)

add_dependencies(NullTracker
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(NullTracker 
  ${catkin_LIBRARIES}
  ${mavros_msgs_LIBRARIES}
  )

# UavManager

add_library(UavManager src/uav_manager.cpp)

add_dependencies(UavManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UavManager
  ${catkin_LIBRARIES}
  ${mavros_msgs_LIBRARIES}
  )

# GainManager

add_library(GainManager src/gain_manager.cpp)

add_dependencies(GainManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(GainManager
  ${catkin_LIBRARIES}
  ${mavros_msgs_LIBRARIES}
  )

# ConstraintManager

add_library(ConstraintManager src/constraint_manager.cpp)

add_dependencies(ConstraintManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(ConstraintManager
  ${catkin_LIBRARIES}
  ${mavros_msgs_LIBRARIES}
  )
