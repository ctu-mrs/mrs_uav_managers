cmake_minimum_required(VERSION 3.5)
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
  mrs_uav_testing
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)

if(COVERAGE)
  message(WARNING "building with --coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

set(LIBRARIES
  MrsUavManagers_GainManager
  MrsUavManagers_ConstraintManager
  MrsUavManagers_ControlManager
  MrsUavManagers_UavManager
  MrsUavManagers_EstimationManager
  MrsUavManagers_TransformManager
  MrsUavManagers_NullTracker
  )

catkin_package(
  INCLUDE_DIRS include test/include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  include test/include
  ${catkin_INCLUDE_DIRS}
  )

# MrsUavManagers_ControlManager

add_library(MrsUavManagers_ControlManager
  src/control_manager/control_manager.cpp
  src/control_manager/output_publisher.cpp
  src/control_manager/common/common.cpp
  )

add_dependencies(MrsUavManagers_ControlManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsUavManagers_ControlManager
  ${catkin_LIBRARIES}
  )

# MrsUavManagers_NullTracker

add_library(MrsUavManagers_NullTracker
  src/null_tracker.cpp
  )

add_dependencies(MrsUavManagers_NullTracker
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(MrsUavManagers_NullTracker
  ${catkin_LIBRARIES}
  )

# MrsUavManagers_UavManager

add_library(MrsUavManagers_UavManager
  src/uav_manager.cpp
  )

add_dependencies(MrsUavManagers_UavManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsUavManagers_UavManager
  ${catkin_LIBRARIES}
  )

# MrsUavManagers_GainManager

add_library(MrsUavManagers_GainManager
  src/gain_manager.cpp
  )

add_dependencies(MrsUavManagers_GainManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsUavManagers_GainManager
  ${catkin_LIBRARIES}
  )

# MrsUavManagers_ConstraintManager

add_library(MrsUavManagers_ConstraintManager
  src/constraint_manager.cpp
  )

add_dependencies(MrsUavManagers_ConstraintManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsUavManagers_ConstraintManager
  ${catkin_LIBRARIES}
  )

# MrsUavManagers_EstimationManager
add_library(MrsUavManagers_EstimationManager
  src/estimation_manager/estimation_manager.cpp
  src/estimation_manager/estimator.cpp
  src/estimation_manager/estimators/state_estimator.cpp
  src/estimation_manager/estimators/agl_estimator.cpp
  )

add_dependencies(MrsUavManagers_EstimationManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsUavManagers_EstimationManager
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# MrsUavManagers_TransformManager
add_library(MrsUavManagers_TransformManager
  src/transform_manager/transform_manager.cpp
  )

add_dependencies(MrsUavManagers_TransformManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsUavManagers_TransformManager
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(CATKIN_ENABLE_TESTING AND MRS_ENABLE_TESTING)

  message(WARNING "Testing enabled.")

  add_subdirectory(test)

endif()

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

install(FILES nodelets.xml plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
