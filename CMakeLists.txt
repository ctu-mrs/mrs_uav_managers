cmake_minimum_required(VERSION 3.28)
project(mrs_uav_managers)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_managers]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options("-Wall" "-Wextra")
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
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
  mrs_uav_testing
  backward_ros
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(LIBRARIES
  MrsUavManagers_EstimationManager
  MrsUavManagers_TransformManager
  MrsUavManagers_ControlManager
  MrsUavManagers_NullTracker
  MrsUavManagers_UavManager
  MrsUavManagers_ConstraintManager
  MrsUavManagers_GainManager
  MrsUavManagers_SafetyAreaManager
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

target_include_directories(MrsUavManagers_EstimationManager
  PRIVATE
    "include"
)

target_link_libraries(MrsUavManagers_EstimationManager
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    mrs_lib::mrs_lib
    pluginlib::pluginlib
    mrs_uav_hw_api::mrs_uav_hw_api
    Eigen3::Eigen
    ${mrs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

rclcpp_components_register_nodes(MrsUavManagers_EstimationManager "mrs_uav_managers::estimation_manager::EstimationManager")

target_compile_definitions(MrsUavManagers_EstimationManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Transform Manager

add_library(MrsUavManagers_TransformManager SHARED
  src/transform_manager/transform_manager.cpp
)

target_include_directories(MrsUavManagers_TransformManager
  PRIVATE
    "include"
)

target_link_libraries(MrsUavManagers_TransformManager
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    mrs_lib::mrs_lib
    pluginlib::pluginlib
    mrs_uav_hw_api::mrs_uav_hw_api
    Eigen3::Eigen
    ${mrs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

rclcpp_components_register_nodes(MrsUavManagers_TransformManager "mrs_uav_managers::transform_manager::TransformManager")

target_compile_definitions(MrsUavManagers_TransformManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Control Manager

add_library(MrsUavManagers_ControlManager SHARED
  src/control_manager/control_manager.cpp
  src/control_manager/output_publisher.cpp
  src/control_manager/common/common.cpp
)

target_include_directories(MrsUavManagers_ControlManager
  PRIVATE
    "include"
)

target_link_libraries(MrsUavManagers_ControlManager
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    mrs_lib::mrs_lib
    pluginlib::pluginlib
    mrs_uav_hw_api::mrs_uav_hw_api
    Eigen3::Eigen
    ${mrs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

rclcpp_components_register_nodes(MrsUavManagers_ControlManager "mrs_uav_managers::control_manager::ControlManager")

target_compile_definitions(MrsUavManagers_ControlManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER} COVERAGE=${COVERAGE})

# Safety Area Manager

add_library(MrsUavManagers_SafetyAreaManager SHARED
  src/safety_area_manager.cpp
)

target_link_libraries(MrsUavManagers_SafetyAreaManager
  PRIVATE
    mrs_uav_managers::mrs_uav_managers
    rclcpp::rclcpp
    rclcpp_components::component
    mrs_lib::mrs_lib
    pluginlib::pluginlib
    mrs_uav_hw_api::mrs_uav_hw_api
    Eigen3::Eigen
    ${mrs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

rclcpp_components_register_nodes(MrsUavManagers_SafetyAreaManager "mrs_uav_managers::safety_area_manager::SafetyAreaManager")

target_compile_definitions(MrsUavManagers_SafetyAreaManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# UAV Manager

add_library(MrsUavManagers_UavManager SHARED
  src/uav_manager.cpp
)

target_include_directories(MrsUavManagers_UavManager
  PRIVATE
    "include"
)

target_link_libraries(MrsUavManagers_UavManager
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    mrs_lib::mrs_lib
    pluginlib::pluginlib
    mrs_uav_hw_api::mrs_uav_hw_api
    Eigen3::Eigen
    ${mrs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

rclcpp_components_register_nodes(MrsUavManagers_UavManager "mrs_uav_managers::uav_manager::UavManager")

target_compile_definitions(MrsUavManagers_UavManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Constraint Manager

add_library(MrsUavManagers_ConstraintManager SHARED
  src/constraint_manager.cpp
)

target_include_directories(MrsUavManagers_ConstraintManager
  PRIVATE
    "include"
)

target_link_libraries(MrsUavManagers_ConstraintManager
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    mrs_lib::mrs_lib
    pluginlib::pluginlib
    mrs_uav_hw_api::mrs_uav_hw_api
    Eigen3::Eigen
    ${mrs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

rclcpp_components_register_nodes(MrsUavManagers_ConstraintManager "mrs_uav_managers::constraint_manager::ConstraintManager")

target_compile_definitions(MrsUavManagers_ConstraintManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Gain Manager

add_library(MrsUavManagers_GainManager SHARED
  src/gain_manager.cpp
)

target_include_directories(MrsUavManagers_GainManager
  PRIVATE
    "include"
)

target_link_libraries(MrsUavManagers_GainManager
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    mrs_lib::mrs_lib
    pluginlib::pluginlib
    mrs_uav_hw_api::mrs_uav_hw_api
    Eigen3::Eigen
    ${mrs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

rclcpp_components_register_nodes(MrsUavManagers_GainManager "mrs_uav_managers::gain_manager::GainManager")

target_compile_definitions(MrsUavManagers_GainManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Null Tracker

add_library(MrsUavManagers_NullTracker SHARED
  src/null_tracker.cpp
)

target_include_directories(MrsUavManagers_NullTracker
  PRIVATE
    "include"
)

target_link_libraries(MrsUavManagers_NullTracker
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    mrs_lib::mrs_lib
    pluginlib::pluginlib
    mrs_uav_hw_api::mrs_uav_hw_api
    Eigen3::Eigen
    ${mrs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

target_compile_definitions(MrsUavManagers_NullTracker PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Public interface of this library

add_library(MrsUavManagers_MrsUavManagers INTERFACE)
add_library(mrs_uav_managers::mrs_uav_managers ALIAS MrsUavManagers_MrsUavManagers)
set_target_properties(MrsUavManagers_MrsUavManagers
  PROPERTIES
    EXPORT_NAME mrs_uav_managers
    OUTPUT_NAME mrs_uav_managers
)

target_sources(MrsUavManagers_MrsUavManagers
  INTERFACE FILE_SET HEADERS BASE_DIRS "include"
  FILES
    # include/control_manager
    "include/control_manager/output_publisher.h"
    # include/mrs_uav_managers
    "include/mrs_uav_managers/agl_estimator.h"
    "include/mrs_uav_managers/controller.h"
    "include/mrs_uav_managers/state_estimator.h"
    "include/mrs_uav_managers/tracker.h"
    # include/mrs_uav_managers/control_manager
    "include/mrs_uav_managers/control_manager/common_handlers.h"
    "include/mrs_uav_managers/control_manager/common.h"
    "include/mrs_uav_managers/control_manager/private_handlers.h"
    # include/mrs_uav_managers/estimation_manager
    "include/mrs_uav_managers/estimation_manager/common_handlers.h"
    "include/mrs_uav_managers/estimation_manager/estimator.h"
    "include/mrs_uav_managers/estimation_manager/private_handlers.h"
    "include/mrs_uav_managers/estimation_manager/support.h"
    "include/mrs_uav_managers/estimation_manager/types.h"
    # include/transform_manager
    "include/transform_manager/tf_mapping_origin.h"
    "include/transform_manager/tf_source.h"
)

target_link_libraries(MrsUavManagers_MrsUavManagers
  INTERFACE
    ${LIBRARIES}
    rclcpp::rclcpp
    mrs_lib::mrs_lib
    pluginlib::pluginlib
    Eigen3::Eigen
    ${mrs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs}
    ${sensor_msgs}
    ${mrs_uav_hw_api}
    ${visualization_msgs}
    ${tf2_geometry_msgs_TARGETS}
)

## --------------------------------------------------------------
## |                       export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(mrs_uav_managers plugins.xml)

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS
    MrsUavManagers_MrsUavManagers
    ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  FILE_SET HEADERS DESTINATION "include/${PROJECT_NAME}"
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
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_libraries(
  ${LIBRARIES}
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
