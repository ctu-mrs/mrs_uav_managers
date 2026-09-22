#pragma once
#include <cstdint>
#include <mrs_msgs/msg/sensor_status.hpp>

// macro variables for the enum definition
#undef X_ENUM_NAME
#undef X_ENUM_BASE_TYPE
#undef X_ENUM_SEQ

#define X_ENUM_NAME sensor_type_t
#define X_ENUM_BASE_TYPE uint8_t
// clang-format off
#define X_ENUM_SEQ  \
  (AUTOPILOT)       \
  (RANGEFINDER)     \
  (GNSS)            \
  (IMU)             \
  (BAROMETER)       \
  (MAGNETOMETER)    \
  (LIDAR)           \
  (CAMERA)          \
  (REMOTE_CONTROLLER)
// clang-format on

// optional macro variables for enum to ROS message conversions
#undef X_ENUM_MSG_TYPE
#undef X_ENUM_MSG_MEMBER
#undef X_ENUM_MSG_PREFIX

#define X_ENUM_MSG_TYPE mrs_msgs::msg::SensorStatus
#define X_ENUM_MSG_MEMBER type
#define X_ENUM_MSG_PREFIX TYPE_

namespace mrs_uav_managers::diagnostics_manager
{

#include <mrs_uav_managers/diagnostics_manager/enums/helpers/enum_macros.hpp>

// generate the enum and the to_string() conversion
DEFINE_ENUM_WITH_CONVERSIONS(X_ENUM_NAME, X_ENUM_BASE_TYPE, X_ENUM_SEQ)

// generate the to_ros()/from_ros() conversions
DEFINE_ENUM_MSG_CONVERSIONS(X_ENUM_NAME, X_ENUM_MSG_TYPE, X_ENUM_MSG_MEMBER, X_ENUM_MSG_PREFIX, X_ENUM_SEQ)

// generate the from_string() conversion (config type: string -> sensor_type_t)
DEFINE_ENUM_STRING_PARSE(X_ENUM_NAME, X_ENUM_SEQ)

} // namespace mrs_uav_managers::diagnostics_manager
