#pragma once
#include <cstdint>
#include <mrs_msgs/msg/state.hpp>

// macro variables for the enum definition
#undef X_ENUM_NAME
#undef X_ENUM_BASE_TYPE
#undef X_ENUM_SEQ

#define X_ENUM_NAME state_t
#define X_ENUM_BASE_TYPE uint8_t
// clang-format off
#define X_ENUM_SEQ \
  (DISARMED)       \
  (ARMED)          \
  (OFFBOARD)       \
  (MANUAL)         \
  (TAKEOFF)        \
  (LAND)           \
  (RC_MODE)        \
  (HOVER)          \
  (GOTO)           \
  (TRAJECTORY)
// clang-format on

// optional macro variables for enum to ROS message conversions
#undef X_ENUM_MSG_TYPE
#undef X_ENUM_MSG_MEMBER
#undef X_ENUM_MSG_PREFIX

#define X_ENUM_MSG_TYPE mrs_msgs::msg::State
#define X_ENUM_MSG_MEMBER state
#define X_ENUM_MSG_PREFIX STATE_

namespace mrs_uav_managers
{

#include <mrs_uav_managers/diagnostics_manager/enums/helpers/enum_macros.hpp>

// generate the enum and the to_string() conversion
DEFINE_ENUM_WITH_CONVERSIONS(X_ENUM_NAME, X_ENUM_BASE_TYPE, X_ENUM_SEQ)

// generate the to_ros() conversion
DEFINE_ENUM_MSG_CONVERSIONS(X_ENUM_NAME, X_ENUM_MSG_TYPE, X_ENUM_MSG_MEMBER, X_ENUM_MSG_PREFIX, X_ENUM_SEQ)

// some more helper functions related to this enum
inline bool is_flying(state_t uav_state) {
  switch (uav_state) {
  case state_t::DISARMED:
  case state_t::ARMED:
  case state_t::OFFBOARD:
    return false;
  default:
    return true;
  }
}

// some more helper functions related to this enum
inline bool is_flying_autonomously(state_t uav_state) {
  switch (uav_state) {
  case state_t::DISARMED:
  case state_t::ARMED:
  case state_t::OFFBOARD:
  case state_t::MANUAL:
    return false;
  default:
    return true;
  }
}

} // namespace mrs_uav_managers
