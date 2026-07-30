#pragma once
#include <cstdint>

// macro variables for the enum definition
#undef X_ENUM_NAME
#undef X_ENUM_BASE_TYPE
#undef X_ENUM_SEQ

#define X_ENUM_NAME robot_type_t
#define X_ENUM_BASE_TYPE uint8_t
// clang-format off
#define X_ENUM_SEQ \
  (MULTIROTOR)     \
  (BOAT)
// clang-format on

namespace mrs_uav_managers
{

#include <mrs_uav_managers/diagnostics_manager/enums/helpers/enum_macros.hpp>

// generate the enum and the to_string() conversion
DEFINE_ENUM_WITH_CONVERSIONS(X_ENUM_NAME, X_ENUM_BASE_TYPE, X_ENUM_SEQ)

} // namespace mrs_uav_managers
