#pragma once
#include <cstdint>

#undef X_ENUM_NAME
#undef X_ENUM_BASE_TYPE
#undef X_ENUM_SEQ

#define X_ENUM_NAME tracker_state_t
#define X_ENUM_BASE_TYPE uint8_t
// clang-format off
#define X_ENUM_SEQ \
  (INVALID)        \
  (IDLE)           \
  (TAKEOFF)        \
  (HOVER)          \
  (REFERENCE)      \
  (TRAJECTORY)     \
  (LAND)
// clang-format on

namespace mrs_uav_managers
{

#include <mrs_uav_managers/diagnostics_manager/enums/helpers/enum_macros.hpp>

DEFINE_ENUM_WITH_CONVERSIONS(X_ENUM_NAME, X_ENUM_BASE_TYPE, X_ENUM_SEQ)

} // namespace mrs_uav_managers
