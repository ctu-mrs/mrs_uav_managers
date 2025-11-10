#pragma once
#include <cstdint>

// macro variables for the enum definition
#undef X_ENUM_NAME
#undef X_ENUM_BASE_TYPE
#undef X_ENUM_SEQ

#define X_ENUM_NAME       robot_type_t
#define X_ENUM_BASE_TYPE  uint8_t
#define X_ENUM_SEQ                      \
                          (MULTIROTOR)  \
                          (BOAT)        



namespace mrs_uav_managers 
{
namespace diagnostics_manager 
{

#include "diagnostics_manager/enums/enum_macros.h"

  // generate the enum and the to_string() conversion
DEFINE_ENUM_WITH_CONVERSIONS(X_ENUM_NAME, X_ENUM_BASE_TYPE, X_ENUM_SEQ)

}   // namespace diagnostics_manager
}  // namespace mrs_uav_managers
