#pragma once
#ifndef ESTIMATION_MANAGER_TYPES_H
#define ESTIMATION_MANAGER_TYPES_H

#include <vector>
#include <string>

namespace mrs_uav_managers
{

namespace estimation_manager
{
/*//{ typedef */

typedef enum
{

  AXIS_X,
  AXIS_Y,
  AXIS_Z

} AxisId_t;

typedef enum
{

  POSITION,
  VELOCITY,
  ACCELERATION,
  ACCELERATION_BIAS,
  ACCELERATION_UNBIASED

} StateId_t;
const int n_StateId_t = 5;

typedef enum
{

  UNINITIALIZED_STATE,
  INITIALIZED_STATE,
  READY_STATE,
  STARTED_STATE,
  RUNNING_STATE,
  STOPPED_STATE,
  ERROR_STATE

} SMStates_t;

typedef enum
{

  METERS,
  DEGREES

} UtmUnits_t;

/*//}*/

namespace sm
{
// clang-format off
  const std::vector<std::string> state_names = {
    "UNINITIALIZED_STATE",
    "INITIALIZED_STATE",
    "READY_STATE",
    "STARTED_STATE",
    "RUNNING_STATE",
    "STOPPED_STATE",
    "ERROR_STATE"
  };
// clang-format on

} // namespace sm

} // namespace estimation_manager

} // namespace mrs_uav_managers

#endif // ESTIMATION_MANAGER_TYPES_H
