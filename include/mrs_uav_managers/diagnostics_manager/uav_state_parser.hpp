#pragma once

#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/hw_api_status.hpp>

#include <mrs_uav_managers/diagnostics_manager/enums/tracker_state.hpp>
#include <mrs_uav_managers/diagnostics_manager/enums/uav_state.hpp>

namespace mrs_uav_managers::diagnostics_manager
{

/* parse_tracker_state() //{ */

inline tracker_state_t parse_tracker_state(const mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr &control_manager_diagnostics) {

  if (control_manager_diagnostics == nullptr)
    return tracker_state_t::UNKNOWN;

  if (control_manager_diagnostics->active_tracker == "NullTracker")
    return tracker_state_t::INVALID;

  switch (control_manager_diagnostics->tracker_status.state) {

  case mrs_msgs::msg::TrackerStatus::STATE_INVALID:
    return tracker_state_t::INVALID;
  case mrs_msgs::msg::TrackerStatus::STATE_IDLE:
    return tracker_state_t::IDLE;
  case mrs_msgs::msg::TrackerStatus::STATE_TAKEOFF:
    return tracker_state_t::TAKEOFF;
  case mrs_msgs::msg::TrackerStatus::STATE_HOVER:
    return tracker_state_t::HOVER;
  case mrs_msgs::msg::TrackerStatus::STATE_REFERENCE:
    return tracker_state_t::REFERENCE;
  case mrs_msgs::msg::TrackerStatus::STATE_TRAJECTORY:
    return tracker_state_t::TRAJECTORY;
  case mrs_msgs::msg::TrackerStatus::STATE_LAND:
    return tracker_state_t::LAND;
  default:
    return tracker_state_t::UNKNOWN;
  }
}

//}

/* parse_uav_state() //{ */

inline state_t parse_uav_state(const mrs_msgs::msg::HwApiStatus::ConstSharedPtr               &hw_api_status,
                               const mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr &control_manager_diagnostics) {
  if (hw_api_status == nullptr || control_manager_diagnostics == nullptr)
    return state_t::UNKNOWN;

  if (!hw_api_status->connected)
    return state_t::LINK_LOST;

  const bool hw_armed = hw_api_status->armed;
  // not armed
  if (!hw_armed)
    return state_t::DISARMED;

  // armed, flying in manual mode
  const bool manual_mode = hw_api_status->mode == "MANUAL";
  if (control_manager_diagnostics->joystick_active && manual_mode)
    return state_t::MANUAL;

  // armed, not flying
  const auto tracker_state = parse_tracker_state(control_manager_diagnostics);
  const bool null_tracker  = tracker_state == tracker_state_t::INVALID;
  if (hw_armed && null_tracker) {
    const bool offboard = hw_api_status->offboard;
    if (offboard)
      return state_t::OFFBOARD;
    return state_t::ARMED;
  }
  // flying using the MRS system in RC joystick mode
  if (control_manager_diagnostics->joystick_active)
    return state_t::RC_MODE;

  // LandoffTracker goes into idle state when deactivating
  if (control_manager_diagnostics->active_tracker == "LandoffTracker" && tracker_state == tracker_state_t::IDLE)
    return state_t::TAKEOFF;

  // unless the RC mode is active, just parse the tracker state
  switch (tracker_state) {
  case tracker_state_t::TAKEOFF:
    return state_t::TAKEOFF;
  case tracker_state_t::HOVER:
    return state_t::HOVER;
  case tracker_state_t::REFERENCE:
    return state_t::GOTO;
  case tracker_state_t::TRAJECTORY:
    return state_t::TRAJECTORY;
  case tracker_state_t::LAND:
    return state_t::LAND;
  default:
    return state_t::UNKNOWN;
  }
}

//}

} // namespace mrs_uav_managers::diagnostics_manager
