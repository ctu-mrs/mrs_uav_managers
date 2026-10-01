#pragma once

#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/hw_api_status.hpp>

#include <mrs_uav_managers/diagnostics_manager/enums/tracker_state.hpp>
#include <mrs_uav_managers/diagnostics_manager/enums/uav_state.hpp>

namespace mrs_uav_managers::diagnostics_manager
{

/* names //{ */

// ControlManager trackers and controllers recognised by name; the configurable ones are matched by their default name
// TODO ControlManager should report these modes explicitly, so that no names are needed here
namespace names
{
inline constexpr char null_tracker[]              = "NullTracker";
inline constexpr char midair_activation_tracker[] = "MidairActivationTracker";
inline constexpr char landoff_tracker[]           = "LandoffTracker";
inline constexpr char eland_controller[]          = "EmergencyController";
inline constexpr char failsafe_controller[]       = "FailsafeController";
} // namespace names

//}

/* parse_tracker_state() //{ */

inline tracker_state_t parse_tracker_state(const mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr &control_manager_diagnostics) {

  if (control_manager_diagnostics == nullptr)
    return tracker_state_t::UNKNOWN;

  if (control_manager_diagnostics->active_tracker == names::null_tracker)
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
                               const mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr &control_manager_diagnostics,
                               const state_t                                                   previous = state_t::UNKNOWN) {
  if (hw_api_status == nullptr || control_manager_diagnostics == nullptr)
    return state_t::UNKNOWN;

  if (!hw_api_status->connected)
    return state_t::NO_LINK;

  if (!hw_api_status->armed)
    return state_t::DISARMED;

  // UavManager is taking over a UAV already in the air: MidairActivationTracker holds it while the autopilot is switched
  // to OFFBOARD (before that, the HW still reports a pilot flying); the tracker never fills its state, so decide by name
  if (control_manager_diagnostics->active_tracker == names::midair_activation_tracker)
    return state_t::MIDAIR;

  // in the air without offboard: a pilot (any RC mode) or the autopilot itself (e.g. PX4 AUTO.*) is flying, not MRS;
  // only an explicit YES starts MANUAL -- UNKNOWN (HW API can't tell) keeps the ARMED fallback below;
  // once MANUAL, only an explicit NO ends it -- a stale in-air source (UNKNOWN) must not look like a landing
  const bool airborne_yes   = hw_api_status->airborne == mrs_msgs::msg::HwApiStatus::AIRBORNE_YES;
  const bool still_airborne = previous == state_t::MANUAL && hw_api_status->airborne != mrs_msgs::msg::HwApiStatus::AIRBORNE_NO;

  if (!hw_api_status->offboard && (airborne_yes || still_airborne))
    return state_t::MANUAL;

  // ControlManager's emergencies: ehover/eland run the eland controller, failsafe the failsafe controller;
  // the eland controller alone is no emergency -- it is also the startup controller and the joystick fallback
  if (control_manager_diagnostics->active_tracker != names::null_tracker && !control_manager_diagnostics->joystick_active) {

    if (control_manager_diagnostics->active_controller == names::failsafe_controller)
      return state_t::FAILSAFE;

    if (control_manager_diagnostics->active_controller == names::eland_controller)
      return control_manager_diagnostics->tracker_status.state == mrs_msgs::msg::TrackerStatus::STATE_LAND ? state_t::ELAND : state_t::EHOVER;
  }

  const auto tracker_state = parse_tracker_state(control_manager_diagnostics);

  // flight phase driven by MRS before this reading (any is_flying_autonomously() state)
  const bool was_flying = is_flying_autonomously(previous);

  if (tracker_state == tracker_state_t::INVALID) {
    // a tracker activated in flight (e.g. LandoffTracker for landing) reports STATE_INVALID until its first update:
    // a switch in progress, not "no tracker" -- keep the flight phase instead of dropping to OFFBOARD mid-air
    if (hw_api_status->offboard && was_flying && control_manager_diagnostics->active_tracker != names::null_tracker)
      return previous;

    return hw_api_status->offboard ? state_t::OFFBOARD : state_t::ARMED;
  }

  // flying using the MRS system in RC joystick mode
  if (control_manager_diagnostics->joystick_active)
    return state_t::RC_MODE;

  // LandoffTracker goes into idle state when deactivating after a takeoff; activated in the air for a landing it is
  // idle before it starts landing -- that is not a takeoff, keep the flight phase
  if (control_manager_diagnostics->active_tracker == names::landoff_tracker && tracker_state == tracker_state_t::IDLE)
    return (was_flying && previous != state_t::TAKEOFF) ? previous : state_t::TAKEOFF;

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
