#include <gtest/gtest.h>

#include <mrs_uav_managers/diagnostics_manager/uav_state_parser.hpp>

using namespace mrs_uav_managers::diagnostics_manager;
using Hw  = mrs_msgs::msg::HwApiStatus;
using Cmd = mrs_msgs::msg::ControlManagerDiagnostics;
using Ts  = mrs_msgs::msg::TrackerStatus;

namespace
{

Hw::ConstSharedPtr hw(bool armed, bool offboard, uint8_t airborne, const std::string &mode) {
  auto m       = std::make_shared<Hw>();
  m->connected = true;
  m->armed     = armed;
  m->offboard  = offboard;
  m->airborne  = airborne;
  m->mode      = mode;
  return m;
}

Cmd::ConstSharedPtr null_tracker(bool joystick = false) {
  auto m                  = std::make_shared<Cmd>();
  m->active_tracker       = "NullTracker";
  m->tracker_status.state = Ts::STATE_INVALID;
  m->joystick_active      = joystick;
  return m;
}

Cmd::ConstSharedPtr tracker(uint8_t tracker_status_state, bool joystick = false) {
  auto m                  = std::make_shared<Cmd>();
  m->active_tracker       = "MpcTracker";
  m->tracker_status.state = tracker_status_state;
  m->joystick_active      = joystick;
  return m;
}

Cmd::ConstSharedPtr hovering(bool joystick = false) {
  return tracker(Ts::STATE_HOVER, joystick);
}

} // namespace

TEST(UavStateParser, AirborneDefaultsToUnknown) {
  EXPECT_EQ(Hw().airborne, Hw::AIRBORNE_UNKNOWN);
}

TEST(UavStateParser, MissingInputsAreUnknown) {
  EXPECT_EQ(parse_uav_state(nullptr, null_tracker()), state_t::UNKNOWN);
  EXPECT_EQ(parse_uav_state(hw(true, false, Hw::AIRBORNE_NO, "POSCTL"), nullptr), state_t::UNKNOWN);
}

TEST(UavStateParser, DisconnectedIsLinkLost) {
  auto m       = std::make_shared<Hw>();
  m->connected = false;
  m->armed     = true;
  m->airborne  = Hw::AIRBORNE_YES; // violates the HW API rule on purpose -- connected must still win
  EXPECT_EQ(parse_uav_state(m, null_tracker()), state_t::NO_LINK);
}

TEST(UavStateParser, DisarmedWinsEvenInAir) {
  EXPECT_EQ(parse_uav_state(hw(false, false, Hw::AIRBORNE_YES, "MANUAL"), null_tracker()), state_t::DISARMED);
}

TEST(UavStateParser, ArmedOnGroundWithoutOffboardIsArmed) {
  for (const std::string mode : {"MANUAL", "POSCTL", "ALTCTL", "AUTO.READY"}) {
    EXPECT_EQ(parse_uav_state(hw(true, false, Hw::AIRBORNE_NO, mode), null_tracker()), state_t::ARMED) << mode;
  }
}

TEST(UavStateParser, UnknownAirborneKeepsArmed) {
  EXPECT_EQ(parse_uav_state(hw(true, false, Hw::AIRBORNE_UNKNOWN, "MANUAL"), null_tracker(true)), state_t::ARMED);
  EXPECT_EQ(parse_uav_state(hw(true, false, 200, "MANUAL"), null_tracker()), state_t::ARMED); // out-of-range value is not YES
}

TEST(UavStateParser, AirborneWithoutOffboardIsManualInEveryMode) {
  const std::vector<std::string> modes = {"MANUAL",      "STABILIZED", "ACRO",      "RATTITUDE",    "ALTCTL",       "POSCTL",
                                          "AUTO.LOITER", "AUTO.RTL",   "AUTO.LAND", "AUTO.MISSION", "CMODE(12345)", ""};
  for (const auto &mode : modes) {
    EXPECT_EQ(parse_uav_state(hw(true, false, Hw::AIRBORNE_YES, mode), null_tracker()), state_t::MANUAL) << mode;
  }
}

TEST(UavStateParser, LostOffboardWithTrackerStillActiveIsManual) {
  EXPECT_EQ(parse_uav_state(hw(true, false, Hw::AIRBORNE_YES, "POSCTL"), hovering()), state_t::MANUAL);
}

TEST(UavStateParser, LingeringJoystickDoesNotChangeManual) {
  EXPECT_EQ(parse_uav_state(hw(true, false, Hw::AIRBORNE_YES, "POSCTL"), null_tracker(true)), state_t::MANUAL);
  // the old rule reported MANUAL here; on the ground it is just ARMED now
  EXPECT_EQ(parse_uav_state(hw(true, false, Hw::AIRBORNE_NO, "MANUAL"), null_tracker(true)), state_t::ARMED);
}

TEST(UavStateParser, OffboardTakeoffIsNotManual) {
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), null_tracker()), state_t::OFFBOARD);
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_NO, "OFFBOARD"), null_tracker()), state_t::OFFBOARD);
}

TEST(UavStateParser, OffboardFlightUnchanged) {
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), hovering()), state_t::HOVER);
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), hovering(true)), state_t::RC_MODE);
}

TEST(UavStateParser, ManualIsStickyWhileAirborneUnknown) {
  // the in-air source went stale during a flight without offboard: must not look like a landing
  EXPECT_EQ(parse_uav_state(hw(true, false, Hw::AIRBORNE_UNKNOWN, "POSCTL"), null_tracker(), state_t::MANUAL), state_t::MANUAL);
  EXPECT_EQ(parse_uav_state(hw(true, false, Hw::AIRBORNE_UNKNOWN, "POSCTL"), hovering(), state_t::MANUAL), state_t::MANUAL);
  EXPECT_EQ(parse_uav_state(hw(true, false, 200, "POSCTL"), null_tracker(), state_t::MANUAL), state_t::MANUAL); // out-of-range is not NO
}

TEST(UavStateParser, ManualEndsOnExplicitLandingDisarmOffboardOrLinkLoss) {
  EXPECT_EQ(parse_uav_state(hw(true, false, Hw::AIRBORNE_NO, "POSCTL"), null_tracker(), state_t::MANUAL), state_t::ARMED);
  EXPECT_EQ(parse_uav_state(hw(false, false, Hw::AIRBORNE_UNKNOWN, "POSCTL"), null_tracker(), state_t::MANUAL), state_t::DISARMED);
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_UNKNOWN, "OFFBOARD"), null_tracker(), state_t::MANUAL), state_t::OFFBOARD);

  auto m       = std::make_shared<Hw>();
  m->connected = false;
  m->armed     = true;
  EXPECT_EQ(parse_uav_state(m, null_tracker(), state_t::MANUAL), state_t::NO_LINK);
}

TEST(UavStateParser, UnknownAirborneWithoutManualBeforeKeepsArmed) {
  // a HW API that never fills airborne must behave as today, whatever the previous state
  for (const auto previous : {state_t::UNKNOWN, state_t::NO_LINK, state_t::DISARMED, state_t::ARMED, state_t::OFFBOARD}) {
    EXPECT_EQ(parse_uav_state(hw(true, false, Hw::AIRBORNE_UNKNOWN, "POSCTL"), null_tracker(), previous), state_t::ARMED) << static_cast<int>(previous);
  }
}

TEST(UavStateParser, AirborneUnknownIrrelevantWhenOffboard) {
  // gap: airborne only matters for the "!offboard" MANUAL check -- once offboard, tracker/joystick resolution
  // must not care that airborne went stale (e.g. right after a takeoff, before the autopilot reports YES)
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_UNKNOWN, "OFFBOARD"), hovering()), state_t::HOVER);
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_UNKNOWN, "OFFBOARD"), hovering(true)), state_t::RC_MODE);
}

TEST(UavStateParser, TrackerStateMapsDirectly) {
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), tracker(Ts::STATE_TAKEOFF)), state_t::TAKEOFF);
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), tracker(Ts::STATE_HOVER)), state_t::HOVER);
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), tracker(Ts::STATE_REFERENCE)), state_t::GOTO);
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), tracker(Ts::STATE_TRAJECTORY)), state_t::TRAJECTORY);
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), tracker(Ts::STATE_LAND)), state_t::LAND);
}

TEST(UavStateParser, LandoffTrackerIdleIsTakeoff) {
  // LandoffTracker goes into idle state when deactivating, which must still read as TAKEOFF
  auto m                  = std::make_shared<Cmd>();
  m->active_tracker       = "LandoffTracker";
  m->tracker_status.state = Ts::STATE_IDLE;
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_NO, "OFFBOARD"), m), state_t::TAKEOFF);
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_NO, "OFFBOARD"), m, state_t::OFFBOARD), state_t::TAKEOFF);
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), m, state_t::TAKEOFF), state_t::TAKEOFF);
}

TEST(UavStateParser, TrackerSwitchInFlightKeepsState) {
  // a tracker activated in flight (e.g. LandoffTracker for landing) reports STATE_INVALID until its first update;
  // that is a switch in progress, not "no tracker" -- the UAV must not read as OFFBOARD (not flying) mid-air
  for (const auto previous : {state_t::HOVER, state_t::GOTO, state_t::TRAJECTORY, state_t::LAND, state_t::RC_MODE, state_t::TAKEOFF}) {
    auto m                  = std::make_shared<Cmd>();
    m->active_tracker       = "LandoffTracker";
    m->tracker_status.state = Ts::STATE_INVALID;
    EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), m, previous), previous) << static_cast<int>(previous);
  }
}

TEST(UavStateParser, NewTrackerOnGroundIsStillOffboard) {
  // the same INVALID status before the UAV flies (a tracker being activated on the ground) keeps OFFBOARD
  auto m                  = std::make_shared<Cmd>();
  m->active_tracker       = "MpcTracker";
  m->tracker_status.state = Ts::STATE_INVALID;
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_NO, "OFFBOARD"), m, state_t::OFFBOARD), state_t::OFFBOARD);
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_NO, "OFFBOARD"), m, state_t::ARMED), state_t::OFFBOARD);
}

TEST(UavStateParser, NullTrackerAfterFlightIsOffboard) {
  // NullTracker really means no tracker: after a landing the state leaves LAND
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_NO, "OFFBOARD"), null_tracker(), state_t::LAND), state_t::OFFBOARD);
}

TEST(UavStateParser, LandoffTrackerIdleInFlightIsNotTakeoff) {
  // LandoffTracker activated in the air (e.g. escalating failsafe -> eland) reports STATE_IDLE before it starts landing;
  // only coming from the ground (or an ongoing takeoff) does IDLE mean TAKEOFF
  auto m                  = std::make_shared<Cmd>();
  m->active_tracker       = "LandoffTracker";
  m->tracker_status.state = Ts::STATE_IDLE;
  for (const auto previous : {state_t::HOVER, state_t::GOTO, state_t::TRAJECTORY, state_t::LAND, state_t::RC_MODE}) {
    EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), m, previous), previous) << static_cast<int>(previous);
  }
}

namespace
{

Cmd::ConstSharedPtr midair_activation_tracker() {
  // MidairActivationTracker never fills tracker_status.state, so it always reads STATE_INVALID
  auto m                  = std::make_shared<Cmd>();
  m->active_tracker       = "MidairActivationTracker";
  m->tracker_status.state = Ts::STATE_INVALID;
  return m;
}

} // namespace

TEST(UavStateParser, MidairActivationTrackerIsMidairActivation) {
  // UavManager activates MidairActivationTracker before it switches the autopilot to OFFBOARD
  EXPECT_EQ(parse_uav_state(hw(true, false, Hw::AIRBORNE_YES, "POSCTL"), midair_activation_tracker(), state_t::MANUAL), state_t::MIDAIR);
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), midair_activation_tracker(), state_t::MIDAIR), state_t::MIDAIR);
  // also when the HW API can't tell or reports the ground (e.g. the simulator's takeoff platform)
  EXPECT_EQ(parse_uav_state(hw(true, false, Hw::AIRBORNE_UNKNOWN, "POSCTL"), midair_activation_tracker()), state_t::MIDAIR);
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_NO, "OFFBOARD"), midair_activation_tracker(), state_t::ARMED), state_t::MIDAIR);
}

TEST(UavStateParser, MidairActivationYieldsToDisarmAndLinkLoss) {
  EXPECT_EQ(parse_uav_state(hw(false, false, Hw::AIRBORNE_YES, "POSCTL"), midair_activation_tracker(), state_t::MIDAIR), state_t::DISARMED);

  auto m       = std::make_shared<Hw>();
  m->connected = false;
  m->armed     = true;
  EXPECT_EQ(parse_uav_state(m, midair_activation_tracker(), state_t::MIDAIR), state_t::NO_LINK);
}

TEST(UavStateParser, MidairActivationSequenceStaysFlying) {
  // MANUAL (pilot flies) -> MIDAIR -> MpcTracker takes over (INVALID first, then braking, then hover)
  state_t s = state_t::MANUAL;
  s         = parse_uav_state(hw(true, false, Hw::AIRBORNE_YES, "POSCTL"), midair_activation_tracker(), s);
  EXPECT_EQ(s, state_t::MIDAIR);
  s = parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), midair_activation_tracker(), s);
  EXPECT_EQ(s, state_t::MIDAIR);
  s = parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), tracker(Ts::STATE_INVALID), s);
  EXPECT_EQ(s, state_t::MIDAIR);
  s = parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), tracker(Ts::STATE_REFERENCE), s);
  EXPECT_EQ(s, state_t::GOTO);
  s = parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), hovering(), s);
  EXPECT_EQ(s, state_t::HOVER);
}

TEST(UavStateParser, MidairActivationIsFlying) {
  EXPECT_TRUE(is_flying(state_t::MIDAIR));
  EXPECT_TRUE(is_flying_autonomously(state_t::MIDAIR));
}

TEST(UavStateParser, MidairActivationMapsToMessage) {
  EXPECT_EQ(to_ros(state_t::MIDAIR), mrs_msgs::msg::State::STATE_MIDAIR);
}

namespace
{

// ehover/eland: LandoffTracker with EmergencyController; failsafe: FailsafeController with any tracker
Cmd::ConstSharedPtr emergency(const std::string &controller, uint8_t tracker_status_state, const std::string &active_tracker = "LandoffTracker",
                              bool joystick = false) {
  auto m                  = std::make_shared<Cmd>();
  m->active_tracker       = active_tracker;
  m->active_controller    = controller;
  m->tracker_status.state = tracker_status_state;
  m->joystick_active      = joystick;
  return m;
}

} // namespace

TEST(UavStateParser, EmergencyControllerHoldingIsEhover) {
  for (const auto tracker_state : {Ts::STATE_IDLE, Ts::STATE_HOVER, Ts::STATE_INVALID}) {
    for (const auto previous : {state_t::HOVER, state_t::GOTO, state_t::MIDAIR}) {
      EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), emergency("EmergencyController", tracker_state), previous), state_t::EHOVER)
          << int(tracker_state) << " " << static_cast<int>(previous);
    }
  }
}

TEST(UavStateParser, EmergencyControllerLandingIsEland) {
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), emergency("EmergencyController", Ts::STATE_LAND), state_t::EHOVER), state_t::ELAND);
}

TEST(UavStateParser, FailsafeControllerIsFailsafe) {
  for (const auto tracker_state : {Ts::STATE_INVALID, Ts::STATE_IDLE, Ts::STATE_HOVER, Ts::STATE_LAND}) {
    EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), emergency("FailsafeController", tracker_state), state_t::ELAND), state_t::FAILSAFE)
        << int(tracker_state);
  }
}

TEST(UavStateParser, EmergencyControllerOnGroundIsNotEhover) {
  // EmergencyController is ControlManager's startup controller: with NullTracker it is no emergency
  EXPECT_EQ(parse_uav_state(hw(true, false, Hw::AIRBORNE_NO, "POSCTL"), emergency("EmergencyController", Ts::STATE_INVALID, "NullTracker")), state_t::ARMED);
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_NO, "OFFBOARD"), emergency("EmergencyController", Ts::STATE_INVALID, "NullTracker")),
            state_t::OFFBOARD);
}

TEST(UavStateParser, JoystickWinsOverEmergencyController) {
  // joystick attitude control falls back to EmergencyController
  EXPECT_EQ(
      parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), emergency("EmergencyController", Ts::STATE_HOVER, "MpcTracker", true), state_t::HOVER),
      state_t::RC_MODE);
}

TEST(UavStateParser, EmergencyYieldsToDisarmLinkLossAndManual) {
  const auto m = emergency("EmergencyController", Ts::STATE_LAND);
  EXPECT_EQ(parse_uav_state(hw(false, true, Hw::AIRBORNE_YES, "OFFBOARD"), m, state_t::ELAND), state_t::DISARMED);
  EXPECT_EQ(parse_uav_state(hw(true, false, Hw::AIRBORNE_YES, "POSCTL"), m, state_t::ELAND), state_t::MANUAL); // the pilot took over

  auto lost       = std::make_shared<Hw>();
  lost->connected = false;
  lost->armed     = true;
  EXPECT_EQ(parse_uav_state(lost, m, state_t::ELAND), state_t::NO_LINK);
}

TEST(UavStateParser, EscalatingFailsafeSequence) {
  // HOVER -> ehover (tracker switched first, then the controller) -> eland -> failsafe
  const auto air = hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD");
  state_t    s   = state_t::HOVER;
  s              = parse_uav_state(air, emergency("MpcController", Ts::STATE_INVALID), s);
  EXPECT_EQ(s, state_t::HOVER);
  s = parse_uav_state(air, emergency("EmergencyController", Ts::STATE_IDLE), s);
  EXPECT_EQ(s, state_t::EHOVER);
  s = parse_uav_state(air, emergency("EmergencyController", Ts::STATE_LAND), s);
  EXPECT_EQ(s, state_t::ELAND);
  s = parse_uav_state(air, emergency("FailsafeController", Ts::STATE_LAND), s);
  EXPECT_EQ(s, state_t::FAILSAFE);
}

TEST(UavStateParser, FailedMidairActivationIsEhover) {
  // the next tracker can't be activated -> UavManager calls ehover
  const auto air = hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD");
  state_t    s   = state_t::MIDAIR;
  s              = parse_uav_state(air, emergency("MidairActivationController", Ts::STATE_INVALID), s);
  EXPECT_EQ(s, state_t::MIDAIR);
  s = parse_uav_state(air, emergency("EmergencyController", Ts::STATE_IDLE), s);
  EXPECT_EQ(s, state_t::EHOVER);
}

TEST(UavStateParser, EmergencyStatesAreFlyingAutonomously) {
  for (const auto st : {state_t::EHOVER, state_t::ELAND, state_t::FAILSAFE}) {
    EXPECT_TRUE(is_flying(st)) << static_cast<int>(st);
    EXPECT_TRUE(is_flying_autonomously(st)) << static_cast<int>(st);
  }
}

TEST(UavStateParser, EmergencyStatesMapToMessage) {
  EXPECT_EQ(to_ros(state_t::EHOVER), mrs_msgs::msg::State::STATE_EHOVER);
  EXPECT_EQ(to_ros(state_t::ELAND), mrs_msgs::msg::State::STATE_ELAND);
  EXPECT_EQ(to_ros(state_t::FAILSAFE), mrs_msgs::msg::State::STATE_FAILSAFE);
  EXPECT_EQ(to_ros(state_t::NO_LINK), mrs_msgs::msg::State::STATE_NO_LINK);
}

TEST(UavStateParser, EhoverRecoveryReturnsToFlight) {
  // the operator switches the controller and the tracker back (in either order); EHOVER lasts until both are done
  const auto air = hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD");

  state_t s = state_t::EHOVER;
  s         = parse_uav_state(air, emergency("Se3Controller", Ts::STATE_IDLE), s); // controller first, LandoffTracker still holds
  EXPECT_EQ(s, state_t::EHOVER);
  s = parse_uav_state(air, emergency("Se3Controller", Ts::STATE_INVALID, "MpcTracker"), s);
  EXPECT_EQ(s, state_t::EHOVER);
  s = parse_uav_state(air, emergency("Se3Controller", Ts::STATE_HOVER, "MpcTracker"), s);
  EXPECT_EQ(s, state_t::HOVER);

  s = state_t::EHOVER;
  s = parse_uav_state(air, emergency("EmergencyController", Ts::STATE_HOVER, "MpcTracker"), s); // tracker first
  EXPECT_EQ(s, state_t::EHOVER);
  s = parse_uav_state(air, emergency("Se3Controller", Ts::STATE_HOVER, "MpcTracker"), s);
  EXPECT_EQ(s, state_t::HOVER);
}

TEST(UavStateParser, EhoverThenLandIsLand) {
  // UavManager's land switches to its landing controller and LandoffTracker starts landing
  EXPECT_EQ(parse_uav_state(hw(true, true, Hw::AIRBORNE_YES, "OFFBOARD"), emergency("MpcController", Ts::STATE_LAND), state_t::EHOVER), state_t::LAND);
}
