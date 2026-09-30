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
  EXPECT_EQ(parse_uav_state(m, null_tracker()), state_t::LINK_LOST);
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
  EXPECT_EQ(parse_uav_state(m, null_tracker(), state_t::MANUAL), state_t::LINK_LOST);
}

TEST(UavStateParser, UnknownAirborneWithoutManualBeforeKeepsArmed) {
  // a HW API that never fills airborne must behave as today, whatever the previous state
  for (const auto previous : {state_t::UNKNOWN, state_t::LINK_LOST, state_t::DISARMED, state_t::ARMED, state_t::OFFBOARD}) {
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
}
