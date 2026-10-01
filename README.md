# MRS UAV Managers

Please follow to the [documentation page](https://ctu-mrs.github.io/docs/features/managers/).

## DiagnosticsManager: UAV state

How `parse_uav_state()` (`include/mrs_uav_managers/diagnostics_manager/uav_state_parser.hpp`) turns `hw_api/status` (`mrs_msgs/HwApiStatus`) and `control_manager/diagnostics` (`mrs_msgs/ControlManagerDiagnostics`) into the single `mrs_msgs/State` published as `diagnostics_manager/uav_state`. The state is re-evaluated from scratch on every reading; the rules are listed below the diagram.

```mermaid
stateDiagram-v2
  direction TB

  [*] --> UNKNOWN
  UNKNOWN --> DISARMED: inputs present
  NO_LINK --> DISARMED: connected, not armed

  note left of NO_LINK
    From every state
    (arrows not shown):
    no link -> NO_LINK
    disarmed -> DISARMED
  end note

  DISARMED --> ARMED: armed
  ARMED --> OFFBOARD: offboard on / off
  OFFBOARD --> ARMED

  ARMED --> MANUAL
  OFFBOARD --> MANUAL
  FLYING --> MANUAL
  MANUAL --> ARMED: landed
  MANUAL --> OFFBOARD: offboard back

  note right of MANUAL
    not offboard + HW says
    airborne: a pilot or the
    autopilot flies, not MRS;
    stays until landed
  end note

  MANUAL --> MIDAIR: MidairActivationTracker
  MIDAIR --> FLYING: next tracker<br/>reports a state

  OFFBOARD --> FLYING: tracker active
  FLYING --> OFFBOARD: no tracker

  FLYING --> EHOVER: ehover
  EHOVER --> ELAND: eland
  ELAND --> FAILSAFE: failsafe
  FLYING --> FAILSAFE: failsafe

  state "Flying (tracker-driven)" as FLYING {
    [*] --> TAKEOFF
    TAKEOFF --> HOVER
    HOVER --> GOTO
    HOVER --> TRAJECTORY
    GOTO --> HOVER
    TRAJECTORY --> HOVER
    HOVER --> LAND
    LAND --> HOVER

    HOVER --> RC_MODE: joystick on
    RC_MODE --> HOVER: joystick off

    TAKEOFF: TAKEOFF<br/>(STATE_TAKEOFF, or<br/>LandoffTracker idle<br/>on the ground)
    GOTO: GOTO<br/>(STATE_REFERENCE)

    note right of TAKEOFF
      follows the tracker's state:
      any can follow any other;
      a tracker switch keeps
      the current state
    end note
  }
```

### How the state is decided

On every reading the parser checks these rules from the top and stops at the first one that matches:

1. Missing input (`hw_api/status` or `control_manager/diagnostics` not received yet) -> `UNKNOWN`.
2. HW API not connected to the autopilot -> `NO_LINK`.
3. Not armed -> `DISARMED`.
4. The active tracker is `MidairActivationTracker` -> `MIDAIR`.
5. Not offboard and the HW says the UAV is airborne -> `MANUAL`. Once in `MANUAL`, it stays there until the HW says the UAV has landed (`airborne == NO`) or offboard comes back. An unknown airborne reading does not end it.
6. A tracker is active (not `NullTracker`), the joystick is not, and ControlManager has switched to an emergency controller:
   - `FailsafeController` -> `FAILSAFE`;
   - `EmergencyController` and the tracker reports `STATE_LAND` -> `ELAND`, otherwise -> `EHOVER`.
7. No active tracker (`NullTracker`, or the tracker reports `STATE_INVALID`):
   - offboard, the previous state was a flight state, and the tracker isn't `NullTracker` -> keep the previous state (a tracker switch in flight);
   - otherwise -> `OFFBOARD` if offboard, `ARMED` if not.
8. Joystick active -> `RC_MODE`.
9. `LandoffTracker` reports `STATE_IDLE` -> `TAKEOFF`, unless the previous state was a flight state other than `TAKEOFF`; then keep the previous state.
10. Otherwise the tracker's state decides: `STATE_TAKEOFF` -> `TAKEOFF`, `STATE_HOVER` -> `HOVER`, `STATE_REFERENCE` -> `GOTO`, `STATE_TRAJECTORY` -> `TRAJECTORY`, `STATE_LAND` -> `LAND`, anything else -> `UNKNOWN`.

"Flight state" means `TAKEOFF`, `HOVER`, `GOTO`, `TRAJECTORY`, `LAND`, `RC_MODE`, `MIDAIR`, `EHOVER`, `ELAND` or `FAILSAFE` (`is_flying_autonomously()`).

### Why some rules exist

- **Matched by name**: `NullTracker` and `MidairActivationTracker` are built into ControlManager. `LandoffTracker`, `EmergencyController` and `FailsafeController` are ControlManager parameters (`landing_takeoff_tracker`, `safety/eland/controller`, `safety/failsafe_controller`) matched against their default values; all names are in one place, `names` in `uav_state_parser.hpp`. Technical debt: ControlManager should report these modes explicitly.
- **The previous state**: the parser keeps no state of its own. It gets the previously published state and uses it only in rules 5, 7 and 9.
- **`MIDAIR` (rule 4)**: UavManager is taking over a UAV already in the air: control output ON, then `MidairActivationTracker` holds the UAV while the autopilot is switched to offboard, then the next tracker takes over. `MidairActivationTracker` never reports a tracker state, so it is recognised by name. It is checked before `MANUAL` because until offboard is confirmed the HW still reports a pilot flying. It counts as flying for `is_flying()` and `is_flying_autonomously()`.
- **Emergencies (rule 6)**: ControlManager's safety levels, in the order the escalating failsafe goes through them. `EHOVER`: holds the position after an error (e.g. a failed mid-air activation or takeoff, a tracker error); the UAV keeps flying until someone commands it. `ELAND`: lands where it is. `FAILSAFE`: descends without position feedback, the last resort. ehover and eland both use the eland controller (`EmergencyController`) with the ehover tracker; failsafe uses `FailsafeController`. `EmergencyController` alone is no emergency: it is ControlManager's startup controller (with `NullTracker`) and the joystick fallback. A pilot taking over (`MANUAL`), disarm and link loss still win.
- **`MANUAL` stays until landed (rule 5)**: if the airborne source goes stale (`UNKNOWN`), that must not look like a landing. Without a previous `MANUAL`, an unknown airborne reading gives `ARMED`/`OFFBOARD` as usual.
- **Tracker switch in flight (rule 7)**: a tracker activated in the air (`LandoffTracker` for land, land home, eland or escalating failsafe; the tracker taking over after a mid-air activation) reports `STATE_INVALID` for its first ~60-240 ms. Without this rule the state would drop to `OFFBOARD` mid-air and `is_flying()` would briefly be false. `NullTracker` really means no tracker, so after a landing the state still goes to `OFFBOARD`.
- **`LandoffTracker` idle (rule 9)**: after a takeoff, `LandoffTracker` goes idle before the next tracker takes over -- still `TAKEOFF`. Activated in the air for a landing it is idle before it starts landing; that is not a takeoff.
- **Inside `Flying`**: the arrows show a typical mission. Any flight state can follow any other, and the joystick (`RC_MODE`) can take over from any of them, not just `HOVER`.

### Transitions and the tests that cover them

Unit tests are `TEST(UavStateParser, ...)` in `test/diagnostics_manager/uav_state_parser/test.cpp`; `manual_after_losing_offboard` (in `test/diagnostics_manager/`) runs in the simulator.

| Transition | Test |
|---|---|
| `HwApiStatus.airborne` defaults to `UNKNOWN` (a HW API that doesn't fill it never starts `MANUAL`) | `AirborneDefaultsToUnknown` |
| missing `hw_api_status`/`control_manager_diagnostics` -> `UNKNOWN` | `MissingInputsAreUnknown` |
| any -> `NO_LINK`: `!connected` (overrides armed/airborne) | `DisconnectedIsLinkLost` |
| any -> `DISARMED`: connected, `!armed` (overrides airborne) | `DisarmedWinsEvenInAir` |
| `DISARMED` -> `ARMED`: armed, `!offboard`, `airborne == NO` | `ArmedOnGroundWithoutOffboardIsArmed`, `UnknownAirborneKeepsArmed` |
| `ARMED` -> `OFFBOARD`: offboard, no active tracker | `OffboardTakeoffIsNotManual` |
| `OFFBOARD` -> `ARMED`: `!offboard`, `airborne == NO`, no active tracker | `LingeringJoystickDoesNotChangeManual` |
| `ARMED` / `OFFBOARD` / `FLYING` -> `MANUAL`: `!offboard`, `airborne == YES` | `AirborneWithoutOffboardIsManualInEveryMode`, `LostOffboardWithTrackerStillActiveIsManual` |
| `HOVER` -> `MANUAL` -> `ARMED`: output switched off in the air (offboard lost, fall, landing) | `manual_after_losing_offboard` (simulation) |
| `MANUAL` -> `MANUAL`: `!offboard`, `airborne == UNKNOWN` (sticky) | `ManualIsStickyWhileAirborneUnknown` |
| `MANUAL` -> `ARMED`: `airborne == NO` | `ManualEndsOnExplicitLandingDisarmOffboardOrLinkLoss` |
| `MANUAL` -> `OFFBOARD`: offboard resumes | `ManualEndsOnExplicitLandingDisarmOffboardOrLinkLoss` |
| `MANUAL` -> `DISARMED`: `!armed` | `ManualEndsOnExplicitLandingDisarmOffboardOrLinkLoss` |
| `MANUAL` -> `NO_LINK`: `!connected` | `ManualEndsOnExplicitLandingDisarmOffboardOrLinkLoss` |
| non-`MANUAL` previous + `airborne == UNKNOWN` -> `ARMED` (no stickiness) | `UnknownAirborneWithoutManualBeforeKeepsArmed` |
| `HOVER` <-> `RC_MODE`: `joystick_active` toggling | `OffboardFlightUnchanged`, `AirborneUnknownIrrelevantWhenOffboard` |
| `OFFBOARD` -> `FLYING`: active tracker, `tracker_status.state` | `OffboardFlightUnchanged`, `TrackerStateMapsDirectly`, `AirborneUnknownIrrelevantWhenOffboard` |
| `FLYING` (`LandoffTracker` deactivating, `STATE_IDLE`) -> `TAKEOFF` | `LandoffTrackerIdleIsTakeoff` |
| `FLYING` (`LandoffTracker` activated in the air, `STATE_IDLE`) keeps the flight phase | `LandoffTrackerIdleInFlightIsNotTakeoff` |
| `FLYING` -> `FLYING`: tracker switch (`STATE_INVALID`, not `NullTracker`) keeps the state | `TrackerSwitchInFlightKeepsState` |
| `OFFBOARD` stays: new tracker `STATE_INVALID` on the ground | `NewTrackerOnGroundIsStillOffboard` |
| `FLYING` -> `OFFBOARD`: `NullTracker` after a flight | `NullTrackerAfterFlightIsOffboard` |
| `MANUAL` / `ARMED` / `OFFBOARD` -> `MIDAIR`: `MidairActivationTracker` (any `airborne`) | `MidairActivationTrackerIsMidairActivation` |
| `MIDAIR` -> `DISARMED` / `NO_LINK` | `MidairActivationYieldsToDisarmAndLinkLoss` |
| `MANUAL` -> `MIDAIR` -> (switch, `STATE_INVALID`) -> `GOTO` -> `HOVER` | `MidairActivationSequenceStaysFlying` |
| `MIDAIR` is flying (`is_flying`, `is_flying_autonomously`), maps to `STATE_MIDAIR` | `MidairActivationIsFlying`, `MidairActivationMapsToMessage` |
| `FLYING` -> `EHOVER`: `EmergencyController`, tracker not landing | `EmergencyControllerHoldingIsEhover` |
| `EHOVER` -> `ELAND`: `EmergencyController`, tracker `STATE_LAND` | `EmergencyControllerLandingIsEland` |
| any flight -> `FAILSAFE`: `FailsafeController` | `FailsafeControllerIsFailsafe` |
| `EmergencyController` with `NullTracker` (startup) is no emergency | `EmergencyControllerOnGroundIsNotEhover` |
| joystick (fallback `EmergencyController`) -> `RC_MODE` | `JoystickWinsOverEmergencyController` |
| emergency -> `DISARMED` / `MANUAL` / `NO_LINK` | `EmergencyYieldsToDisarmLinkLossAndManual` |
| `HOVER` -> `EHOVER` -> `ELAND` -> `FAILSAFE` (escalating failsafe) | `EscalatingFailsafeSequence` |
| `MIDAIR` -> `EHOVER` (failed mid-air activation) | `FailedMidairActivationIsEhover` |
| `EHOVER` / `ELAND` / `FAILSAFE` are flying, map to `STATE_*` | `EmergencyStatesAreFlyingAutonomously`, `EmergencyStatesMapToMessage` |
| `airborne` irrelevant once `offboard == true` (closes a known gap) | `AirborneUnknownIrrelevantWhenOffboard` |
