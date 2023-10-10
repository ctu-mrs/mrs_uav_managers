# MRS UAV Managers

![](.fig/thumbnail.jpg)

This package contains *high-level* flight managers.

> :warning: **Attention please: This page is outdated.**
>
> The MRS UAV System 1.5 is being released and this page needs updating. Plase, keep in mind that the information on this page might not ve valid.

## ControlManager

* integrates the control and tracking part of the MRS UAV pipeline
* dynamically loads [feedback controllers](https://github.com/ctu-mrs/mrs_uav_controllers) and [reference trackers](https://github.com/ctu-mrs/mrs_uav_trackers) plugins
* subscribes to the estimated UAV state and provides it to all loaded trackers and controllers
* runs the iteration of the currently active controller and tracker with every received state estimator message
* outputs the attitude control command to the [HW API](https://github.com/ctu-mrs/mrs_uav_hw_api)
* provides constraints to [trackers](https://github.com/ctu-mrs/mrs_uav_trackers) through a common interface
* allows in-mid-air switching of [feedback controllers](https://github.com/ctu-mrs/mrs_uav_controllers) and [reference trackers](https://github.com/ctu-mrs/mrs_uav_trackers)
* provides common services for
  * setting a **reference**
  * setting a **trajectory reference**
  * human-callable **goto**, **goto relative**, **goto altitude**, **set heading** and **set relative heading**
  * **hover**
  * **emergency hover** after which most of the provided services are disabled
  * **emergency landing** - switches to emergency controller and tracker and lands as fast as possible
  * **failsafe landing** - feedforward landing, only effective in small heights and low speeds
  * **escalating failsafe** - the measures escalate when calling it repeatedly
    1. emergency hover
    2. emergency landing
    3. failsafe landing
    4. disarm
  * transforming references and vectors between frames
  * verifying that references are inside of the allowed safety area
* provides safety routines for checking
  * position control error
  * tilt control error
  * yaw control error
  * estimator innovation error
  * missing estimator data
  * the validity of desired references
* allows controlling the UAV using a ROS-compatible joystick
* allows controlling the UAV using the **RC controller** through our pipeline
  * the RC channels are picked up from [HW API](https://github.com/ctu-mrs/mrs_uav_hw_api)
  * a relative reference is generated for the active tracker
  * **escalating failsafe** can be triggered by a dedicated switch
* provides an **obstacle bumper** feature
  * has priority over user-provided references
  * for slow-flight only

## UavManager

* handles takeoff from the *high-level*
  * checks for pre-condition which might result in takeoff failure
  * switches to the right tracker and controller for the takeoff
  * triggers takeoff
  * switches to the right tracker and controller after the takeoff
* handles a regular landing from the *high-level*
  * descends to a landing height (~2.0 m) using the currently active tracker and controller, given the current constraints
  * switches to the right tracker and controller for the landing
  * triggers the landing
  * monitors the thrust output and decides when the UAV touched the ground
* provides a **land home** feature
  * the UAV flies in the current altitude to its **takeoff** location and lands
* provides a **land there** feature
  * the UAV flies to the desired position at which it lands
* monitors exceeding the **maximum altitude**
  * triggers a priority descend if it exceeds the safety area ceiling
* monitors exceeding the **maximum thrust**
  * triggers a landing if it exceeds a limit
* provides a **flight timer**
  * triggers a landing if it exceeds a limit

## ConstraintManager

* stores and provides sets of dynamics constraints for the [trackers](https://github.com/ctu-mrs/mrs_uav_trackers)
* allows mapping each set of constraints to only some state estimators
* automatically switches to a fallback set of constraints if the current ones are not allowed
* provides a service for switching to a desired set of constraints

## GainManager

* stores and provides sets of gains for the [Se3Controller](https://github.com/ctu-mrs/mrs_uav_controllers)
* allows mapping each set of gains to only some state estimators
* automatically switches to a fallback set of gains if the current ones are not allowed
* provides a service for switching to a desired set of gains
