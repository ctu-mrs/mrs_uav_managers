# MRS UAV Managers

![](.fig/thumbnail.jpg)

> :warning: **Attention please: This README is outdated.**
>
> The MRS UAV System 1.5 is being released and this page needs updating. Please, keep in mind that the information on this page might not be valid.

This package contains *high-level* flight managers.

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
* allows controlling the UAV using **RC** through our pipeline
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

## EstimationManager

* provides the state estimation of the UAV
* dynamically loads default [state estimator](https://github.com/ctu-mrs/mrs_uav_state_estimators) or custom user-made plugins
* publishes following topics:
  * the `uav_state` topic that is used for feedback control in ControlManager
    * **velocities in `uav_state` are in the `headed/frame_id` frame!!**
  * visualizable `odom` topic 
    * **velocities in `odom` are in the `child_frame_id` frame!! (follows the ROS convention for `nav_msgs/Odometry` msg)**
  * current UAV height above terrain on topic `height_agl` (if available)
  * maximum height above terrain that the UAV can fly in (based on the current estimator used for control)
* provides common services for
  * switching of state estimator currently used in control feedback  
* checks health of current state estimator and switches to a healthy one in case current estimator becomes unhealthy 
* if no estimator is healthy, calls failsafe landing

## TransformManager
* provides the following default TFs from the `fcu` frame (can be turned off/on or renamed by custom configs):
  * `fcu_untilted_origin`, `world_origin`, `local_origin`, `stable_origin`, `fixed_origin`, `utm_origin`, `mapping_origin_tf` (if a SLAM algorithm is running)  
* publishes the delay of odometry produced by a SLAM algorithm (if available) on topic `map_delay` 
* additionally can provide custom TFs from `nav_msgs/Odometry` topics by adding them to the `tf_sources` array in custom config
  * the msg can also be republished in another frame by adding the `frame_id` to the `republish_in_frames` array
