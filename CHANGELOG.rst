^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_uav_managers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.4 (2023-01-20)
------------------
* added big dofec simulation configs
* updated dofec params
* fixed launch, env->optenv
* Contributors: Tomas Baca, Vaclav Pritzl

1.0.3 (2022-05-09)
------------------
* added useful prints
* added midair activatioin to simulation yamls
* default gain manager: allow supersoft gains for aloam estimators
* + rc handoff for eland/failsafe
* added constraints override
* refactored agains publisher handler
* refactored control manager
* updated transformer interface
* added install for forgotten NullTracker
* + install in cmakelists
* small config comments fix
* naki: default to se3controller
* add configs: simulation/naki
* add ScopeTimer and its logger to all managers via CommonHandler; disabled by default
* updated brus configs
* Contributors: Pavel Petracek, Tomas Baca

1.0.2 (2021-10-04)
------------------
* fixed cmd odom attitude initialization bug
* added rc mode to control manager diag
* parametrized rc joystick speed
* fixed escalating failsafe sub-state success check
* softened gains for x500
* throttled failsafe disarming print
* updated brus configs
* added land_home check for previous takeoff
* parametrized landing tracking reference thr
* added position cmd check for landing
* added missing checks on position cmd header
* added brus configs
* added midair activation
* fixed motors on when already on
* added brus simulation configs
* added missing callbacks_enabled\_ and fixed velCmd
* fix control manager crash while velocity reference on the ground
* decreased the land home detection radius
* added virtual destructor implementation to null tracker
* fixed destructors in tracker and controller
* updated rc mode carrot distance
* fixed min/max height check in uav manager
* fixed escalating failsafe initialization
* added dofec uav config
* enabled min height check with 0.5 height
* updated max height check to atually use the height
* fixed controller returning null action
* separated escalating failsafe state machine
* added tight gains for f450
* updated naki gains
* updated default constraints
* updated mass loading for simulation
* added speed publisher
* updated naki gains
* ControlManager: updated trajectory snapping
* ControlManager: updated default constraints
* ControlManager: updated odom timeout
* added t18 configs
* disabled trajectory snapping to safety area (by default)
* added controll/tracker deactivation when motors off
* fixed the odom timeout failsafe bug
* Contributors: Matej Petrlik, Pavel Petracek, Tomas Baca

1.0.1 (2021-05-16)
------------------
* version -> 1.0.1
* updated ros::shutdown
* fix fallback gains for ALOAMREP and x500 uav type
* removed disarm from escalating failsafe
* linted, minor bug fixes
* removed CsvTracker
* remapped the joystick topic to be namespaced and command only a single drone
* added velocity command to control manager
* UavManager: minor changes in minHeightCheck
* fixed transfomer warning when not flying on GPS
* made land_home more insistive
* added min height checking to uav manager
* updated safety area loading
* fixed tf check for safety area publishing
* updated MpcTracker's topic remaps in control_manager.launch
* double -> bool fix, controlTimer -> std::async
* fixed landoff tracker in control manager's diagnostics
* fixed potential land home altitude discrepancy
* added diagnostics publisher to UavManager, warning fixes
* Contributors: Matouš Vrba, Tomas Baca

1.0.0 (2021-03-18)
------------------
* Major release

0.0.6 (2021-03-16)
------------------
* major control pipeline refactoring
* c++ refactoring
* Noetic-compatible
* updated controller and tracker interface
* updated safety features
* dynamic constraints update
* Contributors: Daniel Hert, Matej Petrlik, Matej@NUC, Matouš Vrba, Pavel Petracek, Robert Penicka, Tomas Baca, Tomáš Báča, Viktor Walter, eaglemk2, klaxalk, mergify[bot], mrs, uav20, uav35, uav45, uav46, uav49

0.0.5 (2020-02-26)
------------------
* added deadband around retard mode
* undone the new landing mass factor parameters for landing and elanding
* updated automatic shutdown
* Contributors: Tomas Baca

0.0.4 (2020-02-18)
------------------
* added safety area toggle service
* updated land/eland cutoff params -> 2x faster landing detection
* added land_there service to uav_manager
* added yaw error payload release check
* added emergency ungrip before eland
* emergency reference is transformed
* added bumper params service
* ungripping payload while landing home
* added version checks
* changed the motors on pub to offboard on
* added checks for null tracker to emergency callbacks
* added motors-on publisher
* pushing common handlers to controllers
* sprinkled in some Profiler to ControlManger
* added landing_controller option to UavManager
* removed NullTracker switch before landing
* increased joystick carrot distance
* muted null_trackers callback message
* increased timeouts for profiler routines
* added hiccup detector
* added services for reference validation
* sped up land_home
* updated control manager diag
* added motors to diag
* updated the joystick control fallback
* fixed constraint/gain manger setting bug
* gain/const manager don't give up when they fail
* throttled RC mode prints
* gain/const manager: fallbacks only if neccessary
* control error publish only when flying
* updated compilation flags
* fixed bumper's obstacle hugging
* updated odometry missing timeout
* added maxthrust ungripper
* ControlManager: updated cmd odom publisher
* added gain and constraint diagnostics
* added height to point obstacles
* removed target attitude publisher from control manager
* swtiched from target_att to att_cmd
* added odom callbacks setter
* updated debil land timeout
* added odometry callbacks disabler
* fixed the controller init after null tracker
* returning true when activating already active t/c
* throttled partial landing rinfo
* added getMinHeight service
* added trackerResetStatic method to trackers
* added set_min_height service
* refactored disabling of switching callbacks
* reworked switch-tracker/controller callbacks
* workaraounded point obstacles in latlon_origin
* updated bumper description
* updated control error publisher
* updated uav_names param loading
* fixed wrong return value in resetTracker()
* added area coordinate markers
* updated UAV mass in launch files
* fixed bumper infinity bug
* added constraints publisher
* updated transformer calls
* added OFFBOARD fall-out check with subsequent motors(0)
* added WORLD_NAME parameter to launch files
* updated R-mode params
* retard 2.0
* updated defaut constraints to something more sensible
* fixed bumper safety area transformation
* updated the null attitude command
* fixed common handler initialization before tracker loading
* safety area marker in local origin
* fixed max altitude bug
* added routines for validation of odom. and uav state
* added nan checks for cont. and track. commands
* fixed deadlock during null return check from controllers
* added service for resetting a tracker
* added bumper enabler services
* updated safety area markers
* added mavros gps subscriber
* generalized the transforms
* updated nulltracker's response
* extracted the tf transformer from control manager
* fixed rc channel empty detection bug
* updated get_mutexed calls
* refatored mutexes to use get_mutexed()
* fixed wrong mass publisher type
* fixed wrong integral gains
* fixed high integration gains
* added bumper to mpc tracker
* separated rviz visual markers coming from control manager
* fixed tf bug with asin
* changed fcu services to fcu_untilted
* updated disarming routine
* changed set_reference to just reference
* references are transformed by tfs
* added speed tracker
* added transformer
* added reference transform routine
* added odometry switch to uavStateCallback
* fixed race condition bug when failsafing after controller returns null
* Contributors: Matej Petrlik, Pavel Petracek, Pavel Petráček, Petr Stepan, Tomas Baca, Vit Kratky

0.0.3 (2019-10-25)
------------------
* fixed retard mode rc channels
* added action options to rc eland
* extracted escalating failsafe into a standalone function
* move custom config loading after world file and motor params
* updated rinfos during landing
* updated the max thrust eland
* updated rinfo
* enabled max thrust automatic landing
* added status publisher to bumper
* updated partial landing
* added odometry innovation check
* updated partial landing
* fixed retakeoff
* enabled debil land
* disabling switch controller and tracker callbacks in eland, failsafe and
  debil land
* removed remembering of disturbances after second takeoff
* updated takeoff disturbance remembering
* fixed mass estimate during second takeoff
* fixed disabling of partial landing
* partial_landing -> partial_land
* added partial landing routine
* fixed landing disarming bug
* updated prints
* fixed world name in launch files
* udpated safety timer rates, added run-in-progress check for safety timer
  in control timer
* updated limits for eland and failsafe
* updated safety area interface
* fixed mutexing around new safety area methods
* added disabler for obstacle sectors and point obstacles
* updated control manager launch file
* increased carrot distance for rc joystick
* fixed crashing of ControlManager while retarding during eland
* fixed the activation of the first controller -> eland controller
* typo in NullTracker
* default controller for simulation is Se3
* height checking is optional, eland disarm is optional, minor changes
* added constraint override feature for controllers
* added supersoft gains for optflow for simulated uavs
* added yaw angle error check
* reworked control loop evaluation, added control oneshot timer
* updated control manager launch
* Add pavel slam for Chlumin experiment
* improved the mass difference checking during takeoff
* added controller namespace and name into the interface
* updated toggle of joystic control
* updated logitech joystic channels
* more missing gains for odometry types
* added missing gains to all uav types
* Added new estimators to constraint and gain managers
* increased MpcController eland limit
* added custom configs for controllers and trackers
* updated max thrust warning in uav manager
* fixed wrong NsfController address
* added set_integral_terms service to control_manager launch
* updated bumper condition
* updated the default vertical kq
* updating attitude gains
* updated and tuned simulation gains, slightly updated uav gains
* polishing launch files
* updated launch files
* working on custom configs
* forcing standalone when debugging
* added debug and tested standaloning
* 2nd rehaul of launchfiles
* rehauled launch files
* incresed the z jerk, =1 create takeoff problems
* updated remaps
* updated configs for the new "hierarchy" config model
* delete almost all launchfiles
* reworking launch files
* fixed bumper deadlock bug
* updated getStatus() of tracker and controllers
* removed NullTracker's constructor
* fixed some uninitialized bool variables
* utilizing landoff diagnostics for takeoff
* set eland controllers to MpcController
* fixed carrot_distance type to double in control manager
* generalized takeoff and landing for arbitrary initial z
* removed landing cutoff height from the landing condition
* parametrized carrot distance for rc joystic in control manager
* updated the retard mode to be relative to the reference
* patched the mode mask in when no controller is running
* added the initial body disturbance to control manager
* updated Controller.h interface, addid distrurbance visualization
* added pirouette
* parametrized automatic pc shutdown in control manager
* updated the shutdown routine
* fix in velodyne uav launch file
* added more clear rinfo to tracker reactivation during controller
  switching
* swapped switching of tracker and controller during takeoff
* added missing joybumper tracker parameters to simulation launch files
* new odometry launch files structure
* updated simulation gains and constraints
* added minimum thrust param for NullTracker
* updates in joystic control
* fixed Tomas's controller switching
* fixed joystick controller switching
* joystick channels move to config file
* updated launchfiles with JoyBumperTracker
* fallback tracker+controller for joystic are loaded from configs
* updating controllers even when they are not active
* added odometrySwitchRoutine to controller interface
* added control error publisher to control manager
* added acceleration controller for simulation
* added acceleration controller to simulation launchfile
* updated joystic channel from logitech joystic
* arming after landing is TRUE by default, switching to MPC controller
  after takeoff
* Add temporary solution: transpose to input obstacle matrixes
* Add multiple obstacles to safety_zone
* Added check for path between current position ang goto position
* Change the message type of safe_zone
* Add border polygon publisher
* added new sefety zone
* Change to SafetyZone and Polygon
* Contributors: Tomas Baca

0.0.2 (2019-07-01)
------------------
* added more prints for odometry switch
* tweaks in rc eland trigger before takeoff
* fixed control manager crash during startup while RC eland is triggered
* BRICK + BRICKFLOW estimators
* uav manager triggers eland when takeoff fails
* removed the acceleration publisher
* switched to se3 controller after takeoff
* increased the odometry missing timeout
* fixed failsafe heading bug, fixed deadlock with safety timeout
* fixed reactivation of trackers and controllers during ehover and eland
* mpc is default for t650
* Add MpcController as eland for NAKI
* updated joytracker for t650
* updated rc goto
* switch takeoff tracker for naki
* disabled disarm after large tilt control error
* fixed disarm glitch after switching trackers
* updated the takeoff mass condition
* updated the channel numbers
* fixed the rc channel array check
* updated the retard mode
* increased eland and failsafe thresholds for MpcController
* JoyTracker falls back to MpcController
* fixed active_tracker_idx bug in control manager
* fixed race condition in  switching controllers
* fixed the rc joystic mode
* updated remaps for mpc tracker
* added NullTracker activation in the init
* reworked loading of trackers' and controllers' parameters
* split failsafe and eland conditions for se3, mpc and other
* changed the number of "rc control" channel
* improved comments for the control error failsafe in control_manager.yml
* switched SE3 back to default for takeoff and after takeoff
* switched eland controller to Se3
* added mass estimator publisher to control manager
* constraints are passed to controllers
* increased the odom timeout for simulation
* added rc_joystics wiggle switch
* uav_manager needs SE(3)'s gain manager for takeoff
* gain manager will publish status when SE(3) is not active
* tracker is reactivated upon controller switch
* refurbished failsafes for hector slam
* updating takeoff routine with new control switching
* added tilt failsafe edgecase after switching controllers
* added hector estimator
* added failsafe trigger after unsuccessfull controller update
* failsafe trigger when controller update fails
* updated mavros dependency version
* Contributors: Matej Petrlik, Matej Petrlik (desktop), NAKI, Tomas Baca, Tomáš Báča, Vojtech Spurny, uav10, uav3, uav42, uav5, uav60

0.0.1 (2019-05-20)
------------------
