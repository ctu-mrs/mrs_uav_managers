^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_uav_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* default controller for simulation is So3
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
* switched to so3 controller after takeoff
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
* split failsafe and eland conditions for so3, mpc and other
* changed the number of "rc control" channel
* improved comments for the control error failsafe in control_manager.yml
* switched SO3 back to default for takeoff and after takeoff
* switched eland controller to So3
* added mass estimator publisher to control manager
* constraints are passed to controllers
* increased the odom timeout for simulation
* added rc_joystics wiggle switch
* uav_manager needs SO(3)'s gain manager for takeoff
* gain manager will publish status when SO(3) is not active
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
