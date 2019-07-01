^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_uav_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* darpa constraints for icp
* Contributors: Matej Petrlik, Matej Petrlik (desktop), NAKI, Tomas Baca, Tomáš Báča, Vojtech Spurny, uav10, uav3, uav42, uav5, uav60

0.0.1 (2019-05-20)
------------------
