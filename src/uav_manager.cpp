/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

#include <nav_msgs/Odometry.h>
#include <mrs_msgs/Vec1.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/BoolStamped.h>
#include <mrs_msgs/LandoffDiagnostics.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/ConstraintManagerDiagnostics.h>
#include <mrs_msgs/GainManagerDiagnostics.h>

#include <mavros_msgs/State.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/mutex.h>

#include <tf/transform_datatypes.h>

//}

namespace mrs_uav_manager
{

namespace uav_manager
{

/* //{ class UavManager */

// state machine
typedef enum
{

  IDLE_STATE,
  FLY_HOME_STATE,
  LANDING_STATE,

} LandingStates_t;

const char* state_names[3] = {

    "IDLING", "FLYING HOME", "LANDING"};

class UavManager : public nodelet::Nodelet {

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

public:
  virtual void onInit();

  bool callbackTakeoff(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackLand(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackLandHome(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  void callbackOdometry(const nav_msgs::OdometryConstPtr& msg);
  void callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg);
  void callbackAttitudeCommand(const mrs_msgs::AttitudeCommandConstPtr& msg);
  void callbackMavrosState(const mavros_msgs::StateConstPtr& msg);
  void callbackAttitudeCmd(const mrs_msgs::AttitudeCommandConstPtr& msg);
  void callbackMaxHeight(const mrs_msgs::Float64StampedConstPtr& msg);
  void callbackHeight(const mrs_msgs::Float64StampedConstPtr& msg);
  void callbackGains(const mrs_msgs::GainManagerDiagnosticsConstPtr& msg);
  void callbackConstraints(const mrs_msgs::ConstraintManagerDiagnosticsConstPtr& msg);
  void callbackMotors(const mrs_msgs::BoolStampedConstPtr& msg);

  void changeLandingState(LandingStates_t new_state);

  // odometry subscriber
  ros::Subscriber    subscriber_odometry_;
  nav_msgs::Odometry odometry_;
  double             odometry_yaw_;
  double             odometry_roll_;
  double             odometry_pitch_;
  std::mutex         mutex_odometry_;
  bool               got_odometry_ = false;

  // odometry subscriber
  ros::Timer      max_height_timer_;
  ros::Subscriber subscriber_max_height_;
  double          _max_height_;
  std::mutex      mutex_max_height_;
  bool            got_max_height_    = false;
  bool            fixing_max_height_ = false;
  // params
  bool   _max_height_enabled_ = false;
  int    _max_height_checking_rate_;
  double _max_height_offset_;

  // height subscriber
  ros::Subscriber subscriber_height_;
  double          height_;
  std::mutex      mutex_height_;
  bool            got_height_ = false;

  // mass estimation during landing
  double    thrust_mass_estimate_;
  bool      thrust_under_threshold_ = false;
  ros::Time thrust_mass_estimate_first_time_;

  // diagnostics from landoff tracker
  void                         callbackLandoffDiagnostics(const mrs_msgs::LandoffDiagnosticsConstPtr& msg);
  ros::Subscriber              subscriber_landoff_diagnostics_;
  mrs_msgs::LandoffDiagnostics landoff_diagnostics_;
  std::mutex                   mutex_landoff_diagnostics_;
  bool                         got_landoff_diagnostics_ = false;

  // subscriber for motors on/off
  ros::Subscriber       subscriber_motors_;
  mrs_msgs::BoolStamped motors_;
  std::mutex            mutex_motors_;
  bool                  got_motors_ = false;

  // subscriber for gains from gain manager
  ros::Subscriber                  subscriber_gain_diagnostics_;
  ros::Time                        gains_last_time_;
  mrs_msgs::GainManagerDiagnostics gains_;
  std::mutex                       mutex_gains_;
  bool                             _gain_manager_required_ = false;

  // subscriber for constraints from constraint manager
  ros::Subscriber                        subscriber_constraint_manager_diagnostics_;
  ros::Time                              constraints_last_time_;
  mrs_msgs::ConstraintManagerDiagnostics constraints_;
  std::mutex                             mutex_constraints_;
  bool                                   _constraint_manager_required_ = false;

  // subscriber for control manager diagnostics
  ros::Subscriber                     subscriber_control_manager_diagnostics_;
  bool                                got_control_manager_diagnostics_ = false;
  mrs_msgs::ControlManagerDiagnostics control_manager_diagnostics_;
  std::mutex                          mutex_control_manager_diagnostics_;

  // subscriber for target attitude
  ros::Subscriber           subscriber_attitude_cmd_;
  bool                      got_attitude_cmd_ = false;
  mrs_msgs::AttitudeCommand attitude_cmd_;
  std::mutex                mutex_attitude_cmd_;

  // subscriber for mavros state
  ros::Subscriber    subscriber_mavros_state_;
  mavros_msgs::State mavros_state_;
  std::mutex         mutex_mavros_state_;
  bool               got_mavros_state_ = false;

  // service servers
  ros::ServiceServer service_server_takeoff_;
  ros::ServiceServer service_server_land_;
  ros::ServiceServer service_server_land_home_;

  // service clients
  ros::ServiceClient service_client_takeoff_;
  ros::ServiceClient service_client_switch_tracker_;
  ros::ServiceClient service_client_switch_controller_;
  ros::ServiceClient service_client_land_;
  ros::ServiceClient service_client_eland_;
  ros::ServiceClient service_client_motors_;
  ros::ServiceClient service_client_enabled_callbacks_;
  ros::ServiceClient service_client_emergency_reference_;
  ros::ServiceClient service_client_arm_;
  ros::ServiceClient service_client_pirouette_;
  ros::ServiceClient service_client_set_odometry_callbacks_;

  std::mutex mutex_services_;

  // saved takeoff coordinates
  double      takeoff_x_;
  double      takeoff_y_;
  std::string takeoff_frame_id_;

  // to which height to takeoff
  double _takeoff_height_;

  // names of important trackers
  std::string _null_tracker_name_;
  std::string _partial_landing_controller_name_;

  // Takeoff timer
  ros::Timer takeoff_timer_;
  double     _takeoff_timer_rate_;
  bool       takingoff_            = false;
  int        number_of_takeoffs_   = 0;
  double     last_mass_difference_ = 0;
  std::mutex mutex_last_mass_difference_;
  bool       waiting_for_takeoff_ = false;

  // after takeoff
  std::string _after_takeoff_tracker_name_;
  std::string _after_takeoff_controller_name_;
  std::string _takeoff_tracker_name_;
  std::string _takeoff_controller_name_;
  bool        _after_takeoff_pirouette_ = false;

  // Landing timer
  ros::Timer  landing_timer_;
  std::string _landing_tracker_name_;
  double      _landing_cutoff_mass_factor_;
  double      _landing_cutoff_mass_timeout_;
  double      _landing_timer_rate_;
  bool        landing_ = false;
  double      _uav_mass_;
  double      _g_;
  double      landing_uav_mass_;
  bool        _landing_disarm_ = false;
  double      _hover_thrust_a_;
  double      _hover_thrust_b_;

  // landing state machine states
  LandingStates_t current_state_landing_  = IDLE_STATE;
  LandingStates_t previous_state_landing_ = IDLE_STATE;

  // timer callbacks
  void landingTimer(const ros::TimerEvent& event);
  void takeoffTimer(const ros::TimerEvent& event);
  void maxHeightTimer(const ros::TimerEvent& event);
  void flighttimeTimer(const ros::TimerEvent& event);
  void maxthrustTimer(const ros::TimerEvent& event);

  // Timer for checking max flight time
  ros::Timer flighttime_timer_;
  double     _flighttime_timer_rate_;
  double     _flighttime_max_time_;
  bool       _flighttime_timer_enabled_ = false;
  double     flighttime_                = 0;

  // Timer for checking maximum thrust
  ros::Timer maxthrust_timer_;
  bool       _maxthrust_timer_enabled_ = false;
  double     _maxthrust_timer_rate_;
  double     _maxthrust_max_thrust_;
  double     _maxthrust_eland_timeout_;
  double     _maxthrust_ungrip_timeout_;
  bool       maxthrust_above_threshold_ = false;
  ros::Time  maxthrust_first_time_;

  // profiler
  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  void setOdometryCallbacks(const bool input);

  void               ungrip(void);
  ros::ServiceClient service_client_ungrip_;
};

//}

/* //{ onInit() */

void UavManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "UavManager");

  ROS_INFO("[UavManager]: initializing");

  param_loader.load_param("enable_profiler", _profiler_enabled_);

  param_loader.load_param("null_tracker", _null_tracker_name_);
  param_loader.load_param("partial_landing_controller", _partial_landing_controller_name_);

  param_loader.load_param("takeoff/rate", _takeoff_timer_rate_);
  param_loader.load_param("takeoff/after_takeoff/tracker", _after_takeoff_tracker_name_);
  param_loader.load_param("takeoff/after_takeoff/controller", _after_takeoff_controller_name_);
  param_loader.load_param("takeoff/after_takeoff/pirouette", _after_takeoff_pirouette_);
  param_loader.load_param("takeoff/during_takeoff/tracker", _takeoff_tracker_name_);
  param_loader.load_param("takeoff/during_takeoff/controller", _takeoff_controller_name_);
  param_loader.load_param("takeoff/takeoff_height", _takeoff_height_);

  param_loader.load_param("landing/rate", _landing_timer_rate_);
  param_loader.load_param("landing/landing_tracker", _landing_tracker_name_);
  param_loader.load_param("landing/landing_cutoff_mass_factor", _landing_cutoff_mass_factor_);
  param_loader.load_param("landing/landing_cutoff_timeout", _landing_cutoff_mass_timeout_);
  param_loader.load_param("landing/disarm", _landing_disarm_);

  param_loader.load_param("uav_mass", _uav_mass_);
  param_loader.load_param("g", _g_);

  param_loader.load_param("hover_thrust/a", _hover_thrust_a_);
  param_loader.load_param("hover_thrust/b", _hover_thrust_b_);

  param_loader.load_param("max_height_checking/enabled", _max_height_enabled_);
  param_loader.load_param("max_height_checking/rate", _max_height_checking_rate_);
  param_loader.load_param("max_height_checking/safety_height_offset", _max_height_offset_);

  param_loader.load_param("safety_area/max_height", _max_height_);

  param_loader.load_param("require_gain_manager", _gain_manager_required_);
  param_loader.load_param("require_constraint_manager", _constraint_manager_required_);

  param_loader.load_param("flight_timer/enabled", _flighttime_timer_enabled_);
  param_loader.load_param("flight_timer/rate", _flighttime_timer_rate_);
  param_loader.load_param("flight_timer/max_time", _flighttime_max_time_);

  param_loader.load_param("max_thrust/enabled", _maxthrust_timer_enabled_);
  param_loader.load_param("max_thrust/rate", _maxthrust_timer_rate_);
  param_loader.load_param("max_thrust/max_thrust", _maxthrust_max_thrust_);
  param_loader.load_param("max_thrust/eland_timeout", _maxthrust_eland_timeout_);
  param_loader.load_param("max_thrust/ungrip_timeout", _maxthrust_ungrip_timeout_);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[UavManager]: Could not load all parameters!");
    ros::shutdown();
  }

  // --------------------------------------------------------------
  // |             Initialize subscribers and services            |
  // --------------------------------------------------------------

  subscriber_odometry_     = nh_.subscribe("odometry_in", 1, &UavManager::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
  subscriber_attitude_cmd_ = nh_.subscribe("attitude_cmd_in", 1, &UavManager::callbackAttitudeCmd, this, ros::TransportHints().tcpNoDelay());
  subscriber_mavros_state_ = nh_.subscribe("mavros_state_in", 1, &UavManager::callbackMavrosState, this, ros::TransportHints().tcpNoDelay());
  subscriber_max_height_   = nh_.subscribe("max_height_in", 1, &UavManager::callbackMaxHeight, this, ros::TransportHints().tcpNoDelay());
  subscriber_height_       = nh_.subscribe("height_in", 1, &UavManager::callbackHeight, this, ros::TransportHints().tcpNoDelay());
  subscriber_motors_       = nh_.subscribe("motors_in", 1, &UavManager::callbackMotors, this, ros::TransportHints().tcpNoDelay());
  subscriber_landoff_diagnostics_ =
      nh_.subscribe("landoff_diagnostics_in", 1, &UavManager::callbackLandoffDiagnostics, this, ros::TransportHints().tcpNoDelay());
  subscriber_control_manager_diagnostics_ =
      nh_.subscribe("control_manager_diagnostics_in", 1, &UavManager::callbackControlManagerDiagnostics, this, ros::TransportHints().tcpNoDelay());

  subscriber_gain_diagnostics_ = nh_.subscribe("gain_manager_diagnostics_in", 1, &UavManager::callbackGains, this, ros::TransportHints().tcpNoDelay());
  gains_last_time_             = ros::Time(0);

  subscriber_constraint_manager_diagnostics_ =
      nh_.subscribe("constraint_manager_diagnostics_in", 1, &UavManager::callbackConstraints, this, ros::TransportHints().tcpNoDelay());
  constraints_last_time_ = ros::Time(0);

  service_server_takeoff_   = nh_.advertiseService("takeoff_in", &UavManager::callbackTakeoff, this);
  service_server_land_      = nh_.advertiseService("land_in", &UavManager::callbackLand, this);
  service_server_land_home_ = nh_.advertiseService("land_home_in", &UavManager::callbackLandHome, this);

  service_client_takeoff_                = nh_.serviceClient<mrs_msgs::Vec1>("takeoff_out");
  service_client_land_                   = nh_.serviceClient<std_srvs::Trigger>("land_out");
  service_client_eland_                  = nh_.serviceClient<std_srvs::Trigger>("eland_out");
  service_client_switch_tracker_         = nh_.serviceClient<mrs_msgs::String>("switch_tracker_out");
  service_client_switch_controller_      = nh_.serviceClient<mrs_msgs::String>("switch_controller_out");
  service_client_motors_                 = nh_.serviceClient<std_srvs::SetBool>("motors_out");
  service_client_emergency_reference_    = nh_.serviceClient<mrs_msgs::ReferenceStampedSrv>("emergency_reference_out");
  service_client_enabled_callbacks_      = nh_.serviceClient<std_srvs::SetBool>("enable_callbacks_out");
  service_client_arm_                    = nh_.serviceClient<std_srvs::SetBool>("arm_out");
  service_client_pirouette_              = nh_.serviceClient<std_srvs::Trigger>("pirouette_out");
  service_client_set_odometry_callbacks_ = nh_.serviceClient<std_srvs::SetBool>("set_odometry_callbacks_out");
  service_client_ungrip_                 = nh_.serviceClient<std_srvs::Trigger>("ungrip_out");

  // --------------------------------------------------------------
  // |                    landing state machine                   |
  // --------------------------------------------------------------

  changeLandingState(IDLE_STATE);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler_ = mrs_lib::Profiler(nh_, "UavManager", _profiler_enabled_);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  landing_timer_    = nh_.createTimer(ros::Rate(_landing_timer_rate_), &UavManager::landingTimer, this, false, false);
  takeoff_timer_    = nh_.createTimer(ros::Rate(_takeoff_timer_rate_), &UavManager::takeoffTimer, this, false, false);
  flighttime_timer_ = nh_.createTimer(ros::Rate(_flighttime_timer_rate_), &UavManager::flighttimeTimer, this, false, false);
  maxthrust_timer_  = nh_.createTimer(ros::Rate(_maxthrust_timer_rate_), &UavManager::maxthrustTimer, this, false, false);

  if (_max_height_enabled_) {
    max_height_timer_ = nh_.createTimer(ros::Rate(_max_height_checking_rate_), &UavManager::maxHeightTimer, this);
  }

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[UavManager]: initilized");
}

//}

// | ---------------------- state machine --------------------- |

/* //{ changeLandingState() */

void UavManager::changeLandingState(LandingStates_t new_state) {

  // copy member variables
  auto attitude_cmd = mrs_lib::get_mutexed(mutex_attitude_cmd_, attitude_cmd_);

  previous_state_landing_ = current_state_landing_;
  current_state_landing_  = new_state;

  switch (current_state_landing_) {

    case IDLE_STATE:
      break;
    case FLY_HOME_STATE:
      break;
    case LANDING_STATE: {

      landing_uav_mass_ = _uav_mass_ + attitude_cmd.mass_difference;
    } break;
  }

  // just for ROS_INFO
  ROS_INFO("[UavManager]: Switching landing state %s -> %s", state_names[previous_state_landing_], state_names[current_state_landing_]);
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* //{ landingTimer() */

void UavManager::landingTimer(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("landingTimer", _landing_timer_rate_, 0.01, event);

  // copy member variables
  auto control_manager_diagnostics = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);
  auto attitude_cmd                = mrs_lib::get_mutexed(mutex_attitude_cmd_, attitude_cmd_);
  auto odometry                    = mrs_lib::get_mutexed(mutex_odometry_, odometry_);

  double odometry_x, odometry_y;
  odometry_x = odometry.pose.pose.position.x;
  odometry_y = odometry.pose.pose.position.y;

  if (current_state_landing_ == IDLE_STATE) {

    return;

  } else if (current_state_landing_ == FLY_HOME_STATE) {

    // TODO: parametrize the radius
    if (sqrt(pow(odometry_x - takeoff_x_, 2) + pow(odometry_y - takeoff_y_, 2)) < 0.5) {

      // TODO: parametrize the timeout
      ros::Duration wait(5.0);
      wait.sleep();

      ROS_INFO("[UavManager]: landing");

      mrs_msgs::String switch_tracker_out;
      switch_tracker_out.request.value = _landing_tracker_name_;
      if (!service_client_switch_tracker_.call(switch_tracker_out)) {
        ROS_ERROR("[UavManager]: service call for switching to tracker %s failed", _landing_tracker_name_.c_str());
        changeLandingState(LANDING_STATE);
      }

      if (switch_tracker_out.response.success == true) {

        std_srvs::Trigger land_out;
        if (service_client_land_.call(land_out)) {

          if (land_out.response.success) {

            changeLandingState(LANDING_STATE);

          } else {

            ROS_ERROR("[UavManager]: service call for landing was unsuccessful: %s", land_out.response.message.c_str());
            changeLandingState(IDLE_STATE);
          }

        } else {

          ROS_ERROR("[UavManager]: service call for landing failed");
          changeLandingState(IDLE_STATE);
        }

      } else {

        ROS_ERROR("[UavManager]: could not switch to tracker %s: %s", _landing_tracker_name_.c_str(), switch_tracker_out.response.message.c_str());
        changeLandingState(IDLE_STATE);
      }
    }

  } else if (current_state_landing_ == LANDING_STATE) {


    if (_landing_tracker_name_.compare(control_manager_diagnostics.tracker_status.tracker) == 0) {

      // recalculate the mass based on the thrust
      thrust_mass_estimate_ = pow((attitude_cmd.thrust - _hover_thrust_b_) / _hover_thrust_a_, 2) / _g_;
      ROS_INFO_THROTTLE(1.0, "[UavManager]: landing: initial mass: %.2f thrust mass estimate: %.2f", landing_uav_mass_, thrust_mass_estimate_);

      // condition for automatic motor turn off
      if (((thrust_mass_estimate_ < _landing_cutoff_mass_factor_ * landing_uav_mass_) || attitude_cmd.thrust < 0.01)) {

        if (!thrust_under_threshold_) {

          thrust_mass_estimate_first_time_ = ros::Time::now();
          thrust_under_threshold_          = true;
        }

        ROS_INFO_THROTTLE(0.1, "[UavManager]: thrust is under cutoff factor for %.2f s", (ros::Time::now() - thrust_mass_estimate_first_time_).toSec());

      } else {

        thrust_under_threshold_ = false;
      }

      if (thrust_under_threshold_ && ((ros::Time::now() - thrust_mass_estimate_first_time_).toSec() > _landing_cutoff_mass_timeout_)) {

        if (current_state_landing_ == LANDING_STATE) {

          mrs_msgs::String switch_tracker_out;
          switch_tracker_out.request.value = _null_tracker_name_;
          service_client_switch_tracker_.call(switch_tracker_out);

          std_srvs::SetBool enable_callbacks_out;
          enable_callbacks_out.request.data = true;
          service_client_enabled_callbacks_.call(enable_callbacks_out);

          if (_landing_disarm_) {

            ROS_INFO("[UavManager]: disarming after landing");

            std_srvs::SetBool arm_out;
            arm_out.request.data = false;
            service_client_arm_.call(arm_out);
          }

          changeLandingState(IDLE_STATE);

          ROS_INFO("[UavManager]: landing finished");

        } else {  // emergency landing

          std_srvs::SetBool arm_out;
          arm_out.request.data = false;
          service_client_arm_.call(arm_out);

          std_srvs::SetBool enable_callbacks_out;
          enable_callbacks_out.request.data = true;
          service_client_enabled_callbacks_.call(enable_callbacks_out);

          changeLandingState(IDLE_STATE);

          ROS_WARN("[UavManager]: emergency landing finished");
        }

        landing_timer_.stop();
      }

    } else {

      ROS_ERROR("[UavManager]: incorrect tracker detected during landing!");
    }
  }
}

//}

/* //{ takeoffTimer() */

void UavManager::takeoffTimer(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("takeoffTimer", _takeoff_timer_rate_, 0.004, event);

  auto landoff_diagnostics = mrs_lib::get_mutexed(mutex_landoff_diagnostics_, landoff_diagnostics_);

  if (waiting_for_takeoff_) {

    if (landoff_diagnostics.taking_off) {

      waiting_for_takeoff_ = false;
    } else {

      ROS_WARN_THROTTLE(1.0, "[UavManager]: waiting for takeoff confirmation from LandoffTracker");
      return;
    }
  }

  if (takingoff_) {

    if (!landoff_diagnostics.taking_off) {

      ROS_INFO("[UavManager]: take off finished, switching to %s", _after_takeoff_tracker_name_.c_str());

      // if enabled, start the timer for landing after reaching max thrust
      if (_maxthrust_timer_enabled_) {
        maxthrust_timer_.start();
      }

      mrs_msgs::String switch_tracker_out;
      switch_tracker_out.request.value = _after_takeoff_tracker_name_;
      service_client_switch_tracker_.call(switch_tracker_out);

      if (switch_tracker_out.response.success == true) {

        ROS_INFO("[UavManager]: switched to %s", _after_takeoff_tracker_name_.c_str());

      } else {

        ROS_ERROR("[UavManager]: could not switch to %s: %s", _after_takeoff_tracker_name_.c_str(), switch_tracker_out.response.message.c_str());
      }

      mrs_msgs::String switch_controller_out;
      switch_controller_out.request.value = _after_takeoff_controller_name_;
      service_client_switch_controller_.call(switch_controller_out);

      if (switch_controller_out.response.success == true) {

        ROS_INFO("[UavManager]: switched to %s", _after_takeoff_controller_name_.c_str());

      } else {

        ROS_ERROR("[UavManager]: could not switch to %s: %s", _after_takeoff_controller_name_.c_str(), switch_controller_out.response.message.c_str());
      }

      setOdometryCallbacks(true);

      if (_after_takeoff_pirouette_) {

        std_srvs::Trigger pirouette_out;
        service_client_pirouette_.call(pirouette_out);

        if (pirouette_out.response.success == true) {

          ROS_INFO("[UavManager]: initiated after takeoff pirouette");

        } else {

          ROS_INFO("[UavManager]: pirouette not successfull: %s", pirouette_out.response.message.c_str());
        }
      }

      takeoff_timer_.stop();
    }
  }
}

//}

/* //{ maxHeightTimer() */

void UavManager::maxHeightTimer(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("maxHeightTimer", _max_height_checking_rate_, 0.004, event);

  auto max_height               = mrs_lib::get_mutexed(mutex_max_height_, _max_height_);
  auto [odometry, odometry_yaw] = mrs_lib::get_mutexed(mutex_odometry_, odometry_, odometry_yaw_);

  double odometry_x, odometry_y, odometry_z;
  odometry_x = odometry.pose.pose.position.x;
  odometry_y = odometry.pose.pose.position.y;
  odometry_z = odometry.pose.pose.position.z;

  double odometry_x_speed, odometry_y_speed, odometry_z_speed;
  odometry_x_speed = odometry.twist.twist.linear.x;
  odometry_y_speed = odometry.twist.twist.linear.y;
  odometry_z_speed = odometry.twist.twist.linear.z;

  if (!got_max_height_ || !got_odometry_) {
    return;
  }

  if (!fixing_max_height_) {

    if (odometry_z > max_height + 0.25) {

      ROS_WARN("[UavManager]: max height exceeded: %.2f >  %.2f, triggering safety goto", odometry_z, max_height);

      // get the current odometry
      double current_horizontal_speed = sqrt(pow(odometry_x_speed, 2.0) + pow(odometry_y_speed, 2.0));
      double current_heading          = atan2(odometry_y_speed, odometry_x_speed);

      double horizontal_t_stop    = current_horizontal_speed / 1.0;
      double horizontal_stop_dist = (horizontal_t_stop * current_horizontal_speed) / 2.0;
      double stop_dist_x          = cos(current_heading) * horizontal_stop_dist;
      double stop_dist_y          = sin(current_heading) * horizontal_stop_dist;

      mrs_msgs::ReferenceStampedSrv reference_out;
      reference_out.request.header.frame_id = odometry.header.frame_id;
      reference_out.request.header.stamp    = ros::Time::now();

      reference_out.request.reference.position.x = odometry_x + stop_dist_x;
      reference_out.request.reference.position.y = odometry_y + stop_dist_y;
      reference_out.request.reference.position.z = max_height - fabs(_max_height_offset_);
      reference_out.request.reference.yaw        = odometry_yaw;

      {
        std::scoped_lock lock(mutex_services_);

        service_client_emergency_reference_.call(reference_out);
      }

      if (reference_out.response.success == true) {

        ROS_INFO("[UavManager]: descending");

        fixing_max_height_ = true;

      } else {

        ROS_ERROR("[UavManager]: goto failed: %s", reference_out.response.message.c_str());
      }
    }

  } else {

    if (odometry_z < max_height) {

      std_srvs::SetBool enable_callbacks_out;
      enable_callbacks_out.request.data = true;
      service_client_enabled_callbacks_.call(enable_callbacks_out);

      ROS_WARN("[UavManager]: safety height reached, enabling callbacks");

      fixing_max_height_ = false;
    }
  }
}

//}

/* //{ flighttimeTimer() */

void UavManager::flighttimeTimer(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("flighttimeTimer", _flighttime_timer_rate_, 0.1, event);

  flighttime_ += 1.0 / _flighttime_timer_rate_;

  if (flighttime_ > _flighttime_max_time_) {

    flighttime_ = 0;
    flighttime_timer_.stop();

    ROS_INFO("[UavManager]: max flight time achieved, landing");

    mrs_msgs::String switch_tracker_out;
    switch_tracker_out.request.value = _landing_tracker_name_;
    service_client_switch_tracker_.call(switch_tracker_out);

    std_srvs::Trigger land_out;
    if (switch_tracker_out.response.success == true) {

      service_client_land_.call(land_out);

      ros::Duration wait(1.0);
      wait.sleep();

      changeLandingState(LANDING_STATE);

      landing_timer_.start();

    } else {

      changeLandingState(IDLE_STATE);
    }
  }
}

//}

/* //{ maxthrustTimer() */

void UavManager::maxthrustTimer(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("maxthrustTimer", _maxthrust_timer_rate_, 0.002, event);

  // copy member variables
  auto attitude_cmd = mrs_lib::get_mutexed(mutex_attitude_cmd_, attitude_cmd_);

  if (attitude_cmd.thrust >= _maxthrust_max_thrust_) {

    if (!maxthrust_above_threshold_) {

      maxthrust_first_time_      = ros::Time::now();
      maxthrust_above_threshold_ = true;
      ROS_WARN("[UavManager]: max thrust exceeded threshold (%.2f/%.2f)", attitude_cmd.thrust, _maxthrust_max_thrust_);

    } else {

      ROS_WARN_THROTTLE(0.1, "[UavManager]: thrust over threshold (%.2f/%.2f) for %.2f s", attitude_cmd.thrust, _maxthrust_max_thrust_,
                        (ros::Time::now() - maxthrust_first_time_).toSec());
    }

  } else {

    maxthrust_above_threshold_ = false;
  }

  if (maxthrust_above_threshold_ && (ros::Time::now() - maxthrust_first_time_).toSec() > _maxthrust_ungrip_timeout_) {

    ROS_WARN_THROTTLE(1.0, "[UavManager]: thrust over threshold (%.2f/%.2f) for more than %.2f s, ungripping payload", attitude_cmd.thrust,
                      _maxthrust_max_thrust_, _maxthrust_ungrip_timeout_);

    ungrip();
  }

  if (maxthrust_above_threshold_ && (ros::Time::now() - maxthrust_first_time_).toSec() > _maxthrust_eland_timeout_) {

    maxthrust_timer_.stop();

    ROS_ERROR("[UavManager]: thrust over threshold (%.2f/%.2f) for more than %.2f s, calling emergency landing", attitude_cmd.thrust, _maxthrust_max_thrust_,
              _maxthrust_eland_timeout_);

    mrs_msgs::String switch_tracker_out;
    switch_tracker_out.request.value = _landing_tracker_name_;
    service_client_switch_tracker_.call(switch_tracker_out);

    std_srvs::Trigger land_out;
    if (switch_tracker_out.response.success == true) {

      service_client_land_.call(land_out);

      ros::Duration wait(1.0);
      wait.sleep();

      changeLandingState(LANDING_STATE);

      landing_timer_.start();

    } else {

      changeLandingState(IDLE_STATE);
    }
  }
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | --------------------- topic callbacks -------------------- |

/* //{ callbackTrackerStatus() */

void UavManager::callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackControlManagerDiagnostics");

  {
    std::scoped_lock lock(mutex_control_manager_diagnostics_);

    control_manager_diagnostics_ = *msg;

    got_control_manager_diagnostics_ = true;
  }
}

//}

/* //{ callbackMavrosState() */

void UavManager::callbackMavrosState(const mavros_msgs::StateConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackMavrosState");

  {
    std::scoped_lock lock(mutex_mavros_state_);

    mavros_state_ = *msg;

    got_mavros_state_ = true;
  }
}

//}

/* //{ callbackAttitudeCmd() */

void UavManager::callbackAttitudeCmd(const mrs_msgs::AttitudeCommandConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackAttitudeCmd");

  {
    std::scoped_lock lock(mutex_attitude_cmd_);

    attitude_cmd_ = *msg;

    got_attitude_cmd_ = true;
  }
}

//}

/* //{ callbackMaxHeight() */

void UavManager::callbackMaxHeight(const mrs_msgs::Float64StampedConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackMaxHeight");

  {
    std::scoped_lock lock(mutex_max_height_);

    _max_height_ = msg->value;

    got_max_height_ = true;
  }
}

//}

/* //{ callbackHeight() */

void UavManager::callbackHeight(const mrs_msgs::Float64StampedConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackHeight");

  {
    std::scoped_lock lock(mutex_height_);

    height_ = msg->value;

    got_height_ = true;
  }
}

//}

/* //{ callbackGains() */

void UavManager::callbackGains(const mrs_msgs::GainManagerDiagnosticsConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackGains");

  {
    std::scoped_lock lock(mutex_gains_);

    gains_           = *msg;
    gains_last_time_ = ros::Time::now();
  }
}

//}

/* //{ callbackConstraints() */

void UavManager::callbackConstraints(const mrs_msgs::ConstraintManagerDiagnosticsConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackConstraints");

  {
    std::scoped_lock lock(mutex_constraints_);

    constraints_           = *msg;
    constraints_last_time_ = ros::Time::now();
  }
}

//}

/* //{ callbackOdometry() */

void UavManager::callbackOdometry(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackOdometry");

  {
    std::scoped_lock lock(mutex_odometry_);

    odometry_ = *msg;

    // calculate the euler angles
    tf::Quaternion quaternion_odometry;
    quaternionMsgToTF(odometry_.pose.pose.orientation, quaternion_odometry);
    tf::Matrix3x3 m(quaternion_odometry);
    m.getRPY(odometry_roll_, odometry_pitch_, odometry_yaw_);

    got_odometry_ = true;
  }
}

//}

/* //{ callbackMotors() */

void UavManager::callbackMotors(const mrs_msgs::BoolStampedConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackMotors");

  {
    std::scoped_lock lock(mutex_motors_);

    motors_ = *msg;

    got_motors_ = true;
  }
}

//}

/* //{ callbackLandoffDiagnostics() */

void UavManager::callbackLandoffDiagnostics(const mrs_msgs::LandoffDiagnosticsConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackLandoffDiagnostics");

  {
    std::scoped_lock lock(mutex_landoff_diagnostics_);

    landoff_diagnostics_ = *msg;

    got_landoff_diagnostics_ = true;
  }
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackTakeoff() */

bool UavManager::callbackTakeoff([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  // copy member variables
  auto odometry                    = mrs_lib::get_mutexed(mutex_odometry_, odometry_);
  auto motors                      = mrs_lib::get_mutexed(mutex_motors_, motors_);
  auto gains_last_time             = mrs_lib::get_mutexed(mutex_gains_, gains_last_time_);
  auto constraints_last_time       = mrs_lib::get_mutexed(mutex_constraints_, constraints_last_time_);
  auto control_manager_diagnostics = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);
  auto mavros_state                = mrs_lib::get_mutexed(mutex_mavros_state_, mavros_state_);
  auto last_mass_difference        = mrs_lib::get_mutexed(mutex_last_mass_difference_, last_mass_difference_);

  double odometry_x, odometry_y;
  odometry_x = odometry.pose.pose.position.x;
  odometry_y = odometry.pose.pose.position.y;

  char message[200];

  if (!got_odometry_) {
    sprintf((char*)&message, "Can't takeoff, missing odometry!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_mavros_state_ || (ros::Time::now() - mavros_state.header.stamp).toSec() > 5.0) {
    sprintf((char*)&message, "Can't takeoff, missing mavros state!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!mavros_state.armed) {
    sprintf((char*)&message, "Can't takeoff, UAV not armed!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (mavros_state.mode.compare(std::string("OFFBOARD")) != 0) {
    sprintf((char*)&message, "Can't takeoff, UAV not in offboard mode!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_control_manager_diagnostics_) {
    sprintf((char*)&message, "Can't takeoff, missing control manager diagnostics!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_attitude_cmd_) {
    sprintf((char*)&message, "Can't takeoff, missing target attitude!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_max_height_) {
    sprintf((char*)&message, "Can't takeoff, missing max height");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_height_) {
    sprintf((char*)&message, "Can't takeoff, missing height");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_landoff_diagnostics_) {
    sprintf((char*)&message, "Can't takeoff, missing landoff diagnostics");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (_null_tracker_name_.compare(control_manager_diagnostics.tracker_status.tracker) != 0 &&
      _partial_landing_controller_name_.compare(control_manager_diagnostics.controller_status.controller) != 0) {
    sprintf((char*)&message, "Can't takeoff, need '%s' to be active!", _null_tracker_name_.c_str());
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (_gain_manager_required_ && (ros::Time::now() - gains_last_time).toSec() > 5.0) {
    sprintf((char*)&message, "Can't takeoff, GainManager is not running!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (_constraint_manager_required_ && (ros::Time::now() - constraints_last_time).toSec() > 5.0) {
    sprintf((char*)&message, "Can't takeoff, ConstraintManager is not running!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_motors_ || (ros::Time::now() - motors.stamp).toSec() > 1.0 || !motors.data) {
    sprintf((char*)&message, "Can't takeoff, motors are off!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (number_of_takeoffs_ > 0) {

    if (!got_attitude_cmd_) {

      sprintf((char*)&message, "Can't takeoff, missing attitude command!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[UavManager]: %s", message);
      return true;
    }

    if (last_mass_difference > 0.5) {

      sprintf((char*)&message, "Can't takeoff, estimated mass difference is too large: %.2f!", last_mass_difference);
      res.message = message;
      res.success = false;
      ROS_ERROR("[UavManager]: %s", message);
      return true;
    }
  }

  ROS_INFO("[UavManager]: taking off");

  setOdometryCallbacks(false);

  mrs_msgs::String switch_controller_out;
  switch_controller_out.request.value = _takeoff_controller_name_;
  service_client_switch_controller_.call(switch_controller_out);

  mrs_msgs::String switch_tracker_out;
  switch_tracker_out.request.value = _takeoff_tracker_name_;
  service_client_switch_tracker_.call(switch_tracker_out);

  mrs_msgs::Vec1 takeoff_out;
  takeoff_out.request.goal = _takeoff_height_;

  if (switch_tracker_out.response.success == true && switch_controller_out.response.success == true) {

    service_client_takeoff_.call(takeoff_out);

    // if the takeoff was not successful, switch to NullTracker
    if (!takeoff_out.response.success) {

      ROS_ERROR("[UavManager]: takeoff was not successful, calling eland");

      std_srvs::Trigger eland_out;
      service_client_eland_.call(eland_out);

    } else {

      res.success = takeoff_out.response.success;
      res.message = takeoff_out.response.message;

      takeoff_x_        = odometry_x;
      takeoff_y_        = odometry_y;
      takeoff_frame_id_ = odometry.header.frame_id;

      {
        // if enabled, start the timer for measuring the flight time
        if (_flighttime_timer_enabled_) {

          flighttime_timer_.start();
        }
      }

      ROS_INFO("[UavManager]: took off, saving x=%0.2f, y=%0.2f as home position", takeoff_x_, takeoff_y_);

      takingoff_ = true;
      number_of_takeoffs_++;
      waiting_for_takeoff_ = true;

      takeoff_timer_.start();
    }

    // could not activate the landoff tracker!
  } else if (!switch_tracker_out.response.success) {

    res.success = switch_tracker_out.response.success;
    res.message = switch_tracker_out.response.message;
    ROS_ERROR("[UavManager]: could not activate %s for takeoff", _takeoff_tracker_name_.c_str());

    mrs_msgs::String switch_tracker_out;
    switch_tracker_out.request.value = _null_tracker_name_;
    service_client_switch_tracker_.call(switch_tracker_out);

  } else if (!switch_controller_out.response.success) {

    res.success = switch_controller_out.response.success;
    res.message = switch_controller_out.response.message;
    ROS_ERROR("[UavManager]: could not activate %s for takeoff", _takeoff_controller_name_.c_str());

    mrs_msgs::String switch_tracker_out;
    switch_tracker_out.request.value = _null_tracker_name_;
    service_client_switch_tracker_.call(switch_tracker_out);
  }

  return true;
}

//}

/* //{ callbackLand() */

bool UavManager::callbackLand([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  // copy member variables
  auto attitude_cmd = mrs_lib::get_mutexed(mutex_attitude_cmd_, attitude_cmd_);

  char message[100];

  if (!got_odometry_) {
    sprintf((char*)&message, "Can't land, missing odometry!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_control_manager_diagnostics_) {
    sprintf((char*)&message, "Can't land, missing control manager diagnostics!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_attitude_cmd_) {
    sprintf((char*)&message, "Can't land, missing attitude command!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  // stop the eventual takeoff
  waiting_for_takeoff_ = false;
  takingoff_           = false;
  takeoff_timer_.stop();

  {
    std::scoped_lock lock(mutex_last_mass_difference_);

    last_mass_difference_ = attitude_cmd.mass_difference;
  }

  ROS_INFO("[UavManager]: landing");

  setOdometryCallbacks(false);

  flighttime_timer_.stop();

  mrs_msgs::String switch_tracker_out;
  switch_tracker_out.request.value = _landing_tracker_name_;
  if (!service_client_switch_tracker_.call(switch_tracker_out)) {
    sprintf((char*)&message, "Service call for switching to tracker %s failed.", _landing_tracker_name_.c_str());
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (switch_tracker_out.response.success == true) {

    std_srvs::Trigger land_out;
    if (service_client_land_.call(land_out)) {

      if (land_out.response.success) {

        res.success = land_out.response.success;
        res.message = land_out.response.message;

        changeLandingState(LANDING_STATE);

        landing_timer_.start();

      } else {

        sprintf((char*)&message, "Service call for landing was not successfull: %s", land_out.response.message.c_str());
        res.message = message;
        res.success = false;
        ROS_ERROR("[UavManager]: %s", message);

        ROS_INFO("[UavManager]: switching back to NullTracker");
        switch_tracker_out.request.value = _null_tracker_name_;
        service_client_switch_tracker_.call(switch_tracker_out);

        return true;
      }

    } else {

      ROS_ERROR("[UavManager]: service call for landing failed");
      res.success = false;
      res.message = "service call for landing failed";

      ROS_INFO("[UavManager]: switching back to NullTracker");
      switch_tracker_out.request.value = _null_tracker_name_;
      service_client_switch_tracker_.call(switch_tracker_out);
    }

  } else {

    ROS_ERROR("[UavManager]: could not switch to tracker %s: %s", _landing_tracker_name_.c_str(), switch_tracker_out.response.message.c_str());
    res.success = switch_tracker_out.response.success;
    res.message = switch_tracker_out.response.message;
  }

  return true;
}

//}

/* //{ callbackLandHome() */

bool UavManager::callbackLandHome([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  // copy member variables
  auto odometry = mrs_lib::get_mutexed(mutex_odometry_, odometry_);

  double odometry_z;
  odometry_z = odometry.pose.pose.position.z;

  char message[100];

  if (!got_odometry_) {
    sprintf((char*)&message, "Can't land, missing odometry!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_control_manager_diagnostics_) {
    sprintf((char*)&message, "Can't land, missing tracker status!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_attitude_cmd_) {
    sprintf((char*)&message, "Can't land, missing attitude command!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (fixing_max_height_) {
    sprintf((char*)&message, "Can't land, descedning to safety height!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  // stop the eventual takeoff
  waiting_for_takeoff_ = false;
  takingoff_           = false;
  takeoff_timer_.stop();

  ROS_INFO("[UavManager]: landing on home -> x=%0.2f, y=%0.2f", takeoff_x_, takeoff_y_);

  mrs_msgs::ReferenceStampedSrv reference_out;

  reference_out.request.header.frame_id = takeoff_frame_id_;

  reference_out.request.reference.position.x = takeoff_x_;
  reference_out.request.reference.position.y = takeoff_y_;
  reference_out.request.reference.position.z = odometry_z;
  reference_out.request.reference.yaw        = odometry_yaw_;

  {
    std::scoped_lock lock(mutex_services_);

    service_client_emergency_reference_.call(reference_out);
  }

  if (reference_out.response.success == true) {

    res.success = reference_out.response.success;
    res.message = "Flying home for landing";

    ros::Duration wait(1.0);
    wait.sleep();

    changeLandingState(FLY_HOME_STATE);

    landing_timer_.start();

  } else {

    res.success = reference_out.response.success;
    res.message = reference_out.response.message;
    changeLandingState(IDLE_STATE);
  }

  return true;
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* setOdometryCallbacks() //{ */

void UavManager::setOdometryCallbacks(const bool input) {

  ROS_INFO("[UavManager]: switching odometry callabcks to %s", input ? "ON" : "OFF");

  std_srvs::SetBool srv;

  srv.request.data = input;

  bool res = service_client_set_odometry_callbacks_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN("[UavManager]: service call for toggle odometry callbacks returned: %s.", srv.response.message.c_str());
    }

  } else {
    ROS_ERROR("[UavManager]: service call for toggle odometry callbacks failed!");
  }
}

//}

/* ungrip() //{ */

void UavManager::ungrip(void) {

  ROS_INFO_THROTTLE(1.0, "[ControlManager]: ungripping payload");

  std_srvs::Trigger srv;

  bool res = service_client_ungrip_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[ControlManager]: service call for ungripping payload returned: %s.", srv.response.message.c_str());
    }

  } else {
    ROS_WARN_THROTTLE(1.0, "[ControlManager]: service call for ungripping payload failed!");
  }
}

//}

}  // namespace uav_manager

}  // namespace mrs_uav_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_manager::uav_manager::UavManager, nodelet::Nodelet)
