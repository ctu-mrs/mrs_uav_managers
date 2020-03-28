#define VERSION "0.0.5.0"

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
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/ConstraintManagerDiagnostics.h>
#include <mrs_msgs/GainManagerDiagnostics.h>

#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>

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
  FLY_THERE_STATE,
  LANDING_STATE,

} LandingStates_t;

const char* state_names[3] = {

    "IDLING", "FLYING HOME", "LANDING"};

class UavManager : public nodelet::Nodelet {

private:
  ros::NodeHandle nh_;
  std::string     _version_;
  bool            is_initialized_ = false;
  std::string     _uav_name_;

public:
  std::shared_ptr<mrs_lib::Transformer> transformer_;

public:
  virtual void onInit();

  bool callbackTakeoff(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackLand(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackLandHome(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackLandThere(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);
  void callbackOdometry(const nav_msgs::OdometryConstPtr& msg);
  void callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg);
  void callbackAttitudeCommand(const mrs_msgs::AttitudeCommandConstPtr& msg);
  void callbackMavrosState(const mavros_msgs::StateConstPtr& msg);
  void callbackMavrosGps(const sensor_msgs::NavSatFixConstPtr& msg);
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
  ros::Timer      timer_max_height_;
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

  ros::Subscriber subscriber_mavros_gps_;

  // service servers
  ros::ServiceServer service_server_takeoff_;
  ros::ServiceServer service_server_land_;
  ros::ServiceServer service_server_land_home_;
  ros::ServiceServer service_server_land_there_;

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
  mrs_msgs::ReferenceStamped land_there_reference_;

  // to which height to takeoff
  double _takeoff_height_;

  // names of important trackers
  std::string _null_tracker_name_;

  // Takeoff timer
  ros::Timer timer_takeoff_;
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
  ros::Timer  timer_landing_;
  std::string _landing_tracker_name_;
  std::string _landing_controller_name_;
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
  void timerLanding(const ros::TimerEvent& event);
  void timerTakeoff(const ros::TimerEvent& event);
  void timerMaxHeight(const ros::TimerEvent& event);
  void timerFlighttime(const ros::TimerEvent& event);
  void timerMaxthrust(const ros::TimerEvent& event);

  // Timer for checking max flight time
  ros::Timer timer_flighttime_;
  double     _flighttime_timer_rate_;
  double     _flighttime_max_time_;
  bool       _flighttime_timer_enabled_ = false;
  double     flighttime_                = 0;

  // Timer for checking maximum thrust
  ros::Timer timer_maxthrust_;
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

  ROS_INFO("[UavManager]: initializing");

  mrs_lib::ParamLoader param_loader(nh_, "UavManager");

  param_loader.load_param("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[UavManager]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.load_param("uav_name", _uav_name_);

  param_loader.load_param("enable_profiler", _profiler_enabled_);

  param_loader.load_param("null_tracker", _null_tracker_name_);

  param_loader.load_param("takeoff/rate", _takeoff_timer_rate_);
  param_loader.load_param("takeoff/after_takeoff/tracker", _after_takeoff_tracker_name_);
  param_loader.load_param("takeoff/after_takeoff/controller", _after_takeoff_controller_name_);
  param_loader.load_param("takeoff/after_takeoff/pirouette", _after_takeoff_pirouette_);
  param_loader.load_param("takeoff/during_takeoff/tracker", _takeoff_tracker_name_);
  param_loader.load_param("takeoff/during_takeoff/controller", _takeoff_controller_name_);
  param_loader.load_param("takeoff/takeoff_height", _takeoff_height_);

  param_loader.load_param("landing/rate", _landing_timer_rate_);
  param_loader.load_param("landing/landing_tracker", _landing_tracker_name_);
  param_loader.load_param("landing/landing_controller", _landing_controller_name_);
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

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>("ControlManager", _uav_name_);

  // | ----------------------- subscribers ---------------------- |

  subscriber_odometry_     = nh_.subscribe("odometry_in", 1, &UavManager::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
  subscriber_attitude_cmd_ = nh_.subscribe("attitude_cmd_in", 1, &UavManager::callbackAttitudeCmd, this, ros::TransportHints().tcpNoDelay());
  subscriber_mavros_state_ = nh_.subscribe("mavros_state_in", 1, &UavManager::callbackMavrosState, this, ros::TransportHints().tcpNoDelay());
  subscriber_mavros_gps_   = nh_.subscribe("mavros_gps_in", 1, &UavManager::callbackMavrosGps, this, ros::TransportHints().tcpNoDelay());
  subscriber_max_height_   = nh_.subscribe("max_height_in", 1, &UavManager::callbackMaxHeight, this, ros::TransportHints().tcpNoDelay());
  subscriber_height_       = nh_.subscribe("height_in", 1, &UavManager::callbackHeight, this, ros::TransportHints().tcpNoDelay());
  subscriber_motors_       = nh_.subscribe("motors_in", 1, &UavManager::callbackMotors, this, ros::TransportHints().tcpNoDelay());
  subscriber_control_manager_diagnostics_ =
      nh_.subscribe("control_manager_diagnostics_in", 1, &UavManager::callbackControlManagerDiagnostics, this, ros::TransportHints().tcpNoDelay());

  subscriber_gain_diagnostics_ = nh_.subscribe("gain_manager_diagnostics_in", 1, &UavManager::callbackGains, this, ros::TransportHints().tcpNoDelay());
  gains_last_time_             = ros::Time(0);

  subscriber_constraint_manager_diagnostics_ =
      nh_.subscribe("constraint_manager_diagnostics_in", 1, &UavManager::callbackConstraints, this, ros::TransportHints().tcpNoDelay());
  constraints_last_time_ = ros::Time(0);

  // | --------------------- service servers -------------------- |

  service_server_takeoff_    = nh_.advertiseService("takeoff_in", &UavManager::callbackTakeoff, this);
  service_server_land_       = nh_.advertiseService("land_in", &UavManager::callbackLand, this);
  service_server_land_home_  = nh_.advertiseService("land_home_in", &UavManager::callbackLandHome, this);
  service_server_land_there_ = nh_.advertiseService("land_there_in", &UavManager::callbackLandThere, this);

  // | --------------------- service clients -------------------- |

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

  // | ---------------------- state machine --------------------- |

  changeLandingState(IDLE_STATE);

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "UavManager", _profiler_enabled_);

  // | ------------------------- timers ------------------------- |

  timer_landing_    = nh_.createTimer(ros::Rate(_landing_timer_rate_), &UavManager::timerLanding, this, false, false);
  timer_takeoff_    = nh_.createTimer(ros::Rate(_takeoff_timer_rate_), &UavManager::timerTakeoff, this, false, false);
  timer_flighttime_ = nh_.createTimer(ros::Rate(_flighttime_timer_rate_), &UavManager::timerFlighttime, this, false, false);
  timer_maxthrust_  = nh_.createTimer(ros::Rate(_maxthrust_timer_rate_), &UavManager::timerMaxthrust, this, false, false);

  if (_max_height_enabled_) {
    timer_max_height_ = nh_.createTimer(ros::Rate(_max_height_checking_rate_), &UavManager::timerMaxHeight, this);
  }

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[UavManager]: initialized, version %s", VERSION);
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
    case FLY_THERE_STATE:
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

/* //{ timerLanding() */

void UavManager::timerLanding(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("timerLanding", _landing_timer_rate_, 0.1, event);

  // copy member variables
  auto control_manager_diagnostics = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);
  auto attitude_cmd                = mrs_lib::get_mutexed(mutex_attitude_cmd_, attitude_cmd_);
  auto odometry                    = mrs_lib::get_mutexed(mutex_odometry_, odometry_);

  auto res = transformer_->transformSingle(odometry.header.frame_id, land_there_reference_);

  mrs_msgs::ReferenceStamped land_there_current_frame;

  if (res) {

    land_there_current_frame = res.value();
  } else {

    ROS_ERROR("[UavManager]: could not transform the reference into the current frame! land by yourselve pls.");
    return;
  }

  if (current_state_landing_ == IDLE_STATE) {

    return;

  } else if (current_state_landing_ == FLY_THERE_STATE) {

    if (sqrt(pow(odometry.pose.pose.position.x - land_there_current_frame.reference.position.x, 2) +
             pow(odometry.pose.pose.position.y - land_there_current_frame.reference.position.y, 2) +
             pow(odometry.pose.pose.position.z - land_there_current_frame.reference.position.z, 2)) < 0.5) {

      ROS_INFO("[UavManager]: landing");

      mrs_msgs::String switch_controller_out;
      switch_controller_out.request.value = _landing_controller_name_;
      if (!service_client_switch_controller_.call(switch_controller_out)) {
        ROS_ERROR("[UavManager]: service call for switching to the controller failed");
      } else {
        if (!switch_controller_out.response.success) {
          ROS_ERROR("[UavManager]: switch to landing controller failed");
        }
      }

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


    if (_landing_tracker_name_ == control_manager_diagnostics.active_tracker) {

      // recalculate the mass based on the thrust
      thrust_mass_estimate_ = pow((attitude_cmd.thrust - _hover_thrust_b_) / _hover_thrust_a_, 2) / _g_;
      ROS_INFO_THROTTLE(1.0, "[UavManager]: landing: initial mass: %.2f thrust mass estimate: %.2f", landing_uav_mass_, thrust_mass_estimate_);

      // condition for automatic motor turn off
      if (((thrust_mass_estimate_ < _landing_cutoff_mass_factor_ * landing_uav_mass_) || attitude_cmd.thrust < 0.01)) {

        if (!thrust_under_threshold_) {

          thrust_mass_estimate_first_time_ = ros::Time::now();
          thrust_under_threshold_          = true;
        }

        ROS_INFO_THROTTLE(0.5, "[UavManager]: thrust is under cutoff factor for %.2f s", (ros::Time::now() - thrust_mass_estimate_first_time_).toSec());

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

        timer_landing_.stop();
      }

    } else {

      ROS_WARN_THROTTLE(1.0, "[UavManager]: incorrect tracker detected during landing!");
    }
  }
}

//}

/* //{ timerTakeoff() */

void UavManager::timerTakeoff(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("timerTakeoff", _takeoff_timer_rate_, 0.1, event);

  auto control_manager_diagnostics = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);

  if (waiting_for_takeoff_) {

    if (control_manager_diagnostics.active_tracker == _takeoff_tracker_name_ && control_manager_diagnostics.tracker_status.moving_reference) {

      waiting_for_takeoff_ = false;
    } else {

      ROS_WARN_THROTTLE(1.0, "[UavManager]: waiting for takeoff confirmation from the ControlManager");
      return;
    }
  }

  if (takingoff_) {

    if (control_manager_diagnostics.active_tracker != _takeoff_tracker_name_ || !control_manager_diagnostics.tracker_status.moving_reference) {

      ROS_INFO("[UavManager]: take off finished, switching to %s", _after_takeoff_tracker_name_.c_str());

      // if enabled, start the timer for landing after reaching max thrust
      if (_maxthrust_timer_enabled_) {
        timer_maxthrust_.start();
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

      timer_takeoff_.stop();
    }
  }
}

//}

/* //{ timerMaxHeight() */

void UavManager::timerMaxHeight(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("timerMaxHeight", _max_height_checking_rate_, 0.1, event);

  if (!got_max_height_ || !got_odometry_) {
    return;
  }

  auto max_height               = mrs_lib::get_mutexed(mutex_max_height_, _max_height_);
  auto [odometry, odometry_yaw] = mrs_lib::get_mutexed(mutex_odometry_, odometry_, odometry_yaw_);

  double odometry_x, odometry_y, odometry_z;
  odometry_x = odometry.pose.pose.position.x;
  odometry_y = odometry.pose.pose.position.y;
  odometry_z = odometry.pose.pose.position.z;

  double odometry_x_speed, odometry_y_speed;
  odometry_x_speed = odometry.twist.twist.linear.x;
  odometry_y_speed = odometry.twist.twist.linear.y;

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

/* //{ timerFlighttime() */

void UavManager::timerFlighttime(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("timerFlighttime", _flighttime_timer_rate_, 0.1, event);

  flighttime_ += 1.0 / _flighttime_timer_rate_;

  if (flighttime_ > _flighttime_max_time_) {

    flighttime_ = 0;
    timer_flighttime_.stop();

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

      timer_landing_.start();

    } else {

      changeLandingState(IDLE_STATE);
    }
  }
}

//}

/* //{ timerMaxthrust() */

void UavManager::timerMaxthrust(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("timerMaxthrust", _maxthrust_timer_rate_, 0.03, event);

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

    timer_maxthrust_.stop();

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

      timer_landing_.start();

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

/* //{ callbackMavrosGps() */

void UavManager::callbackMavrosGps(const sensor_msgs::NavSatFixConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackMavrosGps");

  transformer_->setCurrentLatLon(msg->latitude, msg->longitude);
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

    transformer_->setCurrentControlFrame(odometry_.header.frame_id);

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

  std::stringstream ss;

  if (!got_odometry_) {
    ss << "can not takeoff, missing odometry!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!got_mavros_state_ || (ros::Time::now() - mavros_state.header.stamp).toSec() > 5.0) {
    ss << "can not takeoff, missing mavros state!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!mavros_state.armed) {
    ss << "can not takeoff, UAV not armed!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (mavros_state.mode != "OFFBOARD") {
    ss << "can not takeoff, UAV not in offboard mode!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!got_control_manager_diagnostics_) {
    ss << "can not takeoff, missing control manager diagnostics!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!got_attitude_cmd_) {
    ss << "can not takeoff, missing target attitude!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (_null_tracker_name_ != control_manager_diagnostics.active_tracker) {
    ss << "can not takeoff, need '" << _null_tracker_name_ << "' to be active!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (_gain_manager_required_ && (ros::Time::now() - gains_last_time).toSec() > 5.0) {
    ss << "can not takeoff, GainManager is not running!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (_constraint_manager_required_ && (ros::Time::now() - constraints_last_time).toSec() > 5.0) {
    ss << "can not takeoff, ConstraintManager is not running!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!got_motors_ || (ros::Time::now() - motors.stamp).toSec() > 1.0 || !motors.data) {
    ss << "can not takeoff, motors are off!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (number_of_takeoffs_ > 0) {

    if (!got_attitude_cmd_) {

      ss << "can not takeoff, missing attitude command!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    if (last_mass_difference > 0.5) {

      ss << std::setprecision(2);
      ss << "can not takeoff, estimated mass difference is too large: " << _null_tracker_name_ << "!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
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

      land_there_reference_.header               = odometry.header;
      land_there_reference_.reference.position.x = odometry.pose.pose.position.x;
      land_there_reference_.reference.position.y = odometry.pose.pose.position.y;
      land_there_reference_.reference.position.z = odometry.pose.pose.position.z;
      land_there_reference_.reference.yaw        = tf::getYaw(odometry.pose.pose.orientation);

      {
        // if enabled, start the timer for measuring the flight time
        if (_flighttime_timer_enabled_) {

          timer_flighttime_.start();
        }
      }

      ROS_INFO("[UavManager]: took off, saving x=%0.2f, y=%0.2f as home position", land_there_reference_.reference.position.x,
               land_there_reference_.reference.position.y);

      takingoff_ = true;
      number_of_takeoffs_++;
      waiting_for_takeoff_ = true;

      timer_takeoff_.start();
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

  std::stringstream ss;

  if (!got_odometry_) {
    ss << "can not land, missing odometry!";
    res.message = ss.str();
    res.success = false;
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    return true;
  }

  if (!got_control_manager_diagnostics_) {
    ss << "can not land, missing control manager diagnostics!";
    res.message = ss.str();
    res.success = false;
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    return true;
  }

  if (!got_attitude_cmd_) {
    ss << "can not land, missing attitude command!";
    res.message = ss.str();
    res.success = false;
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    return true;
  }

  // stop the eventual takeoff
  waiting_for_takeoff_ = false;
  takingoff_           = false;
  timer_takeoff_.stop();

  {
    std::scoped_lock lock(mutex_last_mass_difference_);

    last_mass_difference_ = attitude_cmd.mass_difference;
  }

  ROS_INFO("[UavManager]: landing");

  setOdometryCallbacks(false);

  timer_flighttime_.stop();

  bool suc = true;

  mrs_msgs::String switch_tracker_out;
  switch_tracker_out.request.value = _landing_tracker_name_;
  if (!service_client_switch_tracker_.call(switch_tracker_out)) {
    ss << "service call for switching to tracker '%s' failed.", _landing_tracker_name_.c_str();
    res.message = ss.str();
    res.success = false;
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    suc = false;
    return true;
  }

  mrs_msgs::String switch_controller_out;
  switch_controller_out.request.value = _landing_controller_name_;
  if (!service_client_switch_controller_.call(switch_controller_out)) {
    ss << "service call for switching to controller '%s' failed.", _landing_controller_name_.c_str();
    res.message = ss.str();
    res.success = false;
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    suc = false;
    return true;
  }

  if (suc) {

    std_srvs::Trigger land_out;
    if (service_client_land_.call(land_out)) {

      if (land_out.response.success) {

        res.success = land_out.response.success;
        res.message = land_out.response.message;

        changeLandingState(LANDING_STATE);

        timer_landing_.start();

      } else {

        ss << "service call for landing was not successfull: '%s'", land_out.response.message.c_str();
        res.message = ss.str();
        res.success = false;
        ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());

        return true;
      }

    } else {

      ss << "service call for landing failed";
      res.success = false;
      res.message = ss.str();
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    }

  } else {

    ss << "could not switch to landing tracker or controller: '" << switch_tracker_out.response.message << "'";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    res.success = false;
    res.message = ss.str();
  }

  return true;
}

//}

/* //{ callbackLandHome() */

bool UavManager::callbackLandHome([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  auto odometry = mrs_lib::get_mutexed(mutex_odometry_, odometry_);

  std::stringstream ss;

  if (!got_odometry_) {
    ss << "can not land, missing odometry!";
    res.message = ss.str();
    res.success = false;
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    return true;
  }

  if (!got_control_manager_diagnostics_) {
    ss << "can not land, missing tracker status!";
    res.message = ss.str();
    res.success = false;
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    return true;
  }

  if (!got_attitude_cmd_) {
    ss << "can not land, missing attitude command!";
    res.message = ss.str();
    res.success = false;
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    return true;
  }

  if (fixing_max_height_) {
    ss << "can not land, descedning to safety height!";
    res.message = ss.str();
    res.success = false;
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    return true;
  }

  // stop the eventual takeoff
  waiting_for_takeoff_ = false;
  takingoff_           = false;
  timer_takeoff_.stop();

  ROS_INFO("[UavManager]: landing on home -> x=%0.2f, y=%0.2f", land_there_reference_.reference.position.x, land_there_reference_.reference.position.y);

  ungrip();

  mrs_msgs::ReferenceStampedSrv reference_out;

  land_there_reference_.reference.position.z = odometry.pose.pose.position.z;

  reference_out.request.header.frame_id = land_there_reference_.header.frame_id;
  reference_out.request.header.stamp    = ros::Time::now();
  reference_out.request.reference       = land_there_reference_.reference;

  {
    std::scoped_lock lock(mutex_services_);

    service_client_emergency_reference_.call(reference_out);
  }

  if (reference_out.response.success == true) {

    res.success = reference_out.response.success;
    res.message = "flying home for landing";

    changeLandingState(FLY_THERE_STATE);

    timer_landing_.start();

  } else {

    res.success = reference_out.response.success;
    res.message = reference_out.response.message;
    changeLandingState(IDLE_STATE);
  }

  return true;
}

//}

/* //{ callbackLandThere() */

bool UavManager::callbackLandThere(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res) {

  if (!is_initialized_)
    return false;

  std::stringstream ss;

  if (!got_odometry_) {
    ss << "can not land, missing odometry!";
    res.message = ss.str();
    res.success = false;
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    return true;
  }

  if (!got_control_manager_diagnostics_) {
    ss << "can not land, missing tracker status!";
    res.message = ss.str();
    res.success = false;
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    return true;
  }

  if (!got_attitude_cmd_) {
    ss << "can not land, missing attitude command!";
    res.message = ss.str();
    res.success = false;
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    return true;
  }

  if (fixing_max_height_) {
    ss << "can not land, descedning to safety height!";
    res.message = ss.str();
    res.success = false;
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    return true;
  }

  // stop the eventual takeoff
  waiting_for_takeoff_ = false;
  takingoff_           = false;
  timer_takeoff_.stop();

  ROS_INFO("[UavManager]: landing there -> x=%.2f, y=%.2f, z=%.2f, yaw=%.2f in %s", req.reference.position.x, req.reference.position.y,
           req.reference.position.z, req.reference.yaw, req.header.frame_id.c_str());

  ungrip();

  mrs_msgs::ReferenceStampedSrv reference_out;

  land_there_reference_.header    = req.header;
  land_there_reference_.reference = req.reference;

  reference_out.request.header.frame_id = land_there_reference_.header.frame_id;
  reference_out.request.header.stamp    = ros::Time::now();
  reference_out.request.reference       = land_there_reference_.reference;

  {
    std::scoped_lock lock(mutex_services_);

    service_client_emergency_reference_.call(reference_out);
  }

  if (reference_out.response.success == true) {

    res.success = reference_out.response.success;
    res.message = "Flying there for landing";

    changeLandingState(FLY_THERE_STATE);

    timer_landing_.start();

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

  ROS_INFO_THROTTLE(1.0, "[UavManager]: ungripping payload");

  std_srvs::Trigger srv;

  bool res = service_client_ungrip_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[UavManager]: service call for ungripping payload returned: %s.", srv.response.message.c_str());
    }

  } else {
    ROS_ERROR_THROTTLE(1.0, "[UavManager]: service call for ungripping payload failed!");
  }
}

//}

}  // namespace uav_manager

}  // namespace mrs_uav_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_manager::uav_manager::UavManager, nodelet::Nodelet)
