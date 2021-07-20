#define VERSION "1.0.1.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <nav_msgs/Odometry.h>
#include <mrs_msgs/Vec1.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/Float64.h>
#include <mrs_msgs/BoolStamped.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/ConstraintManagerDiagnostics.h>
#include <mrs_msgs/GainManagerDiagnostics.h>
#include <mrs_msgs/UavManagerDiagnostics.h>
#include <mrs_msgs/OdometryDiag.h>

#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/quadratic_thrust_model.h>

//}

/* using //{ */

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

//}

namespace mrs_uav_managers
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

  void changeLandingState(LandingStates_t new_state);

  // subscribers
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                     sh_odometry_;
  mrs_lib::SubscribeHandler<mrs_msgs::OdometryDiag>                 sh_odometry_diagnostics_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>    sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::BoolStamped>                  sh_motors_;
  mrs_lib::SubscribeHandler<mrs_msgs::AttitudeCommand>              sh_attitude_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>               sh_height_;
  mrs_lib::SubscribeHandler<mavros_msgs::State>                     sh_mavros_state_;
  mrs_lib::SubscribeHandler<mrs_msgs::GainManagerDiagnostics>       sh_gains_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::ConstraintManagerDiagnostics> sh_constraints_diag_;
  mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>                 sh_mavros_gps_;
  mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>               sh_max_height_;

  void callbackMavrosGps(mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>& wrp);
  void callbackOdometry(mrs_lib::SubscribeHandler<nav_msgs::Odometry>& wrp);

  // service servers
  ros::ServiceServer service_server_takeoff_;
  ros::ServiceServer service_server_land_;
  ros::ServiceServer service_server_land_home_;
  ros::ServiceServer service_server_land_there_;

  // service callbacks
  bool callbackTakeoff(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackLand(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackLandHome(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackLandThere(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);

  // service clients
  ros::ServiceClient service_client_takeoff_;
  ros::ServiceClient service_client_switch_tracker_;
  ros::ServiceClient service_client_switch_controller_;
  ros::ServiceClient service_client_land_;
  ros::ServiceClient service_client_eland_;
  ros::ServiceClient service_client_control_callbacks_;
  ros::ServiceClient service_client_emergency_reference_;
  ros::ServiceClient service_client_arm_;
  ros::ServiceClient service_client_pirouette_;
  ros::ServiceClient service_client_odometry_callbacks_;
  ros::ServiceClient service_client_ungrip_;

  // service client wrappers
  bool takeoffSrv(void);
  bool switchTrackerSrv(const std::string& tracker);
  bool switchControllerSrv(const std::string& controller);
  bool landSrv(void);
  bool elandSrv(void);
  void disarmSrv(void);
  bool emergencyReferenceSrv(const mrs_msgs::ReferenceStamped& goal);
  void setOdometryCallbacksSrv(const bool& input);
  void setControlCallbacksSrv(const bool& input);
  void ungripSrv(void);
  void pirouetteSrv(void);

  ros::Timer timer_takeoff_;
  ros::Timer timer_max_height_;
  ros::Timer timer_min_height_;
  ros::Timer timer_landing_;
  ros::Timer timer_maxthrust_;
  ros::Timer timer_flighttime_;
  ros::Timer timer_diagnostics_;

  // timer callbacks
  void timerLanding(const ros::TimerEvent& event);
  void timerTakeoff(const ros::TimerEvent& event);
  void timerMaxHeight(const ros::TimerEvent& event);
  void timerMinHeight(const ros::TimerEvent& event);
  void timerFlighttime(const ros::TimerEvent& event);
  void timerMaxthrust(const ros::TimerEvent& event);
  void timerDiagnostics(const ros::TimerEvent& event);

  // publishers
  ros::Publisher publisher_diagnostics_;

  // max height checking
  bool              _max_height_enabled_ = false;
  int               _max_height_checking_rate_;
  double            _max_height_offset_;
  double            _max_height_;
  std::atomic<bool> fixing_max_height_ = false;

  // min height checking
  bool              _min_height_enabled_ = false;
  int               _min_height_checking_rate_;
  double            _min_height_offset_;
  double            _min_height_;
  std::atomic<bool> fixing_min_height_ = false;

  // mass estimation during landing
  double    thrust_mass_estimate_;
  bool      thrust_under_threshold_ = false;
  ros::Time thrust_mass_estimate_first_time_;

  bool _gain_manager_required_       = false;
  bool _constraint_manager_required_ = false;

  std::tuple<bool, std::string> landImpl(void);
  std::tuple<bool, std::string> landWithDescendImpl(void);

  // saved takeoff coordinates and allows to land there again
  mrs_msgs::ReferenceStamped land_there_reference_;
  std::mutex                 mutex_land_there_reference_;

  // to which height to takeoff
  double _takeoff_height_;

  std::atomic<bool> takeoff_successful_;

  // names of important trackers
  std::string _null_tracker_name_;

  // Takeoff timer
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
  std::string _landing_tracker_name_;
  std::string _landing_controller_name_;
  double      _landing_cutoff_mass_factor_;
  double      _landing_cutoff_mass_timeout_;
  double      _landing_timer_rate_;
  double      _landing_descend_height_;
  bool        landing_ = false;
  double      _uav_mass_;
  double      _g_;
  double      landing_uav_mass_;
  bool        _landing_disarm_ = false;

  // diagnostics timer
  double _diagnostics_timer_rate_;

  mrs_lib::quadratic_thrust_model::MotorParams_t _motor_params_;

  // landing state machine states
  LandingStates_t current_state_landing_  = IDLE_STATE;
  LandingStates_t previous_state_landing_ = IDLE_STATE;

  // Timer for checking max flight time
  double     _flighttime_timer_rate_;
  double     _flighttime_max_time_;
  bool       _flighttime_timer_enabled_ = false;
  double     flighttime_                = 0;
  std::mutex mutex_flighttime_;

  // Timer for checking maximum thrust
  bool      _maxthrust_timer_enabled_ = false;
  double    _maxthrust_timer_rate_;
  double    _maxthrust_max_thrust_;
  double    _maxthrust_eland_timeout_;
  double    _maxthrust_ungrip_timeout_;
  bool      maxthrust_above_threshold_ = false;
  ros::Time maxthrust_first_time_;

  // profiler
  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;
};

//}

/* //{ onInit() */

void UavManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[UavManager]: initializing");

  mrs_lib::ParamLoader param_loader(nh_, "UavManager");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[UavManager]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("uav_name", _uav_name_);

  param_loader.loadParam("enable_profiler", _profiler_enabled_);

  param_loader.loadParam("null_tracker", _null_tracker_name_);

  param_loader.loadParam("takeoff/rate", _takeoff_timer_rate_);
  param_loader.loadParam("takeoff/after_takeoff/tracker", _after_takeoff_tracker_name_);
  param_loader.loadParam("takeoff/after_takeoff/controller", _after_takeoff_controller_name_);
  param_loader.loadParam("takeoff/after_takeoff/pirouette", _after_takeoff_pirouette_);
  param_loader.loadParam("takeoff/during_takeoff/controller", _takeoff_controller_name_);
  param_loader.loadParam("takeoff/during_takeoff/tracker", _takeoff_tracker_name_);
  param_loader.loadParam("takeoff/takeoff_height", _takeoff_height_);

  param_loader.loadParam("landing/rate", _landing_timer_rate_);
  param_loader.loadParam("landing/landing_tracker", _landing_tracker_name_);
  param_loader.loadParam("landing/landing_controller", _landing_controller_name_);
  param_loader.loadParam("landing/landing_cutoff_mass_factor", _landing_cutoff_mass_factor_);
  param_loader.loadParam("landing/landing_cutoff_timeout", _landing_cutoff_mass_timeout_);
  param_loader.loadParam("landing/disarm", _landing_disarm_);
  param_loader.loadParam("landing/descend_height", _landing_descend_height_);

  param_loader.loadParam("uav_mass", _uav_mass_);
  param_loader.loadParam("g", _g_);

  param_loader.loadParam("motor_params/n_motors", _motor_params_.n_motors);
  param_loader.loadParam("motor_params/a", _motor_params_.A);
  param_loader.loadParam("motor_params/b", _motor_params_.B);

  param_loader.loadParam("max_height_checking/enabled", _max_height_enabled_);
  param_loader.loadParam("max_height_checking/rate", _max_height_checking_rate_);
  param_loader.loadParam("max_height_checking/safety_height_offset", _max_height_offset_);

  param_loader.loadParam("min_height_checking/enabled", _min_height_enabled_);
  param_loader.loadParam("min_height_checking/rate", _min_height_checking_rate_);
  param_loader.loadParam("min_height_checking/safety_height_offset", _min_height_offset_);
  param_loader.loadParam("min_height_checking/min_height", _min_height_);

  param_loader.loadParam("safety_area/max_height", _max_height_);

  param_loader.loadParam("require_gain_manager", _gain_manager_required_);
  param_loader.loadParam("require_constraint_manager", _constraint_manager_required_);

  param_loader.loadParam("flight_timer/enabled", _flighttime_timer_enabled_);
  param_loader.loadParam("flight_timer/rate", _flighttime_timer_rate_);
  param_loader.loadParam("flight_timer/max_time", _flighttime_max_time_);

  param_loader.loadParam("max_thrust/enabled", _maxthrust_timer_enabled_);
  param_loader.loadParam("max_thrust/rate", _maxthrust_timer_rate_);
  param_loader.loadParam("max_thrust/max_thrust", _maxthrust_max_thrust_);
  param_loader.loadParam("max_thrust/eland_timeout", _maxthrust_eland_timeout_);
  param_loader.loadParam("max_thrust/ungrip_timeout", _maxthrust_ungrip_timeout_);

  param_loader.loadParam("diagnostics/rate", _diagnostics_timer_rate_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[UavManager]: Could not load all parameters!");
    ros::shutdown();
  }

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>("ControlManager", _uav_name_);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "UavManager";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_odometry_             = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odometry_in", &UavManager::callbackOdometry, this);
  sh_odometry_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::OdometryDiag>(shopts, "odometry_diagnostics_in");
  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in");
  sh_motors_               = mrs_lib::SubscribeHandler<mrs_msgs::BoolStamped>(shopts, "motors_in");
  sh_attitude_cmd_         = mrs_lib::SubscribeHandler<mrs_msgs::AttitudeCommand>(shopts, "attitude_cmd_in");
  sh_height_               = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(shopts, "height_in");
  sh_mavros_state_         = mrs_lib::SubscribeHandler<mavros_msgs::State>(shopts, "mavros_state_in");
  sh_gains_diag_           = mrs_lib::SubscribeHandler<mrs_msgs::GainManagerDiagnostics>(shopts, "gain_manager_diagnostics_in");
  sh_constraints_diag_     = mrs_lib::SubscribeHandler<mrs_msgs::ConstraintManagerDiagnostics>(shopts, "constraint_manager_diagnostics_in");
  sh_mavros_gps_           = mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>(shopts, "mavros_gps_in", &UavManager::callbackMavrosGps, this);
  sh_max_height_           = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(shopts, "max_height_in");

  // | ----------------------- publishers ----------------------- |

  publisher_diagnostics_ = nh_.advertise<mrs_msgs::UavManagerDiagnostics>("diagnostics_out", 1);

  // | --------------------- service servers -------------------- |

  service_server_takeoff_    = nh_.advertiseService("takeoff_in", &UavManager::callbackTakeoff, this);
  service_server_land_       = nh_.advertiseService("land_in", &UavManager::callbackLand, this);
  service_server_land_home_  = nh_.advertiseService("land_home_in", &UavManager::callbackLandHome, this);
  service_server_land_there_ = nh_.advertiseService("land_there_in", &UavManager::callbackLandThere, this);

  // | --------------------- service clients -------------------- |

  service_client_takeoff_             = nh_.serviceClient<mrs_msgs::Vec1>("takeoff_out");
  service_client_land_                = nh_.serviceClient<std_srvs::Trigger>("land_out");
  service_client_eland_               = nh_.serviceClient<std_srvs::Trigger>("eland_out");
  service_client_switch_tracker_      = nh_.serviceClient<mrs_msgs::String>("switch_tracker_out");
  service_client_switch_controller_   = nh_.serviceClient<mrs_msgs::String>("switch_controller_out");
  service_client_emergency_reference_ = nh_.serviceClient<mrs_msgs::ReferenceStampedSrv>("emergency_reference_out");
  service_client_control_callbacks_   = nh_.serviceClient<std_srvs::SetBool>("enable_callbacks_out");
  service_client_arm_                 = nh_.serviceClient<std_srvs::SetBool>("arm_out");
  service_client_pirouette_           = nh_.serviceClient<std_srvs::Trigger>("pirouette_out");
  service_client_odometry_callbacks_  = nh_.serviceClient<std_srvs::SetBool>("set_odometry_callbacks_out");
  service_client_ungrip_              = nh_.serviceClient<std_srvs::Trigger>("ungrip_out");

  // | ---------------------- state machine --------------------- |

  current_state_landing_ = IDLE_STATE;

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "UavManager", _profiler_enabled_);

  // | ------------------------- timers ------------------------- |

  timer_landing_     = nh_.createTimer(ros::Rate(_landing_timer_rate_), &UavManager::timerLanding, this, false, false);
  timer_takeoff_     = nh_.createTimer(ros::Rate(_takeoff_timer_rate_), &UavManager::timerTakeoff, this, false, false);
  timer_flighttime_  = nh_.createTimer(ros::Rate(_flighttime_timer_rate_), &UavManager::timerFlighttime, this, false, false);
  timer_maxthrust_   = nh_.createTimer(ros::Rate(_maxthrust_timer_rate_), &UavManager::timerMaxthrust, this, false, false);
  timer_diagnostics_ = nh_.createTimer(ros::Rate(_diagnostics_timer_rate_), &UavManager::timerDiagnostics, this);

  if (_max_height_enabled_) {
    timer_max_height_ = nh_.createTimer(ros::Rate(_max_height_checking_rate_), &UavManager::timerMaxHeight, this);
  }

  if (_min_height_enabled_) {
    timer_min_height_ = nh_.createTimer(ros::Rate(_min_height_checking_rate_), &UavManager::timerMinHeight, this);
  }

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[UavManager]: initialized, version %s", VERSION);

  ROS_DEBUG("[UavManager]: debug output is enabled");
}

//}

// | ---------------------- state machine --------------------- |

/* //{ changeLandingState() */

void UavManager::changeLandingState(LandingStates_t new_state) {

  // copy member variables
  auto attitude_cmd = sh_attitude_cmd_.getMsg();

  previous_state_landing_ = current_state_landing_;
  current_state_landing_  = new_state;

  switch (current_state_landing_) {

    case LANDING_STATE: {

      landing_uav_mass_ = _uav_mass_ + attitude_cmd->mass_difference;
      break;
    };

    default: {
      break;
    }
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

  auto land_there_reference = mrs_lib::get_mutexed(mutex_land_there_reference_, land_there_reference_);

  // copy member variables
  auto   control_manager_diagnostics = sh_control_manager_diag_.getMsg();
  double desired_thrust              = sh_attitude_cmd_.getMsg()->thrust;
  auto   odometry                    = sh_odometry_.getMsg();

  auto res = transformer_->transformSingle(odometry->header.frame_id, land_there_reference);

  mrs_msgs::ReferenceStamped land_there_current_frame;

  if (res) {
    land_there_current_frame = res.value();
  } else {

    ROS_ERROR("[UavManager]: could not transform the reference into the current frame! land by yourself pls.");
    return;
  }

  if (current_state_landing_ == IDLE_STATE) {

    return;

  } else if (current_state_landing_ == FLY_THERE_STATE) {

    auto [odom_x, odom_y, odom_z] = mrs_lib::getPosition(odometry);
    auto [ref_x, ref_y, ref_z]    = mrs_lib::getPosition(land_there_current_frame);

    double odom_heading, ref_heading;
    try {
      odom_heading = mrs_lib::getHeading(odometry);
      ref_heading  = mrs_lib::getHeading(land_there_current_frame);
    }
    catch (mrs_lib::AttitudeConverter::GetHeadingException& e) {
      ROS_ERROR_THROTTLE(1.0, "[UavManager]: exception caught: '%s'", e.what());
      return;
    }

    if (mrs_lib::geometry::dist(vec3_t(odom_x, odom_y, odom_z), vec3_t(ref_x, ref_y, ref_z)) < 0.05 && fabs(radians::diff(odom_heading, ref_heading)) < 0.05) {

      auto [success, message] = landWithDescendImpl();

      if (!success) {

        ROS_ERROR_THROTTLE(1.0, "[UavManager]: call for landing failed: '%s'", message.c_str());
      }

    } else if (!control_manager_diagnostics->tracker_status.have_goal && control_manager_diagnostics->flying_normally) {

      ROS_WARN_THROTTLE(1.0, "[UavManager]: the tracker does not have a goal while flying home, setting the reference again");

      mrs_msgs::ReferenceStamped reference_out;

      {
        std::scoped_lock lock(mutex_land_there_reference_);

        // get the current altitude in land_there_reference_.header.frame_id;
        geometry_msgs::PoseStamped current_pose;
        current_pose.header.stamp     = ros::Time::now();
        current_pose.header.frame_id  = _uav_name_ + "/fcu";
        current_pose.pose.position.x  = 0;
        current_pose.pose.position.y  = 0;
        current_pose.pose.position.z  = 0;
        current_pose.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

        auto response = transformer_->transformSingle(land_there_reference_.header.frame_id, current_pose);

        if (response) {

          land_there_reference_.reference.position.z = response.value().pose.position.z;
          ROS_DEBUG("[UavManager]: current altitude is %.2f m", land_there_reference_.reference.position.z);

        } else {

          std::stringstream ss;
          ss << "could not transform current height to " << land_there_reference_.header.frame_id;
          ROS_ERROR_STREAM("[UavManager]: " << ss.str());
        }

        reference_out.header.frame_id = land_there_reference_.header.frame_id;
        reference_out.header.stamp    = ros::Time::now();
        reference_out.reference       = land_there_reference_.reference;
      }

      emergencyReferenceSrv(reference_out);
    }

  } else if (current_state_landing_ == LANDING_STATE) {

    // we should not attempt to finish the landing if some other tracked was activated
    if (_landing_tracker_name_ == sh_control_manager_diag_.getMsg()->active_tracker) {

      // recalculate the mass based on the thrust
      thrust_mass_estimate_ = mrs_lib::quadratic_thrust_model::thrustToForce(_motor_params_, desired_thrust) / _g_;
      ROS_INFO_THROTTLE(1.0, "[UavManager]: landing: initial mass: %.2f thrust mass estimate: %.2f", landing_uav_mass_, thrust_mass_estimate_);

      // condition for automatic motor turn off
      if (((thrust_mass_estimate_ < _landing_cutoff_mass_factor_ * landing_uav_mass_) || desired_thrust < 0.01)) {

        if (!thrust_under_threshold_) {

          thrust_mass_estimate_first_time_ = ros::Time::now();
          thrust_under_threshold_          = true;
        }

        ROS_INFO_THROTTLE(0.5, "[UavManager]: thrust is under cutoff factor for %.2f s", (ros::Time::now() - thrust_mass_estimate_first_time_).toSec());

      } else {

        thrust_under_threshold_ = false;
      }

      if (thrust_under_threshold_ && ((ros::Time::now() - thrust_mass_estimate_first_time_).toSec() > _landing_cutoff_mass_timeout_)) {

        switchTrackerSrv(_null_tracker_name_);

        setControlCallbacksSrv(true);

        if (_landing_disarm_) {

          ROS_INFO("[UavManager]: disarming after landing");

          disarmSrv();
        }

        changeLandingState(IDLE_STATE);

        ROS_INFO("[UavManager]: landing finished");

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

  auto control_manager_diagnostics = sh_control_manager_diag_.getMsg();

  if (waiting_for_takeoff_) {

    if (control_manager_diagnostics->active_tracker == _takeoff_tracker_name_ && control_manager_diagnostics->tracker_status.have_goal) {

      waiting_for_takeoff_ = false;
    } else {

      ROS_WARN_THROTTLE(1.0, "[UavManager]: waiting for takeoff confirmation from the ControlManager");
      return;
    }
  }

  if (takingoff_) {

    if (control_manager_diagnostics->active_tracker != _takeoff_tracker_name_ || !control_manager_diagnostics->tracker_status.have_goal) {

      ROS_INFO("[UavManager]: take off finished, switching to %s", _after_takeoff_tracker_name_.c_str());

      // if enabled, start the timer for landing after reaching max thrust
      if (_maxthrust_timer_enabled_) {
        timer_maxthrust_.start();
      }

      switchTrackerSrv(_after_takeoff_tracker_name_);

      switchControllerSrv(_after_takeoff_controller_name_);

      setOdometryCallbacksSrv(true);

      if (_after_takeoff_pirouette_) {

        pirouetteSrv();
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

  if (!sh_max_height_.hasMsg() || !sh_height_.hasMsg() || !sh_odometry_.hasMsg()) {
    return;
  }

  double                     max_height = sh_max_height_.getMsg()->value;
  double                     height     = sh_height_.getMsg()->value;
  nav_msgs::OdometryConstPtr odometry   = sh_odometry_.getMsg();

  auto [odometry_x, odometry_y, odometry_z] = mrs_lib::getPosition(odometry);

  double odometry_heading = 0;
  try {
    odometry_heading = mrs_lib::getHeading(odometry);
  }
  catch (mrs_lib::AttitudeConverter::GetHeadingException& e) {
    ROS_ERROR_THROTTLE(1.0, "[UavManager]: exception caught: '%s'", e.what());
    return;
  }

  if (height > max_height) {

    ROS_WARN_THROTTLE(1.0, "[UavManager]: max height exceeded: %.2f >  %.2f, triggering safety goto", height, max_height);

    mrs_msgs::ReferenceStamped reference_out;
    reference_out.header.frame_id = odometry->header.frame_id;
    reference_out.header.stamp    = ros::Time::now();

    reference_out.reference.position.x = odometry_x;
    reference_out.reference.position.y = odometry_y;
    reference_out.reference.position.z = odometry_z + ((max_height - _max_height_offset_) - height);

    reference_out.reference.heading = odometry_heading;

    setControlCallbacksSrv(false);

    bool success = emergencyReferenceSrv(reference_out);

    if (success) {

      ROS_INFO("[UavManager]: descending");

      fixing_max_height_ = true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[UavManager]: could not descend");

      setControlCallbacksSrv(true);
    }
  }

  if (fixing_max_height_ && height < max_height) {

    setControlCallbacksSrv(true);

    ROS_WARN_THROTTLE(1.0, "[UavManager]: safe height reached");

    fixing_max_height_ = false;
  }
}

//}

/* //{ timerMinHeight() */

void UavManager::timerMinHeight(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  ROS_INFO_ONCE("[UavManager]: min height timer spinning");

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("timerMinHeight", _min_height_checking_rate_, 0.1, event);

  if (!sh_height_.hasMsg() || !sh_odometry_.hasMsg() || !sh_control_manager_diag_.hasMsg()) {
    return;
  }

  auto control_manager_diag = sh_control_manager_diag_.getMsg();

  if (!control_manager_diag->flying_normally) {
    return;
  }

  auto   odometry = sh_odometry_.getMsg();
  double height   = sh_height_.getMsg()->value;

  auto [odometry_x, odometry_y, odometry_z] = mrs_lib::getPosition(odometry);

  double odometry_heading = 0;
  try {
    odometry_heading = mrs_lib::getHeading(odometry);
  }
  catch (mrs_lib::AttitudeConverter::GetHeadingException& e) {
    ROS_ERROR_THROTTLE(1.0, "[UavManager]: exception caught: '%s'", e.what());
    return;
  }

  if (height < _min_height_) {

    ROS_WARN_THROTTLE(1.0, "[UavManager]: min height breached: %.2f < %.2f, triggering safety goto", height, _min_height_);

    mrs_msgs::ReferenceStamped reference_out;
    reference_out.header.frame_id = odometry->header.frame_id;
    reference_out.header.stamp    = ros::Time::now();

    reference_out.reference.position.x = odometry_x;
    reference_out.reference.position.y = odometry_y;
    reference_out.reference.position.z = odometry_z + ((_min_height_ + _min_height_offset_) - height);

    reference_out.reference.heading = odometry_heading;

    setControlCallbacksSrv(false);

    bool success = emergencyReferenceSrv(reference_out);

    if (success) {

      ROS_INFO("[UavManager]: ascending");

      fixing_min_height_ = true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[UavManager]: could not ascend");

      setControlCallbacksSrv(true);
    }
  }

  if (fixing_min_height_ && height > _min_height_) {

    setControlCallbacksSrv(true);

    ROS_WARN_THROTTLE(1.0, "[UavManager]: safe height reached");

    fixing_min_height_ = false;
  }
}

//}

/* //{ timerFlighttime() */

void UavManager::timerFlighttime(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("timerFlighttime", _flighttime_timer_rate_, 0.1, event);

  auto flighttime = mrs_lib::get_mutexed(mutex_flighttime_, flighttime_);

  flighttime += 1.0 / _flighttime_timer_rate_;

  mrs_msgs::Float64 flight_time;
  flight_time.value = flighttime;

  // if enabled, start the timer for measuring the flight time
  if (_flighttime_timer_enabled_) {

    if (flighttime > _flighttime_max_time_) {

      flighttime = 0;
      timer_flighttime_.stop();

      ROS_INFO("[UavManager]: max flight time reached, landing");

      landImpl();
    }
  }

  mrs_lib::set_mutexed(mutex_flighttime_, flighttime, flighttime_);
}

//}

/* //{ timerMaxthrust() */

void UavManager::timerMaxthrust(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  if (!sh_attitude_cmd_.hasMsg()) {
    return;
  }

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("timerMaxthrust", _maxthrust_timer_rate_, 0.03, event);

  auto desired_thrust = sh_attitude_cmd_.getMsg()->thrust;

  if (desired_thrust >= _maxthrust_max_thrust_) {

    if (!maxthrust_above_threshold_) {

      maxthrust_first_time_      = ros::Time::now();
      maxthrust_above_threshold_ = true;
      ROS_WARN_THROTTLE(1.0, "[UavManager]: max thrust exceeded threshold (%.2f/%.2f)", desired_thrust, _maxthrust_max_thrust_);

    } else {

      ROS_WARN_THROTTLE(0.1, "[UavManager]: thrust over threshold (%.2f/%.2f) for %.2f s", desired_thrust, _maxthrust_max_thrust_,
                        (ros::Time::now() - maxthrust_first_time_).toSec());
    }

  } else {

    maxthrust_above_threshold_ = false;
  }

  if (maxthrust_above_threshold_ && (ros::Time::now() - maxthrust_first_time_).toSec() > _maxthrust_ungrip_timeout_) {

    ROS_WARN_THROTTLE(1.0, "[UavManager]: thrust over threshold (%.2f/%.2f) for more than %.2f s, ungripping payload", desired_thrust, _maxthrust_max_thrust_,
                      _maxthrust_ungrip_timeout_);

    ungripSrv();
  }

  if (maxthrust_above_threshold_ && (ros::Time::now() - maxthrust_first_time_).toSec() > _maxthrust_eland_timeout_) {

    timer_maxthrust_.stop();

    ROS_ERROR_THROTTLE(1.0, "[UavManager]: thrust over threshold (%.2f/%.2f) for more than %.2f s, calling for emergency landing", desired_thrust,
                       _maxthrust_max_thrust_, _maxthrust_eland_timeout_);

    elandSrv();
  }
}

//}

/* //{ timerDiagnostics() */

void UavManager::timerDiagnostics(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("timerDiagnostics", _maxthrust_timer_rate_, 0.03, event);

  bool got_gps_est = false;
  bool got_rtk_est = false;

  if (sh_odometry_diagnostics_.hasMsg()) {  // get current position in lat-lon

    auto                     odom_diag      = sh_odometry_diagnostics_.getMsg();
    std::vector<std::string> lat_estimators = odom_diag.get()->available_lat_estimators;

    got_gps_est = std::find(lat_estimators.begin(), lat_estimators.end(), "GPS") != lat_estimators.end();
    got_rtk_est = std::find(lat_estimators.begin(), lat_estimators.end(), "RTK") != lat_estimators.end();
  }

  mrs_msgs::UavManagerDiagnostics diag;

  diag.stamp    = ros::Time::now();
  diag.uav_name = _uav_name_;

  auto flighttime = mrs_lib::get_mutexed(mutex_flighttime_, flighttime_);

  // fill in the acumulated flight time
  diag.flight_time = flighttime;

  if (sh_odometry_.hasMsg()) {  // get current position in lat-lon

    if (got_gps_est || got_rtk_est) {

      nav_msgs::Odometry odom = *sh_odometry_.getMsg();

      geometry_msgs::PoseStamped uav_pose;
      uav_pose.pose = mrs_lib::getPose(odom);

      auto res = transformer_->transformSingle("latlon_origin", uav_pose);

      if (res) {
        diag.cur_latitude  = res.value().pose.position.x;
        diag.cur_longitude = res.value().pose.position.y;
      }
    }
  }

  if (takeoff_successful_) {

    if (got_gps_est || got_rtk_est) {

      auto land_there_reference = mrs_lib::get_mutexed(mutex_land_there_reference_, land_there_reference_);

      auto res = transformer_->transformSingle("latlon_origin", land_there_reference);

      if (res) {
        diag.home_latitude  = res.value().reference.position.x;
        diag.home_longitude = res.value().reference.position.y;
      }
    }
  }

  try {
    publisher_diagnostics_.publish(diag);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", publisher_diagnostics_.getTopic().c_str());
  }
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | --------------------- topic callbacks -------------------- |

/* //{ callbackMavrosGps() */

void UavManager::callbackMavrosGps(mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>& wrp) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackMavrosGps");

  sensor_msgs::NavSatFixConstPtr data = wrp.getMsg();

  transformer_->setCurrentLatLon(data->latitude, data->longitude);
}

//}

/* //{ callbackOdometry() */

void UavManager::callbackOdometry(mrs_lib::SubscribeHandler<nav_msgs::Odometry>& wrp) {

  if (!is_initialized_)
    return;

  nav_msgs::OdometryConstPtr data = wrp.getMsg();

  transformer_->setCurrentControlFrame(data->header.frame_id);
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackTakeoff() */

bool UavManager::callbackTakeoff([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  /* preconditions //{ */

  {
    std::stringstream ss;

    if (!sh_odometry_.hasMsg()) {
      ss << "can not takeoff, missing odometry!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    if (!sh_mavros_state_.hasMsg() || (ros::Time::now() - sh_mavros_state_.lastMsgTime()).toSec() > 5.0) {
      ss << "can not takeoff, missing mavros state!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    if (!sh_mavros_state_.getMsg()->armed) {
      ss << "can not takeoff, UAV not armed!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    if (sh_mavros_state_.getMsg()->mode != "OFFBOARD") {
      ss << "can not takeoff, UAV not in offboard mode!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    {
      if (!sh_control_manager_diag_.hasMsg()) {
        ss << "can not takeoff, missing control manager diagnostics!";
        ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
        res.message = ss.str();
        res.success = false;
        return true;
      }

      if (_null_tracker_name_ != sh_control_manager_diag_.getMsg()->active_tracker) {
        ss << "can not takeoff, need '" << _null_tracker_name_ << "' to be active!";
        ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
        res.message = ss.str();
        res.success = false;
        return true;
      }
    }

    if (!sh_attitude_cmd_.hasMsg()) {
      ss << "can not takeoff, missing target attitude!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    if (_gain_manager_required_ && (ros::Time::now() - sh_gains_diag_.lastMsgTime()).toSec() > 5.0) {
      ss << "can not takeoff, GainManager is not running!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    if (_constraint_manager_required_ && (ros::Time::now() - sh_constraints_diag_.lastMsgTime()).toSec() > 5.0) {
      ss << "can not takeoff, ConstraintManager is not running!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    if (!sh_motors_.hasMsg()) {

      ss << "can not takeoff, missing the motors data!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;

    } else if ((ros::Time::now() - sh_motors_.lastMsgTime()).toSec() > 1.0) {

      ss << "can not takeoff, the motors data is too old!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;

    } else if (!sh_motors_.getMsg()->data) {

      ss << "can not takeoff, the motors are off!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    if (number_of_takeoffs_ > 0) {

      auto last_mass_difference = mrs_lib::get_mutexed(mutex_last_mass_difference_, last_mass_difference_);

      if (last_mass_difference > 1.0) {

        ss << std::setprecision(2);
        ss << "can not takeoff, estimated mass difference is too large: " << _null_tracker_name_ << "!";
        ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
        res.message = ss.str();
        res.success = false;
        return true;
      }
    }
  }

  //}

  auto control_manager_diagnostics = sh_control_manager_diag_.getMsg();
  auto odometry                    = sh_odometry_.getMsg();
  auto [odom_x, odom_y, odom_z]    = mrs_lib::getPosition(sh_odometry_.getMsg());

  double odom_heading;
  try {
    odom_heading = mrs_lib::getHeading(sh_odometry_.getMsg());
  }
  catch (mrs_lib::AttitudeConverter::GetHeadingException& e) {
    ROS_ERROR_THROTTLE(1.0, "[UavManager]: exception caught: '%s'", e.what());

    std::stringstream ss;
    ss << "could not calculate current heading";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
    res.message = ss.str();
    res.success = false;
    return true;
  }

  ROS_INFO("[UavManager]: taking off");

  setOdometryCallbacksSrv(false);

  // activating the takeoff controller
  {
    std::string old_controller      = sh_control_manager_diag_.getMsg()->active_controller;
    bool        controller_switched = switchControllerSrv(_takeoff_controller_name_);

    // if it fails, activate back the old controller
    // this is no big deal since the control outputs are not used
    // until NullTracker is active
    if (!controller_switched) {

      std::stringstream ss;
      ss << "could not activate '" << _takeoff_controller_name_ << "' for takeoff";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.success = false;
      res.message = ss.str();

      switchControllerSrv(old_controller);

      return true;
    }
  }

  // activate the takeoff tracker
  {
    std::string old_tracker      = sh_control_manager_diag_.getMsg()->active_tracker;
    bool        tracker_switched = switchTrackerSrv(_takeoff_tracker_name_);

    // if it fails, activate back the old tracker
    if (!tracker_switched) {

      std::stringstream ss;
      ss << "could not activate '" << _takeoff_tracker_name_ << "' for takeoff";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.success = false;
      res.message = ss.str();

      switchTrackerSrv(old_tracker);

      return true;
    }
  }

  // now the takeoff tracker and controller are active
  // the UAV is basically hovering on the ground
  // (the controller is probably rumping up the thrust now)

  // call the takeoff service at the takeoff tracker
  {
    bool takeoff_successful = takeoffSrv();

    // if the takeoff was not successful, switch to NullTracker
    if (takeoff_successful) {

      // save the current spot for later landing
      {
        std::scoped_lock lock(mutex_land_there_reference_);

        land_there_reference_.header               = odometry->header;
        land_there_reference_.reference.position.x = odom_x;
        land_there_reference_.reference.position.y = odom_y;
        land_there_reference_.reference.position.z = odom_z;
        land_there_reference_.reference.heading    = odom_heading;
      }

      timer_flighttime_.start();

      std::stringstream ss;
      ss << "taking off";
      res.success = true;
      res.message = ss.str();
      ROS_INFO_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());

      takingoff_ = true;
      number_of_takeoffs_++;
      waiting_for_takeoff_ = true;

      // start the takeoff timer
      timer_takeoff_.start();

      takeoff_successful_ = takeoff_successful;

    } else {

      std::stringstream ss;
      ss << "takeoff was not successful";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.success = false;
      res.message = ss.str();

      // if the call for takeoff fails, call for emergency landing
      elandSrv();
    }
  }

  return true;
}

//}

/* //{ callbackLand() */

bool UavManager::callbackLand([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  /* preconditions //{ */

  {
    std::stringstream ss;

    if (!sh_odometry_.hasMsg()) {
      ss << "can not land, missing odometry!";
      res.message = ss.str();
      res.success = false;
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      return true;
    }

    {
      if (!sh_control_manager_diag_.hasMsg()) {
        ss << "can not land, missing control manager diagnostics!";
        res.message = ss.str();
        res.success = false;
        ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
        return true;
      }

      if (_null_tracker_name_ == sh_control_manager_diag_.getMsg()->active_tracker) {
        ss << "can not land, '" << _null_tracker_name_ << "' is active!";
        ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
        res.message = ss.str();
        res.success = false;
        return true;
      }
    }

    if (!sh_attitude_cmd_.hasMsg()) {
      ss << "can not land, missing attitude command!";
      res.message = ss.str();
      res.success = false;
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      return true;
    }
  }

  //}

  auto [success, message] = landWithDescendImpl();

  res.message = message;
  res.success = success;

  return true;
}

//}

/* //{ callbackLandHome() */

bool UavManager::callbackLandHome([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  /* preconditions //{ */

  {
    std::stringstream ss;

    if (!sh_odometry_.hasMsg()) {
      ss << "can not land, missing odometry!";
      res.message = ss.str();
      res.success = false;
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      return true;
    }

    {
      if (!sh_control_manager_diag_.hasMsg()) {
        ss << "can not land, missing tracker status!";
        res.message = ss.str();
        res.success = false;
        ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
        return true;
      }

      if (_null_tracker_name_ == sh_control_manager_diag_.getMsg()->active_tracker) {
        ss << "can not land, '" << _null_tracker_name_ << "' is active!";
        ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
        res.message = ss.str();
        res.success = false;
        return true;
      }
    }

    if (!sh_attitude_cmd_.hasMsg()) {
      ss << "can not land, missing attitude command!";
      res.message = ss.str();
      res.success = false;
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      return true;
    }

    if (fixing_max_height_) {
      ss << "can not land, descedning to safe height!";
      res.message = ss.str();
      res.success = false;
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      return true;
    }

    if (current_state_landing_ != IDLE_STATE) {
      ss << "can not land, already landing!";
      res.message = ss.str();
      res.success = false;
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      return true;
    }
  }

  //}

  ungripSrv();

  mrs_msgs::ReferenceStamped reference_out;

  {
    std::scoped_lock lock(mutex_land_there_reference_);

    // get the current altitude in land_there_reference_.header.frame_id;
    geometry_msgs::PoseStamped current_pose;
    current_pose.header.stamp     = ros::Time::now();
    current_pose.header.frame_id  = _uav_name_ + "/fcu";
    current_pose.pose.position.x  = 0;
    current_pose.pose.position.y  = 0;
    current_pose.pose.position.z  = 0;
    current_pose.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

    auto response = transformer_->transformSingle(land_there_reference_.header.frame_id, current_pose);

    if (response) {

      land_there_reference_.reference.position.z = response.value().pose.position.z;
      ROS_DEBUG("[UavManager]: current altitude is %.2f m", land_there_reference_.reference.position.z);

    } else {

      std::stringstream ss;
      ss << "could not transform current height to " << land_there_reference_.header.frame_id;
      ROS_ERROR_STREAM("[UavManager]: " << ss.str());

      res.success = false;
      res.message = ss.str();
      return true;
    }

    reference_out.header.frame_id = land_there_reference_.header.frame_id;
    reference_out.header.stamp    = ros::Time::now();
    reference_out.reference       = land_there_reference_.reference;
  }

  bool service_success = emergencyReferenceSrv(reference_out);

  if (service_success) {

    std::stringstream ss;
    ss << "flying home for landing";
    ROS_INFO_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());

    res.success = true;
    res.message = ss.str();

    // stop the eventual takeoff
    waiting_for_takeoff_ = false;
    takingoff_           = false;
    timer_takeoff_.stop();

    changeLandingState(FLY_THERE_STATE);

    timer_landing_.start();

  } else {

    std::stringstream ss;
    ss << "can not fly home for landing";
    ROS_ERROR_STREAM("[UavManager]: " << ss.str());

    res.success = false;
    res.message = ss.str();
  }

  return true;
}

//}

/* //{ callbackLandThere() */

bool UavManager::callbackLandThere(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res) {

  if (!is_initialized_)
    return false;

  /* preconditions //{ */

  {
    std::stringstream ss;

    if (!sh_odometry_.hasMsg()) {
      ss << "can not land, missing odometry!";
      res.message = ss.str();
      res.success = false;
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      return true;
    }

    {
      if (!sh_control_manager_diag_.hasMsg()) {
        ss << "can not land, missing tracker status!";
        res.message = ss.str();
        res.success = false;
        ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
        return true;
      }

      if (_null_tracker_name_ == sh_control_manager_diag_.getMsg()->active_tracker) {
        ss << "can not land, '" << _null_tracker_name_ << "' is active!";
        ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
        res.message = ss.str();
        res.success = false;
        return true;
      }
    }

    if (!sh_attitude_cmd_.hasMsg()) {
      ss << "can not land, missing attitude command!";
      res.message = ss.str();
      res.success = false;
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      return true;
    }

    if (fixing_max_height_) {
      ss << "can not land, descedning to safe height!";
      res.message = ss.str();
      res.success = false;
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      return true;
    }
  }

  //}

  ungripSrv();

  mrs_msgs::ReferenceStamped reference_out;

  {
    std::scoped_lock lock(mutex_land_there_reference_);

    land_there_reference_.header    = req.header;
    land_there_reference_.reference = req.reference;

    reference_out.header.frame_id = land_there_reference_.header.frame_id;
    reference_out.header.stamp    = ros::Time::now();
    reference_out.reference       = land_there_reference_.reference;
  }

  bool service_success = emergencyReferenceSrv(reference_out);

  if (service_success) {

    std::stringstream ss;
    ss << "flying there for landing";
    ROS_INFO_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());

    res.success = true;
    res.message = ss.str();

    // stop the eventual takeoff
    waiting_for_takeoff_ = false;
    takingoff_           = false;
    timer_takeoff_.stop();

    changeLandingState(FLY_THERE_STATE);

    timer_landing_.start();

  } else {

    std::stringstream ss;
    ss << "can not fly there for landing";
    ROS_ERROR_STREAM("[UavManager]: " << ss.str());

    res.success = false;
    res.message = ss.str();
  }

  return true;
}

//}

// | ------------------------ routines ------------------------ |

/* landImpl() //{ */

std::tuple<bool, std::string> UavManager::landImpl(void) {

  // activating the landing controller
  {
    std::string old_controller      = sh_control_manager_diag_.getMsg()->active_controller;
    bool        controller_switched = switchControllerSrv(_landing_controller_name_);

    // if it fails, activate eland
    // Tomas: I pressume that its more important to get the UAV to the ground rather than
    // just throw out error.
    if (!controller_switched) {

      std::stringstream ss;
      ss << "could not activate '" << _takeoff_controller_name_ << "' for landing";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());

      elandSrv();

      return std::tuple(false, ss.str());
    }
  }

  // activate the landing tracker
  {
    std::string old_tracker      = sh_control_manager_diag_.getMsg()->active_tracker;
    bool        tracker_switched = switchTrackerSrv(_landing_tracker_name_);

    // if it fails, activate eland
    // Tomas: I pressume that its more important to get the UAV to the ground rather than
    // just throw out error.
    if (!tracker_switched) {

      std::stringstream ss;
      ss << "could not activate '" << _takeoff_tracker_name_ << "' for landing";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());

      elandSrv();

      return std::tuple(false, ss.str());
    }
  }

  // call the landing service
  {
    bool land_successful = landSrv();

    if (land_successful) {

      // stop the eventual takeoff
      waiting_for_takeoff_ = false;
      takingoff_           = false;
      timer_takeoff_.stop();

      // stop counting the flight time
      timer_flighttime_.stop();

      // remember the last valid mass estimated
      // used during subsequent takeoff
      last_mass_difference_ = sh_attitude_cmd_.getMsg()->mass_difference;

      setOdometryCallbacksSrv(false);

      changeLandingState(LANDING_STATE);

      timer_landing_.start();

      std::stringstream ss;
      ss << "landing initiated";
      ROS_INFO_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());

      return std::tuple(true, ss.str());

    } else {

      std::stringstream ss;
      ss << "could not land";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());

      elandSrv();

      return std::tuple(false, ss.str());
    }
  }
}

//}

/* landWithDescendImpl() //{ */

std::tuple<bool, std::string> UavManager::landWithDescendImpl(void) {

  // if the height information is available
  if (sh_height_.hasMsg()) {

    double height = sh_height_.getMsg()->value;

    if (height > 0 && height >= _landing_descend_height_ + 1.0) {

      auto odometry = sh_odometry_.getMsg();

      ungripSrv();

      {
        std::scoped_lock lock(mutex_land_there_reference_);

        // FOR FUTURE ME: Do not change this, we need it to be filled for the final check later
        land_there_reference_.header.frame_id      = "";
        land_there_reference_.header.stamp         = ros::Time::now();
        land_there_reference_.reference.position.x = odometry->pose.pose.position.x;
        land_there_reference_.reference.position.y = odometry->pose.pose.position.y;
        land_there_reference_.reference.position.z = odometry->pose.pose.position.z - (height - _landing_descend_height_);
        land_there_reference_.reference.heading    = mrs_lib::AttitudeConverter(odometry->pose.pose.orientation).getHeading();
      }

      bool service_success = emergencyReferenceSrv(land_there_reference_);

      if (service_success) {

        std::stringstream ss;
        ss << "flying down for landing";
        ROS_INFO_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());

        // stop the eventual takeoff
        waiting_for_takeoff_ = false;
        takingoff_           = false;
        timer_takeoff_.stop();

        changeLandingState(FLY_THERE_STATE);

        timer_landing_.start();

        return std::tuple(true, ss.str());

      } else {

        std::stringstream ss;
        ss << "can not fly down for landing";
        ROS_ERROR_STREAM("[UavManager]: " << ss.str());
      }
    }
  }

  auto [success, message] = landImpl();

  return std::tuple(success, message);
}

//}

// | ----------------- service client wrappers ---------------- |

/* setOdometryCallbacksSrv() //{ */

void UavManager::setOdometryCallbacksSrv(const bool& input) {

  ROS_INFO("[UavManager]: switching odometry callabcks to %s", input ? "ON" : "OFF");

  std_srvs::SetBool srv;

  srv.request.data = input;

  bool res = service_client_odometry_callbacks_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN("[UavManager]: service call for toggle odometry callbacks returned: %s.", srv.response.message.c_str());
    }

  } else {
    ROS_ERROR("[UavManager]: service call for toggle odometry callbacks failed!");
  }
}

//}

/* setControlCallbacksSrv() //{ */

void UavManager::setControlCallbacksSrv(const bool& input) {

  ROS_INFO("[UavManager]: switching control callabcks to %s", input ? "ON" : "OFF");

  std_srvs::SetBool srv;

  srv.request.data = input;

  bool res = service_client_control_callbacks_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN("[UavManager]: service call for setting control callbacks returned: %s.", srv.response.message.c_str());
    }

  } else {
    ROS_ERROR("[UavManager]: service call for setting control callbacks failed!");
  }
}

//}

/* ungripSrv() //{ */

void UavManager::ungripSrv(void) {

  ROS_DEBUG_THROTTLE(1.0, "[UavManager]: ungripping payload");

  std_srvs::Trigger srv;

  bool res = service_client_ungrip_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_DEBUG_THROTTLE(1.0, "[UavManager]: service call for ungripping payload returned: %s.", srv.response.message.c_str());
    }

  } else {
    ROS_DEBUG_THROTTLE(1.0, "[UavManager]: service call for ungripping payload failed!");
  }
}

//}

/* pirouetteSrv() //{ */

void UavManager::pirouetteSrv(void) {

  std_srvs::Trigger srv;

  bool res = service_client_pirouette_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[UavManager]: service call for pirouette returned: %s.", srv.response.message.c_str());
    }

  } else {
    ROS_ERROR_THROTTLE(1.0, "[UavManager]: service call for pirouette failed!");
  }
}

//}

/* disarmSrv() //{ */

void UavManager::disarmSrv(void) {

  std_srvs::SetBool srv;

  srv.request.data = false;

  bool res = service_client_arm_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[UavManager]: service call disarming returned: %s.", srv.response.message.c_str());
    }

  } else {
    ROS_ERROR_THROTTLE(1.0, "[UavManager]: service call for disarming failed!");
  }
}

//}

/* switchControllerSrv() //{ */

bool UavManager::switchControllerSrv(const std::string& controller) {

  ROS_INFO_STREAM("[UavManager]: activating controller '" << controller << "'");

  mrs_msgs::String srv;
  srv.request.value = controller;

  bool res = service_client_switch_controller_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN("[UavManager]: service call for switching controller returned: '%s'", srv.response.message.c_str());
    }

    return srv.response.success;

  } else {

    ROS_ERROR("[UavManager]: service call for switching controller failed!");

    return false;
  }
}

//}

/* switchTrackerSrv() //{ */

bool UavManager::switchTrackerSrv(const std::string& tracker) {

  ROS_INFO_STREAM("[UavManager]: activating tracker '" << tracker << "'");


  mrs_msgs::String srv;
  srv.request.value = tracker;

  bool res = service_client_switch_tracker_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN("[UavManager]: service call for switching tracker returned: '%s'", srv.response.message.c_str());
    }

    return srv.response.success;

  } else {

    ROS_ERROR("[UavManager]: service call for switching tracker failed!");

    return false;
  }
}

//}

/* landSrv() //{ */

bool UavManager::landSrv(void) {

  ROS_INFO("[UavManager]: calling for landing");

  std_srvs::Trigger srv;

  bool res = service_client_land_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN("[UavManager]: service call for landing returned: '%s'", srv.response.message.c_str());
    }

    return srv.response.success;

  } else {

    ROS_ERROR("[UavManager]: service call for landing failed!");

    return false;
  }
}

//}

/* elandSrv() //{ */

bool UavManager::elandSrv(void) {

  ROS_INFO("[UavManager]: calling for eland");

  std_srvs::Trigger srv;

  bool res = service_client_eland_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN("[UavManager]: service call for eland returned: '%s'", srv.response.message.c_str());
    }

    return srv.response.success;

  } else {

    ROS_ERROR("[UavManager]: service call for eland failed!");

    return false;
  }
}

//}

/* takeoffSrv() //{ */

bool UavManager::takeoffSrv(void) {

  ROS_INFO("[UavManager]: calling for takeoff to height '%.2f m'", _takeoff_height_);

  mrs_msgs::Vec1 srv;

  srv.request.goal = _takeoff_height_;

  bool res = service_client_takeoff_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN("[UavManager]: service call for takeoff returned: '%s'", srv.response.message.c_str());
    }

    return srv.response.success;

  } else {

    ROS_ERROR("[UavManager]: service call for takeoff failed!");

    return false;
  }
}

//}

/* emergencyReferenceSrv() //{ */

bool UavManager::emergencyReferenceSrv(const mrs_msgs::ReferenceStamped& goal) {

  ROS_INFO("[UavManager]: calling for emergency reference");

  mrs_msgs::ReferenceStampedSrv srv;

  srv.request.header    = goal.header;
  srv.request.reference = goal.reference;

  bool res = service_client_emergency_reference_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN("[UavManager]: service call for emergency reference returned: '%s'", srv.response.message.c_str());
    }

    return srv.response.success;

  } else {

    ROS_ERROR("[UavManager]: service call for emergency reference failed!");

    return false;
  }
}

//}

}  // namespace uav_manager

}  // namespace mrs_uav_managers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::uav_manager::UavManager, nodelet::Nodelet)
