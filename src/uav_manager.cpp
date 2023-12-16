/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/Vector3Stamped.h>

#include <nav_msgs/Odometry.h>
#include <mrs_msgs/Vec1.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/Float64.h>
#include <mrs_msgs/BoolStamped.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/ConstraintManagerDiagnostics.h>
#include <mrs_msgs/GainManagerDiagnostics.h>
#include <mrs_msgs/UavManagerDiagnostics.h>
#include <mrs_msgs/EstimationDiagnostics.h>
#include <mrs_msgs/HwApiStatus.h>
#include <mrs_msgs/ControllerDiagnostics.h>
#include <mrs_msgs/HwApiCapabilities.h>

#include <sensor_msgs/NavSatFix.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/quadratic_throttle_model.h>

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
  GOTO_STATE,
  LANDING_STATE,

} LandingStates_t;

const char* state_names[3] = {

    "IDLING", "GOTO", "LANDING"};

class UavManager : public nodelet::Nodelet {

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  std::string     _uav_name_;

public:
  std::shared_ptr<mrs_lib::Transformer> transformer_;

public:
  bool                                       scope_timer_enabled_ = false;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;

public:
  virtual void onInit();

  // | ------------------------- HW API ------------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiCapabilities> sh_hw_api_capabilities_;

  // this timer will check till we already got the hardware api diagnostics
  // then it will trigger the initialization of the controllers and finish
  // the initialization of the ControlManager
  ros::Timer timer_hw_api_capabilities_;
  void       timerHwApiCapabilities(const ros::TimerEvent& event);

  void preinitialize(void);
  void initialize(void);

  mrs_msgs::HwApiCapabilities hw_api_capabilities_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::ControllerDiagnostics>        sh_controller_diagnostics_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                     sh_odometry_;
  mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>        sh_estimation_diagnostics_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>    sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<std_msgs::Float64>                      sh_mass_estimate_;
  mrs_lib::SubscribeHandler<std_msgs::Float64>                      sh_throttle_;
  mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>               sh_height_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus>                  sh_hw_api_status_;
  mrs_lib::SubscribeHandler<mrs_msgs::GainManagerDiagnostics>       sh_gains_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::ConstraintManagerDiagnostics> sh_constraints_diag_;
  mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>                 sh_hw_api_gnss_;
  mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>               sh_max_height_;
  mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>               sh_tracker_cmd_;

  void callbackHwApiGNSS(const sensor_msgs::NavSatFix::ConstPtr msg);
  void callbackOdometry(const nav_msgs::Odometry::ConstPtr msg);

  // service servers
  ros::ServiceServer service_server_takeoff_;
  ros::ServiceServer service_server_land_;
  ros::ServiceServer service_server_land_home_;
  ros::ServiceServer service_server_land_there_;
  ros::ServiceServer service_server_midair_activation_;

  // service callbacks
  bool callbackTakeoff(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackLand(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackLandHome(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackLandThere(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);

  // service clients
  mrs_lib::ServiceClientHandler<mrs_msgs::Vec1>                sch_takeoff_;
  mrs_lib::ServiceClientHandler<mrs_msgs::String>              sch_switch_tracker_;
  mrs_lib::ServiceClientHandler<mrs_msgs::String>              sch_switch_controller_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger>             sch_land_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger>             sch_eland_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger>             sch_ehover_;
  mrs_lib::ServiceClientHandler<std_srvs::SetBool>             sch_control_callbacks_;
  mrs_lib::ServiceClientHandler<mrs_msgs::ReferenceStampedSrv> sch_emergency_reference_;
  mrs_lib::ServiceClientHandler<std_srvs::SetBool>             sch_arm_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger>             sch_pirouette_;
  mrs_lib::ServiceClientHandler<std_srvs::SetBool>             sch_odometry_callbacks_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger>             sch_ungrip_;
  mrs_lib::ServiceClientHandler<std_srvs::SetBool>             sch_toggle_control_output_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger>             sch_offboard_;

  // service client wrappers
  bool takeoffSrv(void);
  bool switchTrackerSrv(const std::string& tracker);
  bool switchControllerSrv(const std::string& controller);
  bool landSrv(void);
  bool elandSrv(void);
  bool ehoverSrv(void);
  void disarmSrv(void);
  bool emergencyReferenceSrv(const mrs_msgs::ReferenceStamped& goal);
  void setOdometryCallbacksSrv(const bool& input);
  void setControlCallbacksSrv(const bool& input);
  void ungripSrv(void);
  bool toggleControlOutput(const bool& input);
  void pirouetteSrv(void);
  bool offboardSrv(const bool in);

  ros::Timer timer_takeoff_;
  ros::Timer timer_max_height_;
  ros::Timer timer_min_height_;
  ros::Timer timer_landing_;
  ros::Timer timer_maxthrottle_;
  ros::Timer timer_flighttime_;
  ros::Timer timer_diagnostics_;
  ros::Timer timer_midair_activation_;

  // timer callbacks
  void timerLanding(const ros::TimerEvent& event);
  void timerTakeoff(const ros::TimerEvent& event);
  void timerMaxHeight(const ros::TimerEvent& event);
  void timerMinHeight(const ros::TimerEvent& event);
  void timerFlightTime(const ros::TimerEvent& event);
  void timerMaxthrottle(const ros::TimerEvent& event);
  void timerDiagnostics(const ros::TimerEvent& event);

  // publishers
  mrs_lib::PublisherHandler<mrs_msgs::UavManagerDiagnostics> ph_diag_;

  // max height checking
  bool              _max_height_enabled_ = false;
  double            _max_height_checking_rate_;
  double            _max_height_offset_;
  std::atomic<bool> fixing_max_height_ = false;

  // min height checking
  bool              _min_height_enabled_ = false;
  double            _min_height_checking_rate_;
  double            _min_height_offset_;
  double            _min_height_;
  std::atomic<bool> fixing_min_height_ = false;

  // mass estimation during landing
  double    throttle_mass_estimate_;
  bool      throttle_under_threshold_ = false;
  ros::Time throttle_mass_estimate_first_time_;

  // velocity during landing
  bool      velocity_under_threshold_ = false;
  ros::Time velocity_under_threshold_first_time_;

  bool _gain_manager_required_       = false;
  bool _constraint_manager_required_ = false;

  std::tuple<bool, std::string> landImpl(void);
  std::tuple<bool, std::string> landWithDescendImpl(void);
  std::tuple<bool, std::string> midairActivationImpl(void);

  // saved takeoff coordinates and allows to land there again
  mrs_msgs::ReferenceStamped land_there_reference_;
  std::mutex                 mutex_land_there_reference_;

  // to which height to takeoff
  double _takeoff_height_;

  std::atomic<bool> takeoff_successful_ = false;

  // names of important trackers
  std::string _null_tracker_name_;

  // takeoff timer
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
  double      _landing_tracking_tolerance_translation_;
  double      _landing_tracking_tolerance_heading_;

  // diagnostics timer
  double _diagnostics_timer_rate_;

  mrs_lib::quadratic_throttle_model::MotorParams_t _throttle_model_;

  // landing state machine states
  LandingStates_t current_state_landing_  = IDLE_STATE;
  LandingStates_t previous_state_landing_ = IDLE_STATE;

  // timer for checking max flight time
  double     _flighttime_timer_rate_;
  double     _flighttime_max_time_;
  bool       _flighttime_timer_enabled_ = false;
  double     flighttime_                = 0;
  std::mutex mutex_flighttime_;

  // timer for checking maximum throttle
  bool      _maxthrottle_timer_enabled_ = false;
  double    _maxthrottle_timer_rate_;
  double    _maxthrottle_max_throttle_;
  double    _maxthrottle_eland_timeout_;
  double    _maxthrottle_ungrip_timeout_;
  bool      maxthrottle_above_threshold_ = false;
  ros::Time maxthrottle_first_time_;

  // profiler
  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // midair activation
  void      timerMidairActivation(const ros::TimerEvent& event);
  bool      callbackMidairActivation(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::Time midair_activation_started_;

  double      _midair_activation_timer_rate_;
  std::string _midair_activation_during_controller_;
  std::string _midair_activation_during_tracker_;
  std::string _midair_activation_after_controller_;
  std::string _midair_activation_after_tracker_;

  void changeLandingState(LandingStates_t new_state);
};

//}

/* onInit() //{ */

void UavManager::onInit() {
  preinitialize();
}

//}

/* preinitialize() //{ */

void UavManager::preinitialize(void) {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "ControlManager";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_hw_api_capabilities_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiCapabilities>(shopts, "hw_api_capabilities_in");

  timer_hw_api_capabilities_ = nh_.createTimer(ros::Rate(1.0), &UavManager::timerHwApiCapabilities, this);
}

//}

/* initialize() //{ */

void UavManager::initialize() {

  ROS_INFO("[UavManager]: initializing");

  mrs_lib::ParamLoader param_loader(nh_, "UavManager");

  std::string custom_config_path;
  std::string platform_config_path;
  std::string world_config_path;

  param_loader.loadParam("custom_config", custom_config_path);
  param_loader.loadParam("platform_config", platform_config_path);
  param_loader.loadParam("world_config", world_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  if (platform_config_path != "") {
    param_loader.addYamlFile(platform_config_path);
  }

  if (world_config_path != "") {
    param_loader.addYamlFile(world_config_path);
  }

  param_loader.addYamlFileFromParam("private_config");
  param_loader.addYamlFileFromParam("public_config");

  const std::string yaml_prefix = "mrs_uav_managers/uav_manager/";

  // params passed from the launch file are not prefixed
  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("enable_profiler", _profiler_enabled_);
  param_loader.loadParam("uav_mass", _uav_mass_);
  param_loader.loadParam("g", _g_);

  // motor params are also not prefixed, since they are common to more nodes
  param_loader.loadParam("motor_params/n_motors", _throttle_model_.n_motors);
  param_loader.loadParam("motor_params/a", _throttle_model_.A);
  param_loader.loadParam("motor_params/b", _throttle_model_.B);

  param_loader.loadParam(yaml_prefix + "null_tracker", _null_tracker_name_);

  param_loader.loadParam(yaml_prefix + "takeoff/rate", _takeoff_timer_rate_);
  param_loader.loadParam(yaml_prefix + "takeoff/after_takeoff/tracker", _after_takeoff_tracker_name_);
  param_loader.loadParam(yaml_prefix + "takeoff/after_takeoff/controller", _after_takeoff_controller_name_);
  param_loader.loadParam(yaml_prefix + "takeoff/after_takeoff/pirouette", _after_takeoff_pirouette_);
  param_loader.loadParam(yaml_prefix + "takeoff/during_takeoff/controller", _takeoff_controller_name_);
  param_loader.loadParam(yaml_prefix + "takeoff/during_takeoff/tracker", _takeoff_tracker_name_);
  param_loader.loadParam(yaml_prefix + "takeoff/takeoff_height", _takeoff_height_);

  param_loader.loadParam(yaml_prefix + "landing/rate", _landing_timer_rate_);
  param_loader.loadParam(yaml_prefix + "landing/landing_tracker", _landing_tracker_name_);
  param_loader.loadParam(yaml_prefix + "landing/landing_controller", _landing_controller_name_);
  param_loader.loadParam(yaml_prefix + "landing/landing_cutoff_mass_factor", _landing_cutoff_mass_factor_);
  param_loader.loadParam(yaml_prefix + "landing/landing_cutoff_timeout", _landing_cutoff_mass_timeout_);
  param_loader.loadParam(yaml_prefix + "landing/disarm", _landing_disarm_);
  param_loader.loadParam(yaml_prefix + "landing/descend_height", _landing_descend_height_);
  param_loader.loadParam(yaml_prefix + "landing/tracking_tolerance/translation", _landing_tracking_tolerance_translation_);
  param_loader.loadParam(yaml_prefix + "landing/tracking_tolerance/heading", _landing_tracking_tolerance_heading_);

  param_loader.loadParam(yaml_prefix + "midair_activation/rate", _midair_activation_timer_rate_);
  param_loader.loadParam(yaml_prefix + "midair_activation/during_activation/controller", _midair_activation_during_controller_);
  param_loader.loadParam(yaml_prefix + "midair_activation/during_activation/tracker", _midair_activation_during_tracker_);
  param_loader.loadParam(yaml_prefix + "midair_activation/after_activation/controller", _midair_activation_after_controller_);
  param_loader.loadParam(yaml_prefix + "midair_activation/after_activation/tracker", _midair_activation_after_tracker_);

  param_loader.loadParam(yaml_prefix + "max_height_checking/enabled", _max_height_enabled_);
  param_loader.loadParam(yaml_prefix + "max_height_checking/rate", _max_height_checking_rate_);
  param_loader.loadParam(yaml_prefix + "max_height_checking/safety_height_offset", _max_height_offset_);

  param_loader.loadParam(yaml_prefix + "min_height_checking/enabled", _min_height_enabled_);
  param_loader.loadParam(yaml_prefix + "min_height_checking/rate", _min_height_checking_rate_);
  param_loader.loadParam(yaml_prefix + "min_height_checking/safety_height_offset", _min_height_offset_);
  param_loader.loadParam(yaml_prefix + "min_height_checking/min_height", _min_height_);

  param_loader.loadParam(yaml_prefix + "require_gain_manager", _gain_manager_required_);
  param_loader.loadParam(yaml_prefix + "require_constraint_manager", _constraint_manager_required_);

  param_loader.loadParam(yaml_prefix + "flight_timer/enabled", _flighttime_timer_enabled_);
  param_loader.loadParam(yaml_prefix + "flight_timer/rate", _flighttime_timer_rate_);
  param_loader.loadParam(yaml_prefix + "flight_timer/max_time", _flighttime_max_time_);

  param_loader.loadParam(yaml_prefix + "max_throttle/enabled", _maxthrottle_timer_enabled_);
  param_loader.loadParam(yaml_prefix + "max_throttle/rate", _maxthrottle_timer_rate_);
  param_loader.loadParam(yaml_prefix + "max_throttle/max_throttle", _maxthrottle_max_throttle_);
  param_loader.loadParam(yaml_prefix + "max_throttle/eland_timeout", _maxthrottle_eland_timeout_);
  param_loader.loadParam(yaml_prefix + "max_throttle/ungrip_timeout", _maxthrottle_ungrip_timeout_);

  param_loader.loadParam(yaml_prefix + "diagnostics/rate", _diagnostics_timer_rate_);

  // | ------------------- scope timer logger ------------------- |

  param_loader.loadParam(yaml_prefix + "scope_timer/enabled", scope_timer_enabled_);
  const std::string scope_timer_log_filename = param_loader.loadParam2("scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<mrs_lib::ScopeTimerLogger>(scope_timer_log_filename, scope_timer_enabled_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[UavManager]: Could not load all parameters!");
    ros::shutdown();
  }

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(nh_, "ControlManager");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "UavManager";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_controller_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::ControllerDiagnostics>(shopts, "controller_diagnostics_in");
  sh_odometry_               = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odometry_in", &UavManager::callbackOdometry, this);
  sh_estimation_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>(shopts, "odometry_diagnostics_in");
  sh_control_manager_diag_   = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in");
  sh_mass_estimate_          = mrs_lib::SubscribeHandler<std_msgs::Float64>(shopts, "mass_estimate_in");
  sh_throttle_               = mrs_lib::SubscribeHandler<std_msgs::Float64>(shopts, "throttle_in");
  sh_height_                 = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(shopts, "height_in");
  sh_hw_api_status_          = mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus>(shopts, "hw_api_status_in");
  sh_gains_diag_             = mrs_lib::SubscribeHandler<mrs_msgs::GainManagerDiagnostics>(shopts, "gain_manager_diagnostics_in");
  sh_constraints_diag_       = mrs_lib::SubscribeHandler<mrs_msgs::ConstraintManagerDiagnostics>(shopts, "constraint_manager_diagnostics_in");
  sh_hw_api_gnss_            = mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>(shopts, "hw_api_gnss_in", &UavManager::callbackHwApiGNSS, this);
  sh_max_height_             = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(shopts, "max_height_in");
  sh_tracker_cmd_            = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, "tracker_cmd_in");

  // | ----------------------- publishers ----------------------- |

  ph_diag_ = mrs_lib::PublisherHandler<mrs_msgs::UavManagerDiagnostics>(nh_, "diagnostics_out", 1);

  // | --------------------- service servers -------------------- |

  service_server_takeoff_           = nh_.advertiseService("takeoff_in", &UavManager::callbackTakeoff, this);
  service_server_land_              = nh_.advertiseService("land_in", &UavManager::callbackLand, this);
  service_server_land_home_         = nh_.advertiseService("land_home_in", &UavManager::callbackLandHome, this);
  service_server_land_there_        = nh_.advertiseService("land_there_in", &UavManager::callbackLandThere, this);
  service_server_midair_activation_ = nh_.advertiseService("midair_activation_in", &UavManager::callbackMidairActivation, this);

  // | --------------------- service clients -------------------- |

  sch_takeoff_               = mrs_lib::ServiceClientHandler<mrs_msgs::Vec1>(nh_, "takeoff_out");
  sch_land_                  = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "land_out");
  sch_eland_                 = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "eland_out");
  sch_ehover_                = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "ehover_out");
  sch_switch_tracker_        = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, "switch_tracker_out");
  sch_switch_controller_     = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, "switch_controller_out");
  sch_emergency_reference_   = mrs_lib::ServiceClientHandler<mrs_msgs::ReferenceStampedSrv>(nh_, "emergency_reference_out");
  sch_control_callbacks_     = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "enable_callbacks_out");
  sch_arm_                   = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "arm_out");
  sch_pirouette_             = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "pirouette_out");
  sch_odometry_callbacks_    = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "set_odometry_callbacks_out");
  sch_ungrip_                = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "ungrip_out");
  sch_toggle_control_output_ = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "toggle_control_output_out");
  sch_offboard_              = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "offboard_out");

  // | ---------------------- state machine --------------------- |

  current_state_landing_ = IDLE_STATE;

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "UavManager", _profiler_enabled_);

  // | ------------------------- timers ------------------------- |

  timer_landing_           = nh_.createTimer(ros::Rate(_landing_timer_rate_), &UavManager::timerLanding, this, false, false);
  timer_takeoff_           = nh_.createTimer(ros::Rate(_takeoff_timer_rate_), &UavManager::timerTakeoff, this, false, false);
  timer_flighttime_        = nh_.createTimer(ros::Rate(_flighttime_timer_rate_), &UavManager::timerFlightTime, this, false, false);
  timer_diagnostics_       = nh_.createTimer(ros::Rate(_diagnostics_timer_rate_), &UavManager::timerDiagnostics, this);
  timer_midair_activation_ = nh_.createTimer(ros::Rate(_midair_activation_timer_rate_), &UavManager::timerMidairActivation, this, false, false);
  timer_max_height_        = nh_.createTimer(ros::Rate(_max_height_checking_rate_), &UavManager::timerMaxHeight, this, false,
                                      _max_height_enabled_ && hw_api_capabilities_.produces_distance_sensor);
  timer_min_height_        = nh_.createTimer(ros::Rate(_min_height_checking_rate_), &UavManager::timerMinHeight, this, false,
                                      _min_height_enabled_ && hw_api_capabilities_.produces_distance_sensor);

  bool should_check_throttle = hw_api_capabilities_.accepts_actuator_cmd || hw_api_capabilities_.accepts_control_group_cmd ||
                               hw_api_capabilities_.accepts_attitude_rate_cmd || hw_api_capabilities_.accepts_attitude_cmd;

  timer_maxthrottle_ =
      nh_.createTimer(ros::Rate(_maxthrottle_timer_rate_), &UavManager::timerMaxthrottle, this, false, _maxthrottle_timer_enabled_ && should_check_throttle);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[UavManager]: initialized");

  ROS_DEBUG("[UavManager]: debug output is enabled");
}

//}

//}

// | ---------------------- state machine --------------------- |

/* //{ changeLandingState() */

void UavManager::changeLandingState(LandingStates_t new_state) {

  previous_state_landing_ = current_state_landing_;
  current_state_landing_  = new_state;

  switch (current_state_landing_) {

    case LANDING_STATE: {

      if (sh_mass_estimate_.hasMsg() && (ros::Time::now() - sh_mass_estimate_.lastMsgTime()).toSec() < 1.0) {

        // copy member variables
        auto mass_esimtate = sh_mass_estimate_.getMsg();

        landing_uav_mass_ = mass_esimtate->data;
      }

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

/* timerHwApiCapabilities() //{ */

void UavManager::timerHwApiCapabilities(const ros::TimerEvent& event) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerHwApiCapabilities", 1.0, 1.0, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::timerHwApiCapabilities", scope_timer_logger_, scope_timer_enabled_);

  if (!sh_hw_api_capabilities_.hasMsg()) {
    ROS_INFO_THROTTLE(1.0, "[ControlManager]: waiting for HW API capabilities");
    return;
  }

  hw_api_capabilities_ = *sh_hw_api_capabilities_.getMsg();

  ROS_INFO("[ControlManager]: got HW API capabilities, initializing");

  initialize();

  timer_hw_api_capabilities_.stop();
}

//}

/* //{ timerLanding() */

void UavManager::timerLanding(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerLanding", _landing_timer_rate_, 0.1, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("UavManager::timerLanding", scope_timer_logger_, scope_timer_enabled_);

  auto land_there_reference = mrs_lib::get_mutexed(mutex_land_there_reference_, land_there_reference_);

  // copy member variables
  auto control_manager_diagnostics = sh_control_manager_diag_.getMsg();
  auto odometry                    = sh_odometry_.getMsg();
  auto tracker_cmd                 = sh_tracker_cmd_.getMsg();

  std::optional<double> desired_throttle;

  if (sh_throttle_.hasMsg() && (ros::Time::now() - sh_throttle_.lastMsgTime()).toSec() < 1.0) {
    desired_throttle = sh_throttle_.getMsg()->data;
  }

  auto res = transformer_->transformSingle(land_there_reference, odometry->header.frame_id);

  mrs_msgs::ReferenceStamped land_there_current_frame;

  if (res) {
    land_there_current_frame = res.value();
  } else {

    ROS_ERROR("[UavManager]: could not transform the reference into the current frame! land by yourself pls.");
    return;
  }

  if (current_state_landing_ == IDLE_STATE) {

    return;

  } else if (current_state_landing_ == GOTO_STATE) {

    auto [pos_x, pos_y, pos_z] = mrs_lib::getPosition(*tracker_cmd);
    auto [ref_x, ref_y, ref_z] = mrs_lib::getPosition(land_there_current_frame);

    double pos_heading = tracker_cmd->heading;

    double ref_heading = 0;
    try {
      ref_heading = mrs_lib::getHeading(land_there_current_frame);
    }
    catch (mrs_lib::AttitudeConverter::GetHeadingException& e) {
      ROS_ERROR_THROTTLE(1.0, "[UavManager]: exception caught: '%s'", e.what());
      return;
    }

    if (mrs_lib::geometry::dist(vec3_t(pos_x, pos_y, pos_z), vec3_t(ref_x, ref_y, ref_z)) < _landing_tracking_tolerance_translation_ &&
        fabs(radians::diff(pos_heading, ref_heading)) < _landing_tracking_tolerance_heading_) {

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

        auto response = transformer_->transformSingle(current_pose, land_there_reference_.header.frame_id);

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

      if (desired_throttle) {

        // recalculate the mass based on the throttle
        throttle_mass_estimate_ = mrs_lib::quadratic_throttle_model::throttleToForce(_throttle_model_, desired_throttle.value()) / _g_;
        ROS_INFO_THROTTLE(1.0, "[UavManager]: landing: initial mass: %.2f throttle_mass_estimate: %.2f", landing_uav_mass_, throttle_mass_estimate_);

        // condition for automatic motor turn off
        if (((throttle_mass_estimate_ < _landing_cutoff_mass_factor_ * landing_uav_mass_) || desired_throttle < 0.01)) {

          if (!throttle_under_threshold_) {

            throttle_mass_estimate_first_time_ = ros::Time::now();
            throttle_under_threshold_          = true;
          }

          ROS_INFO_THROTTLE(0.5, "[UavManager]: throttle is under cutoff factor for %.2f s", (ros::Time::now() - throttle_mass_estimate_first_time_).toSec());

        } else {

          throttle_under_threshold_ = false;
        }

        if (throttle_under_threshold_ && ((ros::Time::now() - throttle_mass_estimate_first_time_).toSec() > _landing_cutoff_mass_timeout_)) {

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

        auto odometry = sh_odometry_.getMsg();

        double z_vel = odometry->twist.twist.linear.z;

        ROS_INFO_THROTTLE(1.0, "[UavManager]: landing: z-velocity: %.2f", z_vel);

        // condition for automatic motor turn off
        if (z_vel > -0.1) {

          if (!velocity_under_threshold_) {

            velocity_under_threshold_first_time_ = ros::Time::now();
            velocity_under_threshold_            = true;
          }

          ROS_INFO_THROTTLE(0.5, "[UavManager]: velocity over threshold for %.2f s", (ros::Time::now() - velocity_under_threshold_first_time_).toSec());

        } else {

          velocity_under_threshold_ = false;
        }

        if (velocity_under_threshold_ && ((ros::Time::now() - velocity_under_threshold_first_time_).toSec() > 3.0)) {

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

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerTakeoff", _takeoff_timer_rate_, 0.1, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("UavManager::timerTakeoff", scope_timer_logger_, scope_timer_enabled_);

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

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerMaxHeight", _max_height_checking_rate_, 0.1, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("UavManager::timerMaxHeight", scope_timer_logger_, scope_timer_enabled_);

  if (!sh_max_height_.hasMsg() || !sh_height_.hasMsg() || !sh_odometry_.hasMsg()) {
    ROS_WARN_THROTTLE(10.0, "[UavManager]: maxHeightTimer() not spinning, missing data");
    return;
  }

  auto control_manager_diag = sh_control_manager_diag_.getMsg();

  if (!control_manager_diag->flying_normally) {
    return;
  }

  auto   odometry = sh_odometry_.getMsg();
  double height   = sh_height_.getMsg()->value;

  // transform max z to the height frame
  geometry_msgs::PointStamped point;
  point.header  = sh_max_height_.getMsg()->header;
  point.point.z = sh_max_height_.getMsg()->value;

  auto res = transformer_->transformSingle(point, sh_height_.getMsg()->header.frame_id);

  double max_z_in_height;

  if (res) {
    max_z_in_height = res->point.z;
  } else {
    ROS_WARN_THROTTLE(1.0, "[UavManager]: timerMaxHeight() not working, cannot transform max z to the height frame");
    return;
  }

  auto [odometry_x, odometry_y, odometry_z] = mrs_lib::getPosition(odometry);

  double odometry_heading = 0;
  try {
    odometry_heading = mrs_lib::getHeading(odometry);
  }
  catch (mrs_lib::AttitudeConverter::GetHeadingException& e) {
    ROS_ERROR_THROTTLE(1.0, "[UavManager]: exception caught: '%s'", e.what());
    return;
  }

  if (height > max_z_in_height) {

    ROS_WARN_THROTTLE(1.0, "[UavManager]: max height exceeded: %.2f >  %.2f, triggering safety goto", height, max_z_in_height);

    mrs_msgs::ReferenceStamped reference_out;
    reference_out.header.frame_id = odometry->header.frame_id;
    reference_out.header.stamp    = ros::Time::now();

    reference_out.reference.position.x = odometry_x;
    reference_out.reference.position.y = odometry_y;
    reference_out.reference.position.z = odometry_z + ((max_z_in_height - _max_height_offset_) - height);

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

  if (fixing_max_height_ && height < max_z_in_height) {

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

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerMinHeight", _min_height_checking_rate_, 0.1, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("UavManager::timerMinHeight", scope_timer_logger_, scope_timer_enabled_);

  if (!sh_odometry_.hasMsg() || !sh_height_.hasMsg() || !sh_control_manager_diag_.hasMsg()) {
    ROS_WARN_THROTTLE(10.0, "[UavManager]: minHeightTimer() not spinning, missing data");
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

/* //{ timerFlightTime() */

void UavManager::timerFlightTime(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerFlightTime", _flighttime_timer_rate_, 0.1, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("UavManager::timerFlightTime", scope_timer_logger_, scope_timer_enabled_);

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

/* //{ timerMaxthrottle() */

void UavManager::timerMaxthrottle(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  if (!sh_throttle_.hasMsg() || (ros::Time::now() - sh_throttle_.lastMsgTime()).toSec() > 1.0) {
    return;
  }

  ROS_INFO_ONCE("[UavManager]: max throttle timer spinning");

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerMaxthrottle", _maxthrottle_timer_rate_, 0.03, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("UavManager::timerMaxthrottle", scope_timer_logger_, scope_timer_enabled_);

  auto desired_throttle = sh_throttle_.getMsg()->data;

  if (desired_throttle >= _maxthrottle_max_throttle_) {

    if (!maxthrottle_above_threshold_) {

      maxthrottle_first_time_      = ros::Time::now();
      maxthrottle_above_threshold_ = true;
      ROS_WARN_THROTTLE(1.0, "[UavManager]: max throttle exceeded threshold (%.2f/%.2f)", desired_throttle, _maxthrottle_max_throttle_);

    } else {

      ROS_WARN_THROTTLE(0.1, "[UavManager]: throttle over threshold (%.2f/%.2f) for %.2f s", desired_throttle, _maxthrottle_max_throttle_,
                        (ros::Time::now() - maxthrottle_first_time_).toSec());
    }

  } else {

    maxthrottle_above_threshold_ = false;
  }

  if (maxthrottle_above_threshold_ && (ros::Time::now() - maxthrottle_first_time_).toSec() > _maxthrottle_ungrip_timeout_) {

    ROS_WARN_THROTTLE(1.0, "[UavManager]: throttle over threshold (%.2f/%.2f) for more than %.2f s, ungripping payload", desired_throttle,
                      _maxthrottle_max_throttle_, _maxthrottle_ungrip_timeout_);

    ungripSrv();
  }

  if (maxthrottle_above_threshold_ && (ros::Time::now() - maxthrottle_first_time_).toSec() > _maxthrottle_eland_timeout_) {

    timer_maxthrottle_.stop();

    ROS_ERROR_THROTTLE(1.0, "[UavManager]: throttle over threshold (%.2f/%.2f) for more than %.2f s, calling for emergency landing", desired_throttle,
                       _maxthrottle_max_throttle_, _maxthrottle_eland_timeout_);

    elandSrv();
  }
}

//}

/* //{ timerDiagnostics() */

void UavManager::timerDiagnostics(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerDiagnostics", _maxthrottle_timer_rate_, 0.03, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("UavManager::timerDiagnostics", scope_timer_logger_, scope_timer_enabled_);

  bool got_gps_est = false;
  bool got_rtk_est = false;

  if (sh_estimation_diagnostics_.hasMsg()) {  // get current position in lat-lon

    auto                     estimation_diag  = sh_estimation_diagnostics_.getMsg();
    std::vector<std::string> state_estimators = estimation_diag.get()->switchable_state_estimators;

    got_gps_est = std::find(state_estimators.begin(), state_estimators.end(), "gps_garmin") != state_estimators.end() ||
                  std::find(state_estimators.begin(), state_estimators.end(), "gps_baro") != state_estimators.end();
    got_rtk_est = std::find(state_estimators.begin(), state_estimators.end(), "rtk") != state_estimators.end();
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

      auto res = transformer_->transformSingle(uav_pose, "latlon_origin");

      if (res) {
        diag.cur_latitude  = res.value().pose.position.x;
        diag.cur_longitude = res.value().pose.position.y;
      }
    }
  }

  if (takeoff_successful_) {

    if (got_gps_est || got_rtk_est) {

      auto land_there_reference = mrs_lib::get_mutexed(mutex_land_there_reference_, land_there_reference_);

      auto res = transformer_->transformSingle(land_there_reference, "latlon_origin");

      if (res) {
        diag.home_latitude  = res.value().reference.position.x;
        diag.home_longitude = res.value().reference.position.y;
      }
    }
  }

  ph_diag_.publish(diag);
}

//}

/* //{ timerMidairActivation() */

void UavManager::timerMidairActivation([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  ROS_INFO_THROTTLE(0.1, "[UavManager]: waiting for OFFBOARD");

  if (sh_hw_api_status_.getMsg()->offboard) {

    ROS_INFO("[UavManager]: OFFBOARD detected");

    setOdometryCallbacksSrv(true);

    {
      bool controller_switched = switchControllerSrv(_midair_activation_after_controller_);

      if (!controller_switched) {

        ROS_ERROR("[UavManager]: could not activate '%s'", _midair_activation_after_controller_.c_str());

        ehoverSrv();

        timer_midair_activation_.stop();

        return;
      }
    }

    {
      bool tracker_switched = switchTrackerSrv(_midair_activation_after_tracker_);

      if (!tracker_switched) {

        ROS_ERROR("[UavManager]: could not activate '%s'", _midair_activation_after_tracker_.c_str());

        ehoverSrv();

        timer_midair_activation_.stop();

        return;
      }
    }

    timer_midair_activation_.stop();

    return;
  }

  if ((ros::Time::now() - midair_activation_started_).toSec() > 0.5) {

    ROS_ERROR("[UavManager]: waiting for OFFBOARD timeouted, reverting");

    toggleControlOutput(false);

    timer_midair_activation_.stop();

    return;
  }
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | --------------------- topic callbacks -------------------- |

/* //{ callbackHwApiGNSS() */

void UavManager::callbackHwApiGNSS(const sensor_msgs::NavSatFix::ConstPtr msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackHwApiGNSS");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("UavManager::callbackHwApiGNSS", scope_timer_logger_, scope_timer_enabled_);

  transformer_->setLatLon(msg->latitude, msg->longitude);
}

//}

/* //{ callbackOdometry() */

void UavManager::callbackOdometry(const nav_msgs::Odometry::ConstPtr msg) {

  if (!is_initialized_)
    return;

  transformer_->setDefaultFrame(msg->header.frame_id);
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackTakeoff() */

bool UavManager::callbackTakeoff([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  ROS_INFO("[UavManager]: takeoff called by service");

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

    if (!sh_hw_api_status_.hasMsg() || (ros::Time::now() - sh_hw_api_status_.lastMsgTime()).toSec() > 5.0) {
      ss << "can not takeoff, missing HW API status!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    if (!sh_hw_api_status_.getMsg()->armed) {
      ss << "can not takeoff, UAV not armed!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    if (!sh_hw_api_status_.getMsg()->offboard) {
      ss << "can not takeoff, UAV not in offboard mode!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    {
      if (!sh_control_manager_diag_.hasMsg() && (ros::Time::now() - sh_control_manager_diag_.lastMsgTime()).toSec() > 5.0) {
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

    if (!sh_controller_diagnostics_.hasMsg()) {
      ss << "can not takeoff, missing controller diagnostics!";
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

    if (!sh_control_manager_diag_.getMsg()->output_enabled) {

      ss << "can not takeoff, Control Manager's output is disabled!";
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

  // let's sleep before calling take off.. the motors are rumping-up anyway
  // this solves on-time-happening race condition in the landoff tracker
  ros::Duration(0.3).sleep();

  // now the takeoff tracker and controller are active
  // the UAV is basically hovering on the ground
  // (the controller is probably rumping up the throttle now)

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

  ROS_INFO("[UavManager]: land called by service");

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

    if (!sh_controller_diagnostics_.hasMsg()) {
      ss << "can not land, missing controller diagnostics!";
      res.message = ss.str();
      res.success = false;
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      return true;
    }

    if (!sh_tracker_cmd_.hasMsg()) {
      ss << "can not land, missing position cmd!";
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

  ROS_INFO("[UavManager]: land home called by service");

  /* preconditions //{ */

  {
    std::stringstream ss;

    if (number_of_takeoffs_ == 0) {
      ss << "can not land home, did not takeoff before!";
      res.message = ss.str();
      res.success = false;
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      return true;
    }

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

    if (!sh_controller_diagnostics_.hasMsg()) {
      ss << "can not land, missing controller diagnostics command!";
      res.message = ss.str();
      res.success = false;
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      return true;
    }

    if (!sh_tracker_cmd_.hasMsg()) {
      ss << "can not land, missing position cmd!";
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

    auto response = transformer_->transformSingle(current_pose, land_there_reference_.header.frame_id);

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

    throttle_under_threshold_          = false;
    throttle_mass_estimate_first_time_ = ros::Time(0);

    changeLandingState(GOTO_STATE);

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

  ROS_INFO("[UavManager]: land there called by service");

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

    if (!sh_controller_diagnostics_.hasMsg()) {
      ss << "can not land, missing controller diagnostics!";
      res.message = ss.str();
      res.success = false;
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      return true;
    }

    if (!sh_tracker_cmd_.hasMsg()) {
      ss << "can not land, missing position cmd!";
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

    throttle_under_threshold_          = false;
    throttle_mass_estimate_first_time_ = ros::Time(0);

    changeLandingState(GOTO_STATE);

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

/* //{ callbackMidairActivation() */

bool UavManager::callbackMidairActivation([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  ROS_INFO("[UavManager]: midair activation called by service");

  /* preconditions //{ */

  {
    std::stringstream ss;

    if (!sh_odometry_.hasMsg()) {
      ss << "can not activate, missing odometry!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    if (!sh_hw_api_status_.hasMsg() || (ros::Time::now() - sh_hw_api_status_.lastMsgTime()).toSec() > 5.0) {
      ss << "can not activate, missing HW API status!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    if (!sh_hw_api_status_.getMsg()->armed) {
      ss << "can not activate, UAV not armed!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    if (sh_hw_api_status_.getMsg()->offboard) {
      ss << "can not activate, UAV already in offboard mode!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    {
      if (!sh_control_manager_diag_.hasMsg()) {
        ss << "can not activate, missing control manager diagnostics!";
        ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
        res.message = ss.str();
        res.success = false;
        return true;
      }

      if (_null_tracker_name_ != sh_control_manager_diag_.getMsg()->active_tracker) {
        ss << "can not activate, need '" << _null_tracker_name_ << "' to be active!";
        ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
        res.message = ss.str();
        res.success = false;
        return true;
      }
    }

    if (!sh_controller_diagnostics_.hasMsg()) {
      ss << "can not activate, missing controller diagnostics!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    if (_gain_manager_required_ && (ros::Time::now() - sh_gains_diag_.lastMsgTime()).toSec() > 5.0) {
      ss << "can not activate, GainManager is not running!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    if (_constraint_manager_required_ && (ros::Time::now() - sh_constraints_diag_.lastMsgTime()).toSec() > 5.0) {
      ss << "can not activate, ConstraintManager is not running!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }

    if (number_of_takeoffs_ > 0) {
      ss << "can not activate, we flew already!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());
      res.message = ss.str();
      res.success = false;
      return true;
    }
  }

  //}

  auto [success, message] = midairActivationImpl();

  res.message = message;
  res.success = success;

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

      auto controller_diagnostics = sh_controller_diagnostics_.getMsg();

      // remember the last valid mass estimated
      // used during subsequent takeoff
      if (controller_diagnostics->mass_estimator) {
        last_mass_difference_ = controller_diagnostics->mass_difference;
      }

      setOdometryCallbacksSrv(false);

      changeLandingState(LANDING_STATE);

      throttle_under_threshold_          = false;
      throttle_mass_estimate_first_time_ = ros::Time(0);

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

        changeLandingState(GOTO_STATE);

        throttle_under_threshold_          = false;
        throttle_mass_estimate_first_time_ = ros::Time(0);

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

/* midairActivationImpl() //{ */

std::tuple<bool, std::string> UavManager::midairActivationImpl(void) {

  // 1. activate the mid-air activation controller
  // the controller will output hover-like control output
  std::string old_controller;
  {
    old_controller           = sh_control_manager_diag_.getMsg()->active_controller;
    bool controller_switched = switchControllerSrv(_midair_activation_during_controller_);

    if (!controller_switched) {

      std::stringstream ss;
      ss << "could not activate '" << _midair_activation_during_controller_ << "' for midair activation";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());

      return std::tuple(false, ss.str());
    }
  }

  // 2. turn Control Manager's output ON
  {
    bool output_enabled = toggleControlOutput(true);

    if (!output_enabled) {

      switchControllerSrv(old_controller);

      std::stringstream ss;
      ss << "could not enable Control Manager's output";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());

      return std::tuple(false, ss.str());
    }
  }

  // 3. activate the mid-air activation tracker
  // this will cause the Control Manager to output something else than min-throttle
  std::string old_tracker;
  {
    old_tracker = sh_control_manager_diag_.getMsg()->active_tracker;

    bool tracker_switched = switchTrackerSrv(_midair_activation_during_tracker_);

    if (!tracker_switched) {

      switchControllerSrv(old_controller);
      toggleControlOutput(false);

      std::stringstream ss;
      ss << "could not activate '" << _midair_activation_during_tracker_ << "' for midair activation";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());

      return std::tuple(false, ss.str());
    }
  }

  // 4. wait for 50 ms, that should be enough for the Pixhawk to start getting data
  ros::Duration(0.05).sleep();

  // 5. turn on the OFFBOARD MODE
  // since now, the UAV should be under our control
  {
    bool offboard_set = offboardSrv(true);

    if (!offboard_set) {

      switchTrackerSrv(old_tracker);
      switchControllerSrv(old_controller);
      toggleControlOutput(false);

      std::stringstream ss;
      ss << "could not activate offboard mode";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());

      return std::tuple(false, ss.str());
    }
  }

  // remember this time, later check for timeout
  midair_activation_started_ = ros::Time::now();

  // start the timer which should check if the offboard is on, activate proper controller and tracker or timeout
  timer_midair_activation_.start();

  std::stringstream ss;
  ss << "midair activation initiated, starting the timer";
  ROS_INFO_STREAM_THROTTLE(1.0, "[UavManager]: " << ss.str());

  return std::tuple(true, ss.str());
}

//}

// | ----------------- service client wrappers ---------------- |

/* setOdometryCallbacksSrv() //{ */

void UavManager::setOdometryCallbacksSrv(const bool& input) {

  ROS_INFO("[UavManager]: switching odometry callbacks to %s", input ? "ON" : "OFF");

  std_srvs::SetBool srv;

  srv.request.data = input;

  bool res = sch_odometry_callbacks_.call(srv);

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

  ROS_INFO("[UavManager]: switching control callbacks to %s", input ? "ON" : "OFF");

  std_srvs::SetBool srv;

  srv.request.data = input;

  bool res = sch_control_callbacks_.call(srv);

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

  bool res = sch_ungrip_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_DEBUG_THROTTLE(1.0, "[UavManager]: service call for ungripping payload returned: %s.", srv.response.message.c_str());
    }

  } else {
    ROS_DEBUG_THROTTLE(1.0, "[UavManager]: service call for ungripping payload failed!");
  }
}

//}

/* toggleControlOutput() //{ */

bool UavManager::toggleControlOutput(const bool& input) {

  ROS_DEBUG_THROTTLE(1.0, "[UavManager]: toggling control output %s", input ? "ON" : "OFF");

  std_srvs::SetBool srv;

  srv.request.data = input;

  bool res = sch_toggle_control_output_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_DEBUG_THROTTLE(1.0, "[UavManager]: service call for control output returned: %s.", srv.response.message.c_str());
      return false;
    } else {
      return true;
    }

  } else {
    ROS_DEBUG_THROTTLE(1.0, "[UavManager]: service call for control output failed!");
    return false;
  }
}

//}

/* offboardSrv() //{ */

bool UavManager::offboardSrv(const bool in) {

  ROS_DEBUG_THROTTLE(1.0, "[UavManager]: setting offboard to %d", in);

  std_srvs::Trigger srv;

  bool res = sch_offboard_.call(srv);

  if (!res) {

    ROS_DEBUG_THROTTLE(1.0, "[UavManager]: service call for offboard failed!");
    return false;

  } else {

    if (!srv.response.success) {
      ROS_DEBUG_THROTTLE(1.0, "[UavManager]: service call for offboard failed, returned: %s", srv.response.message.c_str());
      return false;
    } else {
      return true;
    }
  }
}

//}

/* pirouetteSrv() //{ */

void UavManager::pirouetteSrv(void) {

  std_srvs::Trigger srv;

  bool res = sch_pirouette_.call(srv);

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

  bool res = sch_arm_.call(srv);

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

  bool res = sch_switch_controller_.call(srv);

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

  bool res = sch_switch_tracker_.call(srv);

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

  bool res = sch_land_.call(srv);

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

  bool res = sch_eland_.call(srv);

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

/* ehoverSrv() //{ */

bool UavManager::ehoverSrv(void) {

  ROS_INFO("[UavManager]: calling for ehover");

  std_srvs::Trigger srv;

  bool res = sch_ehover_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN("[UavManager]: service call for ehover returned: '%s'", srv.response.message.c_str());
    }

    return srv.response.success;

  } else {

    ROS_ERROR("[UavManager]: service call for ehover failed!");

    return false;
  }
}

//}

/* takeoffSrv() //{ */

bool UavManager::takeoffSrv(void) {

  ROS_INFO("[UavManager]: calling for takeoff to height '%.2f m'", _takeoff_height_);

  mrs_msgs::Vec1 srv;

  srv.request.goal = _takeoff_height_;

  bool res = sch_takeoff_.call(srv);

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

  ROS_INFO_THROTTLE(1.0, "[UavManager]: calling for emergency reference");

  mrs_msgs::ReferenceStampedSrv srv;

  srv.request.header    = goal.header;
  srv.request.reference = goal.reference;

  bool res = sch_emergency_reference_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[UavManager]: service call for emergency reference returned: '%s'", srv.response.message.c_str());
    }

    return srv.response.success;

  } else {

    ROS_ERROR_THROTTLE(1.0, "[UavManager]: service call for emergency reference failed!");

    return false;
  }
}

//}

}  // namespace uav_manager

}  // namespace mrs_uav_managers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::uav_manager::UavManager, nodelet::Nodelet)
