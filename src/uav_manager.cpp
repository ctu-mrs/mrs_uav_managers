/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <mrs_msgs/srv/vec1.hpp>
#include <mrs_msgs/srv/vec4.hpp>
#include <mrs_msgs/srv/string.hpp>
#include <mrs_msgs/msg/tracker_command.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>
#include <mrs_msgs/msg/float64.hpp>
#include <mrs_msgs/msg/bool_stamped.hpp>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/msg/constraint_manager_diagnostics.hpp>
#include <mrs_msgs/msg/gain_manager_diagnostics.hpp>
#include <mrs_msgs/msg/uav_manager_diagnostics.hpp>
#include <mrs_msgs/msg/estimation_diagnostics.hpp>
#include <mrs_msgs/msg/hw_api_status.hpp>
#include <mrs_msgs/msg/controller_diagnostics.hpp>
#include <mrs_msgs/msg/hw_api_capabilities.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <mrs_lib/node.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/service_server_handler.h>
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

using namespace std::chrono_literals;

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

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

const char *state_names[3] = {

    "IDLING", "GOTO", "LANDING"};

class UavManager : public mrs_lib::Node {

public:
  UavManager(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  rclcpp::TimerBase::SharedPtr timer_preinitialization_;
  void                         timerPreInitialization();

  bool        is_initialized_ = false;
  std::string _uav_name_;

public:
  std::shared_ptr<mrs_lib::Transformer> transformer_;

public:
  bool                                       scope_timer_enabled_ = false;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;

public:
  // | ------------------------- HW API ------------------------- |

  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiCapabilities> sh_hw_api_capabilities_;

  // this timer will check till we already got the hardware api diagnostics
  // then it will trigger the initialization of the controllers and finish
  // the initialization of the ControlManager
  rclcpp::TimerBase::SharedPtr timer_hw_api_capabilities_;
  void                         timerHwApiCapabilities();

  void initialize(void);

  mrs_msgs::msg::HwApiCapabilities hw_api_capabilities_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControllerDiagnostics>        sh_controller_diagnostics_;
  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>                     sh_odometry_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>        sh_estimation_diagnostics_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>    sh_control_manager_diag_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Float64>                      sh_mass_estimate_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Float64>                      sh_throttle_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>               sh_height_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus>                  sh_hw_api_status_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::GainManagerDiagnostics>       sh_gains_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ConstraintManagerDiagnostics> sh_constraints_diag_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix>                 sh_hw_api_gnss_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>               sh_max_height_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>               sh_tracker_cmd_;

  void callbackHwApiGNSS(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // service servers
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>             ss_takeoff_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>             ss_land_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>             ss_land_home_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv> ss_land_there_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>             ss_midair_activation_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>             ss_min_height_check_;

  // service callbacks
  bool callbackTakeoff(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool callbackLand(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool callbackLandHome(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool callbackLandThere(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request>  request,
                         const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response);

  // service clients
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::Vec1>                sch_takeoff_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sch_switch_tracker_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sch_switch_controller_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>             sch_land_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>             sch_eland_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>             sch_ehover_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>             sch_control_callbacks_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv> sch_emergency_reference_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>             sch_arm_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>             sch_odometry_callbacks_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>             sch_ungrip_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>             sch_toggle_control_output_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>             sch_offboard_;

  // service client wrappers
  bool takeoffSrv(void);
  bool switchTrackerSrv(const std::string &tracker);
  bool switchControllerSrv(const std::string &controller);
  bool landSrv(void);
  bool elandSrv(void);
  bool ehoverSrv(void);
  void disarmSrv(void);
  bool emergencyReferenceSrv(const mrs_msgs::msg::ReferenceStamped &goal);
  void setOdometryCallbacksSrv(const bool &input);
  void setControlCallbacksSrv(const bool &input);
  void ungripSrv(void);
  bool toggleControlOutput(const bool &input);
  bool offboardSrv(const bool in);

  std::shared_ptr<TimerType> timer_takeoff_;
  std::shared_ptr<TimerType> timer_max_height_;
  std::shared_ptr<TimerType> timer_min_height_;
  std::shared_ptr<TimerType> timer_landing_;
  std::shared_ptr<TimerType> timer_maxthrottle_;
  std::shared_ptr<TimerType> timer_flighttime_;
  std::shared_ptr<TimerType> timer_diagnostics_;
  std::shared_ptr<TimerType> timer_midair_activation_;

  // timer callbacks
  void timerLanding();
  void timerTakeoff();
  void timerMaxHeight();
  void timerMinHeight();
  void timerFlightTime();
  void timerMaxthrottle();
  void timerDiagnostics();

  // publishers
  mrs_lib::PublisherHandler<mrs_msgs::msg::UavManagerDiagnostics> ph_diag_;

  // max height checking
  bool              _max_height_enabled_ = false;
  double            _max_height_checking_rate_;
  double            _max_height_offset_;
  std::atomic<bool> fixing_max_height_ = false;

  // min height checking
  std::atomic<bool> min_height_check_ = false;
  double            _min_height_checking_rate_;
  double            _min_height_offset_;
  double            _min_height_;
  std::atomic<bool> fixing_min_height_ = false;

  // mass estimation during landing
  double       throttle_mass_estimate_;
  bool         throttle_under_threshold_ = false;
  rclcpp::Time throttle_mass_estimate_first_time_;

  // velocity during landing
  bool         velocity_under_threshold_ = false;
  rclcpp::Time velocity_under_threshold_first_time_;

  bool _gain_manager_required_       = false;
  bool _constraint_manager_required_ = false;

  std::tuple<bool, std::string> landImpl(void);
  std::tuple<bool, std::string> landWithDescendImpl(void);
  std::tuple<bool, std::string> midairActivationImpl(void);

  // saved takeoff coordinates and allows to land there again
  mrs_msgs::msg::ReferenceStamped land_there_reference_;
  std::mutex                      mutex_land_there_reference_;

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
  bool         _maxthrottle_timer_enabled_ = false;
  double       _maxthrottle_timer_rate_;
  double       _maxthrottle_max_throttle_;
  double       _maxthrottle_eland_timeout_;
  double       _maxthrottle_ungrip_timeout_;
  bool         maxthrottle_above_threshold_ = false;
  rclcpp::Time maxthrottle_first_time_;

  // profiler
  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // midair activation
  void         timerMidairActivation();
  bool         callbackMidairActivation(const std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
                                        const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  rclcpp::Time midair_activation_started_;

  bool callbackMinHeightCheck(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  double      _midair_activation_timer_rate_;
  std::string _midair_activation_during_controller_;
  std::string _midair_activation_during_tracker_;
  std::string _midair_activation_after_controller_;
  std::string _midair_activation_after_tracker_;

  void changeLandingState(LandingStates_t new_state);
};

//}

/* UavManager() //{ */

UavManager::UavManager(rclcpp::NodeOptions options) : mrs_lib::Node("uav_manager", options) {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_hw_api_capabilities_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiCapabilities>(shopts, "~/hw_api_capabilities_in");

  timer_hw_api_capabilities_ = node_->create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&UavManager::timerHwApiCapabilities, this));
}

//}

/* initialize() //{ */

void UavManager::initialize() {

  RCLCPP_INFO(node_->get_logger(), "initializing");

  mrs_lib::ParamLoader param_loader(node_);

  std::string custom_config_path;
  std::string platform_config_path;
  std::string world_config_path;

  throttle_mass_estimate_first_time_   = rclcpp::Time(0, 0, clock_->get_clock_type());
  velocity_under_threshold_first_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  maxthrottle_first_time_              = rclcpp::Time(0, 0, clock_->get_clock_type());
  midair_activation_started_           = rclcpp::Time(0, 0, clock_->get_clock_type());

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

  // preinitialize the uav mass for landing detection, important!
  landing_uav_mass_ = _uav_mass_;

  param_loader.loadParam("g", _g_);

  // motor params are also not prefixed, since they are common to more nodes
  param_loader.loadParam("motor_params/n_motors", _throttle_model_.n_motors);
  param_loader.loadParam("motor_params/a", _throttle_model_.A);
  param_loader.loadParam("motor_params/b", _throttle_model_.B);

  param_loader.loadParam(yaml_prefix + "null_tracker", _null_tracker_name_);

  param_loader.loadParam(yaml_prefix + "takeoff/rate", _takeoff_timer_rate_);
  param_loader.loadParam(yaml_prefix + "takeoff/after_takeoff/tracker", _after_takeoff_tracker_name_);
  param_loader.loadParam(yaml_prefix + "takeoff/after_takeoff/controller", _after_takeoff_controller_name_);
  param_loader.loadParam(yaml_prefix + "takeoff/during_takeoff/controller", _takeoff_controller_name_);
  param_loader.loadParam(yaml_prefix + "takeoff/during_takeoff/tracker", _takeoff_tracker_name_);
  param_loader.loadParam(yaml_prefix + "takeoff/takeoff_height", _takeoff_height_);

  if (_takeoff_height_ < 0.5 || _takeoff_height_ > 10.0) {
    RCLCPP_ERROR(node_->get_logger(), "the takeoff height (%.2f m) has to be between 0.5 and 10 meters", _takeoff_height_);
    rclcpp::shutdown();
    exit(1);
  }

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

  {
    bool tmp;

    param_loader.loadParam(yaml_prefix + "min_height_checking/enabled", tmp);

    min_height_check_ = tmp;
  }

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
  const std::string scope_timer_log_filename = param_loader.loadParam2(yaml_prefix + "scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<mrs_lib::ScopeTimerLogger>(node_, scope_timer_log_filename, scope_timer_enabled_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(node_);
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_controller_diagnostics_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControllerDiagnostics>(shopts, "~/controller_diagnostics_in");
  sh_odometry_               = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/odometry_in", &UavManager::callbackOdometry, this);
  sh_estimation_diagnostics_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>(shopts, "~/odometry_diagnostics_in");
  sh_control_manager_diag_   = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(shopts, "~/control_manager_diagnostics_in");
  sh_mass_estimate_          = mrs_lib::SubscriberHandler<std_msgs::msg::Float64>(shopts, "~/mass_estimate_in");
  sh_throttle_               = mrs_lib::SubscriberHandler<std_msgs::msg::Float64>(shopts, "~/throttle_in");
  sh_height_                 = mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>(shopts, "~/height_in");
  sh_hw_api_status_          = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus>(shopts, "~/hw_api_status_in");
  sh_gains_diag_             = mrs_lib::SubscriberHandler<mrs_msgs::msg::GainManagerDiagnostics>(shopts, "~/gain_manager_diagnostics_in");
  sh_constraints_diag_       = mrs_lib::SubscriberHandler<mrs_msgs::msg::ConstraintManagerDiagnostics>(shopts, "~/constraint_manager_diagnostics_in");
  sh_hw_api_gnss_            = mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix>(shopts, "~/hw_api_gnss_in", &UavManager::callbackHwApiGNSS, this);
  sh_max_height_             = mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>(shopts, "~/max_height_in");
  sh_tracker_cmd_            = mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>(shopts, "~/tracker_cmd_in");

  // | ----------------------- publishers ----------------------- |

  ph_diag_ = mrs_lib::PublisherHandler<mrs_msgs::msg::UavManagerDiagnostics>(node_, "~/diagnostics_out");

  // | --------------------- service servers -------------------- |

  ss_takeoff_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/takeoff_in", std::bind(&UavManager::callbackTakeoff, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS(),
      cbkgrp_ss_);
  ss_land_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/land_in", std::bind(&UavManager::callbackLand, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_land_home_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/land_home_in", std::bind(&UavManager::callbackLandHome, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS(),
      cbkgrp_ss_);
  ss_land_there_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv>(
      node_, "~/land_there_in", std::bind(&UavManager::callbackLandThere, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS(),
      cbkgrp_ss_);
  ss_midair_activation_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/midair_activation_in", std::bind(&UavManager::callbackMidairActivation, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_min_height_check_ = mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>(
      node_, "~/enable_min_height_check_in", std::bind(&UavManager::callbackMinHeightCheck, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  // | --------------------- service clients -------------------- |

  sch_takeoff_               = mrs_lib::ServiceClientHandler<mrs_msgs::srv::Vec1>(node_, "~/takeoff_out", cbkgrp_sc_);
  sch_land_                  = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/land_out", cbkgrp_sc_);
  sch_eland_                 = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/eland_out", cbkgrp_sc_);
  sch_ehover_                = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/ehover_out", cbkgrp_sc_);
  sch_switch_tracker_        = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/switch_tracker_out", cbkgrp_sc_);
  sch_switch_controller_     = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/switch_controller_out", cbkgrp_sc_);
  sch_emergency_reference_   = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "~/emergency_reference_out", cbkgrp_sc_);
  sch_control_callbacks_     = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "~/enable_callbacks_out", cbkgrp_sc_);
  sch_arm_                   = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "~/arm_out", cbkgrp_sc_);
  sch_odometry_callbacks_    = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "~/set_odometry_callbacks_out", cbkgrp_sc_);
  sch_ungrip_                = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/ungrip_out", cbkgrp_sc_);
  sch_toggle_control_output_ = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "~/toggle_control_output_out", cbkgrp_sc_);
  sch_offboard_              = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/offboard_out", cbkgrp_sc_);

  // | ---------------------- state machine --------------------- |

  current_state_landing_ = IDLE_STATE;

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(node_, "UavManager", _profiler_enabled_);

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_timers_;

  mrs_lib::TimerHandlerOptions timer_opts_no_start;

  timer_opts_no_start.node           = node_;
  timer_opts_no_start.autostart      = false;
  timer_opts_no_start.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fcn = std::bind(&UavManager::timerLanding, this);

    timer_landing_ = std::make_shared<TimerType>(timer_opts_no_start, rclcpp::Rate(_landing_timer_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&UavManager::timerTakeoff, this);

    timer_takeoff_ = std::make_shared<TimerType>(timer_opts_no_start, rclcpp::Rate(_takeoff_timer_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&UavManager::timerFlightTime, this);

    timer_flighttime_ = std::make_shared<TimerType>(timer_opts_no_start, rclcpp::Rate(_flighttime_timer_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&UavManager::timerDiagnostics, this);

    timer_diagnostics_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_diagnostics_timer_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&UavManager::timerMidairActivation, this);

    timer_midair_activation_ = std::make_shared<TimerType>(timer_opts_no_start, rclcpp::Rate(_midair_activation_timer_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&UavManager::timerMaxHeight, this);

    mrs_lib::TimerHandlerOptions opts;

    opts.node      = node_;
    opts.autostart = _max_height_enabled_ && hw_api_capabilities_.produces_distance_sensor;

    timer_max_height_ = std::make_shared<TimerType>(opts, rclcpp::Rate(_max_height_checking_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&UavManager::timerMinHeight, this);

    mrs_lib::TimerHandlerOptions opts;

    opts.node      = node_;
    opts.autostart = min_height_check_ && hw_api_capabilities_.produces_distance_sensor;

    timer_min_height_ = std::make_shared<TimerType>(opts, rclcpp::Rate(_min_height_checking_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&UavManager::timerMaxthrottle, this);

    mrs_lib::TimerHandlerOptions opts;

    opts.node      = node_;
    opts.autostart = hw_api_capabilities_.accepts_actuator_cmd || hw_api_capabilities_.accepts_control_group_cmd ||
                     hw_api_capabilities_.accepts_attitude_rate_cmd || hw_api_capabilities_.accepts_attitude_cmd;

    timer_maxthrottle_ = std::make_shared<TimerType>(opts, rclcpp::Rate(_maxthrottle_timer_rate_, clock_), callback_fcn);
  }

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");

  RCLCPP_DEBUG(node_->get_logger(), "debug output is enabled");
}

//}

// | ---------------------- state machine --------------------- |

/* //{ changeLandingState() */

void UavManager::changeLandingState(LandingStates_t new_state) {

  previous_state_landing_ = current_state_landing_;
  current_state_landing_  = new_state;

  switch (current_state_landing_) {

  case LANDING_STATE: {

    // we used to have a check here for freshness of the data.. this was removed,
    // since even old data on the mass estimate might be better than the nominal mass
    // from the parameter
    if (sh_mass_estimate_.hasMsg()) {

      // copy member variables
      auto mass_esimtate = sh_mass_estimate_.getMsg();

      landing_uav_mass_ = mass_esimtate->data;

    } else {

      landing_uav_mass_ = _uav_mass_;
    }

    break;
  };

  default: {
    break;
  }
  }

  // just for ROS_INFO
  RCLCPP_INFO(node_->get_logger(), "Switching landing state %s -> %s", state_names[previous_state_landing_], state_names[current_state_landing_]);
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerHwApiCapabilities() //{ */

void UavManager::timerHwApiCapabilities() {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerHwApiCapabilities");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "UavManager::timerHwApiCapabilities", scope_timer_logger_, scope_timer_enabled_);

  if (!sh_hw_api_capabilities_.hasMsg()) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "waiting for HW API capabilities");
    return;
  }

  hw_api_capabilities_ = *sh_hw_api_capabilities_.getMsg();

  RCLCPP_INFO(node_->get_logger(), "got HW API capabilities, initializing");

  initialize();

  timer_hw_api_capabilities_->cancel();
}

//}

/* //{ timerLanding() */

void UavManager::timerLanding() {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerLanding");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "UavManager::timerLanding", scope_timer_logger_, scope_timer_enabled_);

  auto land_there_reference = mrs_lib::get_mutexed(mutex_land_there_reference_, land_there_reference_);

  // copy member variables
  auto control_manager_diagnostics = sh_control_manager_diag_.getMsg();
  auto odometry                    = sh_odometry_.getMsg();
  auto tracker_cmd                 = sh_tracker_cmd_.getMsg();

  std::optional<double> desired_throttle;

  if (sh_throttle_.hasMsg() && (clock_->now() - sh_throttle_.lastMsgTime()).seconds() < 1.0) {
    desired_throttle = sh_throttle_.getMsg()->data;
  }

  auto res = transformer_->transformSingle(land_there_reference, odometry->header.frame_id);

  mrs_msgs::msg::ReferenceStamped land_there_current_frame;

  if (res) {
    land_there_current_frame = res.value();
  } else {

    RCLCPP_ERROR(node_->get_logger(), "could not transform the reference into the current frame! land by yourself pls.");
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
    catch (mrs_lib::AttitudeConverter::GetHeadingException &e) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "exception caught: '%s'", e.what());
      return;
    }

    if (mrs_lib::geometry::dist(vec3_t(pos_x, pos_y, pos_z), vec3_t(ref_x, ref_y, ref_z)) < _landing_tracking_tolerance_translation_ &&
        fabs(radians::diff(pos_heading, ref_heading)) < _landing_tracking_tolerance_heading_) {

      auto [success, message] = landWithDescendImpl();

      if (!success) {

        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "call for landing failed: '%s'", message.c_str());
      }

    } else if (!control_manager_diagnostics->tracker_status.have_goal && control_manager_diagnostics->flying_normally) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the tracker does not have a goal while flying home, setting the reference again");

      mrs_msgs::msg::ReferenceStamped reference_out;

      {
        std::scoped_lock lock(mutex_land_there_reference_);

        // get the current altitude in land_there_reference_.header.frame_id;
        geometry_msgs::msg::PoseStamped current_pose;
        current_pose.header.stamp     = clock_->now();
        current_pose.header.frame_id  = _uav_name_ + "/fcu";
        current_pose.pose.position.x  = 0;
        current_pose.pose.position.y  = 0;
        current_pose.pose.position.z  = 0;
        current_pose.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

        auto response = transformer_->transformSingle(current_pose, land_there_reference_.header.frame_id);

        if (response) {

          land_there_reference_.reference.position.z = response.value().pose.position.z;
          RCLCPP_DEBUG(node_->get_logger(), "current altitude is %.2f m", land_there_reference_.reference.position.z);

        } else {

          std::stringstream ss;
          ss << "could not transform current height to " << land_there_reference_.header.frame_id;
          RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());
        }

        reference_out.header.frame_id = land_there_reference_.header.frame_id;
        reference_out.header.stamp    = clock_->now();
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
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "landing: initial mass: %.2f throttle_mass_estimate: %.2f", landing_uav_mass_,
                             throttle_mass_estimate_);

        // condition for automatic motor turn off
        if (((throttle_mass_estimate_ < _landing_cutoff_mass_factor_ * landing_uav_mass_) || desired_throttle < 0.01)) {

          if (!throttle_under_threshold_) {

            throttle_mass_estimate_first_time_ = clock_->now();
            throttle_under_threshold_          = true;
          }

          RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 500, "throttle is under cutoff factor for %.2f s",
                               (clock_->now() - throttle_mass_estimate_first_time_).seconds());

        } else {

          throttle_under_threshold_ = false;
        }

        if (throttle_under_threshold_ && ((clock_->now() - throttle_mass_estimate_first_time_).seconds() > _landing_cutoff_mass_timeout_)) {

          switchTrackerSrv(_null_tracker_name_);

          setControlCallbacksSrv(true);

          if (_landing_disarm_) {

            RCLCPP_INFO(node_->get_logger(), "disarming after landing");

            disarmSrv();
          }

          changeLandingState(IDLE_STATE);

          RCLCPP_INFO(node_->get_logger(), "landing finished");

          timer_landing_->stop();
        }

      } else {

        auto odometry = sh_odometry_.getMsg();

        double z_vel = odometry->twist.twist.linear.z;

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "landing: z-velocity: %.2f", z_vel);

        // condition for automatic motor turn off
        if (z_vel > -0.1) {

          if (!velocity_under_threshold_) {

            velocity_under_threshold_first_time_ = clock_->now();
            velocity_under_threshold_            = true;
          }

          RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 500, "velocity over threshold for %.2f s",
                               (clock_->now() - velocity_under_threshold_first_time_).seconds());

        } else {

          velocity_under_threshold_ = false;
        }

        if (velocity_under_threshold_ && ((clock_->now() - velocity_under_threshold_first_time_).seconds() > 3.0)) {

          switchTrackerSrv(_null_tracker_name_);

          setControlCallbacksSrv(true);

          if (_landing_disarm_) {

            RCLCPP_INFO(node_->get_logger(), "disarming after landing");

            disarmSrv();
          }

          changeLandingState(IDLE_STATE);

          RCLCPP_INFO(node_->get_logger(), "landing finished");

          timer_landing_->stop();
        }
      }

    } else {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "incorrect tracker detected during landing!");
    }
  }
}

//}

/* //{ timerTakeoff() */

void UavManager::timerTakeoff() {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerTakeoff");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "UavManager::timerTakeoff", scope_timer_logger_, scope_timer_enabled_);

  auto control_manager_diagnostics = sh_control_manager_diag_.getMsg();

  if (waiting_for_takeoff_) {

    if (control_manager_diagnostics->active_tracker == _takeoff_tracker_name_ && control_manager_diagnostics->tracker_status.have_goal) {

      waiting_for_takeoff_ = false;
    } else {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "waiting for takeoff confirmation from the ControlManager");
      return;
    }
  }

  if (takingoff_) {

    if (control_manager_diagnostics->active_tracker != _takeoff_tracker_name_ || !control_manager_diagnostics->tracker_status.have_goal) {

      auto [odom_x, odom_y, odom_z] = mrs_lib::getPosition(sh_odometry_.getMsg());

      double odom_heading;
      try {
        odom_heading = mrs_lib::getHeading(sh_odometry_.getMsg());
      }
      catch (mrs_lib::AttitudeConverter::GetHeadingException &e) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "exception caught: '%s'", e.what());
        return;
      }
      // this is needed for land_home to work with vins_kickoff estimator
      // if there are any problems with this, it shoud be sufficient to only overwrite the frame_id, without the position and heading here
      {
        std::scoped_lock lock(mutex_land_there_reference_);
        land_there_reference_.header               = sh_odometry_.getMsg()->header;
        land_there_reference_.reference.position.x = odom_x;
        land_there_reference_.reference.position.y = odom_y;
        land_there_reference_.reference.position.z = odom_z;
        land_there_reference_.reference.heading    = odom_heading;
      }

      RCLCPP_INFO(node_->get_logger(), "take off finished, switching to %s", _after_takeoff_tracker_name_.c_str());

      switchTrackerSrv(_after_takeoff_tracker_name_);

      switchControllerSrv(_after_takeoff_controller_name_);

      setOdometryCallbacksSrv(true);

      timer_takeoff_->stop();
    }
  }
}

//}

/* //{ timerMaxHeight() */

void UavManager::timerMaxHeight() {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerMaxHeight");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "UavManager::timerMaxHeight", scope_timer_logger_, scope_timer_enabled_);

  if (!sh_max_height_.hasMsg() || !sh_height_.hasMsg() || !sh_control_manager_diag_.hasMsg() || !sh_odometry_.hasMsg()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 10000, "maxHeightTimer() not spinning, missing data");
    return;
  }

  auto control_manager_diag = sh_control_manager_diag_.getMsg();

  if (!fixing_max_height_ && !control_manager_diag->flying_normally) {
    return;
  }

  auto   odometry = sh_odometry_.getMsg();
  double height   = sh_height_.getMsg()->value;

  // transform max z to the height frame
  geometry_msgs::msg::PointStamped point;
  point.header  = sh_max_height_.getMsg()->header;
  point.point.z = sh_max_height_.getMsg()->value;

  auto res = transformer_->transformSingle(point, sh_height_.getMsg()->header.frame_id);

  double max_z_in_height;

  if (res) {
    max_z_in_height = res->point.z;
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "timerMaxHeight() not working, cannot transform max z to the height frame");
    return;
  }

  auto [odometry_x, odometry_y, odometry_z] = mrs_lib::getPosition(odometry);

  double odometry_heading = 0;
  try {
    odometry_heading = mrs_lib::getHeading(odometry);
  }
  catch (mrs_lib::AttitudeConverter::GetHeadingException &e) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "exception caught: '%s'", e.what());
    return;
  }

  if (height > max_z_in_height) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "max height exceeded: %.2f >  %.2f, triggering safety goto", height, max_z_in_height);

    mrs_msgs::msg::ReferenceStamped reference_out;
    reference_out.header.frame_id = odometry->header.frame_id;
    reference_out.header.stamp    = clock_->now();

    reference_out.reference.position.x = odometry_x;
    reference_out.reference.position.y = odometry_y;
    reference_out.reference.position.z = odometry_z + ((max_z_in_height - _max_height_offset_) - height);

    reference_out.reference.heading = odometry_heading;

    setControlCallbacksSrv(false);

    bool success = emergencyReferenceSrv(reference_out);

    if (success) {

      RCLCPP_INFO(node_->get_logger(), "descending");

      fixing_max_height_ = true;

    } else {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "could not descend");

      setControlCallbacksSrv(true);
    }
  }

  if (fixing_max_height_ && height < max_z_in_height) {

    setControlCallbacksSrv(true);

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "safe height reached");

    fixing_max_height_ = false;
  }
}

//}

/* //{ timerMinHeight() */

void UavManager::timerMinHeight() {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "min height timer spinning");

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerMinHeight");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "UavManager::timerMinHeight", scope_timer_logger_, scope_timer_enabled_);

  if (!sh_odometry_.hasMsg() || !sh_height_.hasMsg() || !sh_control_manager_diag_.hasMsg() || !sh_odometry_.hasMsg()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 10000, "minHeightTimer() not spinning, missing data");
    return;
  }

  auto control_manager_diag = sh_control_manager_diag_.getMsg();

  if (!fixing_min_height_ && !control_manager_diag->flying_normally) {
    return;
  }

  auto   odometry = sh_odometry_.getMsg();
  double height   = sh_height_.getMsg()->value;

  auto [odometry_x, odometry_y, odometry_z] = mrs_lib::getPosition(odometry);

  double odometry_heading = 0;

  try {
    odometry_heading = mrs_lib::getHeading(odometry);
  }
  catch (mrs_lib::AttitudeConverter::GetHeadingException &e) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "exception caught: '%s'", e.what());
    return;
  }

  if (height < _min_height_) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "min height breached: %.2f < %.2f, triggering safety goto", height, _min_height_);

    mrs_msgs::msg::ReferenceStamped reference_out;
    reference_out.header.frame_id = odometry->header.frame_id;
    reference_out.header.stamp    = clock_->now();

    reference_out.reference.position.x = odometry_x;
    reference_out.reference.position.y = odometry_y;
    reference_out.reference.position.z = odometry_z + ((_min_height_ + _min_height_offset_) - height);

    reference_out.reference.heading = odometry_heading;

    setControlCallbacksSrv(false);

    bool success = emergencyReferenceSrv(reference_out);

    if (success) {

      RCLCPP_INFO(node_->get_logger(), "ascending");

      fixing_min_height_ = true;

    } else {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "could not ascend");

      setControlCallbacksSrv(true);
    }
  }

  if (fixing_min_height_ && height > _min_height_) {

    setControlCallbacksSrv(true);

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "safe height reached");

    fixing_min_height_ = false;
  }
}

//}

/* //{ timerFlightTime() */

void UavManager::timerFlightTime() {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerFlightTime");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "UavManager::timerFlightTime", scope_timer_logger_, scope_timer_enabled_);

  auto flighttime = mrs_lib::get_mutexed(mutex_flighttime_, flighttime_);

  flighttime += 1.0 / _flighttime_timer_rate_;

  mrs_msgs::msg::Float64 flight_time;
  flight_time.value = flighttime;

  // if enabled, start the timer for measuring the flight time
  if (_flighttime_timer_enabled_) {

    if (flighttime > _flighttime_max_time_) {

      flighttime = 0;
      timer_flighttime_->stop();

      RCLCPP_INFO(node_->get_logger(), "max flight time reached, landing");

      landImpl();
    }
  }

  mrs_lib::set_mutexed(mutex_flighttime_, flighttime, flighttime_);
}

//}

/* //{ timerMaxthrottle() */

void UavManager::timerMaxthrottle() {

  if (!is_initialized_) {
    return;
  }

  if (!sh_throttle_.hasMsg() || (clock_->now() - sh_throttle_.lastMsgTime()).seconds() > 1.0) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "max throttle timer spinning");

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerMaxthrottle");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "UavManager::timerMaxthrottle", scope_timer_logger_, scope_timer_enabled_);

  auto desired_throttle = sh_throttle_.getMsg()->data;

  if (desired_throttle >= _maxthrottle_max_throttle_) {

    if (!maxthrottle_above_threshold_) {

      maxthrottle_first_time_      = clock_->now();
      maxthrottle_above_threshold_ = true;
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "max throttle exceeded threshold (%.2f/%.2f)", desired_throttle, _maxthrottle_max_throttle_);

    } else {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 100, "throttle over threshold (%.2f/%.2f) for %.2f s", desired_throttle, _maxthrottle_max_throttle_,
                           (clock_->now() - maxthrottle_first_time_).seconds());
    }

  } else {

    maxthrottle_above_threshold_ = false;
  }

  if (maxthrottle_above_threshold_ && (clock_->now() - maxthrottle_first_time_).seconds() > _maxthrottle_ungrip_timeout_) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "throttle over threshold (%.2f/%.2f) for more than %.2f s, ungripping payload", desired_throttle,
                         _maxthrottle_max_throttle_, _maxthrottle_ungrip_timeout_);

    ungripSrv();
  }

  if (maxthrottle_above_threshold_ && (clock_->now() - maxthrottle_first_time_).seconds() > _maxthrottle_eland_timeout_) {

    timer_maxthrottle_->stop();

    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "throttle over threshold (%.2f/%.2f) for more than %.2f s, calling for emergency landing",
                          desired_throttle, _maxthrottle_max_throttle_, _maxthrottle_eland_timeout_);

    elandSrv();
  }
}

//}

/* //{ timerDiagnostics() */

void UavManager::timerDiagnostics() {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerDiagnostics");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "UavManager::timerDiagnostics", scope_timer_logger_, scope_timer_enabled_);

  bool got_gps_est = false;
  bool got_rtk_est = false;

  if (sh_estimation_diagnostics_.hasMsg()) { // get current position in lat-lon

    auto                     estimation_diag  = sh_estimation_diagnostics_.getMsg();
    std::vector<std::string> state_estimators = estimation_diag.get()->switchable_state_estimators;

    got_gps_est = std::find(state_estimators.begin(), state_estimators.end(), "gps_garmin") != state_estimators.end() ||
                  std::find(state_estimators.begin(), state_estimators.end(), "gps_baro") != state_estimators.end();
    got_rtk_est = std::find(state_estimators.begin(), state_estimators.end(), "rtk") != state_estimators.end();
  }

  mrs_msgs::msg::UavManagerDiagnostics diag;

  diag.stamp    = clock_->now();
  diag.uav_name = _uav_name_;

  auto flighttime = mrs_lib::get_mutexed(mutex_flighttime_, flighttime_);

  // fill in the acumulated flight time
  diag.flight_time = flighttime;

  if (sh_odometry_.hasMsg()) { // get current position in lat-lon

    if (got_gps_est || got_rtk_est) {

      nav_msgs::msg::Odometry odom = *sh_odometry_.getMsg();

      geometry_msgs::msg::PoseStamped uav_pose;
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

void UavManager::timerMidairActivation() {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 100, "waiting for OFFBOARD");

  if (sh_hw_api_status_.getMsg()->offboard) {

    RCLCPP_INFO(node_->get_logger(), "OFFBOARD detected");

    setOdometryCallbacksSrv(true);

    {
      bool controller_switched = switchControllerSrv(_midair_activation_after_controller_);

      if (!controller_switched) {

        RCLCPP_ERROR(node_->get_logger(), "could not activate '%s'", _midair_activation_after_controller_.c_str());

        ehoverSrv();

        timer_midair_activation_->stop();

        return;
      }
    }

    {
      bool tracker_switched = switchTrackerSrv(_midair_activation_after_tracker_);

      if (!tracker_switched) {

        RCLCPP_ERROR(node_->get_logger(), "could not activate '%s'", _midair_activation_after_tracker_.c_str());

        ehoverSrv();

        timer_midair_activation_->stop();

        return;
      }
    }

    timer_midair_activation_->stop();

    return;
  }

  if ((clock_->now() - midair_activation_started_).seconds() > 0.5) {

    RCLCPP_ERROR(node_->get_logger(), "waiting for OFFBOARD timeouted, reverting");

    toggleControlOutput(false);

    timer_midair_activation_->stop();

    return;
  }
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | --------------------- topic callbacks -------------------- |

/* //{ callbackHwApiGNSS() */

void UavManager::callbackHwApiGNSS(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackHwApiGNSS");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "UavManager::callbackHwApiGNSS", scope_timer_logger_, scope_timer_enabled_);

  transformer_->setLatLon(msg->latitude, msg->longitude);
}

//}

/* //{ callbackOdometry() */

void UavManager::callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  transformer_->setDefaultFrame(msg->header.frame_id);
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackTakeoff() */

bool UavManager::callbackTakeoff([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "takeoff called by service");

  /* preconditions //{ */

  {
    std::stringstream ss;

    if (!sh_odometry_.hasMsg()) {
      ss << "can not takeoff, missing odometry!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->message = ss.str();
      response->success = false;
      return true;
    }

    if (!sh_hw_api_status_.hasMsg() || (clock_->now() - sh_hw_api_status_.lastMsgTime()).seconds() > 5.0) {
      ss << "can not takeoff, missing HW API status!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->message = ss.str();
      response->success = false;
      return true;
    }

    if (!sh_hw_api_status_.getMsg()->armed) {
      ss << "can not takeoff, UAV not armed!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->message = ss.str();
      response->success = false;
      return true;
    }

    if (!sh_hw_api_status_.getMsg()->offboard) {
      ss << "can not takeoff, UAV not in offboard mode!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->message = ss.str();
      response->success = false;
      return true;
    }

    {
      if (!sh_control_manager_diag_.hasMsg() && (clock_->now() - sh_control_manager_diag_.lastMsgTime()).seconds() > 5.0) {
        ss << "can not takeoff, missing control manager diagnostics!";
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
        response->message = ss.str();
        response->success = false;
        return true;
      }

      if (_null_tracker_name_ != sh_control_manager_diag_.getMsg()->active_tracker) {
        ss << "can not takeoff, need '" << _null_tracker_name_ << "' to be active!";
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
        response->message = ss.str();
        response->success = false;
        return true;
      }
    }

    if (!sh_controller_diagnostics_.hasMsg()) {
      ss << "can not takeoff, missing controller diagnostics!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->message = ss.str();
      response->success = false;
      return true;
    }

    if (_gain_manager_required_ && (!sh_gains_diag_.hasMsg() || (clock_->now() - sh_gains_diag_.lastMsgTime()).seconds() > 5.0)) {
      ss << "can not takeoff, GainManager is not running!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->message = ss.str();
      response->success = false;
      return true;
    }

    if (_constraint_manager_required_ && (!sh_constraints_diag_.hasMsg() || (clock_->now() - sh_constraints_diag_.lastMsgTime()).seconds() > 5.0)) {
      ss << "can not takeoff, ConstraintManager is not running!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->message = ss.str();
      response->success = false;
      return true;
    }

    if (!sh_control_manager_diag_.getMsg()->output_enabled) {

      ss << "can not takeoff, Control Manager's output is disabled!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->message = ss.str();
      response->success = false;
      return true;
    }

    if (number_of_takeoffs_ > 0) {

      auto last_mass_difference = mrs_lib::get_mutexed(mutex_last_mass_difference_, last_mass_difference_);

      if (last_mass_difference > 1.0) {

        ss << std::setprecision(2);
        ss << "can not takeoff, estimated mass difference is too large: " << _null_tracker_name_ << "!";
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
        response->message = ss.str();
        response->success = false;
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
  catch (mrs_lib::AttitudeConverter::GetHeadingException &e) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "exception caught: '%s'", e.what());

    std::stringstream ss;
    ss << "could not calculate current heading";
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    response->message = ss.str();
    response->success = false;
    return true;
  }

  RCLCPP_INFO(node_->get_logger(), "taking off");

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
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->success = false;
      response->message = ss.str();

      toggleControlOutput(false);
      disarmSrv();

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
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->success = false;
      response->message = ss.str();

      toggleControlOutput(false);
      disarmSrv();

      return true;
    }
  }

  // let's sleep before calling take off.. the motors are rumping-up anyway
  // this solves on-time-happening race condition in the landoff tracker
  clock_->sleep_for(0.3s);

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

      timer_flighttime_->start();

      std::stringstream ss;
      ss << "taking off";
      response->success = true;
      response->message = ss.str();
      RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

      takingoff_ = true;
      number_of_takeoffs_++;
      waiting_for_takeoff_ = true;

      // start the takeoff timer
      timer_takeoff_->start();

      takeoff_successful_ = takeoff_successful;

    } else {

      std::stringstream ss;
      ss << "takeoff was not successful";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->success = false;
      response->message = ss.str();

      // if the call for takeoff fails, call for emergency landing
      elandSrv();
    }
  }

  return true;
}

//}

/* //{ callbackLand() */

bool UavManager::callbackLand([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "land called by service");

  /* preconditions //{ */

  {
    std::stringstream ss;

    if (!sh_odometry_.hasMsg()) {
      ss << "can not land, missing odometry!";
      response->message = ss.str();
      response->success = false;
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return true;
    }

    {
      if (!sh_control_manager_diag_.hasMsg()) {
        ss << "can not land, missing control manager diagnostics!";
        response->message = ss.str();
        response->success = false;
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
        return true;
      }

      if (_null_tracker_name_ == sh_control_manager_diag_.getMsg()->active_tracker) {
        ss << "can not land, '" << _null_tracker_name_ << "' is active!";
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
        response->message = ss.str();
        response->success = false;
        return true;
      }
    }

    if (!sh_controller_diagnostics_.hasMsg()) {
      ss << "can not land, missing controller diagnostics!";
      response->message = ss.str();
      response->success = false;
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return true;
    }

    if (!sh_tracker_cmd_.hasMsg()) {
      ss << "can not land, missing position cmd!";
      response->message = ss.str();
      response->success = false;
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return true;
    }
  }

  //}

  auto [success, message] = landWithDescendImpl();

  response->message = message;
  response->success = success;

  return true;
}

//}

/* //{ callbackLandHome() */

bool UavManager::callbackLandHome([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "land home called by service");

  /* preconditions //{ */

  {
    std::stringstream ss;

    if (number_of_takeoffs_ == 0) {
      ss << "can not land home, did not takeoff before!";
      response->message = ss.str();
      response->success = false;
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return true;
    }

    if (!sh_odometry_.hasMsg()) {
      ss << "can not land, missing odometry!";
      response->message = ss.str();
      response->success = false;
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return true;
    }

    {
      if (!sh_control_manager_diag_.hasMsg()) {
        ss << "can not land, missing tracker status!";
        response->message = ss.str();
        response->success = false;
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
        return true;
      }

      if (_null_tracker_name_ == sh_control_manager_diag_.getMsg()->active_tracker) {
        ss << "can not land, '" << _null_tracker_name_ << "' is active!";
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
        response->message = ss.str();
        response->success = false;
        return true;
      }
    }

    if (!sh_controller_diagnostics_.hasMsg()) {
      ss << "can not land, missing controller diagnostics command!";
      response->message = ss.str();
      response->success = false;
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return true;
    }

    if (!sh_tracker_cmd_.hasMsg()) {
      ss << "can not land, missing position cmd!";
      response->message = ss.str();
      response->success = false;
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return true;
    }

    if (fixing_max_height_) {
      ss << "can not land, descedning to safe height!";
      response->message = ss.str();
      response->success = false;
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return true;
    }

    if (current_state_landing_ != IDLE_STATE) {
      ss << "can not land, already landing!";
      response->message = ss.str();
      response->success = false;
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return true;
    }
  }

  //}

  ungripSrv();

  mrs_msgs::msg::ReferenceStamped reference_out;

  {
    std::scoped_lock lock(mutex_land_there_reference_);

    // get the current altitude in land_there_reference_.header.frame_id;
    geometry_msgs::msg::PoseStamped current_pose;

    current_pose.header.stamp     = clock_->now();
    current_pose.header.frame_id  = _uav_name_ + "/fcu";
    current_pose.pose.position.x  = 0;
    current_pose.pose.position.y  = 0;
    current_pose.pose.position.z  = 0;
    current_pose.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

    auto res = transformer_->transformSingle(current_pose, land_there_reference_.header.frame_id);

    if (res) {

      land_there_reference_.reference.position.z = res.value().pose.position.z;
      RCLCPP_DEBUG(node_->get_logger(), "current altitude is %.2f m", land_there_reference_.reference.position.z);

    } else {

      std::stringstream ss;
      ss << "could not transform current height to " << land_there_reference_.header.frame_id;
      RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());

      response->success = false;
      response->message = ss.str();
      return true;
    }

    reference_out.header.frame_id = land_there_reference_.header.frame_id;
    reference_out.header.stamp    = clock_->now();
    reference_out.reference       = land_there_reference_.reference;

    land_there_reference_ = reference_out;
  }

  bool service_success = emergencyReferenceSrv(reference_out);

  if (service_success) {

    std::stringstream ss;
    ss << "flying home for landing";
    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    response->success = true;
    response->message = ss.str();

    // stop the eventual takeoff
    waiting_for_takeoff_ = false;
    takingoff_           = false;
    timer_takeoff_->stop();

    throttle_under_threshold_          = false;
    throttle_mass_estimate_first_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

    changeLandingState(GOTO_STATE);

    timer_landing_->start();

  } else {

    std::stringstream ss;
    ss << "can not fly home for landing";
    RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());

    response->success = false;
    response->message = ss.str();
  }

  return true;
}

//}

/* //{ callbackLandThere() */

bool UavManager::callbackLandThere(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request>  request,
                                   const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "land there called by service");

  /* preconditions //{ */

  {
    std::stringstream ss;

    if (!sh_odometry_.hasMsg()) {
      ss << "can not land, missing odometry!";
      response->message = ss.str();
      response->success = false;
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return true;
    }

    {
      if (!sh_control_manager_diag_.hasMsg()) {
        ss << "can not land, missing tracker status!";
        response->message = ss.str();
        response->success = false;
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
        return true;
      }

      if (_null_tracker_name_ == sh_control_manager_diag_.getMsg()->active_tracker) {
        ss << "can not land, '" << _null_tracker_name_ << "' is active!";
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
        response->message = ss.str();
        response->success = false;
        return true;
      }
    }

    if (!sh_controller_diagnostics_.hasMsg()) {
      ss << "can not land, missing controller diagnostics!";
      response->message = ss.str();
      response->success = false;
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return true;
    }

    if (!sh_tracker_cmd_.hasMsg()) {
      ss << "can not land, missing position cmd!";
      response->message = ss.str();
      response->success = false;
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return true;
    }

    if (fixing_max_height_) {
      ss << "can not land, descedning to safe height!";
      response->message = ss.str();
      response->success = false;
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return true;
    }
  }

  //}

  ungripSrv();

  auto odometry = sh_odometry_.getMsg();

  // | ------ transform the reference to the current frame ------ |

  mrs_msgs::msg::ReferenceStamped reference_in;
  reference_in.header    = request->header;
  reference_in.reference = request->reference;

  auto result = transformer_->transformSingle(reference_in, odometry->header.frame_id);

  if (!result) {
    std::stringstream ss;
    ss << "can not transform the reference to the current control frame!";
    response->message = ss.str();
    response->success = false;
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    return true;
  }

  {
    std::scoped_lock lock(mutex_land_there_reference_);

    land_there_reference_.header               = odometry->header;
    land_there_reference_.reference            = reference_in.reference;
    land_there_reference_.reference.position.z = odometry->pose.pose.position.z;
  }

  bool service_success = emergencyReferenceSrv(land_there_reference_);

  if (service_success) {

    std::stringstream ss;
    ss << "flying there for landing";
    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    response->success = true;
    response->message = ss.str();

    // stop the eventual takeoff
    waiting_for_takeoff_ = false;
    takingoff_           = false;
    timer_takeoff_->stop();

    throttle_under_threshold_          = false;
    throttle_mass_estimate_first_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

    changeLandingState(GOTO_STATE);

    timer_landing_->start();

  } else {

    std::stringstream ss;
    ss << "can not fly there for landing";
    RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());

    response->success = false;
    response->message = ss.str();
  }

  return true;
}

//}

/* //{ callbackMidairActivation() */

bool UavManager::callbackMidairActivation([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                          const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "midair activation called by service");

  /* preconditions //{ */

  {
    std::stringstream ss;

    if (!sh_odometry_.hasMsg()) {
      ss << "can not activate, missing odometry!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->message = ss.str();
      response->success = false;
      return true;
    }

    if (!sh_hw_api_status_.hasMsg() || (clock_->now() - sh_hw_api_status_.lastMsgTime()).seconds() > 5.0) {
      ss << "can not activate, missing HW API status!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->message = ss.str();
      response->success = false;
      return true;
    }

    if (!sh_hw_api_status_.getMsg()->armed) {
      ss << "can not activate, UAV not armed!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->message = ss.str();
      response->success = false;
      return true;
    }

    if (sh_hw_api_status_.getMsg()->offboard) {
      ss << "can not activate, UAV already in offboard mode!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->message = ss.str();
      response->success = false;
      return true;
    }

    {
      if (!sh_control_manager_diag_.hasMsg()) {
        ss << "can not activate, missing control manager diagnostics!";
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
        response->message = ss.str();
        response->success = false;
        return true;
      }

      if (_null_tracker_name_ != sh_control_manager_diag_.getMsg()->active_tracker) {
        ss << "can not activate, need '" << _null_tracker_name_ << "' to be active!";
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
        response->message = ss.str();
        response->success = false;
        return true;
      }
    }

    if (!sh_controller_diagnostics_.hasMsg()) {
      ss << "can not activate, missing controller diagnostics!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->message = ss.str();
      response->success = false;
      return true;
    }

    if (_gain_manager_required_ && (clock_->now() - sh_gains_diag_.lastMsgTime()).seconds() > 5.0) {
      ss << "can not activate, GainManager is not running!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->message = ss.str();
      response->success = false;
      return true;
    }

    if (_constraint_manager_required_ && (clock_->now() - sh_constraints_diag_.lastMsgTime()).seconds() > 5.0) {
      ss << "can not activate, ConstraintManager is not running!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->message = ss.str();
      response->success = false;
      return true;
    }

    if (number_of_takeoffs_ > 0) {
      ss << "can not activate, we flew already!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      response->message = ss.str();
      response->success = false;
      return true;
    }
  }

  //}

  auto [success, message] = midairActivationImpl();

  response->message = message;
  response->success = success;

  return true;
}

//}

/* //{ callbackMinHeightCheck() */

bool UavManager::callbackMinHeightCheck(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                                        const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  min_height_check_ = request->data;

  std::stringstream ss;

  ss << "min height check " << (min_height_check_ ? "enabled" : "disabled");

  if (min_height_check_) {
    timer_min_height_->start();
  } else {
    timer_min_height_->stop();
  }

  RCLCPP_INFO_STREAM(node_->get_logger(), "" << ss.str());

  response->message = ss.str();
  response->success = true;

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
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

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
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

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
      timer_takeoff_->stop();

      // stop counting the flight time
      timer_flighttime_->stop();

      auto controller_diagnostics = sh_controller_diagnostics_.getMsg();

      // remember the last valid mass estimated
      // used during subsequent takeoff
      if (controller_diagnostics->mass_estimator) {
        last_mass_difference_ = controller_diagnostics->mass_difference;
      }

      setOdometryCallbacksSrv(false);

      changeLandingState(LANDING_STATE);

      throttle_under_threshold_          = false;
      throttle_mass_estimate_first_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

      timer_landing_->start();

      std::stringstream ss;
      ss << "landing initiated";
      RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

      return std::tuple(true, ss.str());

    } else {

      std::stringstream ss;
      ss << "could not land";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

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
        land_there_reference_.header.stamp         = clock_->now();
        land_there_reference_.reference.position.x = odometry->pose.pose.position.x;
        land_there_reference_.reference.position.y = odometry->pose.pose.position.y;
        land_there_reference_.reference.position.z = odometry->pose.pose.position.z - (height - _landing_descend_height_);
        land_there_reference_.reference.heading    = mrs_lib::AttitudeConverter(odometry->pose.pose.orientation).getHeading();
      }

      bool service_success = emergencyReferenceSrv(land_there_reference_);

      if (service_success) {

        std::stringstream ss;
        ss << "flying down for landing";
        RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

        // stop the eventual takeoff
        waiting_for_takeoff_ = false;
        takingoff_           = false;
        timer_takeoff_->stop();

        changeLandingState(GOTO_STATE);

        throttle_under_threshold_          = false;
        throttle_mass_estimate_first_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

        timer_landing_->start();

        return std::tuple(true, ss.str());

      } else {

        std::stringstream ss;
        ss << "can not fly down for landing";
        RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());
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
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

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
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

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
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

      return std::tuple(false, ss.str());
    }
  }

  // 4. wait for 100 ms, that should be enough for the Pixhawk to start getting data
  clock_->sleep_for(0.2s);

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
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

      return std::tuple(false, ss.str());
    }
  }

  // remember this time, later check for timeout
  midair_activation_started_ = clock_->now();

  // start the timer which should check if the offboard is on, activate proper controller and tracker or timeout
  timer_midair_activation_->start();

  std::stringstream ss;
  ss << "midair activation initiated, starting the timer";
  RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

  return std::tuple(true, ss.str());
}

//}

// | ----------------- service client wrappers ---------------- |

/* setOdometryCallbacksSrv() //{ */

void UavManager::setOdometryCallbacksSrv(const bool &input) {

  RCLCPP_INFO(node_->get_logger(), "switching odometry callbacks to %s", input ? "ON" : "OFF");

  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();

  request->data = input;

  auto response = sch_odometry_callbacks_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_WARN(node_->get_logger(), "service call for toggle odometry callbacks returned: %s.", response.value()->message.c_str());
    }

  } else {
    RCLCPP_ERROR(node_->get_logger(), "service call for toggle odometry callbacks failed!");
  }
}

//}

/* setControlCallbacksSrv() //{ */

void UavManager::setControlCallbacksSrv(const bool &input) {

  RCLCPP_INFO(node_->get_logger(), "switching control callbacks to %s", input ? "ON" : "OFF");

  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();

  request->data = input;

  auto response = sch_control_callbacks_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_WARN(node_->get_logger(), "service call for setting control callbacks returned: %s.", response.value()->message.c_str());
    }

  } else {
    RCLCPP_ERROR(node_->get_logger(), "service call for setting control callbacks failed!");
  }
}

//}

/* ungripSrv() //{ */

void UavManager::ungripSrv(void) {

  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "ungripping payload");

  std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto response = sch_ungrip_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for ungripping payload returned: %s.", response.value()->message.c_str());
    }

  } else {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for ungripping payload failed!");
  }
}

//}

/* toggleControlOutput() //{ */

bool UavManager::toggleControlOutput(const bool &input) {

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "toggling control output %s", input ? "ON" : "OFF");

  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();

  request->data = input;

  auto response = sch_toggle_control_output_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for control output returned: %s.", response.value()->message.c_str());
      return false;
    } else {
      return true;
    }

  } else {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for control output failed!");
    return false;
  }
}

//}

/* offboardSrv() //{ */

bool UavManager::offboardSrv(const bool in) {

  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "setting offboard to %d", in);

  std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto response = sch_offboard_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for offboard failed, returned: %s", response.value()->message.c_str());
      return false;
    } else {
      return true;
    }

  } else {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for offboard failed!");
    return false;
  }
}

//}

/* disarmSrv() //{ */

void UavManager::disarmSrv(void) {

  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();

  request->data = false;

  auto response = sch_arm_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "service call disarming returned: %s.", response.value()->message.c_str());
    }

  } else {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for disarming failed!");
  }
}

//}

/* switchControllerSrv() //{ */

bool UavManager::switchControllerSrv(const std::string &controller) {

  RCLCPP_INFO_STREAM(node_->get_logger(), "activating controller '" << controller << "'");

  std::shared_ptr<mrs_msgs::srv::String::Request> request = std::make_shared<mrs_msgs::srv::String::Request>();

  request->value = controller;

  auto response = sch_switch_controller_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_WARN(node_->get_logger(), "service call for switching controller returned: '%s'", response.value()->message.c_str());
    }

    return response.value()->success;

  } else {

    RCLCPP_ERROR(node_->get_logger(), "service call for switching controller failed!");

    return false;
  }
}

//}

/* switchTrackerSrv() //{ */

bool UavManager::switchTrackerSrv(const std::string &tracker) {

  RCLCPP_INFO_STREAM(node_->get_logger(), "activating tracker '" << tracker << "'");

  std::shared_ptr<mrs_msgs::srv::String::Request> request = std::make_shared<mrs_msgs::srv::String::Request>();

  request->value = tracker;

  auto response = sch_switch_tracker_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_WARN(node_->get_logger(), "service call for switching tracker returned: '%s'", response.value()->message.c_str());
    }

    return response.value()->success;

  } else {

    RCLCPP_ERROR(node_->get_logger(), "service call for switching tracker failed!");

    return false;
  }
}

//}

/* landSrv() //{ */

bool UavManager::landSrv(void) {

  RCLCPP_INFO(node_->get_logger(), "calling for landing");

  std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto response = sch_land_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_WARN(node_->get_logger(), "service call for landing returned: '%s'", response.value()->message.c_str());
    }

    return response.value()->success;

  } else {

    RCLCPP_ERROR(node_->get_logger(), "service call for landing failed!");

    return false;
  }
}

//}

/* elandSrv() //{ */

bool UavManager::elandSrv(void) {

  RCLCPP_INFO(node_->get_logger(), "calling for eland");

  std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto response = sch_eland_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_WARN(node_->get_logger(), "service call for eland returned: '%s'", response.value()->message.c_str());
    }

    return response.value()->success;

  } else {

    RCLCPP_ERROR(node_->get_logger(), "service call for eland failed!");

    return false;
  }
}

//}

/* ehoverSrv() //{ */

bool UavManager::ehoverSrv(void) {

  RCLCPP_INFO(node_->get_logger(), "calling for ehover");

  std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto response = sch_ehover_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_WARN(node_->get_logger(), "service call for ehover returned: '%s'", response.value()->message.c_str());
    }

    return response.value()->success;

  } else {

    RCLCPP_ERROR(node_->get_logger(), "service call for ehover failed!");

    return false;
  }
}

//}

/* takeoffSrv() //{ */

bool UavManager::takeoffSrv(void) {

  RCLCPP_INFO(node_->get_logger(), "calling for takeoff to height '%.2f m'", _takeoff_height_);

  std::shared_ptr<mrs_msgs::srv::Vec1::Request> request = std::make_shared<mrs_msgs::srv::Vec1::Request>();

  request->goal = _takeoff_height_;

  auto response = sch_takeoff_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_WARN(node_->get_logger(), "service call for takeoff returned: '%s'", response.value()->message.c_str());
    }

    return response.value()->success;

  } else {

    RCLCPP_ERROR(node_->get_logger(), "service call for takeoff failed!");

    return false;
  }
}

//}

/* emergencyReferenceSrv() //{ */

bool UavManager::emergencyReferenceSrv(const mrs_msgs::msg::ReferenceStamped &goal) {

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "calling for emergency reference");

  std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();

  request->header    = goal.header;
  request->reference = goal.reference;

  auto response = sch_emergency_reference_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for emergency reference returned: '%s'", response.value()->message.c_str());
    }

    return response.value()->success;

  } else {

    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for emergency reference failed!");

    return false;
  }
}

//}

} // namespace uav_manager

} // namespace mrs_uav_managers

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_managers::uav_manager::UavManager)
