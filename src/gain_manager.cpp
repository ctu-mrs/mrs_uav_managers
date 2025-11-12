/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <mrs_msgs/msg/estimation_diagnostics.hpp>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/gain_manager_diagnostics.hpp>
#include <mrs_msgs/srv/string.hpp>

#include <mrs_lib/node.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/subscriber_handler.h>

#include <rcl_interfaces/srv/set_parameters.hpp>

//}

/* using //{ */

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

namespace gain_manager
{

/* //{ class GainManager */

typedef struct
{

  double kpxy, kiwxy, kibxy, kvxy, kaxy;
  double kpz, kvz, kaz;
  double kiwxy_lim, kibxy_lim;
  double km, km_lim;
  double kqrp, kqy;

  std::string name;

} Gains_t;

class GainManager : public mrs_lib::Node {

public:
  GainManager(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  void initialize();

  std::atomic<bool> is_initialized_ = false;

  // | ----------------------- parameters ----------------------- |

  std::vector<std::string> _current_state_estimators_;

  std::vector<std::string>       _gain_names_;
  std::map<std::string, Gains_t> _gains_;

  std::map<std::string, std::vector<std::string>> _map_type_allowed_gains_;
  std::map<std::string, std::string>              _map_type_default_gains_;
  ;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<rcl_interfaces::srv::SetParameters> sc_set_gains_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>     sh_estimation_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diag_;

  // | --------------------- gain management -------------------- |

  bool setGains(std::string gains_name);

  mrs_lib::ServiceServerHandler<mrs_msgs::srv::String> ss_set_gains_;

  bool callbackSetGains(const std::shared_ptr<mrs_msgs::srv::String::Request> request, const std::shared_ptr<mrs_msgs::srv::String::Response> response);

  std::string last_estimator_name_;
  std::mutex  mutex_last_estimator_name_;

  void                       timerGainManagement();
  std::shared_ptr<TimerType> timer_gain_management_;
  double                     _gain_management_rate_;

  std::string current_gains_;
  std::mutex  mutex_current_gains_;

  // | ------------------ diagnostics publisher ----------------- |
  //
  mrs_lib::PublisherHandler<mrs_msgs::msg::GainManagerDiagnostics> ph_diagnostics_;

  void                       timerDiagnostics();
  std::shared_ptr<TimerType> timer_diagnostics_;
  double                     _diagnostics_rate_;

  // | ------------------------ profiler ------------------------ |

  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // | ------------------- scope timer logger ------------------- |

  bool                                       scope_timer_enabled_ = false;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;

  // | ------------------------- helpers ------------------------ |

  bool stringInVector(const std::string &value, const std::vector<std::string> &vector);
};

//}

/* GainManager() //{ */

GainManager::GainManager(rclcpp::NodeOptions options) : mrs_lib::Node("control_manager", options) {

  this->initialize();
}

//}

/* //{ initialize() */

void GainManager::initialize() {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // | ------------------------- params ------------------------- |

  mrs_lib::ParamLoader param_loader(node_);

  std::string custom_config_path;
  std::string platform_config_path;

  param_loader.loadParam("custom_config", custom_config_path);
  param_loader.loadParam("platform_config", platform_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  if (platform_config_path != "") {
    param_loader.addYamlFile(platform_config_path);
  }

  param_loader.addYamlFileFromParam("private_config");
  param_loader.addYamlFileFromParam("public_config");
  param_loader.addYamlFileFromParam("public_gains");

  const std::string yaml_prefix = "mrs_uav_managers/gain_manager/";

  // params passed from the launch file are not prefixed
  param_loader.loadParam("enable_profiler", _profiler_enabled_);

  param_loader.loadParam(yaml_prefix + "gains", _gain_names_);

  param_loader.loadParam(yaml_prefix + "estimator_types", _current_state_estimators_);

  param_loader.loadParam(yaml_prefix + "rate", _gain_management_rate_);
  param_loader.loadParam(yaml_prefix + "diagnostics_rate", _diagnostics_rate_);

  // | ------------------- scope timer logger ------------------- |

  param_loader.loadParam(yaml_prefix + "scope_timer/enabled", scope_timer_enabled_);
  const std::string scope_timer_log_filename = param_loader.loadParam2(yaml_prefix + "scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<mrs_lib::ScopeTimerLogger>(node_, scope_timer_log_filename, scope_timer_enabled_);

  std::vector<std::string>::iterator it;

  // loading gain_names
  for (it = _gain_names_.begin(); it != _gain_names_.end(); ++it) {

    RCLCPP_INFO_STREAM(node_->get_logger(), "loading gains '" << *it << "'");

    Gains_t new_gains;

    param_loader.loadParam(yaml_prefix + *it + "/horizontal/kp", new_gains.kpxy);
    param_loader.loadParam(yaml_prefix + *it + "/horizontal/kv", new_gains.kvxy);
    param_loader.loadParam(yaml_prefix + *it + "/horizontal/ka", new_gains.kaxy);
    param_loader.loadParam(yaml_prefix + *it + "/horizontal/kib", new_gains.kibxy);
    param_loader.loadParam(yaml_prefix + *it + "/horizontal/kiw", new_gains.kiwxy);
    param_loader.loadParam(yaml_prefix + *it + "/horizontal/kib_lim", new_gains.kibxy_lim);
    param_loader.loadParam(yaml_prefix + *it + "/horizontal/kiw_lim", new_gains.kiwxy_lim);

    param_loader.loadParam(yaml_prefix + *it + "/vertical/kp", new_gains.kpz);
    param_loader.loadParam(yaml_prefix + *it + "/vertical/kv", new_gains.kvz);
    param_loader.loadParam(yaml_prefix + *it + "/vertical/ka", new_gains.kaz);

    param_loader.loadParam(yaml_prefix + *it + "/attitude/kq_roll_pitch", new_gains.kqrp);
    param_loader.loadParam(yaml_prefix + *it + "/attitude/kq_yaw", new_gains.kqy);

    param_loader.loadParam(yaml_prefix + *it + "/mass_estimator/km", new_gains.km);
    param_loader.loadParam(yaml_prefix + *it + "/mass_estimator/km_lim", new_gains.km_lim);

    _gains_.insert(std::pair<std::string, Gains_t>(*it, new_gains));
  }

  // loading the allowed gains lists
  for (it = _current_state_estimators_.begin(); it != _current_state_estimators_.end(); ++it) {

    std::vector<std::string> temp_vector;
    param_loader.loadParam(yaml_prefix + "allowed_gains/" + *it, temp_vector);

    std::vector<std::string>::iterator it2;
    for (it2 = temp_vector.begin(); it2 != temp_vector.end(); ++it2) {
      if (!stringInVector(*it2, _gain_names_)) {
        RCLCPP_ERROR(node_->get_logger(), "the element '%s' of %s/allowed_gains is not a valid gain!", it2->c_str(), it->c_str());
        rclcpp::shutdown();
        exit(1);
      }
    }

    _map_type_allowed_gains_.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));
  }

  // loading the default gains
  for (it = _current_state_estimators_.begin(); it != _current_state_estimators_.end(); ++it) {

    std::string temp_str;
    param_loader.loadParam(yaml_prefix + "default_gains/" + *it, temp_str);

    if (!stringInVector(temp_str, _map_type_allowed_gains_.at(*it))) {
      RCLCPP_ERROR(node_->get_logger(), "the element '%s' of %s/allowed_gains is not a valid gain!", temp_str.c_str(), it->c_str());
      rclcpp::shutdown();
      exit(1);
    }

    _map_type_default_gains_.insert(std::pair<std::string, std::string>(*it, temp_str));
  }

  RCLCPP_INFO(node_->get_logger(), "done loading dynamical params");

  current_gains_       = "";
  last_estimator_name_ = "";

  // | ------------------------ services ------------------------ |

  ss_set_gains_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::String>(
      node_, "~/set_gains_in", std::bind(&GainManager::callbackSetGains, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS(),
      cbkgrp_ss_);

  sc_set_gains_ = mrs_lib::ServiceClientHandler<rcl_interfaces::srv::SetParameters>(node_, "~/set_gains_out", cbkgrp_sc_);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_estimation_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>(shopts, "~/estimation_diagnostics_in");

  sh_control_manager_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(shopts, "~/control_manager_diagnostics_in");

  // | ----------------------- publishers ----------------------- |

  ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_msgs::msg::GainManagerDiagnostics>(node_, "~/diagnostics_out");

  // | ------------------------- timers ------------------------- |
  //
  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fcn = std::bind(&GainManager::timerGainManagement, this);

    timer_gain_management_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_gain_management_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&GainManager::timerDiagnostics, this);

    timer_diagnostics_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_diagnostics_rate_, clock_), callback_fcn);
  }

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(node_, "GainManager", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");

  RCLCPP_DEBUG(node_->get_logger(), "debug output is enabled");
}

//}

// --------------------------------------------------------------
// |                           methods                          |
// --------------------------------------------------------------

/* setGains() //{ */

bool GainManager::setGains(std::string gains_name) {

  std::map<std::string, Gains_t>::iterator it;
  it = _gains_.find(gains_name);

  if (it == _gains_.end()) {
    RCLCPP_WARN(node_->get_logger(), "can not set gains for '%s', the mode is not on a list!", gains_name.c_str());
    return false;
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

  {
    auto param = rcl_interfaces::msg::Parameter();

    param.name = "mrs_uav_controllers/se3_controller/horizontal.kpxy";

    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = it->second.kpxy;

    request->parameters.push_back(param);
  }

  {
    auto param = rcl_interfaces::msg::Parameter();

    param.name = "mrs_uav_controllers/se3_controller/horizontal.kvxy";

    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = it->second.kvxy;

    request->parameters.push_back(param);
  }

  {
    auto param = rcl_interfaces::msg::Parameter();

    param.name = "mrs_uav_controllers/se3_controller/horizontal.kaxy";

    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = it->second.kaxy;

    request->parameters.push_back(param);
  }

  {
    auto param = rcl_interfaces::msg::Parameter();

    param.name = "mrs_uav_controllers/se3_controller/attitude.kq_roll_pitch";

    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = it->second.kqrp;

    request->parameters.push_back(param);
  }

  {
    auto param = rcl_interfaces::msg::Parameter();

    param.name = "mrs_uav_controllers/se3_controller/horizontal.kibxy";

    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = it->second.kibxy;

    request->parameters.push_back(param);
  }

  {
    auto param = rcl_interfaces::msg::Parameter();

    param.name = "mrs_uav_controllers/se3_controller/horizontal.kiwxy";

    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = it->second.kiwxy;

    request->parameters.push_back(param);
  }

  {
    auto param = rcl_interfaces::msg::Parameter();

    param.name = "mrs_uav_controllers/se3_controller/horizontal.kibxy_lim";

    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = it->second.kibxy_lim;

    request->parameters.push_back(param);
  }

  {
    auto param = rcl_interfaces::msg::Parameter();

    param.name = "mrs_uav_controllers/se3_controller/horizontal.kiwxy_lim";

    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = it->second.kiwxy_lim;

    request->parameters.push_back(param);
  }

  {
    auto param = rcl_interfaces::msg::Parameter();

    param.name = "mrs_uav_controllers/se3_controller/vertical.kpz";

    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = it->second.kpz;

    request->parameters.push_back(param);
  }

  {
    auto param = rcl_interfaces::msg::Parameter();

    param.name = "mrs_uav_controllers/se3_controller/vertical.kvz";

    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = it->second.kvz;

    request->parameters.push_back(param);
  }

  {
    auto param = rcl_interfaces::msg::Parameter();

    param.name = "mrs_uav_controllers/se3_controller/vertical.kaz";

    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = it->second.kaz;

    request->parameters.push_back(param);
  }

  {
    auto param = rcl_interfaces::msg::Parameter();

    param.name = "mrs_uav_controllers/se3_controller/attitude.kq_yaw";

    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = it->second.kqy;

    request->parameters.push_back(param);
  }

  {
    auto param = rcl_interfaces::msg::Parameter();

    param.name = "mrs_uav_controllers/se3_controller/mass.km";

    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = it->second.km;

    request->parameters.push_back(param);
  }

  {
    auto param = rcl_interfaces::msg::Parameter();

    param.name = "mrs_uav_controllers/se3_controller/mass.km_lim";

    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = it->second.km_lim;

    request->parameters.push_back(param);
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "setting up gains for '%s'", gains_name.c_str());

  auto response = sc_set_gains_.callSync(request);

  if (!response) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the service for setting gains has failed!");
    return false;

  } else {

    for (auto res : response.value()->results) {
      if (!res.successful) {
        RCLCPP_ERROR(node_->get_logger(), "could not set param: %s", res.reason.c_str());
      }
    }

    mrs_lib::set_mutexed(mutex_current_gains_, gains_name, current_gains_);
    return true;
  }
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | -------------------- service callbacks ------------------- |

/* //{ callbackSetGains() */

bool GainManager::callbackSetGains(const std::shared_ptr<mrs_msgs::srv::String::Request>  request,
                                   const std::shared_ptr<mrs_msgs::srv::String::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  std::stringstream ss;

  if (!sh_estimation_diag_.hasMsg()) {

    ss << "missing estimation diagnostics";

    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    response->message = ss.str();
    response->success = false;
    return true;
  }

  auto estimation_diagnostics = sh_estimation_diag_.getMsg();

  if (!stringInVector(request->value, _gain_names_)) {

    ss << "the gains '" << request->value.c_str() << "' do not exist (in the GainManager's config)";

    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    response->message = ss.str();
    response->success = false;
    return true;
  }

  if (!stringInVector(request->value, _map_type_allowed_gains_.at(estimation_diagnostics->current_state_estimator))) {

    ss << "the gains '" << request->value.c_str() << "' are not allowed given the current state estimator";

    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    response->message = ss.str();
    response->success = false;
    return true;
  }

  // try to set the gains
  if (!setGains(request->value)) {

    ss << "the Se3Controller could not set the gains";

    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    response->message = ss.str();
    response->success = false;
    return true;

  } else {

    ss << "the gains '" << request->value.c_str() << "' are set";

    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    response->message = ss.str();
    response->success = true;
    return true;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerGainManagement() //{ */

void GainManager::timerGainManagement() {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("gainManagementTimer");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "GainManager::gainManagementTimer", scope_timer_logger_, scope_timer_enabled_);

  if (!sh_estimation_diag_.hasMsg()) {
    return;
  }

  if (!sh_control_manager_diag_.hasMsg()) {
    return;
  }

  auto estimation_diagnostics = sh_estimation_diag_.getMsg();

  auto control_manager_diagnostics = sh_estimation_diag_.getMsg();

  auto current_gains       = mrs_lib::get_mutexed(mutex_current_gains_, current_gains_);
  auto last_estimator_name = mrs_lib::get_mutexed(mutex_last_estimator_name_, last_estimator_name_);

  // | --- automatically set _gains_ when currrent state estimator changes -- |
  if (estimation_diagnostics->current_state_estimator != last_estimator_name) {

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "the state estimator has changed! %s -> %s", last_estimator_name_.c_str(),
                         estimation_diagnostics->current_state_estimator.c_str());

    std::map<std::string, std::string>::iterator it;
    it = _map_type_default_gains_.find(estimation_diagnostics->current_state_estimator);

    if (it == _map_type_default_gains_.end()) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the state estimator '%s' was not specified in the gain_manager's config!",
                           estimation_diagnostics->current_state_estimator.c_str());

    } else {

      // if the current gains are within the allowed estimator types, do nothing
      if (stringInVector(current_gains, _map_type_allowed_gains_.at(estimation_diagnostics->current_state_estimator))) {

        last_estimator_name = estimation_diagnostics->current_state_estimator;

        // else, try to set the default gains
      } else {

        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the current gains '%s' are not within the allowed gains for '%s'", current_gains.c_str(),
                             estimation_diagnostics->current_state_estimator.c_str());

        if (setGains(it->second)) {

          last_estimator_name = estimation_diagnostics->current_state_estimator;

          RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "gains set to default: '%s'", it->second.c_str());

        } else {

          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not set gains!");
        }
      }
    }
  }

  mrs_lib::set_mutexed(mutex_last_estimator_name_, last_estimator_name, last_estimator_name_);
}

//}

/* timerDiagnostics() //{ */

void GainManager::timerDiagnostics() {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerDiagnostics");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "GainManager::timerDiagnostics", scope_timer_logger_, scope_timer_enabled_);

  if (!sh_estimation_diag_.hasMsg()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 10000, "can not do gain management, missing estimator diagnostics!");
    return;
  }

  if (!sh_control_manager_diag_.hasMsg()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 10000, "can not do gain management, missing control manager diagnostics!");
    return;
  }

  auto current_gains = mrs_lib::get_mutexed(mutex_current_gains_, current_gains_);

  if (current_gains == "") { // this could happend just before timerGainManagement() finishes
    return;
  }

  auto estimation_diagnostics = sh_estimation_diag_.getMsg();

  mrs_msgs::msg::GainManagerDiagnostics diagnostics;

  diagnostics.stamp        = clock_->now();
  diagnostics.current_name = current_gains;
  diagnostics.loaded       = _gain_names_;

  // get the available gains
  {
    std::map<std::string, std::vector<std::string>>::iterator it;
    it = _map_type_allowed_gains_.find(estimation_diagnostics->current_state_estimator);

    if (it == _map_type_allowed_gains_.end()) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the estimator name '%s' was not specified in the gain_manager's config!",
                           estimation_diagnostics->current_state_estimator.c_str());
    } else {
      diagnostics.available = it->second;
    }
  }

  // get the current gain values
  {
    std::map<std::string, Gains_t>::iterator it;
    it = _gains_.find(current_gains);

    if (it == _gains_.end()) {
      RCLCPP_ERROR(node_->get_logger(), "current gains '%s' not found in the gain list!", current_gains.c_str());
      return;
    }

    diagnostics.current_values.kpxy = it->second.kpxy;
    diagnostics.current_values.kvxy = it->second.kvxy;
    diagnostics.current_values.kaxy = it->second.kaxy;

    diagnostics.current_values.kqrp = it->second.kqrp;

    diagnostics.current_values.kibxy     = it->second.kibxy;
    diagnostics.current_values.kibxy_lim = it->second.kibxy_lim;

    diagnostics.current_values.kiwxy     = it->second.kiwxy;
    diagnostics.current_values.kiwxy_lim = it->second.kiwxy_lim;

    diagnostics.current_values.kpz = it->second.kpz;
    diagnostics.current_values.kvz = it->second.kvz;
    diagnostics.current_values.kaz = it->second.kaz;

    diagnostics.current_values.kqy = it->second.kqy;

    diagnostics.current_values.km     = it->second.km;
    diagnostics.current_values.km_lim = it->second.km_lim;
  }

  ph_diagnostics_.publish(diagnostics);
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* stringInVector() //{ */

bool GainManager::stringInVector(const std::string &value, const std::vector<std::string> &vector) {

  if (std::find(vector.begin(), vector.end(), value) == vector.end()) {
    return false;
  } else {
    return true;
  }
}

//}

} // namespace gain_manager

} // namespace mrs_uav_managers

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_managers::gain_manager::GainManager)
