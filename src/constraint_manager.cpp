/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <mrs_msgs/msg/constraint_manager_diagnostics.hpp>
#include <mrs_msgs/msg/estimation_diagnostics.hpp>
#include <mrs_msgs/srv/dynamics_constraints_srv.hpp>
#include <mrs_msgs/srv/string.hpp>
#include <mrs_msgs/srv/constraints_override.hpp>

#include <mrs_lib/node.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/subscriber_handler.h>

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

namespace constraint_manager
{

/* //{ class ConstraintManager */

class ConstraintManager : public mrs_lib::Node {

public:
  ConstraintManager(rclcpp::NodeOptions options);

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

  std::vector<std::string> _estimator_type_names_;

  std::vector<std::string>                                                               _constraint_names_;
  std::map<std::string, std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request>> _constraints_;

  std::map<std::string, std::vector<std::string>> _map_type_allowed_constraints_;
  std::map<std::string, std::string>              _map_type_default_constraints_;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<mrs_msgs::srv::DynamicsConstraintsSrv> sc_set_constraints_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics> sh_estimation_diag_;

  // | ------------- constraint management ------------- |

  bool setConstraints(std::string constraints_names);

  mrs_lib::ServiceServerHandler<mrs_msgs::srv::String> ss_set_constraints_;

  bool callbackSetConstraints(const std::shared_ptr<mrs_msgs::srv::String::Request> request, const std::shared_ptr<mrs_msgs::srv::String::Response> response);

  std::string last_estimator_name_;
  std::mutex  mutex_last_estimator_name_;

  void                       timerConstraintManagement();
  std::shared_ptr<TimerType> timer_constraint_management_;
  double                     _constraint_management_rate_;

  std::string current_constraints_;
  std::mutex  mutex_current_constraints_;

  // | ------------------ constraints override ------------------ |

  mrs_lib::ServiceServerHandler<mrs_msgs::srv::ConstraintsOverride> ss_constraints_override_;

  bool callbackConstraintsOverride(const std::shared_ptr<mrs_msgs::srv::ConstraintsOverride::Request>  request,
                                   const std::shared_ptr<mrs_msgs::srv::ConstraintsOverride::Response> response);

  std::atomic<bool>                                            override_constraints_         = false;
  std::atomic<bool>                                            constraints_override_updated_ = false;
  std::mutex                                                   mutex_constraints_override_;
  std::shared_ptr<mrs_msgs::srv::ConstraintsOverride::Request> constraints_override_;

  // | ------------------ diagnostics publisher ----------------- |

  mrs_lib::PublisherHandler<mrs_msgs::msg::ConstraintManagerDiagnostics> ph_diagnostics_;

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

/* ConstraintManager::ConstraintManager() //{ */

ConstraintManager::ConstraintManager(rclcpp::NodeOptions options) : mrs_lib::Node("control_manager", options) {

  this->initialize();
}

//}

/* //{ initialize() */

void ConstraintManager::initialize() {

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
  param_loader.addYamlFileFromParam("public_constraints");

  const std::string yaml_prefix = "mrs_uav_managers/constraint_manager/";

  // params passed from the launch file are not prefixed
  param_loader.loadParam("enable_profiler", _profiler_enabled_);

  param_loader.loadParam(yaml_prefix + "constraints", _constraint_names_);

  param_loader.loadParam(yaml_prefix + "estimator_types", _estimator_type_names_);

  param_loader.loadParam(yaml_prefix + "rate", _constraint_management_rate_);
  param_loader.loadParam(yaml_prefix + "diagnostics_rate", _diagnostics_rate_);

  std::vector<std::string>::iterator it;

  // loading constraint names
  for (it = _constraint_names_.begin(); it != _constraint_names_.end(); ++it) {

    RCLCPP_INFO_STREAM(node_->get_logger(), "loading constraints '" << *it << "'");

    std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> new_constraints;

    new_constraints = std::make_shared<mrs_msgs::srv::DynamicsConstraintsSrv::Request>();

    param_loader.loadParam(yaml_prefix + *it + "/horizontal/speed", new_constraints->constraints.horizontal_speed);
    param_loader.loadParam(yaml_prefix + *it + "/horizontal/acceleration", new_constraints->constraints.horizontal_acceleration);
    param_loader.loadParam(yaml_prefix + *it + "/horizontal/jerk", new_constraints->constraints.horizontal_jerk);
    param_loader.loadParam(yaml_prefix + *it + "/horizontal/snap", new_constraints->constraints.horizontal_snap);

    param_loader.loadParam(yaml_prefix + *it + "/vertical/ascending/speed", new_constraints->constraints.vertical_ascending_speed);
    param_loader.loadParam(yaml_prefix + *it + "/vertical/ascending/acceleration", new_constraints->constraints.vertical_ascending_acceleration);
    param_loader.loadParam(yaml_prefix + *it + "/vertical/ascending/jerk", new_constraints->constraints.vertical_ascending_jerk);
    param_loader.loadParam(yaml_prefix + *it + "/vertical/ascending/snap", new_constraints->constraints.vertical_ascending_snap);

    param_loader.loadParam(yaml_prefix + *it + "/vertical/descending/speed", new_constraints->constraints.vertical_descending_speed);
    param_loader.loadParam(yaml_prefix + *it + "/vertical/descending/acceleration", new_constraints->constraints.vertical_descending_acceleration);
    param_loader.loadParam(yaml_prefix + *it + "/vertical/descending/jerk", new_constraints->constraints.vertical_descending_jerk);
    param_loader.loadParam(yaml_prefix + *it + "/vertical/descending/snap", new_constraints->constraints.vertical_descending_snap);

    param_loader.loadParam(yaml_prefix + *it + "/heading/speed", new_constraints->constraints.heading_speed);
    param_loader.loadParam(yaml_prefix + *it + "/heading/acceleration", new_constraints->constraints.heading_acceleration);
    param_loader.loadParam(yaml_prefix + *it + "/heading/jerk", new_constraints->constraints.heading_jerk);
    param_loader.loadParam(yaml_prefix + *it + "/heading/snap", new_constraints->constraints.heading_snap);

    param_loader.loadParam(yaml_prefix + *it + "/angular_speed/roll", new_constraints->constraints.roll_rate);
    param_loader.loadParam(yaml_prefix + *it + "/angular_speed/pitch", new_constraints->constraints.pitch_rate);
    param_loader.loadParam(yaml_prefix + *it + "/angular_speed/yaw", new_constraints->constraints.yaw_rate);

    double tilt_deg;

    param_loader.loadParam(yaml_prefix + *it + "/tilt", tilt_deg);

    new_constraints->constraints.tilt = M_PI * (tilt_deg / 180.0);

    _constraints_.insert(std::pair<std::string, std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request>>(*it, new_constraints));
  }

  // loading the allowed constraints lists
  for (it = _estimator_type_names_.begin(); it != _estimator_type_names_.end(); ++it) {

    std::vector<std::string> temp_vector;
    param_loader.loadParam(yaml_prefix + "allowed_constraints/" + *it, temp_vector);

    std::vector<std::string>::iterator it2;
    for (it2 = temp_vector.begin(); it2 != temp_vector.end(); ++it2) {
      if (!stringInVector(*it2, _constraint_names_)) {
        RCLCPP_ERROR(node_->get_logger(), "the element '%s' of %s/allowed_constraints is not a valid constraint!", it2->c_str(), it->c_str());
        rclcpp::shutdown();
        exit(1);
      }
    }

    _map_type_allowed_constraints_.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));
  }

  // loading the default constraints
  for (it = _estimator_type_names_.begin(); it != _estimator_type_names_.end(); ++it) {

    std::string temp_str;
    param_loader.loadParam(yaml_prefix + "default_constraints/" + *it, temp_str);

    if (!stringInVector(temp_str, _map_type_allowed_constraints_.at(*it))) {
      RCLCPP_ERROR(node_->get_logger(), "the element '%s' of %s/allowed_constraints is not a valid constraint!", temp_str.c_str(), it->c_str());
      rclcpp::shutdown();
      exit(1);
    }

    _map_type_default_constraints_.insert(std::pair<std::string, std::string>(*it, temp_str));
  }

  RCLCPP_INFO(node_->get_logger(), "done loading dynamic params");

  current_constraints_ = "";
  last_estimator_name_ = "";

  // | ------------------------ services ------------------------ |

  ss_set_constraints_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::String>(
      node_, "~/set_constraints_in", std::bind(&ConstraintManager::callbackSetConstraints, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_constraints_override_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::ConstraintsOverride>(
      node_, "~/constraints_override_in", std::bind(&ConstraintManager::callbackConstraintsOverride, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  sc_set_constraints_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::DynamicsConstraintsSrv>(node_, "~/set_constraints_out", cbkgrp_sc_);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_estimation_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>(shopts, "~/estimation_diagnostics_in");

  // | ----------------------- publishers ----------------------- |

  ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_msgs::msg::ConstraintManagerDiagnostics>(node_, "~/diagnostics_out");

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fcn = std::bind(&ConstraintManager::timerConstraintManagement, this);

    timer_constraint_management_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_constraint_management_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&ConstraintManager::timerDiagnostics, this);

    timer_diagnostics_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_diagnostics_rate_, clock_), callback_fcn);
  }

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(node_, "ConstraintManager", _profiler_enabled_);

  // | ------------------- scope timer logger ------------------- |

  param_loader.loadParam(yaml_prefix + "scope_timer/enabled", scope_timer_enabled_);
  const std::string scope_timer_log_filename = param_loader.loadParam2(yaml_prefix + "scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<mrs_lib::ScopeTimerLogger>(node_, scope_timer_log_filename, scope_timer_enabled_);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

// --------------------------------------------------------------
// |                           methods                          |
// --------------------------------------------------------------

/* setConstraints() //{ */

bool ConstraintManager::setConstraints(std::string constraints_name) {

  std::map<std::string, std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request>>::iterator it;
  it = _constraints_.find(constraints_name);

  if (it == _constraints_.end()) {
    RCLCPP_ERROR(node_->get_logger(), "could not setConstraints(), the constraint name '%s' is not on the list", constraints_name.c_str());
    return false;
  }

  auto request = std::make_shared<mrs_msgs::srv::DynamicsConstraintsSrv::Request>(*it->second);

  if (override_constraints_) {

    auto constraints_override = mrs_lib::get_mutexed(mutex_constraints_override_, constraints_override_);

    if (constraints_override->acceleration_horizontal > 0 && constraints_override->acceleration_horizontal <= request->constraints.horizontal_acceleration) {
      request->constraints.horizontal_acceleration = constraints_override->acceleration_horizontal;
    } else {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "required horizontal acceleration override is out of bounds");
    }

    if (constraints_override->acceleration_vertical > 0 &&
        constraints_override->acceleration_vertical <= request->constraints.vertical_ascending_acceleration &&
        constraints_override->acceleration_vertical <= request->constraints.vertical_descending_acceleration) {
      request->constraints.vertical_ascending_acceleration  = constraints_override->acceleration_vertical;
      request->constraints.vertical_descending_acceleration = constraints_override->acceleration_vertical;
    } else {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "required vertical acceleration override is out of bounds");
    }
  }

  auto response = sc_set_constraints_.callSync(request);

  if (!response) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the service for setting constraints has failed!");
    return false;

  } else {

    if (response.value()->success) {

      mrs_lib::set_mutexed(mutex_current_constraints_, constraints_name, current_constraints_);
      return true;

    } else {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "set service for setting constraints returned: '%s'", response.value()->message.c_str());
      return false;
    }
  }
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | -------------------- service callbacks ------------------- |

/* //{ callbackSetConstraints() */

bool ConstraintManager::callbackSetConstraints(const std::shared_ptr<mrs_msgs::srv::String::Request>  request,
                                               const std::shared_ptr<mrs_msgs::srv::String::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  std::stringstream ss;

  if (!sh_estimation_diag_.hasMsg()) {

    ss << "missing odometry diagnostics";

    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    response->message = ss.str();
    response->success = false;
    return true;
  }

  auto estimation_diagnostics = sh_estimation_diag_.getMsg();

  if (!stringInVector(request->value, _constraint_names_)) {

    ss << "the constraints '" << request->value.c_str() << "' do not exist (in the ConstraintManager's config)";

    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    response->message = ss.str();
    response->success = false;
    return true;
  }

  if (!stringInVector(request->value, _map_type_allowed_constraints_.at(estimation_diagnostics->current_state_estimator))) {

    ss << "the constraints '" << request->value.c_str() << "' are not allowed given the current odometry type";

    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    response->message = ss.str();
    response->success = false;
    return true;
  }

  override_constraints_ = false;

  // try to set the constraints
  if (!setConstraints(request->value)) {

    ss << "the ControlManager could not set the constraints";

    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    response->message = ss.str();
    response->success = false;
    return true;

  } else {

    ss << "the constraints '" << request->value.c_str() << "' were set";

    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    response->message = ss.str();
    response->success = true;
    return true;
  }
}

//}

/* callackConstraintsOverride() //{ */

bool ConstraintManager::callbackConstraintsOverride(const std::shared_ptr<mrs_msgs::srv::ConstraintsOverride::Request>  request,
                                                    const std::shared_ptr<mrs_msgs::srv::ConstraintsOverride::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  {
    std::scoped_lock lock(mutex_constraints_override_);

    constraints_override_ = request;
  }

  override_constraints_         = true;
  constraints_override_updated_ = true;

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 100, "setting constraints override");

  response->message = "override set";
  response->success = true;

  return true;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerConstraintManagement() //{ */

void ConstraintManager::timerConstraintManagement() {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerConstraintManagement");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ContraintManager::timerConstraintManagement", scope_timer_logger_, scope_timer_enabled_);

  auto current_constraints = mrs_lib::get_mutexed(mutex_current_constraints_, current_constraints_);
  auto last_estimator_name = mrs_lib::get_mutexed(mutex_last_estimator_name_, last_estimator_name_);

  if (!sh_estimation_diag_.hasMsg()) {
    return;
  }

  auto estimation_diagnostics = sh_estimation_diag_.getMsg();

  // | --- automatically set constraints when the state estimator changes -- |
  if (estimation_diagnostics->current_state_estimator != last_estimator_name_) {

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "the state estimator has changed! %s -> %s", last_estimator_name_.c_str(),
                         estimation_diagnostics->current_state_estimator.c_str());

    std::map<std::string, std::string>::iterator it;
    it = _map_type_default_constraints_.find(estimation_diagnostics->current_state_estimator);

    if (it == _map_type_default_constraints_.end()) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the state estimator type '%s' was not specified in the constraint_manager's config!",
                           estimation_diagnostics->current_state_estimator.c_str());

    } else {

      // if the current constraints are within the allowed state estimator types, do nothing
      if (stringInVector(current_constraints, _map_type_allowed_constraints_.at(estimation_diagnostics->current_state_estimator))) {

        last_estimator_name = estimation_diagnostics->current_state_estimator;

        // else, try to set the initial constraints
      } else {

        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the current constraints '%s' are not within the allowed constraints for '%s'",
                             current_constraints.c_str(), estimation_diagnostics->current_state_estimator.c_str());

        if (setConstraints(it->second)) {

          last_estimator_name = estimation_diagnostics->current_state_estimator;

          RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "constraints set to initial: '%s'", it->second.c_str());

        } else {

          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not set constraints!");
        }
      }
    }
  }

  if (constraints_override_updated_) {

    std::map<std::string, std::string>::iterator it;
    it = _map_type_default_constraints_.find(last_estimator_name_);

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 100, "re-setting constraints with user value override");

    if (setConstraints(it->second)) {
      constraints_override_updated_ = false;
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not re-set the constraints!");
    }
  }

  mrs_lib::set_mutexed(mutex_last_estimator_name_, last_estimator_name, last_estimator_name_);
}

//}

/* timerDiagnostics() //{ */

void ConstraintManager::timerDiagnostics() {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerDiagnostics");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ContraintManager::timerDiagnostics", scope_timer_logger_, scope_timer_enabled_);

  if (!sh_estimation_diag_.hasMsg()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 10000, "can not do constraint management, missing estimation diagnostics!");
    return;
  }

  auto estimation_diagnostics = sh_estimation_diag_.getMsg();

  auto current_constraints = mrs_lib::get_mutexed(mutex_current_constraints_, current_constraints_);

  if (current_constraints == "") { // this could happend just before timerConstraintManagement() finishes
    return;
  }

  mrs_msgs::msg::ConstraintManagerDiagnostics diagnostics;

  diagnostics.stamp        = clock_->now();
  diagnostics.current_name = current_constraints;
  diagnostics.loaded       = _constraint_names_;

  // get the available constraints
  {
    std::map<std::string, std::vector<std::string>>::iterator it;
    it = _map_type_allowed_constraints_.find(estimation_diagnostics->current_state_estimator);

    if (it == _map_type_allowed_constraints_.end()) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the state estimator '%s' was not specified in the constraint_manager's config!",
                           estimation_diagnostics->current_state_estimator.c_str());
    } else {
      diagnostics.available = it->second;
    }
  }

  // get the current constraint values
  {
    std::map<std::string, std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request>>::iterator it;
    it = _constraints_.find(current_constraints);

    if (it == _constraints_.end()) {
      RCLCPP_ERROR(node_->get_logger(), "current constraints '%s' not found in the constraint list!", current_constraints.c_str());
      return;
    }

    diagnostics.current_values = it->second->constraints;
  }

  ph_diagnostics_.publish(diagnostics);
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* stringInVector() //{ */

bool ConstraintManager::stringInVector(const std::string &value, const std::vector<std::string> &vector) {

  if (std::find(vector.begin(), vector.end(), value) == vector.end()) {
    return false;
  } else {
    return true;
  }
}

//}

} // namespace constraint_manager

} // namespace mrs_uav_managers

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_managers::constraint_manager::ConstraintManager)
