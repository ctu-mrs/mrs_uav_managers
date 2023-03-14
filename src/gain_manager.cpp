#define VERSION "1.0.4.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/String.h>

#include <mrs_msgs/String.h>
#include <mrs_msgs/EstimationDiagnostics.h>
#include <mrs_msgs/EstimatorType.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/GainManagerDiagnostics.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/subscribe_handler.h>

#include <dynamic_reconfigure/ReconfigureRequest.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

//}

namespace mrs_uav_managers
{

namespace gain_manager
{

/* //{ class GainManager */

typedef struct
{

  double kpxy, kiwxy, kibxy, kvxy, kaxy, kqxy;
  double kpz, kvz, kaz, kqz;
  double kiwxy_lim, kibxy_lim;
  double km, km_lim;

  std::string name;

} Gains_t;

class GainManager : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  std::string     _version_;
  bool            is_initialized_ = false;

  // | ----------------------- parameters ----------------------- |

  std::vector<std::string> _current_state_estimators_;

  std::vector<std::string>       _gain_names_;
  std::map<std::string, Gains_t> _gains_;

  std::map<std::string, std::vector<std::string>> _map_type_allowed_gains_;
  std::map<std::string, std::string>              _map_type_fallback_gains_;

  // | --------------------- service clients -------------------- |

  ros::ServiceClient service_client_set_gains_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>              sh_estimation_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;

  // | --------------------- gain management -------------------- |

  bool setGains(std::string gains_name);

  ros::ServiceServer service_server_set_gains_;
  bool               callbackSetGains(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);

  std::string last_estimator_name_;
  std::mutex                          mutex_last_estimator_name_;

  void       timerGainManagement(const ros::TimerEvent& event);
  ros::Timer timer_gain_management_;
  int        _gain_management_rate_;

  std::string current_gains_;
  std::mutex  mutex_current_gains_;

  // | ------------------ diagnostics publisher ----------------- |

  mrs_lib::PublisherHandler<mrs_msgs::GainManagerDiagnostics> ph_diagnostics_;

  void       timerDiagnostics(const ros::TimerEvent& event);
  ros::Timer timer_diagnostics_;
  int        _diagnostics_rate_;

  // | ------------------------ profiler ------------------------ |

  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // | ------------------- scope timer logger ------------------- |

  bool                                       scope_timer_enabled_ = false;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;

  // | ------------------------- helpers ------------------------ |

  bool stringInVector(const std::string& value, const std::vector<std::string>& vector);
};

//}

/* //{ onInit() */

void GainManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[GainManager]: initializing");

  // | ------------------------- params ------------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "GainManager");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[GainManager]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("enable_profiler", _profiler_enabled_);

  param_loader.loadParam("gains", _gain_names_);

  param_loader.loadParam("estimator_types", _current_state_estimators_);

  param_loader.loadParam("rate", _gain_management_rate_);
  param_loader.loadParam("diagnostics_rate", _diagnostics_rate_);

  // | ------------------- scope timer logger ------------------- |

  param_loader.loadParam("scope_timer/enabled", scope_timer_enabled_);
  const std::string scope_timer_log_filename = param_loader.loadParam2("scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<mrs_lib::ScopeTimerLogger>(scope_timer_log_filename, scope_timer_enabled_);

  std::vector<std::string>::iterator it;

  // loading gain_names
  for (it = _gain_names_.begin(); it != _gain_names_.end(); ++it) {

    ROS_INFO_STREAM("[GainManager]: loading gains '" << *it << "'");

    Gains_t new_gains;

    param_loader.loadParam(*it + "/horizontal/kp", new_gains.kpxy);
    param_loader.loadParam(*it + "/horizontal/kv", new_gains.kvxy);
    param_loader.loadParam(*it + "/horizontal/ka", new_gains.kaxy);
    param_loader.loadParam(*it + "/horizontal/attitude/kq", new_gains.kqxy);
    param_loader.loadParam(*it + "/horizontal/kib", new_gains.kibxy);
    param_loader.loadParam(*it + "/horizontal/kiw", new_gains.kiwxy);
    param_loader.loadParam(*it + "/horizontal/kib_lim", new_gains.kibxy_lim);
    param_loader.loadParam(*it + "/horizontal/kiw_lim", new_gains.kiwxy_lim);

    param_loader.loadParam(*it + "/vertical/kp", new_gains.kpz);
    param_loader.loadParam(*it + "/vertical/kv", new_gains.kvz);
    param_loader.loadParam(*it + "/vertical/ka", new_gains.kaz);
    param_loader.loadParam(*it + "/vertical/attitude/kq", new_gains.kqz);

    param_loader.loadParam(*it + "/mass_estimator/km", new_gains.km);
    param_loader.loadParam(*it + "/mass_estimator/km_lim", new_gains.km_lim);

    _gains_.insert(std::pair<std::string, Gains_t>(*it, new_gains));
  }

  // loading the allowed gains lists
  for (it = _current_state_estimators_.begin(); it != _current_state_estimators_.end(); ++it) {

    std::vector<std::string> temp_vector;
    param_loader.loadParam("gain_management/allowed_gains/" + *it, temp_vector);

    std::vector<std::string>::iterator it2;
    for (it2 = temp_vector.begin(); it2 != temp_vector.end(); ++it2) {
      if (!stringInVector(*it2, _gain_names_)) {
        ROS_ERROR("[GainManager]: the element '%s' of %s/allowed_gains is not a valid gain!", it2->c_str(), it->c_str());
        ros::shutdown();
      }
    }

    _map_type_allowed_gains_.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));
  }

  // loading the fallback gains
  for (it = _current_state_estimators_.begin(); it != _current_state_estimators_.end(); ++it) {

    std::string temp_str;
    param_loader.loadParam("gain_management/fallback_gains/" + *it, temp_str);

    if (!stringInVector(temp_str, _map_type_allowed_gains_.at(*it))) {
      ROS_ERROR("[GainManager]: the element '%s' of %s/allowed_gains is not a valid gain!", temp_str.c_str(), it->c_str());
      ros::shutdown();
    }

    _map_type_fallback_gains_.insert(std::pair<std::string, std::string>(*it, temp_str));
  }

  ROS_INFO("[GainManager]: done loading dynamical params");

  current_gains_       = "";
  last_estimator_name_ = "";

  // | ------------------------ services ------------------------ |

  service_server_set_gains_ = nh_.advertiseService("set_gains_in", &GainManager::callbackSetGains, this);

  service_client_set_gains_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_gains_out");

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "GainManager";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_estimation_diag_            = mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>(shopts, "estimation_diagnostics_in");
  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in");

  // | ----------------------- publishers ----------------------- |

  ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_msgs::GainManagerDiagnostics>(nh_, "diagnostics_out", 1);

  // | ------------------------- timers ------------------------- |

  timer_gain_management_ = nh_.createTimer(ros::Rate(_gain_management_rate_), &GainManager::timerGainManagement, this);
  timer_diagnostics_     = nh_.createTimer(ros::Rate(_diagnostics_rate_), &GainManager::timerDiagnostics, this);

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "GainManager", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[GainManager]: could not load all parameters!");
    ros::shutdown();
  }

  is_initialized_ = true;

  ROS_INFO("[GainManager]: initialized, version %s", VERSION);

  ROS_DEBUG("[GainManager]: debug output is enabled");
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
    ROS_WARN("[GainManager]: can not set gains for '%s', the mode is not on a list!", gains_name.c_str());
    return false;
  }

  dynamic_reconfigure::Config          conf;
  dynamic_reconfigure::DoubleParameter param;

  param.name  = "kpxy";
  param.value = it->second.kpxy;
  conf.doubles.push_back(param);

  param.name  = "kvxy";
  param.value = it->second.kvxy;
  conf.doubles.push_back(param);

  param.name  = "kaxy";
  param.value = it->second.kaxy;
  conf.doubles.push_back(param);

  param.name  = "kqxy";
  param.value = it->second.kqxy;
  conf.doubles.push_back(param);

  param.name  = "kibxy";
  param.value = it->second.kibxy;
  conf.doubles.push_back(param);

  param.name  = "kiwxy";
  param.value = it->second.kiwxy;
  conf.doubles.push_back(param);

  param.name  = "kibxy_lim";
  param.value = it->second.kibxy_lim;
  conf.doubles.push_back(param);

  param.name  = "kiwxy_lim";
  param.value = it->second.kiwxy_lim;
  conf.doubles.push_back(param);

  param.name  = "kpz";
  param.value = it->second.kpz;
  conf.doubles.push_back(param);

  param.name  = "kvz";
  param.value = it->second.kvz;
  conf.doubles.push_back(param);

  param.name  = "kaz";
  param.value = it->second.kaz;
  conf.doubles.push_back(param);

  param.name  = "kqz";
  param.value = it->second.kqz;
  conf.doubles.push_back(param);

  param.name  = "km";
  param.value = it->second.km;
  conf.doubles.push_back(param);

  param.name  = "km_lim";
  param.value = it->second.km_lim;
  conf.doubles.push_back(param);

  dynamic_reconfigure::ReconfigureRequest  srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;

  srv_req.config = conf;

  dynamic_reconfigure::Reconfigure reconf;
  reconf.request = srv_req;

  ROS_INFO_THROTTLE(1.0, "[GainManager]: setting up gains for '%s'", gains_name.c_str());

  bool res = service_client_set_gains_.call(reconf);

  if (!res) {

    ROS_WARN_THROTTLE(1.0, "[GainManager]: the service for setting gains has failed!");
    return false;

  } else {

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

bool GainManager::callbackSetGains(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  std::stringstream ss;

  if (!sh_estimation_diag_.hasMsg()) {

    ss << "missing estimation diagnostics";

    ROS_ERROR_STREAM_THROTTLE(1.0, "[GainManager]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  auto estimation_diagnostics = *sh_estimation_diag_.getMsg();

  if (!stringInVector(req.value, _gain_names_)) {

    ss << "the gains '" << req.value.c_str() << "' do not exist (in the GainManager's config)";

    ROS_ERROR_STREAM_THROTTLE(1.0, "[GainManager]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!stringInVector(req.value, _map_type_allowed_gains_.at(estimation_diagnostics.current_state_estimator))) {

    ss << "the gains '" << req.value.c_str() << "' are not allowed given the current state estimator";

    ROS_WARN_STREAM_THROTTLE(1.0, "[GainManager]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  // try to set the gains
  if (!setGains(req.value)) {

    ss << "the Se3Controller could not set the gains";

    ROS_ERROR_STREAM_THROTTLE(1.0, "[GainManager]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;

  } else {

    ss << "the gains '" << req.value.c_str() << "' are set";

    ROS_INFO_STREAM_THROTTLE(1.0, "[GainManager]: " << ss.str());

    res.message = ss.str();
    res.success = true;
    return true;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* gainManagementTimer() //{ */

void GainManager::timerGainManagement(const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("gainManagementTimer", _gain_management_rate_, 0.01, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("GainManager::gainManagementTimer", scope_timer_logger_, scope_timer_enabled_);

  if (!sh_estimation_diag_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[GainManager]: can not do constraint management, missing estimation diagnostics!");
    return;
  }

  auto estimation_diagnostics = *sh_estimation_diag_.getMsg();

  if (!sh_control_manager_diag_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[GainManager]: can not do constraint management, missing control manager diagnostics!");
    return;
  }

  auto control_manager_diagnostics = *sh_estimation_diag_.getMsg();

  auto current_gains       = mrs_lib::get_mutexed(mutex_current_gains_, current_gains_);
  auto last_estimator_name = mrs_lib::get_mutexed(mutex_last_estimator_name_, last_estimator_name_);

  // | --- automatically set _gains_ when currrent state estimator changes -- |
  if (estimation_diagnostics.current_state_estimator != last_estimator_name) {

    ROS_INFO_THROTTLE(1.0, "[GainManager]: the state estimator has changed! %s -> %s", last_estimator_name_.c_str(), estimation_diagnostics.current_state_estimator.c_str());

    std::map<std::string, std::string>::iterator it;
    it = _map_type_fallback_gains_.find(estimation_diagnostics.current_state_estimator);

    if (it == _map_type_fallback_gains_.end()) {

      ROS_WARN_THROTTLE(1.0, "[GainManager]: the state estimator '%s' was not specified in the gain_manager's config!",
                        estimation_diagnostics.current_state_estimator.c_str());

    } else {

      // if the current gains are within the allowed estimator types, do nothing
      if (stringInVector(current_gains, _map_type_allowed_gains_.at(estimation_diagnostics.current_state_estimator))) {

        last_estimator_name = estimation_diagnostics.current_state_estimator;

        // else, try to set the fallback gains
      } else {

        ROS_WARN_THROTTLE(1.0, "[GainManager]: the current gains '%s' are not within the allowed gains for '%s'", current_gains.c_str(),
                          estimation_diagnostics.current_state_estimator.c_str());

        if (setGains(it->second)) {

          last_estimator_name = estimation_diagnostics.current_state_estimator;

          ROS_INFO_THROTTLE(1.0, "[GainManager]: gains set to fallback: '%s'", it->second.c_str());

        } else {

          ROS_WARN_THROTTLE(1.0, "[GainManager]: could not set gains!");
        }
      }
    }
  }

  mrs_lib::set_mutexed(mutex_last_estimator_name_, last_estimator_name, last_estimator_name_);
}

//}

/* dignosticsTimer() //{ */

void GainManager::timerDiagnostics(const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerDiagnostics", _diagnostics_rate_, 0.01, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("GainManager::timerDiagnostics", scope_timer_logger_, scope_timer_enabled_);

  if (!sh_estimation_diag_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[GainManager]: can not do constraint management, missing estimator diagnostics!");
    return;
  }

  auto estimation_diagnostics = *sh_estimation_diag_.getMsg();

  auto current_gains = mrs_lib::get_mutexed(mutex_current_gains_, current_gains_);

  mrs_msgs::GainManagerDiagnostics diagnostics;

  diagnostics.stamp        = ros::Time::now();
  diagnostics.current_name = current_gains;
  diagnostics.loaded       = _gain_names_;

  // get the available gains
  {
    std::map<std::string, std::vector<std::string>>::iterator it;
    it = _map_type_allowed_gains_.find(estimation_diagnostics.current_state_estimator);

    if (it == _map_type_allowed_gains_.end()) {
      ROS_WARN_THROTTLE(1.0, "[GainManager]: the estimator name '%s' was not specified in the gain_manager's config!",
                        estimation_diagnostics.current_state_estimator.c_str());
    } else {
      diagnostics.available = it->second;
    }
  }

  // get the current gain values
  {
    std::map<std::string, Gains_t>::iterator it;
    it = _gains_.find(current_gains);

    diagnostics.current_values.kpxy = it->second.kpxy;
    diagnostics.current_values.kvxy = it->second.kvxy;
    diagnostics.current_values.kaxy = it->second.kaxy;

    diagnostics.current_values.kqxy = it->second.kqxy;
    diagnostics.current_values.kqxy = it->second.kqxy;

    diagnostics.current_values.kibxy     = it->second.kibxy;
    diagnostics.current_values.kibxy_lim = it->second.kibxy_lim;

    diagnostics.current_values.kiwxy     = it->second.kiwxy;
    diagnostics.current_values.kiwxy_lim = it->second.kiwxy_lim;

    diagnostics.current_values.kpz = it->second.kpz;
    diagnostics.current_values.kvz = it->second.kvz;
    diagnostics.current_values.kaz = it->second.kaz;

    diagnostics.current_values.kqz = it->second.kqz;
    diagnostics.current_values.kqz = it->second.kqz;

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

bool GainManager::stringInVector(const std::string& value, const std::vector<std::string>& vector) {

  if (std::find(vector.begin(), vector.end(), value) == vector.end()) {
    return false;
  } else {
    return true;
  }
}

//}

}  // namespace gain_manager

}  // namespace mrs_uav_managers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::gain_manager::GainManager, nodelet::Nodelet)
