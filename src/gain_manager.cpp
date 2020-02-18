#define VERSION "0.0.4.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/String.h>

#include <mrs_msgs/String.h>
#include <mrs_msgs/OdometryDiag.h>
#include <mrs_msgs/EstimatorType.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/GainManagerDiagnostics.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/mutex.h>

#include <dynamic_reconfigure/ReconfigureRequest.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

//}

namespace mrs_uav_manager
{

namespace gain_manager
{

/* //{ class GainManager */

typedef struct
{

  double kpxy, kiwxy, kibxy, kvxy, kaxy, kqxy, kwxy;
  double kpz, kvz, kaz, kqz, kwz;
  double kiwxy_lim, kibxy_lim;
  double km, km_lim;

  std::string name;

} Gains_t;

class GainManager : public nodelet::Nodelet {

private:
  ros::NodeHandle nh_;
  std::string     _version_;
  bool            is_initialized_ = false;

private:
  std::vector<std::string> _estimator_type_names_;

  std::vector<std::string>       _gain_names_;
  std::map<std::string, Gains_t> _gains_;

private:
  std::map<std::string, std::vector<std::string>> _map_type_allowed_gains_;
  std::map<std::string, std::string>              _map_type_fallback_gains_;

public:
  virtual void onInit();
  bool         callbackSetGains(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);
  void         callbackOdometryDiagnostics(const mrs_msgs::OdometryDiagConstPtr &msg);
  void         callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr &msg);

  bool setGains(std::string gains_name);

  bool stringInVector(const std::string &value, const std::vector<std::string> &vector);

private:
  ros::ServiceServer service_server_set_gains_;

  ros::ServiceClient service_client_set_gains_;

  ros::Publisher publisher_diagnostics_;

private:
  ros::Subscriber        subscriber_odometry_diagnostics_;
  bool                   got_odometry_diagnostics_ = false;
  mrs_msgs::OdometryDiag odometry_diagnostics_;
  std::mutex             mutex_odometry_diagnostics_;

  // | ------------- gain management ------------- |

private:
  mrs_msgs::EstimatorType::_type_type last_estimator_type_;
  std::mutex                          mutex_last_estimator_type_;

  void       gainsManagementTimer(const ros::TimerEvent &event);
  ros::Timer gains_management_timer_;

  void       diagnosticsTimer(const ros::TimerEvent &event);
  ros::Timer diagnostics_timer_;

  int _rate_;
  int _diagnostics_rate_;

  // | --------------------- gain management -------------------- |

private:
  ros::Subscriber                     subscriber_control_manager_diagnostics_;
  bool                                got_control_manager_diagnostics_ = false;
  mrs_msgs::ControlManagerDiagnostics control_manager_diagnostics_;
  std::mutex                          mutex_control_manager_diagnostics_;

  std::string current_gains_;
  std::mutex  mutex_current_gains_;

  // | ------------------------ profiler_ ------------------------ |
private:
  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;
  ;
};

//}

/* //{ onInit() */

void GainManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[GainManager]: initializing");

  // | ------------------------- params ------------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "GainManager");

  param_loader.load_param("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[GainManager]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.load_param("enable_profiler", _profiler_enabled_);

  param_loader.load_param("gains", _gain_names_);

  param_loader.load_param("estimator_types", _estimator_type_names_);

  param_loader.load_param("rate", _rate_);
  param_loader.load_param("diagnostics_rate", _diagnostics_rate_);

  std::vector<std::string>::iterator it;

  // loading gain_names
  for (it = _gain_names_.begin(); it != _gain_names_.end(); ++it) {
    ROS_INFO_STREAM("[GainManager]: loading gains \"" << *it << "\"");

    Gains_t new_gains;

    param_loader.load_param(*it + "/horizontal/kp", new_gains.kpxy);
    param_loader.load_param(*it + "/horizontal/kv", new_gains.kvxy);
    param_loader.load_param(*it + "/horizontal/ka", new_gains.kaxy);
    param_loader.load_param(*it + "/horizontal/attitude/kq", new_gains.kqxy);
    param_loader.load_param(*it + "/horizontal/attitude/kw", new_gains.kwxy);
    param_loader.load_param(*it + "/horizontal/kib", new_gains.kibxy);
    param_loader.load_param(*it + "/horizontal/kiw", new_gains.kiwxy);
    param_loader.load_param(*it + "/horizontal/kib_lim", new_gains.kibxy_lim);
    param_loader.load_param(*it + "/horizontal/kiw_lim", new_gains.kiwxy_lim);

    param_loader.load_param(*it + "/vertical/kp", new_gains.kpz);
    param_loader.load_param(*it + "/vertical/kv", new_gains.kvz);
    param_loader.load_param(*it + "/vertical/ka", new_gains.kaz);
    param_loader.load_param(*it + "/vertical/attitude/kq", new_gains.kqz);
    param_loader.load_param(*it + "/vertical/attitude/kw", new_gains.kwz);

    param_loader.load_param(*it + "/weight_estimator/km", new_gains.km);
    param_loader.load_param(*it + "/weight_estimator/km_lim", new_gains.km_lim);

    _gains_.insert(std::pair<std::string, Gains_t>(*it, new_gains));
  }

  // loading the allowed gains lists
  for (it = _estimator_type_names_.begin(); it != _estimator_type_names_.end(); ++it) {

    std::vector<std::string> temp_vector;
    param_loader.load_param("gain_management/allowed_gains/" + *it, temp_vector);

    std::vector<std::string>::iterator it2;
    for (it2 = temp_vector.begin(); it2 != temp_vector.end(); ++it2) {
      if (!stringInVector(*it2, _gain_names_)) {
        ROS_ERROR("[GainManager]: the element '%s' of %s_allowed_gains is not a valid gain!", it2->c_str(), it->c_str());
        ros::shutdown();
      }
    }

    _map_type_allowed_gains_.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));
  }

  // loading the fallback gains
  for (it = _estimator_type_names_.begin(); it != _estimator_type_names_.end(); ++it) {

    std::string temp_str;
    param_loader.load_param("gain_management/fallback_gains/" + *it, temp_str);

    if (!stringInVector(temp_str, _map_type_allowed_gains_.at(*it))) {
      ROS_ERROR("[GainManager]: the element '%s' of %s_allowed_gains is not a valid gain!", temp_str.c_str(), it->c_str());
      ros::shutdown();
    }

    _map_type_fallback_gains_.insert(std::pair<std::string, std::string>(*it, temp_str));
  }

  ROS_INFO("[GainManager]: done loading dynamical params");

  current_gains_       = "";
  last_estimator_type_ = -1;

  // | ------------------------ services ------------------------ |

  service_server_set_gains_ = nh_.advertiseService("set_gains_in", &GainManager::callbackSetGains, this);

  service_client_set_gains_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_gains_out");

  // | ----------------------- subscribers ---------------------- |
  subscriber_odometry_diagnostics_ =
      nh_.subscribe("odometry_diagnostics_in", 1, &GainManager::callbackOdometryDiagnostics, this, ros::TransportHints().tcpNoDelay());
  subscriber_control_manager_diagnostics_ =
      nh_.subscribe("control_manager_diagnostics_in", 1, &GainManager::callbackControlManagerDiagnostics, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  publisher_diagnostics_ = nh_.advertise<mrs_msgs::GainManagerDiagnostics>("diagnostics_out", 1);

  // | ------------------------- timers ------------------------- |

  gains_management_timer_ = nh_.createTimer(ros::Rate(_rate_), &GainManager::gainsManagementTimer, this);
  diagnostics_timer_      = nh_.createTimer(ros::Rate(_diagnostics_rate_), &GainManager::diagnosticsTimer, this);

  // --------------------------------------------------------------
  // |                          profiler_                          |
  // --------------------------------------------------------------

  profiler_ = mrs_lib::Profiler(nh_, "GainManager", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[GainManager]: Could not load all parameters!");
    ros::shutdown();
  }

  is_initialized_ = true;

  ROS_INFO("[GainManager]: initialized, version %s", VERSION);
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
    ROS_WARN("[GainManager]: Can't set gains for '%s', the mode is not on a list!", gains_name.c_str());
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

  param.name  = "kwxy";
  param.value = it->second.kwxy;
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

  param.name  = "kwz";
  param.value = it->second.kwz;
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

// | --------------------- topic callbacks -------------------- |

/* //{ callbackOdometryDiagnostics() */

void GainManager::callbackOdometryDiagnostics(const mrs_msgs::OdometryDiagConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackOdometryDiagnostics");

  ROS_INFO_ONCE("[GainManager]: odom diag");

  {
    std::scoped_lock lock(mutex_odometry_diagnostics_);

    odometry_diagnostics_ = *msg;
  }

  got_odometry_diagnostics_ = true;
}

//}

/* callbackControlManagerDiagnostics() //{ */

void GainManager::callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackControlManagerDiagnostics");

  {
    std::scoped_lock lock(mutex_control_manager_diagnostics_);

    control_manager_diagnostics_ = *msg;
  }

  got_control_manager_diagnostics_ = true;
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackSetGains() */

bool GainManager::callbackSetGains(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

  if (!is_initialized_)
    return false;

  auto odometry_diagnostics = mrs_lib::get_mutexed(mutex_odometry_diagnostics_, odometry_diagnostics_);

  char message[200];

  if (!stringInVector(req.value, _gain_names_)) {

    sprintf((char *)&message, "The gains '%s' do not exist (in the gain_manager's config).", req.value.c_str());
    res.message = message;
    res.success = false;
    ROS_ERROR("[GainManager]: %s", message);
    return true;
  }

  if (!stringInVector(req.value, _map_type_allowed_gains_.at(odometry_diagnostics.estimator_type.name))) {

    sprintf((char *)&message, "The gains '%s' are not allowed given the current odometry.type.", req.value.c_str());
    res.message = message;
    res.success = false;
    ROS_ERROR("[GainManager]: %s", message);
    return true;
  }

  // try to set the _gains_
  if (!setGains(req.value)) {

    res.message = "the controller can't set the gains";
    res.success = false;
    return true;

  } else {

    sprintf((char *)&message, "The gains '%s' are set.", req.value.c_str());
    res.message = message;
    res.success = true;
    ROS_INFO("[GainManager]: %s", message);
    return true;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* gainManagementTimer() //{ */

void GainManager::gainsManagementTimer(const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("gainManagementTimer", _rate_, 0.01, event);

  if (!got_odometry_diagnostics_) {
    ROS_WARN_THROTTLE(1.0, "[GainManager]: can't do gain management, missing odometry diagnostics!");
    return;
  }

  auto control_manager_diagnostics = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);
  auto odometry_diagnostics        = mrs_lib::get_mutexed(mutex_odometry_diagnostics_, odometry_diagnostics_);
  auto current_gains               = mrs_lib::get_mutexed(mutex_current_gains_, current_gains_);
  auto last_estimator_type         = mrs_lib::get_mutexed(mutex_last_estimator_type_, last_estimator_type_);

  // | --- automatically set _gains_ when odometry.type changes -- |
  if (odometry_diagnostics.estimator_type.type != last_estimator_type) {

    ROS_INFO_THROTTLE(1.0, "[GainManager]: the odometry.type has changed! %d -> %d", last_estimator_type, odometry_diagnostics.estimator_type.type);

    std::map<std::string, std::string>::iterator it;
    it = _map_type_fallback_gains_.find(odometry_diagnostics.estimator_type.name);

    if (it == _map_type_fallback_gains_.end()) {

      ROS_WARN_THROTTLE(1.0, "[GainManager]: the odometry.type \"%s\" was not specified in the gain_manager's config!",
                        odometry_diagnostics.estimator_type.name.c_str());

    } else {

      // if the current gains are within the allowed odometry types, do nothing
      if (stringInVector(current_gains, _map_type_allowed_gains_.at(odometry_diagnostics.estimator_type.name))) {

        last_estimator_type = odometry_diagnostics.estimator_type.type;

        // else, try to set the fallback gains
      } else {

        ROS_WARN_THROTTLE(1.0, "[GainManager]: the current gains \"%s\" are not within the allowed gains for \"%s\"", current_gains.c_str(),
                          odometry_diagnostics.estimator_type.name.c_str());

        if (setGains(it->second)) {

          last_estimator_type = odometry_diagnostics.estimator_type.type;

          ROS_INFO_THROTTLE(1.0, "[GainManager]: gains set to fallback: \"%s\"", it->second.c_str());

        } else {

          ROS_WARN_THROTTLE(1.0, "[GainManager]: could not set gains!");
        }
      }
    }
  }

  mrs_lib::set_mutexed(mutex_last_estimator_type_, last_estimator_type, last_estimator_type_);
}

//}

/* dignosticsTimer() //{ */

void GainManager::diagnosticsTimer(const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  auto odometry_diagnostics = mrs_lib::get_mutexed(mutex_odometry_diagnostics_, odometry_diagnostics_);
  auto current_gains        = mrs_lib::get_mutexed(mutex_current_gains_, current_gains_);

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("diagnosticsTimer", _diagnostics_rate_, 0.01, event);

  mrs_msgs::GainManagerDiagnostics diagnostics;

  diagnostics.stamp        = ros::Time::now();
  diagnostics.current_name = current_gains;
  diagnostics.loaded       = _gain_names_;

  // get the available gains
  {
    std::map<std::string, std::vector<std::string>>::iterator it;
    it = _map_type_allowed_gains_.find(odometry_diagnostics.estimator_type.name);

    if (it == _map_type_allowed_gains_.end()) {
      ROS_WARN_THROTTLE(1.0, "[GainManager]: the odometry.type \"%s\" was not specified in the gain_manager's config!",
                        odometry_diagnostics.estimator_type.name.c_str());
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

  try {
    publisher_diagnostics_.publish(diagnostics);
  }
  catch (...) {
    ROS_ERROR("[GainManager]: Exception caught during publishing topic %s.", publisher_diagnostics_.getTopic().c_str());
  }
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

}  // namespace gain_manager

}  // namespace mrs_uav_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_manager::gain_manager::GainManager, nodelet::Nodelet)
