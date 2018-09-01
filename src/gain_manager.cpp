#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/String.h>

#include <mrs_msgs/String.h>
#include <mrs_msgs/OdometryDiag.h>
#include <mrs_msgs/OdometryMode.h>
#include <mrs_msgs/ControllerStatus.h>
#include <mrs_msgs/TrackerConstraints.h>
#include <mrs_msgs/TrackerConstraintsRequest.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>

#include <dynamic_reconfigure/ReconfigureRequest.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#define STRING_EQUAL 0

namespace mrs_mav_manager
{

/* //{ class GainManager */

typedef struct
{

  double kpxy, kiwxy, kibxy, kvxy, kaxy;
  double kpz, kvz, kaz;
  double kiwxy_lim, kibxy_lim;
  double km, km_lim;

  std::string name;

} Gains_t;

class GainManager : public nodelet::Nodelet {

private:
  ros::NodeHandle nh_;
  bool            is_initialized = false;

private:
  std::vector<std::string> odometry_mode_names_;

  std::vector<std::string>       gain_names_;
  std::map<std::string, Gains_t> gains;

  std::vector<std::string>                                   constraint_names_;
  std::map<std::string, mrs_msgs::TrackerConstraintsRequest> constraints;

private:
private:
  std::map<std::string, std::vector<std::string>> map_mode_allowed_gains;
  std::map<std::string, std::string>              map_mode_fallback_gains;

private:
  std::map<std::string, std::vector<std::string>> map_mode_allowed_constraints;
  std::map<std::string, std::string>              map_mode_fallback_constraints;

public:
  virtual void onInit();
  bool         callbackSetGains(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);
  bool         callbackSetConstraints(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);
  void         callbackOdometryDiagnostics(const mrs_msgs::OdometryDiagConstPtr &msg);
  void         callbackControllerStatus(const mrs_msgs::ControllerStatusConstPtr &msg);

  bool setGains(std::string gains_name);
  bool setConstraints(std::string constraints_names);

  bool stringInVector(const std::string &value, const std::vector<std::string> &vector);

private:
  ros::ServiceServer service_server_set_gains;
  ros::ServiceServer service_server_set_constraints;

  ros::ServiceClient service_client_set_gains;
  ros::ServiceClient service_client_set_constraints;

  ros::Publisher publisher_current_gains;
  ros::Publisher publisher_current_constraints;

private:
  ros::Subscriber        subscriber_odometry_diagnostics;
  bool                   got_odometry_diagnostics = false;
  mrs_msgs::OdometryDiag odometry_diagnostics;
  std::mutex             mutex_odometry_diagnostics;

  // | ------------- constraint and gain management ------------- |

private:
  mrs_msgs::OdometryMode::_mode_type last_odometry_mode;

  void       managementTimer(const ros::TimerEvent &event);
  ros::Timer management_timer;
  double     rate_;

  // | --------------------- gain management -------------------- |

private:
  ros::Subscriber            subscriber_controller_status;
  bool                       got_controller_status = false;
  mrs_msgs::ControllerStatus controller_status;
  std::mutex                 mutex_controller_status;

  std::string current_gains;

  // | ------------------ constraint management ----------------- |

  std::string current_constraints;

  // | ------------------------ profiler ------------------------ |
private:
  mrs_lib::Profiler *profiler;
  bool               profiler_enabled_ = false;
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

  param_loader.load_param("enable_profiler", profiler_enabled_);

  param_loader.load_param("gains", gain_names_);
  param_loader.load_param("constraints", constraint_names_);

  param_loader.load_param("odometry_modes", odometry_mode_names_);
  param_loader.load_param("rate", rate_);

  std::vector<std::string>::iterator it;

  // loading gain_names
  for (it = gain_names_.begin(); it != gain_names_.end(); ++it) {
    ROS_INFO_STREAM("[GainManager]: loading gains \"" << *it << "\"");

    Gains_t new_gains;

    param_loader.load_param(*it + "/horizontal/kp", new_gains.kpxy);
    param_loader.load_param(*it + "/horizontal/kv", new_gains.kvxy);
    param_loader.load_param(*it + "/horizontal/ka", new_gains.kaxy);
    param_loader.load_param(*it + "/horizontal/kib", new_gains.kibxy);
    param_loader.load_param(*it + "/horizontal/kiw", new_gains.kiwxy);
    param_loader.load_param(*it + "/horizontal/kib_lim", new_gains.kibxy_lim);
    param_loader.load_param(*it + "/horizontal/kiw_lim", new_gains.kiwxy_lim);

    param_loader.load_param(*it + "/vertical/kp", new_gains.kpz);
    param_loader.load_param(*it + "/vertical/kv", new_gains.kvz);
    param_loader.load_param(*it + "/vertical/ka", new_gains.kaz);

    param_loader.load_param(*it + "/weight_estimator/km", new_gains.km);
    param_loader.load_param(*it + "/weight_estimator/km_lim", new_gains.km_lim);

    gains.insert(std::pair<std::string, Gains_t>(*it, new_gains));
  }

  // loading the allowed gains lists
  for (it = odometry_mode_names_.begin(); it != odometry_mode_names_.end(); ++it) {

    std::vector<std::string> temp_vector;
    param_loader.load_param("gain_management/allowed_gains/" + *it, temp_vector);

    std::vector<std::string>::iterator it2;
    for (it2 = temp_vector.begin(); it2 != temp_vector.end(); ++it2) {
      if (!stringInVector(*it2, gain_names_)) {
        ROS_ERROR("[GainManager]: the element '%s' of %s_allowed_gains is not a valid gain!", it2->c_str(), it->c_str());
        ros::shutdown();
      }
    }

    map_mode_allowed_gains.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));
  }

  // loading the fallback gains
  for (it = odometry_mode_names_.begin(); it != odometry_mode_names_.end(); ++it) {

    std::string temp_str;
    param_loader.load_param("gain_management/fallback_gains/" + *it, temp_str);

    if (!stringInVector(temp_str, map_mode_allowed_gains.at(*it))) {
      ROS_ERROR("[GainManager]: the element '%s' of %s_allowed_gains is not a valid gain!", temp_str.c_str(), it->c_str());
      ros::shutdown();
    }

    map_mode_fallback_gains.insert(std::pair<std::string, std::string>(*it, temp_str));
  }

  // loading constraint names
  for (it = constraint_names_.begin(); it != constraint_names_.end(); ++it) {
    ROS_INFO_STREAM("[GainManager]: loading constraints \"" << *it << "\"");

    mrs_msgs::TrackerConstraintsRequest new_constraints;

    param_loader.load_param(*it + "/horizontal/speed", new_constraints.horizontal_speed);
    param_loader.load_param(*it + "/horizontal/acceleration", new_constraints.horizontal_acceleration);
    param_loader.load_param(*it + "/horizontal/jerk", new_constraints.horizontal_jerk);

    param_loader.load_param(*it + "/vertical/ascending/speed", new_constraints.vertical_ascending_speed);
    param_loader.load_param(*it + "/vertical/ascending/acceleration", new_constraints.vertical_ascending_acceleration);
    param_loader.load_param(*it + "/vertical/ascending/jerk", new_constraints.vertical_ascending_jerk);

    param_loader.load_param(*it + "/vertical/descending/speed", new_constraints.vertical_descending_speed);
    param_loader.load_param(*it + "/vertical/descending/acceleration", new_constraints.vertical_descending_acceleration);
    param_loader.load_param(*it + "/vertical/descending/jerk", new_constraints.vertical_descending_jerk);

    param_loader.load_param(*it + "/yaw/speed", new_constraints.yaw_speed);
    param_loader.load_param(*it + "/yaw/acceleration", new_constraints.yaw_acceleration);
    param_loader.load_param(*it + "/yaw/jerk", new_constraints.yaw_jerk);

    constraints.insert(std::pair<std::string, mrs_msgs::TrackerConstraintsRequest>(*it, new_constraints));
  }

  // loading the allowed constraints lists
  for (it = odometry_mode_names_.begin(); it != odometry_mode_names_.end(); ++it) {

    std::vector<std::string> temp_vector;
    param_loader.load_param("constraint_management/allowed_constraints/" + *it, temp_vector);

    std::vector<std::string>::iterator it2;
    for (it2 = temp_vector.begin(); it2 != temp_vector.end(); ++it2) {
      if (!stringInVector(*it2, constraint_names_)) {
        ROS_ERROR("[GainManager]: the element '%s' of %s_allowed_constraints is not a valid constraint!", it2->c_str(), it->c_str());
        ros::shutdown();
      }
    }

    map_mode_allowed_constraints.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));
  }

  // loading the fallback constraints
  for (it = odometry_mode_names_.begin(); it != odometry_mode_names_.end(); ++it) {

    std::string temp_str;
    param_loader.load_param("constraint_management/fallback_constraints/" + *it, temp_str);

    if (!stringInVector(temp_str, map_mode_allowed_constraints.at(*it))) {
      ROS_ERROR("[GainManager]: the element '%s' of %s_allowed_constraints is not a valid constraint!", temp_str.c_str(), it->c_str());
      ros::shutdown();
    }

    map_mode_fallback_constraints.insert(std::pair<std::string, std::string>(*it, temp_str));
  }

  ROS_INFO("[GainManager]: done loading dynamical params");

  current_gains       = "";
  current_constraints = "";
  last_odometry_mode  = -1;

  // | ------------------------ services ------------------------ |

  service_server_set_gains       = nh_.advertiseService("set_gains_in", &GainManager::callbackSetGains, this);
  service_server_set_constraints = nh_.advertiseService("set_constraints_in", &GainManager::callbackSetConstraints, this);

  service_client_set_gains       = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_gains_out");
  service_client_set_constraints = nh_.serviceClient<mrs_msgs::TrackerConstraints>("set_constraints_out");

  // | ----------------------- subscribers ---------------------- |
  subscriber_odometry_diagnostics =
      nh_.subscribe("odometry_diagnostics_in", 1, &GainManager::callbackOdometryDiagnostics, this, ros::TransportHints().tcpNoDelay());
  subscriber_controller_status = nh_.subscribe("controller_status_in", 1, &GainManager::callbackControllerStatus, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  publisher_current_gains       = nh_.advertise<std_msgs::String>("current_gains_out", 1);
  publisher_current_constraints = nh_.advertise<std_msgs::String>("current_constraints_out", 1);

  // | ------------------------- timers ------------------------- |

  management_timer = nh_.createTimer(ros::Rate(rate_), &GainManager::managementTimer, this);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler = new mrs_lib::Profiler(nh_, "GainManager", profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[GainManager]: initilized");
}

//}

// --------------------------------------------------------------
// |                           methods                          |
// --------------------------------------------------------------

/* setGains() //{ */

bool GainManager::setGains(std::string gains_name) {

  std::map<std::string, Gains_t>::iterator it;
  it = gains.find(gains_name);

  if (it == gains.end()) {
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

  service_client_set_gains.call(reconf);

  current_gains = gains_name;

  ROS_INFO("[GainManager]: setting up gains for '%s'", gains_name.c_str());

  return true;
}

//}

/* setConstraints() //{ */

bool GainManager::setConstraints(std::string constraints_names) {

  std::map<std::string, mrs_msgs::TrackerConstraintsRequest>::iterator it;
  it = constraints.find(constraints_names);

  if (it == constraints.end()) {
    return false;
  }

  mrs_msgs::TrackerConstraints new_constraints;

  new_constraints.request = it->second;

  service_client_set_constraints.call(new_constraints);

  current_constraints = constraints_names;

  return new_constraints.response.success;
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | --------------------- topic callbacks -------------------- |

/* //{ callbackOdometryDiagnostics() */

void GainManager::callbackOdometryDiagnostics(const mrs_msgs::OdometryDiagConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackOdometryDiagnostics");

  mutex_odometry_diagnostics.lock();
  { odometry_diagnostics = *msg; }
  mutex_odometry_diagnostics.unlock();

  got_odometry_diagnostics = true;
}

//}

/* callbackControllerStatus() //{ */

void GainManager::callbackControllerStatus(const mrs_msgs::ControllerStatusConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackControllerStatus");

  mutex_controller_status.lock();
  { controller_status = *msg; }
  mutex_controller_status.unlock();

  got_controller_status = true;
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackSetGains() */

bool GainManager::callbackSetGains(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

  if (!is_initialized)
    return false;

  char message[200];

  if (!stringInVector(req.value, gain_names_)) {

    sprintf((char *)&message, "The gains '%s' do not exist (in the gain_manager's config).", req.value.c_str());
    res.message = message;
    res.success = false;
    ROS_ERROR("[GainManager]: %s", message);
    return true;
  }

  if (!stringInVector(req.value, map_mode_allowed_gains.at(odometry_diagnostics.odometry_mode.name))) {

    sprintf((char *)&message, "The gains '%s' are not allowed given the current odometry mode.", req.value.c_str());
    res.message = message;
    res.success = false;
    ROS_ERROR("[GainManager]: %s", message);
    return true;
  }

  // try to set the gains
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

/* //{ callbackSetGains() */

bool GainManager::callbackSetConstraints(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

  if (!is_initialized)
    return false;

  char message[200];

  if (!stringInVector(req.value, constraint_names_)) {

    sprintf((char *)&message, "The constraints '%s' do not exist (in the gain_manager's config).", req.value.c_str());
    res.message = message;
    res.success = false;
    ROS_ERROR("[GainManager]: %s", message);
    return true;
  }

  if (!stringInVector(req.value, map_mode_allowed_constraints.at(odometry_diagnostics.odometry_mode.name))) {

    sprintf((char *)&message, "The constraints '%s' are not allowed given the current odometry mode.", req.value.c_str());
    res.message = message;
    res.success = false;
    ROS_ERROR("[GainManager]: %s", message);
    return true;
  }

  // try to set the gains
  if (!setConstraints(req.value)) {

    res.message = "the control_manager can't set the constraints";
    res.success = false;
    return true;

  } else {

    sprintf((char *)&message, "The constraints '%s' are set.", req.value.c_str());
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

/* managementTimer() //{ */

void GainManager::managementTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("gainManagementTimer", rate_, 0.01, event);

  if (!got_odometry_diagnostics) {
    ROS_WARN_THROTTLE(1.0, "[GainManager]: can't do gain management, missing odometry diagnostics!");
    return;
  }

  mutex_controller_status.lock();
  if (!(got_controller_status && controller_status.controller.compare("mrs_controllers/NsfController") == STRING_EQUAL)) {
    ROS_WARN_THROTTLE(1.0, "[GainManager]: can't do gain management, the NSF controller is not running!");
    mutex_controller_status.unlock();
    return;
  }
  mutex_controller_status.unlock();

  // | --- automatically set gains when odometry mode schanges -- |
  if (odometry_diagnostics.odometry_mode.mode != last_odometry_mode) {

    ROS_WARN("[GainManager]: the odometry mode has changed! %d -> %d", last_odometry_mode, odometry_diagnostics.odometry_mode.mode);

    std::map<std::string, std::string>::iterator it;
    it = map_mode_fallback_gains.find(odometry_diagnostics.odometry_mode.name);

    if (it == map_mode_fallback_gains.end()) {

      ROS_ERROR("[GainManager]: the odometry mode %s was not specified in the gain_manager's config!", odometry_diagnostics.odometry_mode.name.c_str());

    } else {
      if (setGains(it->second)) {
        last_odometry_mode = odometry_diagnostics.odometry_mode.mode;
      }
    }

    it = map_mode_fallback_constraints.find(odometry_diagnostics.odometry_mode.name);

    if (it == map_mode_fallback_constraints.end()) {

      ROS_ERROR("[GainManager]: the odometry mode %s was not specified in the constraint_manager's config!", odometry_diagnostics.odometry_mode.name.c_str());

    } else {
      if (setConstraints(it->second)) {
        last_odometry_mode = odometry_diagnostics.odometry_mode.mode;
      }
    }
  }

  std_msgs::String str_out;
  str_out.data = current_gains;

  try {
    publisher_current_gains.publish(str_out);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", publisher_current_gains.getTopic().c_str());
  }

  str_out.data = current_constraints;
  try {
    publisher_current_constraints.publish(str_out);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", publisher_current_constraints.getTopic().c_str());
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

}  // namespace mrs_mav_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mav_manager::GainManager, nodelet::Nodelet)
