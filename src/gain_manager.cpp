#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/String.h>

#include <mrs_msgs/String.h>
#include <mrs_msgs/OdometryDiag.h>
#include <mrs_msgs/EstimatorType.h>
#include <mrs_msgs/ControllerStatus.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>

#include <dynamic_reconfigure/ReconfigureRequest.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#define STRING_EQUAL 0

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
  bool            is_initialized = false;

private:
  std::vector<std::string> estimator_type_names_;

  std::vector<std::string>       gain_names_;
  std::map<std::string, Gains_t> gains;

private:
  std::map<std::string, std::vector<std::string>> map_type_allowed_gains;
  std::map<std::string, std::string>              map_type_fallback_gains;

public:
  virtual void onInit();
  bool         callbackSetGains(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);
  void         callbackOdometryDiagnostics(const mrs_msgs::OdometryDiagConstPtr &msg);
  void         callbackControllerStatus(const mrs_msgs::ControllerStatusConstPtr &msg);

  bool setGains(std::string gains_name);

  bool stringInVector(const std::string &value, const std::vector<std::string> &vector);

private:
  ros::ServiceServer service_server_set_gains;

  ros::ServiceClient service_client_set_gains;

  ros::Publisher publisher_current_gains;

private:
  ros::Subscriber        subscriber_odometry_diagnostics;
  bool                   got_odometry_diagnostics = false;
  mrs_msgs::OdometryDiag odometry_diagnostics;
  std::mutex             mutex_odometry_diagnostics;

  // | ------------- gain management ------------- |

private:
  mrs_msgs::EstimatorType::_type_type last_estimator_type;

  void       gainsManagementTimer(const ros::TimerEvent &event);
  ros::Timer gains_management_timer;

  int rate_;

  // | --------------------- gain management -------------------- |

private:
  ros::Subscriber            subscriber_controller_status;
  bool                       got_controller_status = false;
  mrs_msgs::ControllerStatus controller_status;
  std::mutex                 mutex_controller_status;

  std::string current_gains;

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

  param_loader.load_param("estimator_types", estimator_type_names_);
  param_loader.load_param("rate", rate_);

  std::vector<std::string>::iterator it;

  // loading gain_names
  for (it = gain_names_.begin(); it != gain_names_.end(); ++it) {
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

    gains.insert(std::pair<std::string, Gains_t>(*it, new_gains));
  }

  // loading the allowed gains lists
  for (it = estimator_type_names_.begin(); it != estimator_type_names_.end(); ++it) {

    std::vector<std::string> temp_vector;
    param_loader.load_param("gain_management/allowed_gains/" + *it, temp_vector);

    std::vector<std::string>::iterator it2;
    for (it2 = temp_vector.begin(); it2 != temp_vector.end(); ++it2) {
      if (!stringInVector(*it2, gain_names_)) {
        ROS_ERROR("[GainManager]: the element '%s' of %s_allowed_gains is not a valid gain!", it2->c_str(), it->c_str());
        ros::shutdown();
      }
    }

    map_type_allowed_gains.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));
  }

  // loading the fallback gains
  for (it = estimator_type_names_.begin(); it != estimator_type_names_.end(); ++it) {

    std::string temp_str;
    param_loader.load_param("gain_management/fallback_gains/" + *it, temp_str);

    if (!stringInVector(temp_str, map_type_allowed_gains.at(*it))) {
      ROS_ERROR("[GainManager]: the element '%s' of %s_allowed_gains is not a valid gain!", temp_str.c_str(), it->c_str());
      ros::shutdown();
    }

    map_type_fallback_gains.insert(std::pair<std::string, std::string>(*it, temp_str));
  }

  ROS_INFO("[GainManager]: done loading dynamical params");

  current_gains       = "";
  last_estimator_type = -1;

  // | ------------------------ services ------------------------ |

  service_server_set_gains = nh_.advertiseService("set_gains_in", &GainManager::callbackSetGains, this);

  service_client_set_gains = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_gains_out");

  // | ----------------------- subscribers ---------------------- |
  subscriber_odometry_diagnostics =
      nh_.subscribe("odometry_diagnostics_in", 1, &GainManager::callbackOdometryDiagnostics, this, ros::TransportHints().tcpNoDelay());
  subscriber_controller_status = nh_.subscribe("controller_status_in", 1, &GainManager::callbackControllerStatus, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  publisher_current_gains = nh_.advertise<std_msgs::String>("current_gains_out", 1);

  // | ------------------------- timers ------------------------- |

  gains_management_timer = nh_.createTimer(ros::Rate(rate_), &GainManager::gainsManagementTimer, this);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler = new mrs_lib::Profiler(nh_, "GainManager", profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[GainManager]: Could not load all parameters!");
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

  service_client_set_gains.call(reconf);

  current_gains = gains_name;

  ROS_INFO("[GainManager]: setting up gains for '%s'", gains_name.c_str());

  return true;
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

  {
    std::scoped_lock lock(mutex_odometry_diagnostics);

    odometry_diagnostics = *msg;
  }

  got_odometry_diagnostics = true;
}

//}

/* callbackControllerStatus() //{ */

void GainManager::callbackControllerStatus(const mrs_msgs::ControllerStatusConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackControllerStatus");

  {
    std::scoped_lock lock(mutex_controller_status);

    controller_status = *msg;
  }

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

  if (!stringInVector(req.value, map_type_allowed_gains.at(odometry_diagnostics.estimator_type.name))) {

    sprintf((char *)&message, "The gains '%s' are not allowed given the current odometry.type.", req.value.c_str());
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

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* gainManagementTimer() //{ */

void GainManager::gainsManagementTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("gainManagementTimer", rate_, 0.01, event);

  if (!got_odometry_diagnostics) {
    ROS_WARN_THROTTLE(1.0, "[GainManager]: can't do gain management, missing odometry diagnostics!");
    return;
  }

  {
    std::scoped_lock lock(mutex_controller_status);

    if (!(got_controller_status && controller_status.controller.compare("mrs_controllers/So3Controller") == STRING_EQUAL)) {
      ROS_INFO_THROTTLE(1.0, "[GainManager]: can't do gain management, the SO3 controller is not running!");
      return;
    }
  }

  // | --- automatically set gains when odometry.type changes -- |
  if (odometry_diagnostics.estimator_type.type != last_estimator_type) {

    ROS_WARN_THROTTLE(1.0, "[GainManager]: the odometry.type has changed! %d -> %d", last_estimator_type, odometry_diagnostics.estimator_type.type);

    std::map<std::string, std::string>::iterator it;
    it = map_type_fallback_gains.find(odometry_diagnostics.estimator_type.name);

    if (it == map_type_fallback_gains.end()) {
      ROS_ERROR("[GainManager]: the odometry.type %s was not specified in the gain_manager's config!", odometry_diagnostics.estimator_type.name.c_str());
    } else {
      if (setGains(it->second)) {
        last_estimator_type = odometry_diagnostics.estimator_type.type;
      } else {
        ROS_ERROR_THROTTLE(1.0, "[GainManager]: service call to set gains failed!");
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
