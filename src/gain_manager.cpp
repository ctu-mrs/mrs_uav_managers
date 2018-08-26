#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/String.h>

#include <mrs_msgs/String.h>
#include <mrs_msgs/OdometryDiag.h>
#include <mrs_msgs/OdometryMode.h>
#include <mrs_msgs/ControllerStatus.h>

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
  std::vector<std::string>       gain_names;
  std::map<std::string, Gains_t> gains;

  // | --------------------- fallback gains --------------------- |
private:
  std::string VIO_fallback_gain;
  std::string ICP_fallback_gain;
  std::string RTK_fallback_gain;
  std::string OPTFLOWGPS_fallback_gain;
  std::string GPS_fallback_gain;
  std::string OPTFLOW_fallback_gain;
  std::string OTHER_fallback_gain;

  // | ---------------------- allowed gains --------------------- |
private:
  std::vector<std::string> VIO_allowed_gains;
  std::vector<std::string> ICP_allowed_gains;
  std::vector<std::string> RTK_allowed_gains;
  std::vector<std::string> OPTFLOWGPS_allowed_gains;
  std::vector<std::string> GPS_allowed_gains;
  std::vector<std::string> OPTFLOW_allowed_gains;
  std::vector<std::string> OTHER_allowed_gains;

public:
  virtual void onInit();
  bool         callbackSetGains(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);
  void         callbackOdometryDiagnostics(const mrs_msgs::OdometryDiagConstPtr &msg);
  void         callbackControllerStatus(const mrs_msgs::ControllerStatusConstPtr &msg);

  bool setGains(std::string gains_name);

  bool stringInVector(const std::string &value, const std::vector<std::string> &vector);

private:
  ros::ServiceServer service_server_takeoff;
  ros::ServiceClient service_client_set_gains;

  ros::Publisher publisher_current_gains;

  // | -------------------- gain scheduling -------------------- |
private:
  ros::Timer gain_scheduling_timer;
  double     gain_scheduling_rate_;

  void gainSchedulingTimer(const ros::TimerEvent &event);

private:
  ros::Subscriber        subscriber_odometry_diagnostics;
  bool                   got_odometry_diagnostics = false;
  mrs_msgs::OdometryDiag odometry_diagnostics;
  std::mutex             mutex_odometry_diagnostics;

private:
  ros::Subscriber            subscriber_controller_status;
  bool                       got_controller_status = false;
  mrs_msgs::ControllerStatus controller_status;
  std::mutex                 mutex_controller_status;

private:
  std::string                        current_gains;
  mrs_msgs::OdometryMode::_mode_type last_odometry_mode;

  // | ------------------------ profiler ------------------------ |
private:
  mrs_lib::Profiler *profiler;
  mrs_lib::Routine * routine_callback_odometry_diagnostics;
  mrs_lib::Routine * routine_callback_controller_status;
  mrs_lib::Routine * routine_gain_schedulling_timer;
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
  param_loader.load_param("gains", gain_names);
  param_loader.load_param("gain_scheduling/rate", gain_scheduling_rate_);

  std::vector<std::string>::iterator it;
  for (it = gain_names.begin(); it != gain_names.end(); ++it) {
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

  // load the allowed gain lists
  param_loader.load_param("gain_scheduling/VIO_allowed", VIO_allowed_gains);
  for (it = VIO_allowed_gains.begin(); it != VIO_allowed_gains.end(); ++it) {
    if (!stringInVector(*it, gain_names)) {
      ROS_ERROR("[GainManager]: the element of VIO_allowed_gains ('%s') is not a valid gain!", it->c_str());
      ros::shutdown();
    }
  }

  param_loader.load_param("gain_scheduling/ICP_allowed", ICP_allowed_gains);
  for (it = ICP_allowed_gains.begin(); it != ICP_allowed_gains.end(); ++it) {
    if (!stringInVector(*it, gain_names)) {
      ROS_ERROR("[GainManager]: the element of ICP_allowed_gains ('%s') is not a valid gain!", it->c_str());
      ros::shutdown();
    }
  }

  param_loader.load_param("gain_scheduling/RTK_allowed", RTK_allowed_gains);
  for (it = RTK_allowed_gains.begin(); it != RTK_allowed_gains.end(); ++it) {
    if (!stringInVector(*it, gain_names)) {
      ROS_ERROR("[GainManager]: the element of RTK_allowed_gains ('%s') is not a valid gain!", it->c_str());
      ros::shutdown();
    }
  }

  param_loader.load_param("gain_scheduling/OPTFLOWGPS_allowed", OPTFLOWGPS_allowed_gains);
  for (it = OPTFLOWGPS_allowed_gains.begin(); it != OPTFLOWGPS_allowed_gains.end(); ++it) {
    if (!stringInVector(*it, gain_names)) {
      ROS_ERROR("[GainManager]: the element of OPTFLOWGPS_allowed_gains ('%s') is not a valid gain!", it->c_str());
      ros::shutdown();
    }
  }

  param_loader.load_param("gain_scheduling/GPS_allowed", GPS_allowed_gains);
  for (it = GPS_allowed_gains.begin(); it != GPS_allowed_gains.end(); ++it) {
    if (!stringInVector(*it, gain_names)) {
      ROS_ERROR("[GainManager]: the element of GPS_allowed_gains ('%s') is not a valid gain!", it->c_str());
      ros::shutdown();
    }
  }

  param_loader.load_param("gain_scheduling/OPTFLOW_allowed", OPTFLOW_allowed_gains);
  for (it = OPTFLOW_allowed_gains.begin(); it != OPTFLOW_allowed_gains.end(); ++it) {
    if (!stringInVector(*it, gain_names)) {
      ROS_ERROR("[GainManager]: the element of OPTFLOW_allowed_gains ('%s') is not a valid gain!", it->c_str());
      ros::shutdown();
    }
  }

  param_loader.load_param("gain_scheduling/OTHER_allowed", OTHER_allowed_gains);
  for (it = OTHER_allowed_gains.begin(); it != OTHER_allowed_gains.end(); ++it) {
    if (!stringInVector(*it, gain_names)) {
      ROS_ERROR("[GainManager]: the element of OTHER_allowed_gains ('%s') is not a valid gain!", it->c_str());
      ros::shutdown();
    }
  }

  // load the fallback gains and check that they are in the allowed
  param_loader.load_param("gain_scheduling/VIO_fallback", VIO_fallback_gain);
  if (!stringInVector(VIO_fallback_gain, VIO_allowed_gains)) {
    ROS_ERROR("[GainManager]: the VIO_fallback_gain '%s' is not cantained in VIO_allowed_gains!", VIO_fallback_gain.c_str());
    ros::shutdown();
  }

  param_loader.load_param("gain_scheduling/ICP_fallback", ICP_fallback_gain);
  if (!stringInVector(ICP_fallback_gain, ICP_allowed_gains)) {
    ROS_ERROR("[GainManager]: the ICP_fallback_gain '%s' is not cantained in ICP_allowed_gains!", ICP_fallback_gain.c_str());
    ros::shutdown();
  }

  param_loader.load_param("gain_scheduling/RTK_fallback", RTK_fallback_gain);
  if (!stringInVector(RTK_fallback_gain, RTK_allowed_gains)) {
    ROS_ERROR("[GainManager]: the RTK_fallback_gain '%s' is not cantained in RTK_allowed_gains!", RTK_fallback_gain.c_str());
    ros::shutdown();
  }

  param_loader.load_param("gain_scheduling/OPTFLOWGPS_fallback", OPTFLOWGPS_fallback_gain);
  if (!stringInVector(OPTFLOWGPS_fallback_gain, OPTFLOWGPS_allowed_gains)) {
    ROS_ERROR("[GainManager]: the OPTFLOWGPS_fallback_gain '%s' is not cantained in OPTFLOWGPS_allowed_gains!", OPTFLOWGPS_fallback_gain.c_str());
    ros::shutdown();
  }

  param_loader.load_param("gain_scheduling/GPS_fallback", GPS_fallback_gain);
  if (!stringInVector(GPS_fallback_gain, GPS_allowed_gains)) {
    ROS_ERROR("[GainManager]: the GPS_fallback_gain '%s' is not cantained in GPS_allowed_gains!", GPS_fallback_gain.c_str());
    ros::shutdown();
  }

  param_loader.load_param("gain_scheduling/OPTFLOW_fallback", OPTFLOW_fallback_gain);
  if (!stringInVector(OPTFLOW_fallback_gain, OPTFLOW_allowed_gains)) {
    ROS_ERROR("[GainManager]: the OPTFLOW_fallback_gain '%s' is not cantained in OPTFLOW_allowed_gains!", OPTFLOW_fallback_gain.c_str());
    ros::shutdown();
  }

  param_loader.load_param("gain_scheduling/OTHER_fallback", OTHER_fallback_gain);
  if (!stringInVector(OTHER_fallback_gain, OTHER_allowed_gains)) {
    ROS_ERROR("[GainManager]: the OTHER_fallback_gain '%s' is not cantained in OTHER_allowed_gains!", OTHER_fallback_gain.c_str());
    ros::shutdown();
  }

  current_gains      = "";
  last_odometry_mode = mrs_msgs::OdometryMode::OTHER;

  // | ------------------------ services ------------------------ |

  service_server_takeoff   = nh_.advertiseService("set_gains_in", &GainManager::callbackSetGains, this);
  service_client_set_gains = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_gains_out");

  // | ----------------------- subscribers ---------------------- |
  subscriber_odometry_diagnostics =
      nh_.subscribe("odometry_diagnostics_in", 1, &GainManager::callbackOdometryDiagnostics, this, ros::TransportHints().tcpNoDelay());
  subscriber_controller_status = nh_.subscribe("controller_status_in", 1, &GainManager::callbackControllerStatus, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  publisher_current_gains = nh_.advertise<std_msgs::String>("current_gains_out", 1);

  // | ------------------------- timers ------------------------- |

  gain_scheduling_timer = nh_.createTimer(ros::Rate(gain_scheduling_rate_), &GainManager::gainSchedulingTimer, this);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler                              = new mrs_lib::Profiler(nh_, "GainManager");
  routine_gain_schedulling_timer        = profiler->registerRoutine("gainSchedullingTimer", gain_scheduling_rate_, 0.01);
  routine_callback_odometry_diagnostics = profiler->registerRoutine("callbackOdometryDiagnostics");
  routine_callback_controller_status    = profiler->registerRoutine("callbackControllerStatus");

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

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackSetGains() */

bool GainManager::callbackSetGains(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

  if (!is_initialized)
    return false;

  if (!setGains(req.value)) {
    res.message = "those gains do not exist";
    res.success = false;
    return true;
  }

  res.message = "New gains set";
  res.success = true;

  return true;
}

//}

/* //{ callbackOdometryDiagnostics() */

void GainManager::callbackOdometryDiagnostics(const mrs_msgs::OdometryDiagConstPtr &msg) {

  if (!is_initialized)
    return;

  routine_callback_odometry_diagnostics->start();

  mutex_odometry_diagnostics.lock();
  { odometry_diagnostics = *msg; }
  mutex_odometry_diagnostics.unlock();

  got_odometry_diagnostics = true;

  routine_callback_odometry_diagnostics->end();
}

//}

/* callbackControllerStatus() //{ */

void GainManager::callbackControllerStatus(const mrs_msgs::ControllerStatusConstPtr &msg) {

  if (!is_initialized)
    return;

  routine_callback_controller_status->start();

  mutex_controller_status.lock();
  { controller_status = *msg; }
  mutex_controller_status.unlock();

  got_controller_status = true;

  routine_callback_controller_status->end();
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* gainschedulingTimer() //{ */

void GainManager::gainSchedulingTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  if (!got_odometry_diagnostics) {
    ROS_WARN_THROTTLE(1.0, "[MavManager]: can't do gain schedulling, missing odometry diagnostics!");
    return;
  }

  mutex_controller_status.lock();
  if (!(got_controller_status && controller_status.controller.compare("mrs_controllers/NsfController") == STRING_EQUAL)) {
    ROS_WARN_THROTTLE(1.0, "[MavManager]: can't do gain schedulling, the NSF controller is not running!");
    mutex_controller_status.unlock();
    return;
  }
  mutex_controller_status.unlock();

  routine_gain_schedulling_timer->start(event);

  // someone should put some fancy ifs here...
  if (odometry_diagnostics.odometry_mode.mode != last_odometry_mode) {

    ROS_WARN("[GainManager]: the odometry mode has changed! %d -> %d", last_odometry_mode, odometry_diagnostics.odometry_mode.mode);

    bool success = false;

    switch (odometry_diagnostics.odometry_mode.mode) {

      case mrs_msgs::OdometryMode::VIO:
        success = setGains(VIO_fallback_gain);
        break;

      case mrs_msgs::OdometryMode::ICP:
        success = setGains(ICP_fallback_gain);
        break;

      case mrs_msgs::OdometryMode::RTK:
        success = setGains(RTK_fallback_gain);
        break;

      case mrs_msgs::OdometryMode::OPTFLOWGPS:
        success = setGains(OPTFLOWGPS_fallback_gain);
        break;

      case mrs_msgs::OdometryMode::GPS:
        success = setGains(GPS_fallback_gain);
        break;

      case mrs_msgs::OdometryMode::OPTFLOW:
        success = setGains(OPTFLOW_fallback_gain);
        break;

      case mrs_msgs::OdometryMode::OTHER:
        success = setGains(OTHER_fallback_gain);
        break;
    }

    if (success) {
      last_odometry_mode = odometry_diagnostics.odometry_mode.mode;
    }
  }

  routine_gain_schedulling_timer->start(event);
}

//}

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
