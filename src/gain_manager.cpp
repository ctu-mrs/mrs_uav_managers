#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_msgs/String.h>

#include <mrs_lib/Profiler.h>

#include <mrs_lib/ParamLoader.h>

#include <dynamic_reconfigure/ReconfigureRequest.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

namespace mrs_mav_manager
{

//{ class GainManager

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
  std::vector<std::string> gain_names;
  /* std::vector<Gains_t> gains; */
  std::map<std::string, Gains_t> gains;

public:
  virtual void onInit();
  bool         callbackSetGains(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);

private:
  ros::ServiceServer service_server_takeoff;
  ros::ServiceClient service_client_set_gains;
};

//}

//{ onInit()

void GainManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[GainManager]: initializing");

  // | ------------------------- params ------------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "GainManager");
  param_loader.load_param("gains", gain_names);

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

    ROS_INFO("[GainManager]: %f", new_gains.kpxy);

    gains.insert(std::pair<std::string, Gains_t>(*it, new_gains));
  }

  // | ------------------------ services ------------------------ |

  service_server_takeoff   = nh_.advertiseService("set_gains_in", &GainManager::callbackSetGains, this);
  service_client_set_gains = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_gains_out");

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[GainManager]: initilized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

//{ callbackSetGains()

bool GainManager::callbackSetGains(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

  if (!is_initialized)
    return false;

  std::map<std::string, Gains_t>::iterator it;
  it = gains.find(req.value);

  if (it == gains.end()) {
    res.message = "those gains do not exist";
    res.success = false;
    return true;
  }

  dynamic_reconfigure::Config          conf;
  dynamic_reconfigure::DoubleParameter param;

  ROS_INFO("[GainManager]: %f", it->second.kpz);

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

  ROS_INFO("[GainManager]: setting up gains for '%s'", req.value.c_str());

  res.message = "New gains set";
  res.success = true;

  return true;
}

//}

}  // namespace mrs_mav_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mav_manager::GainManager, nodelet::Nodelet)
