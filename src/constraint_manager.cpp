#define VERSION "1.0.2.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/String.h>

#include <mrs_msgs/ConstraintManagerDiagnostics.h>
#include <mrs_msgs/OdometryDiag.h>
#include <mrs_msgs/EstimatorType.h>
#include <mrs_msgs/DynamicsConstraintsSrv.h>
#include <mrs_msgs/DynamicsConstraintsSrvRequest.h>
#include <mrs_msgs/String.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>

#include <dynamic_reconfigure/ReconfigureRequest.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

//}

namespace mrs_uav_managers
{

namespace constraint_manager
{

/* //{ class ConstraintManager */

class ConstraintManager : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  std::string     _version_;
  bool            is_initialized_ = false;

  // | ----------------------- parameters ----------------------- |

  std::vector<std::string> _estimator_type_names_;

  std::vector<std::string>                                       _constraint_names_;
  std::map<std::string, mrs_msgs::DynamicsConstraintsSrvRequest> _constraints_;

  std::map<std::string, std::vector<std::string>> _map_type_allowed_constraints_;
  std::map<std::string, std::string>              _map_type_fallback_constraints_;

  // | --------------------- service clients -------------------- |

  ros::ServiceClient service_client_set_constraints_;

  // | ------------------ odometry diagnostics ------------------ |

  ros::Subscriber subscriber_odometry_diagnostics_;
  void            callbackOdometryDiagnostics(const mrs_msgs::OdometryDiagConstPtr &msg);
  bool            got_odometry_diagnostics_ = false;
  std::mutex      mutex_odometry_diagnostics_;

  mrs_msgs::OdometryDiag odometry_diagnostics_;

  // | ------------- constraint management ------------- |

  bool setConstraints(std::string constraints_names);

  ros::ServiceServer service_server_set_constraints_;
  bool               callbackSetConstraints(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);

  mrs_msgs::EstimatorType::_type_type last_estimator_type_;
  std::mutex                          mutex_last_estimator_type_;

  void       timerConstraintManagement(const ros::TimerEvent &event);
  ros::Timer timer_constraint_management_;
  int        _constraint_management_rate_;

  std::string current_constraints_;
  std::mutex  mutex_current_constraints_;

  // | ------------------ diagnostics publisher ----------------- |

  void           timerDiagnostics(const ros::TimerEvent &event);
  ros::Publisher publisher_diagnostics_;
  ros::Timer     timer_diagnostics_;
  int            _diagnostics_rate_;

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

/* //{ onInit() */

void ConstraintManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[ConstraintManager]: initializing");

  // | ------------------------- params ------------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "ConstraintManager");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[ConstraintManager]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("enable_profiler", _profiler_enabled_);

  param_loader.loadParam("constraints", _constraint_names_);

  param_loader.loadParam("estimator_types", _estimator_type_names_);

  param_loader.loadParam("rate", _constraint_management_rate_);
  param_loader.loadParam("diagnostics_rate", _diagnostics_rate_);

  std::vector<std::string>::iterator it;

  // loading constraint names
  for (it = _constraint_names_.begin(); it != _constraint_names_.end(); ++it) {

    ROS_INFO_STREAM("[ConstraintManager]: loading constraints '" << *it << "'");

    mrs_msgs::DynamicsConstraintsSrvRequest new_constraints;

    param_loader.loadParam(*it + "/horizontal/speed", new_constraints.constraints.horizontal_speed);
    param_loader.loadParam(*it + "/horizontal/acceleration", new_constraints.constraints.horizontal_acceleration);
    param_loader.loadParam(*it + "/horizontal/jerk", new_constraints.constraints.horizontal_jerk);
    param_loader.loadParam(*it + "/horizontal/snap", new_constraints.constraints.horizontal_snap);

    param_loader.loadParam(*it + "/vertical/ascending/speed", new_constraints.constraints.vertical_ascending_speed);
    param_loader.loadParam(*it + "/vertical/ascending/acceleration", new_constraints.constraints.vertical_ascending_acceleration);
    param_loader.loadParam(*it + "/vertical/ascending/jerk", new_constraints.constraints.vertical_ascending_jerk);
    param_loader.loadParam(*it + "/vertical/ascending/snap", new_constraints.constraints.vertical_ascending_snap);

    param_loader.loadParam(*it + "/vertical/descending/speed", new_constraints.constraints.vertical_descending_speed);
    param_loader.loadParam(*it + "/vertical/descending/acceleration", new_constraints.constraints.vertical_descending_acceleration);
    param_loader.loadParam(*it + "/vertical/descending/jerk", new_constraints.constraints.vertical_descending_jerk);
    param_loader.loadParam(*it + "/vertical/descending/snap", new_constraints.constraints.vertical_descending_snap);

    param_loader.loadParam(*it + "/heading/speed", new_constraints.constraints.heading_speed);
    param_loader.loadParam(*it + "/heading/acceleration", new_constraints.constraints.heading_acceleration);
    param_loader.loadParam(*it + "/heading/jerk", new_constraints.constraints.heading_jerk);
    param_loader.loadParam(*it + "/heading/snap", new_constraints.constraints.heading_snap);

    param_loader.loadParam(*it + "/angular_speed/roll", new_constraints.constraints.roll_rate);
    param_loader.loadParam(*it + "/angular_speed/pitch", new_constraints.constraints.pitch_rate);
    param_loader.loadParam(*it + "/angular_speed/yaw", new_constraints.constraints.yaw_rate);

    param_loader.loadParam(*it + "/tilt", new_constraints.constraints.tilt);

    _constraints_.insert(std::pair<std::string, mrs_msgs::DynamicsConstraintsSrvRequest>(*it, new_constraints));
  }

  // loading the allowed constraints lists
  for (it = _estimator_type_names_.begin(); it != _estimator_type_names_.end(); ++it) {

    std::vector<std::string> temp_vector;
    param_loader.loadParam("constraint_management/allowed_constraints/" + *it, temp_vector);

    std::vector<std::string>::iterator it2;
    for (it2 = temp_vector.begin(); it2 != temp_vector.end(); ++it2) {
      if (!stringInVector(*it2, _constraint_names_)) {
        ROS_ERROR("[ConstraintManager]: the element '%s' of %s/allowed_constraints is not a valid constraint!", it2->c_str(), it->c_str());
        ros::shutdown();
      }
    }

    _map_type_allowed_constraints_.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));
  }

  // loading the fallback constraints
  for (it = _estimator_type_names_.begin(); it != _estimator_type_names_.end(); ++it) {

    std::string temp_str;
    param_loader.loadParam("constraint_management/fallback_constraints/" + *it, temp_str);

    if (!stringInVector(temp_str, _map_type_allowed_constraints_.at(*it))) {
      ROS_ERROR("[ConstraintManager]: the element '%s' of %s/allowed_constraints is not a valid constraint!", temp_str.c_str(), it->c_str());
      ros::shutdown();
    }

    _map_type_fallback_constraints_.insert(std::pair<std::string, std::string>(*it, temp_str));
  }

  ROS_INFO("[ConstraintManager]: done loading dynamical params");

  current_constraints_ = "";
  last_estimator_type_ = -1;

  // | ------------------------ services ------------------------ |

  service_server_set_constraints_ = nh_.advertiseService("set_constraints_in", &ConstraintManager::callbackSetConstraints, this);

  service_client_set_constraints_ = nh_.serviceClient<mrs_msgs::DynamicsConstraintsSrv>("set_constraints_out");

  // | ----------------------- subscribers ---------------------- |
  subscriber_odometry_diagnostics_ =
      nh_.subscribe("odometry_diagnostics_in", 1, &ConstraintManager::callbackOdometryDiagnostics, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  publisher_diagnostics_ = nh_.advertise<mrs_msgs::ConstraintManagerDiagnostics>("diagnostics_out", 1);

  // | ------------------------- timers ------------------------- |

  timer_constraint_management_ = nh_.createTimer(ros::Rate(_constraint_management_rate_), &ConstraintManager::timerConstraintManagement, this);
  timer_diagnostics_           = nh_.createTimer(ros::Rate(_diagnostics_rate_), &ConstraintManager::timerDiagnostics, this);

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "ConstraintManager", _profiler_enabled_);

  // | ------------------- scope timer logger ------------------- |

  param_loader.loadParam("scope_timer/enabled", scope_timer_enabled_);
  const std::string scope_timer_log_filename = param_loader.loadParam2("scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<mrs_lib::ScopeTimerLogger>(scope_timer_log_filename, scope_timer_enabled_);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ConstraintManager]: Could not load all parameters!");
    ros::shutdown();
  }

  is_initialized_ = true;

  ROS_INFO("[ConstraintManager]: initialized, version %s", VERSION);

  ROS_DEBUG("[ConstraintManager]: debug output is enabled");
}

//}

// --------------------------------------------------------------
// |                           methods                          |
// --------------------------------------------------------------

/* setConstraints() //{ */

bool ConstraintManager::setConstraints(std::string constraints_name) {

  std::map<std::string, mrs_msgs::DynamicsConstraintsSrvRequest>::iterator it;
  it = _constraints_.find(constraints_name);

  if (it == _constraints_.end()) {
    ROS_ERROR("[ConstraintManager]: could not setConstraints(), the constraint name '%s' is not on the list", constraints_name.c_str());
    return false;
  }

  mrs_msgs::DynamicsConstraintsSrv srv_call;

  srv_call.request = it->second;

  bool res = service_client_set_constraints_.call(srv_call);

  if (!res) {

    ROS_WARN_THROTTLE(1.0, "[ConstraintManager]: the service for setting constraints has failed!");
    return false;

  } else {

    if (srv_call.response.success) {

      mrs_lib::set_mutexed(mutex_current_constraints_, constraints_name, current_constraints_);
      return true;

    } else {

      ROS_WARN_THROTTLE(1.0, "[ConstraintManager]: set service for setting constraints returned: '%s'", srv_call.response.message.c_str());
      return false;
    }
  }
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | --------------------- topic callbacks -------------------- |

/* //{ callbackOdometryDiagnostics() */

void ConstraintManager::callbackOdometryDiagnostics(const mrs_msgs::OdometryDiagConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackOdometryDiagnostics");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ContraintManager::callbackOdometryDiagnostics", scope_timer_logger_, scope_timer_enabled_);

  {
    std::scoped_lock lock(mutex_odometry_diagnostics_);

    odometry_diagnostics_ = *msg;
  }

  got_odometry_diagnostics_ = true;
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackSetConstraints() */

bool ConstraintManager::callbackSetConstraints(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

  if (!is_initialized_)
    return false;

  auto odometry_diagnostics = mrs_lib::get_mutexed(mutex_odometry_diagnostics_, odometry_diagnostics_);

  std::stringstream ss;

  if (!stringInVector(req.value, _constraint_names_)) {

    ss << "the constraints '" << req.value.c_str() << "' do not exist (in the ConstraintManager's config)";

    ROS_ERROR_STREAM_THROTTLE(1.0, "[ConstraintManager]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!stringInVector(req.value, _map_type_allowed_constraints_.at(odometry_diagnostics.estimator_type.name))) {

    ss << "the constraints '" << req.value.c_str() << "' are not allowed given the current odometry type";

    ROS_WARN_STREAM_THROTTLE(1.0, "[ConstraintManager]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  // try to set the constraints
  if (!setConstraints(req.value)) {

    ss << "the ControlManager could not set the constraints";

    ROS_ERROR_STREAM_THROTTLE(1.0, "[ConstraintManager]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;

  } else {

    ss << "the constraints '" << req.value.c_str() << "' were set";

    ROS_INFO_STREAM_THROTTLE(1.0, "[ConstraintManager]: " << ss.str());

    res.message = ss.str();
    res.success = true;
    return true;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerConstraintManagement() //{ */

void ConstraintManager::timerConstraintManagement(const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerConstraintManagement", _constraint_management_rate_, 0.01, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ContraintManager::timerConstraintManagement", scope_timer_logger_, scope_timer_enabled_);

  auto odometry_diagnostics = mrs_lib::get_mutexed(mutex_odometry_diagnostics_, odometry_diagnostics_);
  auto current_constraints  = mrs_lib::get_mutexed(mutex_current_constraints_, current_constraints_);
  auto last_estimator_type  = mrs_lib::get_mutexed(mutex_last_estimator_type_, last_estimator_type_);

  if (!got_odometry_diagnostics_) {
    ROS_WARN_THROTTLE(1.0, "[ConstraintManager]: can not do constraint management, missing odometry diagnostics!");
    return;
  }

  // | --- automatically set constraints when odometry.type changes -- |
  if (odometry_diagnostics.estimator_type.type != last_estimator_type) {

    ROS_INFO_THROTTLE(1.0, "[ConstraintManager]: the odometry type has changed! %d -> %d", last_estimator_type, odometry_diagnostics.estimator_type.type);

    std::map<std::string, std::string>::iterator it;
    it = _map_type_fallback_constraints_.find(odometry_diagnostics.estimator_type.name);

    if (it == _map_type_fallback_constraints_.end()) {

      ROS_WARN_THROTTLE(1.0, "[ConstraintManager]: the odometry type '%s' was not specified in the constraint_manager's config!",
                        odometry_diagnostics.estimator_type.name.c_str());

    } else {

      // if the current constraints are within the allowed odometry types, do nothing
      if (stringInVector(current_constraints, _map_type_allowed_constraints_.at(odometry_diagnostics.estimator_type.name))) {

        last_estimator_type = odometry_diagnostics.estimator_type.type;

        // else, try to set the fallback constraints
      } else {

        ROS_WARN_THROTTLE(1.0, "[ConstraintManager]: the current constraints '%s' are not within the allowed constraints for '%s'", current_constraints.c_str(),
                          odometry_diagnostics.estimator_type.name.c_str());

        if (setConstraints(it->second)) {

          last_estimator_type = odometry_diagnostics.estimator_type.type;

          ROS_INFO_THROTTLE(1.0, "[ConstraintManager]: constraints set to fallback: '%s'", it->second.c_str());

        } else {

          ROS_WARN_THROTTLE(1.0, "[ConstraintManager]: could not set constraints!");
        }
      }
    }
  }

  mrs_lib::set_mutexed(mutex_last_estimator_type_, last_estimator_type, last_estimator_type_);
}

//}

/* timerDiagnostics() //{ */

void ConstraintManager::timerDiagnostics(const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  if (!got_odometry_diagnostics_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerDiagnostics", _diagnostics_rate_, 0.01, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ContraintManager::timerDiagnostics", scope_timer_logger_, scope_timer_enabled_);

  auto odometry_diagnostics = mrs_lib::get_mutexed(mutex_odometry_diagnostics_, odometry_diagnostics_);
  auto current_constraints  = mrs_lib::get_mutexed(mutex_current_constraints_, current_constraints_);

  mrs_msgs::ConstraintManagerDiagnostics diagnostics;

  diagnostics.stamp        = ros::Time::now();
  diagnostics.current_name = current_constraints;
  diagnostics.loaded       = _constraint_names_;

  // get the available constraints
  {
    std::map<std::string, std::vector<std::string>>::iterator it;
    it = _map_type_allowed_constraints_.find(odometry_diagnostics.estimator_type.name);

    if (it == _map_type_allowed_constraints_.end()) {
      ROS_WARN_THROTTLE(1.0, "[ConstraintManager]: the odometry.type '%s' was not specified in the constraint_manager's config!",
                        odometry_diagnostics.estimator_type.name.c_str());
    } else {
      diagnostics.available = it->second;
    }
  }

  // get the current constraint values
  {
    std::map<std::string, mrs_msgs::DynamicsConstraintsSrvRequest>::iterator it;
    it = _constraints_.find(current_constraints);

    diagnostics.current_values = it->second.constraints;
  }

  try {
    publisher_diagnostics_.publish(diagnostics);
  }
  catch (...) {
    ROS_ERROR("[ConstraintManager]: exception caught during publishing topic %s", publisher_diagnostics_.getTopic().c_str());
  }
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

}  // namespace constraint_manager

}  // namespace mrs_uav_managers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::constraint_manager::ConstraintManager, nodelet::Nodelet)
