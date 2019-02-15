#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/String.h>

#include <mrs_msgs/String.h>
#include <mrs_msgs/OdometryDiag.h>
#include <mrs_msgs/EstimatorType.h>
#include <mrs_msgs/TrackerConstraints.h>
#include <mrs_msgs/TrackerConstraintsRequest.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>

#include <dynamic_reconfigure/ReconfigureRequest.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#define STRING_EQUAL 0

namespace mrs_uav_manager
{

  /* //{ class ConstraintManager */

  class ConstraintManager : public nodelet::Nodelet {

  private:
    ros::NodeHandle nh_;
    bool            is_initialized = false;

  private:
    std::vector<std::string> estimator_type_names_;

    std::vector<std::string>                                   constraint_names_;
    std::map<std::string, mrs_msgs::TrackerConstraintsRequest> constraints;

  private:
    std::map<std::string, std::vector<std::string>> map_type_allowed_constraints;
    std::map<std::string, std::string>              map_type_fallback_constraints;

  public:
    virtual void onInit();
    bool         callbackSetConstraints(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);
    void         callbackOdometryDiagnostics(const mrs_msgs::OdometryDiagConstPtr &msg);

    bool setConstraints(std::string constraints_names);

    bool stringInVector(const std::string &value, const std::vector<std::string> &vector);

  private:
    ros::ServiceServer service_server_set_constraints;

    ros::ServiceClient service_client_set_constraints;

    ros::Publisher publisher_current_constraints;

  private:
    ros::Subscriber        subscriber_odometry_diagnostics;
    bool                   got_odometry_diagnostics = false;
    mrs_msgs::OdometryDiag odometry_diagnostics;
    std::mutex             mutex_odometry_diagnostics;

    // | ------------- constraint management ------------- |

  private:
    mrs_msgs::EstimatorType::_type_type last_estimator_type;

    void       constraintsManagementTimer(const ros::TimerEvent &event);
    ros::Timer constraints_management_timer;

    int rate_;

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

  void ConstraintManager::onInit() {

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    ros::Time::waitForValid();

    ROS_INFO("[ConstraintManager]: initializing");

    // | ------------------------- params ------------------------- |

    mrs_lib::ParamLoader param_loader(nh_, "ConstraintManager");

    param_loader.load_param("enable_profiler", profiler_enabled_);

    param_loader.load_param("constraints", constraint_names_);

    param_loader.load_param("estimator_types", estimator_type_names_);
    param_loader.load_param("rate", rate_);

    std::vector<std::string>::iterator it;

    // loading constraint names
    for (it = constraint_names_.begin(); it != constraint_names_.end(); ++it) {
      ROS_INFO_STREAM("[ConstraintManager]: loading constraints \"" << *it << "\"");

      mrs_msgs::TrackerConstraintsRequest new_constraints;

      param_loader.load_param(*it + "/horizontal/speed", new_constraints.horizontal_speed);
      param_loader.load_param(*it + "/horizontal/acceleration", new_constraints.horizontal_acceleration);
      param_loader.load_param(*it + "/horizontal/jerk", new_constraints.horizontal_jerk);
      param_loader.load_param(*it + "/horizontal/snap", new_constraints.horizontal_snap);

      param_loader.load_param(*it + "/vertical/ascending/speed", new_constraints.vertical_ascending_speed);
      param_loader.load_param(*it + "/vertical/ascending/acceleration", new_constraints.vertical_ascending_acceleration);
      param_loader.load_param(*it + "/vertical/ascending/jerk", new_constraints.vertical_ascending_jerk);
      param_loader.load_param(*it + "/vertical/ascending/snap", new_constraints.vertical_ascending_snap);

      param_loader.load_param(*it + "/vertical/descending/speed", new_constraints.vertical_descending_speed);
      param_loader.load_param(*it + "/vertical/descending/acceleration", new_constraints.vertical_descending_acceleration);
      param_loader.load_param(*it + "/vertical/descending/jerk", new_constraints.vertical_descending_jerk);
      param_loader.load_param(*it + "/vertical/descending/snap", new_constraints.vertical_descending_snap);

      param_loader.load_param(*it + "/yaw/speed", new_constraints.yaw_speed);
      param_loader.load_param(*it + "/yaw/acceleration", new_constraints.yaw_acceleration);
      param_loader.load_param(*it + "/yaw/jerk", new_constraints.yaw_jerk);
      param_loader.load_param(*it + "/yaw/snap", new_constraints.yaw_snap);

      constraints.insert(std::pair<std::string, mrs_msgs::TrackerConstraintsRequest>(*it, new_constraints));
    }

    // loading the allowed constraints lists
    for (it = estimator_type_names_.begin(); it != estimator_type_names_.end(); ++it) {

      std::vector<std::string> temp_vector;
      param_loader.load_param("constraint_management/allowed_constraints/" + *it, temp_vector);

      std::vector<std::string>::iterator it2;
      for (it2 = temp_vector.begin(); it2 != temp_vector.end(); ++it2) {
        if (!stringInVector(*it2, constraint_names_)) {
          ROS_ERROR("[ConstraintManager]: the element '%s' of %s_allowed_constraints is not a valid constraint!", it2->c_str(), it->c_str());
          ros::shutdown();
        }
      }

      map_type_allowed_constraints.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));
    }

    // loading the fallback constraints
    for (it = estimator_type_names_.begin(); it != estimator_type_names_.end(); ++it) {

      std::string temp_str;
      param_loader.load_param("constraint_management/fallback_constraints/" + *it, temp_str);

      if (!stringInVector(temp_str, map_type_allowed_constraints.at(*it))) {
        ROS_ERROR("[ConstraintManager]: the element '%s' of %s_allowed_constraints is not a valid constraint!", temp_str.c_str(), it->c_str());
        ros::shutdown();
      }

      map_type_fallback_constraints.insert(std::pair<std::string, std::string>(*it, temp_str));
    }

    ROS_INFO("[ConstraintManager]: done loading dynamical params");

    current_constraints = "";
    last_estimator_type = -1;

    // | ------------------------ services ------------------------ |

    service_server_set_constraints = nh_.advertiseService("set_constraints_in", &ConstraintManager::callbackSetConstraints, this);

    service_client_set_constraints = nh_.serviceClient<mrs_msgs::TrackerConstraints>("set_constraints_out");

    // | ----------------------- subscribers ---------------------- |
    subscriber_odometry_diagnostics =
        nh_.subscribe("odometry_diagnostics_in", 1, &ConstraintManager::callbackOdometryDiagnostics, this, ros::TransportHints().tcpNoDelay());

    // | ----------------------- publishers ----------------------- |

    publisher_current_constraints = nh_.advertise<std_msgs::String>("current_constraints_out", 1);

    // | ------------------------- timers ------------------------- |

    constraints_management_timer = nh_.createTimer(ros::Rate(rate_), &ConstraintManager::constraintsManagementTimer, this);

    // --------------------------------------------------------------
    // |                          profiler                          |
    // --------------------------------------------------------------

    profiler = new mrs_lib::Profiler(nh_, "ConstraintManager", profiler_enabled_);

    // | ----------------------- finish init ---------------------- |

    if (!param_loader.loaded_successfully()) {
      ROS_ERROR("[ConstraintManager]: Could not load all parameters!");
      ros::shutdown();
    }

    is_initialized = true;

    ROS_INFO("[ConstraintManager]: initilized");
  }

  //}

  // --------------------------------------------------------------
  // |                           methods                          |
  // --------------------------------------------------------------

  /* setConstraints() //{ */

  bool ConstraintManager::setConstraints(std::string constraints_name) {

    std::map<std::string, mrs_msgs::TrackerConstraintsRequest>::iterator it;
    it = constraints.find(constraints_name);

    if (it == constraints.end()) {
      ROS_ERROR("[ConstraintManager]: could not setConstraints(), the constraint name \"%s\" is not on the list", constraints_name.c_str());
      return false;
    }

    mrs_msgs::TrackerConstraints new_constraints;

    new_constraints.request = it->second;

    service_client_set_constraints.call(new_constraints);

    current_constraints = constraints_name;

    return new_constraints.response.success;
  }

  //}

  // --------------------------------------------------------------
  // |                          callbacks                         |
  // --------------------------------------------------------------

  // | --------------------- topic callbacks -------------------- |

  /* //{ callbackOdometryDiagnostics() */

  void ConstraintManager::callbackOdometryDiagnostics(const mrs_msgs::OdometryDiagConstPtr &msg) {

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

  // | -------------------- service callbacks ------------------- |

  /* //{ callbackSetConstraints() */

  bool ConstraintManager::callbackSetConstraints(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

    if (!is_initialized)
      return false;

    char message[200];

    if (!stringInVector(req.value, constraint_names_)) {

      sprintf((char *)&message, "The constraints '%s' do not exist (in the ConstraintManager's config).", req.value.c_str());
      res.message = message;
      res.success = false;
      ROS_ERROR("[ConstraintManager]: %s", message);
      return true;
    }

    if (!stringInVector(req.value, map_type_allowed_constraints.at(odometry_diagnostics.estimator_type.name))) {

      sprintf((char *)&message, "The constraints '%s' are not allowed given the current odometry.type.", req.value.c_str());
      res.message = message;
      res.success = false;
      ROS_ERROR("[ConstraintManager]: %s", message);
      return true;
    }

    // try to set the constraints
    if (!setConstraints(req.value)) {

      res.message = "the control_manager can't set the constraints";
      res.success = false;
      return true;

    } else {

      sprintf((char *)&message, "The constraints '%s' are set.", req.value.c_str());
      res.message = message;
      res.success = true;
      ROS_INFO("[ConstraintManager]: %s", message);
      return true;
    }
  }

  //}

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  /* constraintsManagementTimer() //{ */

  void ConstraintManager::constraintsManagementTimer(const ros::TimerEvent &event) {

    if (!is_initialized)
      return;

    mrs_lib::Routine profiler_routine = profiler->createRoutine("constraintsManagementTimer", rate_, 0.01, event);

    if (!got_odometry_diagnostics) {
      ROS_WARN_THROTTLE(1.0, "[ConstraintManager]: can't do constrint management, missing odometry diagnostics!");
      return;
    }

    // | --- automatically set constraints when odometry.type schanges -- |
    if (odometry_diagnostics.estimator_type.type != last_estimator_type) {

      ROS_ERROR_THROTTLE(1.0, "[ConstraintManager]: the odometry.type has changed! %d -> %d", last_estimator_type, odometry_diagnostics.estimator_type.type);

      std::map<std::string, std::string>::iterator it;

      it = map_type_fallback_constraints.find(odometry_diagnostics.estimator_type.name);

      if (it == map_type_fallback_constraints.end()) {

        ROS_ERROR("[ConstraintManager]: the odometry.type %s was not specified in the constraint_manager's config!",
                  odometry_diagnostics.estimator_type.name.c_str());

      } else {
        if (setConstraints(it->second)) {
          last_estimator_type = odometry_diagnostics.estimator_type.type;
        } else {
          ROS_ERROR_THROTTLE(1.0, "[ConstraintManager]: service call to set constraints failed!");
        }
      }
    }

    std_msgs::String str_out;
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

  bool ConstraintManager::stringInVector(const std::string &value, const std::vector<std::string> &vector) {

    if (std::find(vector.begin(), vector.end(), value) == vector.end()) {
      return false;
    } else {
      return true;
    }
  }

  //}

}  // namespace mrs_uav_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_manager::ConstraintManager, nodelet::Nodelet)
