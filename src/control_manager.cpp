/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_msgs/String.h>
#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <mrs_msgs/TrackerStatus.h>
#include <mrs_msgs/ControllerStatus.h>
#include <mrs_msgs/Float64Stamped.h>

#include <mrs_lib/ConvexPolygon.h>
#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Utils.h>

#include <sensor_msgs/Joy.h>

#include <mrs_uav_manager/Controller.h>
#include <mrs_uav_manager/Tracker.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <std_srvs/SetBool.h>

#include <pluginlib/class_loader.h>

#include <nodelet/loader.h>

#include <mutex>
#include <eigen3/Eigen/Eigen>
#include <tf/transform_datatypes.h>

//}

#define STRING_EQUAL 0

namespace mrs_uav_manager
{

  /* //{ class ControlManager */

  class ControlManager : public nodelet::Nodelet {

  public:
    virtual void onInit();

  private:
    ros::NodeHandle nh_;
    bool            is_initialized = false;
    std::string     uav_name_;

  private:
    pluginlib::ClassLoader<mrs_uav_manager::Tracker> *   tracker_loader;
    pluginlib::ClassLoader<mrs_uav_manager::Controller> *controller_loader;

    std::vector<std::string> tracker_names;
    std::vector<std::string> controller_names;

    std::vector<boost::shared_ptr<mrs_uav_manager::Tracker>>    tracker_list;
    std::vector<boost::shared_ptr<mrs_uav_manager::Controller>> controller_list;

    std::string null_tracker_name_;
    std::string hover_tracker_name_;

    std::string failsafe_controller_name_;

    std::mutex mutex_tracker_list;
    std::mutex mutex_controller_list;

    ros::Subscriber    subscriber_odometry;
    nav_msgs::Odometry odometry;
    double             odometry_x;
    double             odometry_y;
    double             odometry_z;
    double             odometry_yaw;
    double             odometry_roll;
    double             odometry_pitch;
    std::mutex         mutex_odometry;
    bool               got_odometry = false;

    ros::Subscriber subscriber_max_height;
    double          max_height;
    bool            got_max_height = false;
    std::mutex      mutex_max_height;
    std::mutex      mutex_min_height;

    int  active_tracker_idx      = 0;
    int  active_controller_idx   = 0;
    int  hover_tracker_idx       = 0;
    int  failsafe_controller_idx = 0;
    bool motors                  = false;

    int status_timer_rate_ = 0;
    int safety_timer_rate_ = 0;

    ros::Publisher publisher_control_output;
    ros::Publisher publisher_position_cmd;
    ros::Publisher publisher_attitude_cmd;
    ros::Publisher publisher_thrust_force;
    ros::Publisher publisher_cmd_odom;
    ros::Publisher publisher_target_attitude;
    ros::Publisher publisher_tracker_status;
    ros::Publisher publisher_controller_status;

    ros::ServiceServer service_switch_tracker;
    ros::ServiceServer service_switch_controller;
    ros::ServiceServer service_hover;
    ros::ServiceServer service_ehover;
    ros::ServiceServer service_motors;
    ros::ServiceServer service_enable_callbacks;
    ros::ServiceServer service_set_constraints;
    ros::ServiceServer service_use_joystick;

    ros::ServiceServer service_goto;
    ros::ServiceServer service_goto_fcu;
    ros::ServiceServer service_goto_relative;
    ros::ServiceServer service_goto_altitude;
    ros::ServiceServer service_set_yaw;
    ros::ServiceServer service_set_yaw_relative;

    ros::ServiceServer service_emergencyGoTo;

    ros::Subscriber subscriber_goto;
    ros::Subscriber subscriber_goto_fcu;
    ros::Subscriber subscriber_goto_relative;
    ros::Subscriber subscriber_goto_altitude;
    ros::Subscriber subscriber_set_yaw;
    ros::Subscriber subscriber_set_yaw_relative;

    mrs_msgs::PositionCommand::ConstPtr last_position_cmd;
    std::mutex                          mutex_last_position_cmd;

    mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd;
    std::mutex                          mutex_last_attitude_cmd;

  private:
    mrs_uav_manager::MotorParams motor_params_;
    double                       g_;

  private:
    double max_tilt_angle_;
    double failsafe_hover_control_error_;
    double failsafe_land_control_error_;

  private:
    mrs_lib::ConvexPolygon *      safety_area_polygon;
    bool                          use_safety_area_;
    double                        min_height;
    mrs_uav_manager::SafetyArea_t safety_area;

    bool   isPointInSafetyArea2d(const double x, const double y);
    bool   isPointInSafetyArea3d(const double x, const double y, const double z);
    double getMinHeight(void);
    double getMaxHeight(void);

  private:
    void callbackOdometry(const nav_msgs::OdometryConstPtr &msg);
    void callbackMaxHeight(const mrs_msgs::Float64StampedConstPtr &msg);

    bool callbackSwitchTracker(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);
    bool callbackSwitchController(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);

    bool callbackGoToService(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res);
    bool callbackGoToFcuService(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res);
    bool callbackGoToRelativeService(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res);
    bool callbackGoToAltitudeService(mrs_msgs::Vec1::Request &req, mrs_msgs::Vec1::Response &res);
    bool callbackSetYawService(mrs_msgs::Vec1::Request &req, mrs_msgs::Vec1::Response &res);
    bool callbackSetYawRelativeService(mrs_msgs::Vec1::Request &req, mrs_msgs::Vec1::Response &res);

    void callbackGoToTopic(const mrs_msgs::TrackerPointStampedConstPtr &msg);
    void callbackGoToFcuTopic(const mrs_msgs::TrackerPointStampedConstPtr &msg);
    void callbackGoToRelativeTopic(const mrs_msgs::TrackerPointStampedConstPtr &msg);
    void callbackGoToAltitudeTopic(const std_msgs::Float64ConstPtr &msg);
    void callbackSetYawTopic(const std_msgs::Float64ConstPtr &msg);
    void callbackSetYawRelativeTopic(const std_msgs::Float64ConstPtr &msg);

    bool callbackEmergencyGoToService(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res);

    bool ehover(std::string &message_out);
    bool callbackHoverService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool callbackEHoverService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool callbackMotors(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    bool callbackEnableCallbacks(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool callbackSetConstraints(mrs_msgs::TrackerConstraints::Request &req, mrs_msgs::TrackerConstraints::Response &res);

  private:
    Eigen::Vector2d rotateVector(const Eigen::Vector2d vector_in, double angle);

  private:
    bool callbacks_enabled = true;

  private:
    ros::Timer status_timer;
    void       statusTimer(const ros::TimerEvent &event);

  private:
    ros::Timer safety_timer;
    void       safetyTimer(const ros::TimerEvent &event);
    bool       running_safety_timer = false;
    double     reseting_odometry    = false;

  private:
    mrs_lib::Profiler *profiler;
    bool               profiler_enabled_ = false;

  private:
    ros::Subscriber subscriber_joystick;
    void            callbackJoystic(const sensor_msgs::Joy &msg);
    bool            callbackUseJoystick([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    ros::Time       joy_last_start_time;
    bool            joy_start_pressed = false;
  };

  //}

  /* //{ onInit() */

  void ControlManager::onInit() {

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    ros::Time::waitForValid();

    ROS_INFO("[ControlManager]: initializing");

    last_attitude_cmd = mrs_msgs::AttitudeCommand::Ptr();
    last_position_cmd = mrs_msgs::PositionCommand::Ptr();

    // --------------------------------------------------------------
    // |                           params                           |
    // --------------------------------------------------------------

    mrs_lib::ParamLoader param_loader(nh_, "ControlManager");

    param_loader.load_param("uav_name", uav_name_);

    param_loader.load_param("enable_profiler", profiler_enabled_);

    param_loader.load_param("status_timer_rate", status_timer_rate_);

    param_loader.load_param("safety/hover_tracker", hover_tracker_name_);
    param_loader.load_param("safety/failsafe_controller", failsafe_controller_name_);

    param_loader.load_param("safety/max_tilt_angle", max_tilt_angle_);
    param_loader.load_param("safety/failsafe_hover_control_error", failsafe_hover_control_error_);
    param_loader.load_param("safety/failsafe_land_control_error", failsafe_land_control_error_);

    param_loader.load_param("safety/timer_rate", safety_timer_rate_);

    param_loader.load_param("hover_thrust/a", motor_params_.hover_thrust_a);
    param_loader.load_param("hover_thrust/b", motor_params_.hover_thrust_b);
    param_loader.load_param("g", g_);

    // --------------------------------------------------------------
    // |                        load trackers                       |
    // --------------------------------------------------------------

    param_loader.load_param("trackers", tracker_names);
    param_loader.load_param("null_tracker", null_tracker_name_);
    tracker_names.insert(tracker_names.begin(), null_tracker_name_);

    tracker_loader = new pluginlib::ClassLoader<mrs_uav_manager::Tracker>("mrs_uav_manager", "mrs_uav_manager::Tracker");

    for (unsigned long i = 0; i < tracker_names.size(); i++) {

      std::string tracker_name = tracker_names[i];

      try {
        ROS_INFO("[ControlManager]: Trying to load tracker %s", tracker_name.c_str());
        { tracker_list.push_back(tracker_loader->createInstance(tracker_name.c_str())); }
      }
      catch (pluginlib::CreateClassException &ex1) {
        ROS_ERROR("[ControlManager]: CreateClassException for tracker %s", tracker_name.c_str());
        ROS_ERROR("[ControlManager]: Error: %s", ex1.what());
        return;
      }
      catch (pluginlib::PluginlibException &ex) {
        ROS_ERROR("[ControlManager]: PluginlibException for tracker %s", tracker_name.c_str());
        ROS_ERROR("[ControlManager]: Error: %s", ex.what());
        return;
      }
    }

    ROS_INFO("[ControlManager]: trackers were loaded");

    for (unsigned long i = 0; i < tracker_list.size(); i++) {

      try {
        ROS_INFO("[ControlManager]: Initializing tracker %d: %s", (int)i, tracker_names[i].c_str());
        tracker_list[i]->initialize(nh_, &safety_area);
      }
      catch (std::runtime_error &ex) {
        ROS_ERROR("[ControlManager]: Exception caught during tracker initialization: %s", ex.what());
      }
    }

    max_tilt_angle_ = (max_tilt_angle_ / 180.0) * 3.141592;

    ROS_INFO("[ControlManager]: trackers were activated");

    // --------------------------------------------------------------
    // |                      load controllers                      |
    // --------------------------------------------------------------

    param_loader.load_param("controllers", controller_names);

    controller_loader = new pluginlib::ClassLoader<mrs_uav_manager::Controller>("mrs_uav_manager", "mrs_uav_manager::Controller");

    for (unsigned long i = 0; i < controller_names.size(); i++) {

      std::string controller_name = controller_names[i];

      try {
        ROS_INFO("[ControlManager]: Loading controller %s", controller_name.c_str());
        controller_list.push_back(controller_loader->createInstance(controller_name.c_str()));
      }
      catch (pluginlib::CreateClassException &ex1) {
        ROS_ERROR("[ControlManager]: CreateClassException for controller %s", controller_name.c_str());
        ROS_ERROR("[ControlManager]: Error: %s", ex1.what());
        return;
      }
      catch (pluginlib::PluginlibException &ex) {
        ROS_ERROR("[ControlManager]: PluginlibException for controller %s", controller_name.c_str());
        ROS_ERROR("[ControlManager]: Error: %s", ex.what());
        return;
      }
    }

    ROS_INFO("[ControlManager]: controllers were loaded");

    for (unsigned long i = 0; i < controller_list.size(); i++) {
      try {
        ROS_INFO("[ControlManager]: Initializing controller %d: %s", (int)i, controller_names[i].c_str());
        controller_list[i]->initialize(nh_, motor_params_);
      }
      catch (std::runtime_error &ex) {
        ROS_ERROR("[ControlManager]: Exception caught during controller initialization: %s", ex.what());
      }
    }

    ROS_INFO("[ControlManager]: controllers were initialized");

    // --------------------------------------------------------------
    // |     check the existance of safety trackers/controllers     |
    // --------------------------------------------------------------

    // check if the hover_tracker is within the loaded trackers
    bool hover_tracker_check = false;
    for (unsigned long i = 0; i < tracker_names.size(); i++) {

      std::string tracker_name = tracker_names[i];

      if (tracker_name.compare(hover_tracker_name_) == 0) {
        hover_tracker_check = true;
        hover_tracker_idx   = i;
        break;
      }
    }
    if (!hover_tracker_check) {
      ROS_ERROR("[ControlManager]: the safety/hover_tracker is not within the loaded trackers");
      ros::shutdown();
    }

    // check if the safety_controller is within the loaded controllers
    bool failsafe_controller_check = false;
    for (unsigned long i = 0; i < controller_names.size(); i++) {

      std::string controller_name = controller_names[i];

      if (controller_name.compare(failsafe_controller_name_) == 0) {
        failsafe_controller_check = true;
        failsafe_controller_idx   = i;
        break;
      }
    }
    if (!failsafe_controller_check) {
      ROS_ERROR("[ControlManager]: the safety/failsafe_controller is not within the loaded controllers");
      ros::shutdown();
    }

    // --------------------------------------------------------------
    // |           active the first controller on the list          |
    // --------------------------------------------------------------

    ROS_INFO("[ControlManager]: Activating the first controller on the list (%s)", controller_names[0].c_str());

    controller_list[active_controller_idx]->activate(last_attitude_cmd);

    motors = false;

    // --------------------------------------------------------------
    // |                         safety area                        |
    // --------------------------------------------------------------

    param_loader.load_param("safety_area/use_safety_area", use_safety_area_);
    param_loader.load_param("safety_area/min_height", min_height);
    param_loader.load_param("safety_area/max_height", max_height);

    if (use_safety_area_) {

      Eigen::MatrixXd safety_area_points = param_loader.load_matrix_dynamic2("safety_area/safety_area", -1, 2);

      try {

        safety_area_polygon = new mrs_lib::ConvexPolygon(safety_area_points);
      }
      catch (mrs_lib::ConvexPolygon::WrongNumberOfVertices) {

        ROS_ERROR("[ControlManager]: Exception caught. Wrong number of vertices was supplied to create the safety area.");
        ros::shutdown();
      }
      catch (mrs_lib::ConvexPolygon::PolygonNotConvexException) {

        ROS_ERROR("[ControlManager]: Exception caught. Polygon supplied to create the safety area is not convex.");
        ros::shutdown();
      }
      catch (mrs_lib::ConvexPolygon::WrongNumberOfColumns) {
        ROS_ERROR("[ControlManager]: Exception caught. Wrong number of columns was supplied to the safety area.");
        ros::shutdown();
      }
    }

    safety_area.use_safety_area       = use_safety_area_;
    safety_area.isPointInSafetyArea2d = boost::bind(&ControlManager::isPointInSafetyArea2d, this, _1, _2);
    safety_area.isPointInSafetyArea3d = boost::bind(&ControlManager::isPointInSafetyArea3d, this, _1, _2, _3);
    safety_area.getMaxHeight          = boost::bind(&ControlManager::getMaxHeight, this);
    safety_area.getMinHeight          = boost::bind(&ControlManager::getMinHeight, this);

    // --------------------------------------------------------------
    // |                          profiler                          |
    // --------------------------------------------------------------

    profiler = new mrs_lib::Profiler(nh_, "ControlManager", profiler_enabled_);

    // --------------------------------------------------------------
    // |                         publishers                         |
    // --------------------------------------------------------------

    publisher_control_output    = nh_.advertise<mavros_msgs::AttitudeTarget>("control_output_out", 1);
    publisher_position_cmd      = nh_.advertise<mrs_msgs::PositionCommand>("position_cmd_out", 1);
    publisher_attitude_cmd      = nh_.advertise<mrs_msgs::AttitudeCommand>("attitude_cmd_out", 1);
    publisher_thrust_force      = nh_.advertise<mrs_msgs::Float64Stamped>("thrust_force_out", 1);
    publisher_cmd_odom          = nh_.advertise<nav_msgs::Odometry>("cmd_odom_out", 1);
    publisher_target_attitude   = nh_.advertise<mavros_msgs::AttitudeTarget>("target_attitude_out", 1);
    publisher_tracker_status    = nh_.advertise<mrs_msgs::TrackerStatus>("tracker_status_out", 1);
    publisher_controller_status = nh_.advertise<mrs_msgs::ControllerStatus>("controller_status_out", 1);

    // --------------------------------------------------------------
    // |                         subscribers                        |
    // --------------------------------------------------------------

    subscriber_odometry   = nh_.subscribe("odometry_in", 1, &ControlManager::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
    subscriber_max_height = nh_.subscribe("max_height_in", 1, &ControlManager::callbackMaxHeight, this, ros::TransportHints().tcpNoDelay());
    subscriber_joystick   = nh_.subscribe("joystick_in", 1, &ControlManager::callbackJoystic, this, ros::TransportHints().tcpNoDelay());

    // | -------------------- general services -------------------- |

    service_switch_tracker    = nh_.advertiseService("switch_tracker_in", &ControlManager::callbackSwitchTracker, this);
    service_switch_controller = nh_.advertiseService("switch_controller_in", &ControlManager::callbackSwitchController, this);
    service_hover             = nh_.advertiseService("hover_in", &ControlManager::callbackHoverService, this);
    service_ehover            = nh_.advertiseService("ehover_in", &ControlManager::callbackEHoverService, this);
    service_motors            = nh_.advertiseService("motors_in", &ControlManager::callbackMotors, this);
    service_enable_callbacks  = nh_.advertiseService("enable_callbacks_in", &ControlManager::callbackEnableCallbacks, this);
    service_set_constraints   = nh_.advertiseService("set_constraints_in", &ControlManager::callbackSetConstraints, this);
    service_use_joystick      = nh_.advertiseService("use_joystick_in", &ControlManager::callbackUseJoystick, this);

    // | ---------------- setpoint command services --------------- |

    service_goto             = nh_.advertiseService("goto_in", &ControlManager::callbackGoToService, this);
    service_goto_fcu         = nh_.advertiseService("goto_fcu_in", &ControlManager::callbackGoToFcuService, this);
    service_goto_relative    = nh_.advertiseService("goto_relative_in", &ControlManager::callbackGoToRelativeService, this);
    service_goto_altitude    = nh_.advertiseService("goto_altitude_in", &ControlManager::callbackGoToAltitudeService, this);
    service_set_yaw          = nh_.advertiseService("set_yaw_in", &ControlManager::callbackSetYawService, this);
    service_set_yaw_relative = nh_.advertiseService("set_yaw_relative_in", &ControlManager::callbackSetYawRelativeService, this);

    // | ----------------- setpoint command topics ---------------- |

    subscriber_goto          = nh_.subscribe("goto_in", 1, &ControlManager::callbackGoToTopic, this, ros::TransportHints().tcpNoDelay());
    subscriber_goto_fcu      = nh_.subscribe("goto_fcu_in", 1, &ControlManager::callbackGoToFcuTopic, this, ros::TransportHints().tcpNoDelay());
    subscriber_goto_relative = nh_.subscribe("goto_relative_in", 1, &ControlManager::callbackGoToRelativeTopic, this, ros::TransportHints().tcpNoDelay());
    subscriber_goto_altitude = nh_.subscribe("goto_altitude_in", 1, &ControlManager::callbackGoToAltitudeTopic, this, ros::TransportHints().tcpNoDelay());
    subscriber_set_yaw       = nh_.subscribe("set_yaw_in", 1, &ControlManager::callbackSetYawTopic, this, ros::TransportHints().tcpNoDelay());
    subscriber_set_yaw_relative =
        nh_.subscribe("set_yaw_relative_in", 1, &ControlManager::callbackSetYawRelativeTopic, this, ros::TransportHints().tcpNoDelay());

    // | --------------------- other services --------------------- |

    service_emergencyGoTo = nh_.advertiseService("emergency_goto_in", &ControlManager::callbackEmergencyGoToService, this);

    // --------------------------------------------------------------
    // |                           timers                           |
    // --------------------------------------------------------------

    status_timer = nh_.createTimer(ros::Rate(status_timer_rate_), &ControlManager::statusTimer, this);
    safety_timer = nh_.createTimer(ros::Rate(safety_timer_rate_), &ControlManager::safetyTimer, this);

    // | ----------------------- finish init ---------------------- |

    if (!param_loader.loaded_successfully()) {
      ROS_ERROR("[ControlManager]: Could not load all parameters!");
      ros::shutdown();
    }

    is_initialized = true;

    ROS_INFO("[ControlManager]: initialized");
  }

  //}

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  /* //{ statusTimer() */

  void ControlManager::statusTimer(const ros::TimerEvent &event) {

    if (!is_initialized)
      return;

    mrs_lib::Routine profiler_routine = profiler->createRoutine("stausTimer", status_timer_rate_, 0.01, event);

    // --------------------------------------------------------------
    // |                publishing the tracker status               |
    // --------------------------------------------------------------

    mrs_msgs::TrackerStatus::Ptr tracker_status_ptr;
    mrs_msgs::TrackerStatus      tracker_status;

    tracker_status_ptr = tracker_list[active_tracker_idx]->getStatus();

    tracker_status = mrs_msgs::TrackerStatus(*tracker_status_ptr);

    tracker_status.tracker = tracker_names[active_tracker_idx];
    tracker_status.stamp   = ros::Time::now();

    try {
      publisher_tracker_status.publish(mrs_msgs::TrackerStatusConstPtr(new mrs_msgs::TrackerStatus(tracker_status)));
    }
    catch (...) {
      ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_tracker_status.getTopic().c_str());
    }

    // --------------------------------------------------------------
    // |              publishing the controller status              |
    // --------------------------------------------------------------

    mrs_msgs::ControllerStatus::Ptr controller_status_ptr;
    mrs_msgs::ControllerStatus      controller_status;

    controller_status_ptr = controller_list[active_controller_idx]->getStatus();

    controller_status = mrs_msgs::ControllerStatus(*controller_status_ptr);

    controller_status.controller = controller_names[active_controller_idx];
    controller_status.stamp      = ros::Time::now();

    try {
      publisher_controller_status.publish(mrs_msgs::ControllerStatusConstPtr(new mrs_msgs::ControllerStatus(controller_status)));
    }
    catch (...) {
      ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_controller_status.getTopic().c_str());
    }
  }

  //}

  /* //{ safetyTimer() */

  void ControlManager::safetyTimer(const ros::TimerEvent &event) {

    mrs_lib::ScopeUnset unset_running(running_safety_timer);

    if (!is_initialized)
      return;

    mrs_lib::Routine profiler_routine = profiler->createRoutine("safetyTimer", safety_timer_rate_, 0.04, event);

    if (!got_odometry || active_tracker_idx <= 0) {
      return;
    }

    if (reseting_odometry) {
      ROS_ERROR("[MpcTracker]: MPC tried to run while reseting odometry");
      return;
    }

    {
      std::scoped_lock lock(mutex_last_position_cmd, mutex_last_attitude_cmd);

      if (!(last_position_cmd != mrs_msgs::PositionCommand::Ptr() && last_attitude_cmd != mrs_msgs::AttitudeCommand::Ptr())) {
        return;
      }
    }

    char message[200];

    double position_error_x;
    double position_error_y;
    double position_error_z;
    {
      std::scoped_lock lock(mutex_last_position_cmd, mutex_odometry);

      position_error_x = last_position_cmd->position.x - odometry_x;
      position_error_y = last_position_cmd->position.y - odometry_y;
      position_error_z = last_position_cmd->position.z - odometry_z;
    }


    double control_error = sqrt(pow(position_error_x, 2) + pow(position_error_y, 2) + pow(position_error_z, 2));

    // --------------------------------------------------------------
    // |   activate failsafe hover for tilt_angle/controller error  |
    // --------------------------------------------------------------

    {
      std::scoped_lock lock(mutex_odometry);

      if (odometry_pitch > max_tilt_angle_ || odometry_roll > max_tilt_angle_ || control_error > failsafe_hover_control_error_) {

        // check if the controller is not active
        if (hover_tracker_idx != active_tracker_idx) {

          ROS_ERROR("[ControlManager]: Activating safety hover: pitch=%f, roll=%f, control_error=%f", odometry_pitch, odometry_roll, control_error);

          std::string message_out;
          ehover(message_out);
        }
      }
    }

    // --------------------------------------------------------------
    // |   activate the failsafe controller in case of large error  |
    // --------------------------------------------------------------

    if (control_error > failsafe_land_control_error_) {

      if (failsafe_controller_idx != active_controller_idx) {

        ROS_ERROR("[ControlManager]: Activating safety land: control_error=%f", control_error);

        try {

          ROS_INFO("[ControlManager]: Activating controller %s", failsafe_controller_name_.c_str());
          {
            std::scoped_lock lock(mutex_last_attitude_cmd, mutex_controller_list);

            controller_list[failsafe_controller_idx]->activate(last_attitude_cmd);
          }
          sprintf((char *)&message, "Controller %s has been activated", failsafe_controller_name_.c_str());
          ROS_INFO("[ControlManager]: %s", message);

          // super important, switch the active controller idx
          try {
            {
              std::scoped_lock lock(mutex_controller_list);

              controller_list[active_controller_idx]->deactivate();
            }
            active_controller_idx = failsafe_controller_idx;
          }
          catch (std::runtime_error &exrun) {
            ROS_ERROR("[ControlManager]: Could not deactivate controller %s", controller_names[active_tracker_idx].c_str());
          }
        }
        catch (std::runtime_error &exrun) {
          ROS_ERROR("[ControlManager]: Error during activation of controller %s", failsafe_controller_name_.c_str());
          ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
        }
      }
    }
  }

  //}

  // --------------------------------------------------------------
  // |                          callbacks                         |
  // --------------------------------------------------------------

  // | --------------------- topic callbacks -------------------- |

  /* //{ callbackOdometry() */

  void ControlManager::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

    if (!is_initialized)
      return;

    if (!got_max_height) {
      ROS_ERROR("[MpcTracker]: the safety timer is in the middle of an iteration, waiting for it to finish");
      return;
    }

    // | -- prepare an OdometryConstPtr for trackers&controllers -- |

    nav_msgs::Odometry::ConstPtr odometry_const_ptr(new nav_msgs::Odometry(*msg));

    // | ----- check for change in odometry frame of reference ---- |

    if (got_odometry) {
      if (odometry.child_frame_id.compare(msg->child_frame_id) != STRING_EQUAL) {

        ROS_INFO("[ControlManager]: detecting switch of odometry frame");

        reseting_odometry = true;

        // we have to stop safety timer, otherwise it will interfere
        safety_timer.stop();
        // wait for the safety timer to stop if its running
        while (running_safety_timer) {
          ROS_INFO("[MpcTracker]: waiting for safety timer to finish");
          ros::Duration wait(0.001);
          wait.sleep();
        }

        tracker_list[active_tracker_idx]->switchOdometrySource(odometry_const_ptr);
      }
    }

    // --------------------------------------------------------------
    // |                      copy the odometry                     |
    // --------------------------------------------------------------

    {
      std::scoped_lock lock(mutex_odometry);

      odometry = *msg;

      odometry_x = odometry.pose.pose.position.x;
      odometry_y = odometry.pose.pose.position.y;
      odometry_z = odometry.pose.pose.position.z;

      // calculate the euler angles
      tf::Quaternion quaternion_odometry;
      quaternionMsgToTF(odometry.pose.pose.orientation, quaternion_odometry);
      tf::Matrix3x3 m(quaternion_odometry);
      m.getRPY(odometry_roll, odometry_pitch, odometry_yaw);
    }

    got_odometry = true;

    // --------------------------------------------------------------
    // |                     Update the trackers                    |
    // --------------------------------------------------------------

    mrs_msgs::PositionCommand::ConstPtr tracker_output_cmd;

    {
      std::scoped_lock lock(mutex_last_position_cmd, mutex_tracker_list);

      try {

        // for each tracker
        for (unsigned int i = 0; i < tracker_list.size(); i++) {

          if ((int)i == active_tracker_idx) {

            // if it is the active one, update and retrieve the command
            tracker_output_cmd = tracker_list[i]->update(odometry_const_ptr);

          } else {

            // if it is not the active one, just update without retrieving the command
            tracker_list[i]->update(odometry_const_ptr);
          }
        }

        if (mrs_msgs::PositionCommand::Ptr() != tracker_output_cmd) {

          last_position_cmd = tracker_output_cmd;

        } else if (active_tracker_idx > 0) {

          ROS_WARN_THROTTLE(1.0, "[ControlManager]: The tracker %s return empty command!", tracker_names[active_tracker_idx].c_str());
          // TODO: switch to failsave tracker, or stop outputting commands
          ROS_ERROR_THROTTLE(1.0, "[ControlManager]: TODO");

        } else if (active_tracker_idx == 0) {

          last_position_cmd = tracker_output_cmd;
        }
      }
      catch (std::runtime_error &exrun) {
        ROS_INFO("[ControlManager]: Exception while updateing trackers.");
        ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
      }
    }

    tf::Quaternion desired_orientation;

    {
      std::scoped_lock lock(mutex_last_position_cmd);

      if (last_position_cmd != mrs_msgs::PositionCommand::Ptr()) {

        // publish the odom topic (position command for debugging, e.g. rviz)
        nav_msgs::Odometry cmd_odom;
        cmd_odom.header.stamp         = ros::Time::now();
        cmd_odom.header.frame_id      = "local_origin";
        cmd_odom.pose.pose.position   = tracker_output_cmd->position;
        cmd_odom.twist.twist.linear.x = tracker_output_cmd->velocity.x;
        cmd_odom.twist.twist.linear.y = tracker_output_cmd->velocity.y;
        cmd_odom.twist.twist.linear.z = tracker_output_cmd->velocity.z;
        desired_orientation           = tf::createQuaternionFromRPY(0, 0, tracker_output_cmd->yaw);
        desired_orientation.normalize();

        if (tracker_output_cmd->use_quat_attitude) {
          cmd_odom.pose.pose.orientation = tracker_output_cmd->attitude;
        } else {
          quaternionTFToMsg(desired_orientation, cmd_odom.pose.pose.orientation);
        }

        try {
          publisher_cmd_odom.publish(nav_msgs::OdometryConstPtr(new nav_msgs::Odometry(cmd_odom)));
        }
        catch (...) {
          ROS_ERROR("Exception caught during publishing topic %s.", publisher_cmd_odom.getTopic().c_str());
        }

        // publish the full command structure
        try {
          publisher_position_cmd.publish(mrs_msgs::PositionCommandConstPtr(last_position_cmd));  // the last_position_cmd is already a ConstPtr
        }
        catch (...) {
          ROS_ERROR("Exception caught during publishing topic %s.", publisher_position_cmd.getTopic().c_str());
        }
      }
    }

    // --------------------------------------------------------------
    // |                   Update the controller                    |
    // --------------------------------------------------------------

    mrs_msgs::AttitudeCommand::ConstPtr controller_output_cmd;

    {
      std::scoped_lock lock(mutex_last_position_cmd, mutex_controller_list);

      if (last_position_cmd != mrs_msgs::PositionCommand::Ptr()) {

        try {
          controller_output_cmd = controller_list[active_controller_idx]->update(odometry_const_ptr, last_position_cmd);
        }
        catch (std::runtime_error &exrun) {
          ROS_INFO("[ControlManager]: Exception while updating the active controller.");
          ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
        }
      }
    }

    // | --------- publish the attitude_cmd for debugging --------- |

    if (controller_output_cmd != mrs_msgs::AttitudeCommand::Ptr()) {
      try {
        publisher_attitude_cmd.publish(mrs_msgs::AttitudeCommandConstPtr(controller_output_cmd));  // the control command is already a ConstPtr
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_attitude_cmd.getTopic().c_str());
      }
    }

    // | ------------ publish the desired thrust force ------------ |

    if (controller_output_cmd != mrs_msgs::AttitudeCommand::Ptr()) {

      mrs_msgs::Float64Stamped thrust_out;
      thrust_out.header.stamp = ros::Time::now();
      thrust_out.value        = (pow((controller_output_cmd->thrust - motor_params_.hover_thrust_b) / motor_params_.hover_thrust_a, 2) / g_) * 10.0;

      try {
        publisher_thrust_force.publish(thrust_out);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_thrust_force.getTopic().c_str());
      }
    }

    // --------------------------------------------------------------
    // |                 Publish the control command                |
    // --------------------------------------------------------------

    mavros_msgs::AttitudeTarget attitude_target;
    attitude_target.header.stamp    = ros::Time::now();
    attitude_target.header.frame_id = "base_link";

    bool should_publish = false;

    if (active_tracker_idx == 0 || !motors) {

      if (!motors) {
        ROS_WARN_THROTTLE(1.0, "[ControlManager]: motors are off");
      } else {
        ROS_WARN_THROTTLE(1.0, "[ControlManager]: NullTracker is active, publishing zeros...");
      }

      desired_orientation = tf::createQuaternionFromRPY(odometry_roll, odometry_pitch, odometry_yaw);
      desired_orientation.normalize();
      quaternionTFToMsg(desired_orientation, attitude_target.orientation);

      attitude_target.thrust = 0.0;

      should_publish = true;

    } else if (active_tracker_idx > 0 && controller_output_cmd == mrs_msgs::AttitudeCommand::Ptr()) {

      ROS_WARN_THROTTLE(1.0, "[ControlManager]: the controller (%s) returned nil command! Not publishing anything...",
                        controller_names[active_controller_idx].c_str());

      // convert the RPY to quaternion
      desired_orientation = tf::createQuaternionFromRPY(odometry_roll, odometry_pitch, odometry_yaw);
      desired_orientation.normalize();
      quaternionTFToMsg(desired_orientation, attitude_target.orientation);

      attitude_target.thrust = 0.0;

      should_publish = true;

    } else if (controller_output_cmd != mrs_msgs::AttitudeCommand::Ptr()) {

      {
        std::scoped_lock lock(mutex_last_attitude_cmd);

        last_attitude_cmd = controller_output_cmd;
      }

      attitude_target.thrust = controller_output_cmd->thrust;

      if (last_attitude_cmd->mode_mask == last_attitude_cmd->MODE_EULER_ATTITUDE) {

        // convert the RPY to quaternion
        desired_orientation = tf::createQuaternionFromRPY(controller_output_cmd->euler_attitude.x, controller_output_cmd->euler_attitude.y,
                                                          controller_output_cmd->euler_attitude.z);
        desired_orientation.normalize();
        quaternionTFToMsg(desired_orientation, attitude_target.orientation);

        attitude_target.body_rate.x = 0.0;
        attitude_target.body_rate.y = 0.0;
        attitude_target.body_rate.z = 0.0;

        attitude_target.type_mask = attitude_target.IGNORE_YAW_RATE | attitude_target.IGNORE_ROLL_RATE | attitude_target.IGNORE_PITCH_RATE;

      } else if (last_attitude_cmd->mode_mask == last_attitude_cmd->MODE_QUATER_ATTITUDE) {

        attitude_target.orientation = last_attitude_cmd->quter_attitude;

        attitude_target.body_rate.x = 0.0;
        attitude_target.body_rate.y = 0.0;
        attitude_target.body_rate.z = 0.0;

        attitude_target.type_mask = attitude_target.IGNORE_YAW_RATE | attitude_target.IGNORE_ROLL_RATE | attitude_target.IGNORE_PITCH_RATE;

      } else if (last_attitude_cmd->mode_mask == last_attitude_cmd->MODE_ATTITUDE_RATE) {

        attitude_target.body_rate.x = last_attitude_cmd->attitude_rate.x;
        attitude_target.body_rate.y = last_attitude_cmd->attitude_rate.y;
        attitude_target.body_rate.z = last_attitude_cmd->attitude_rate.z;

        attitude_target.orientation = last_attitude_cmd->quter_attitude;

        attitude_target.type_mask = attitude_target.IGNORE_ATTITUDE;

        // when controlling with angular rates, PixHawk does not publish the
        // target_attitude topic anymore, thus we need do it here
        try {
          publisher_target_attitude.publish(mavros_msgs::AttitudeTargetConstPtr(new mavros_msgs::AttitudeTarget(attitude_target)));
        }
        catch (...) {
          ROS_ERROR("Exception caught during publishing topic %s.", publisher_control_output.getTopic().c_str());
        }
      }

      should_publish = true;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: not publishing a control command");
    }

    if (should_publish) {

      // test the output
      if (!std::isfinite(attitude_target.orientation.x)) {
        ROS_ERROR("NaN detected in variable \"attitude_target.orientation.x\"!!!");
        return;
      }
      if (!std::isfinite(attitude_target.orientation.y)) {
        ROS_ERROR("NaN detected in variable \"attitude_target.orientation.y\"!!!");
        return;
      }
      if (!std::isfinite(attitude_target.orientation.z)) {
        ROS_ERROR("NaN detected in variable \"attitude_target.orientation.z\"!!!");
        return;
      }
      if (!std::isfinite(attitude_target.orientation.w)) {
        ROS_ERROR("NaN detected in variable \"attitude_target.orientation.w\"!!!");
        return;
      }
      if (!std::isfinite(attitude_target.thrust)) {
        ROS_ERROR("NaN detected in variable \"attitude_target.thrust\"!!!");
        return;
      }

      try {
        publisher_control_output.publish(mavros_msgs::AttitudeTargetConstPtr(new mavros_msgs::AttitudeTarget(attitude_target)));
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_control_output.getTopic().c_str());
      }
    }

    if (reseting_odometry) {

      safety_timer.start();
      reseting_odometry = false;
    }
  }

  //}

  /* callbackMaxHeight() //{ */

  void ControlManager::callbackMaxHeight(const mrs_msgs::Float64StampedConstPtr &msg) {

    if (!is_initialized)
      return;

    mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackMaxHeight");

    {
      std::scoped_lock lock(mutex_max_height);

      got_max_height = true;
      max_height     = msg->value;
    }
  }

  //}

  /* callbackJoystick() //{ */

  void ControlManager::callbackJoystic(const sensor_msgs::Joy &msg) {

    if (!is_initialized)
      return;


    mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackJoy");

    if ((msg.buttons[0] == 1 || msg.buttons[6] == 1) && tracker_names[active_tracker_idx].compare("mrs_trackers/JoyTracker") == 0 &&
        controller_names[active_controller_idx].compare("mrs_controllers/AttitudeController") == 0) {

      mrs_msgs::StringRequest controller_srv;
      controller_srv.value = "mrs_controllers/So3Controller";

      mrs_msgs::StringRequest tracker_srv;
      tracker_srv.value = "mrs_trackers/MpcTracker";

      mrs_msgs::StringResponse response;

      callbackSwitchTracker(tracker_srv, response);
      callbackSwitchController(controller_srv, response);
    }

    if (msg.buttons[7] == 1) {

      joy_last_start_time = ros::Time::now();
      joy_start_pressed   = true;

    } else if (joy_start_pressed && (ros::Time::now() - joy_last_start_time).toSec() > 3.0) {

      joy_start_pressed = false;

      mrs_msgs::StringRequest controller_srv;
      controller_srv.value = "mrs_controllers/AttitudeController";

      mrs_msgs::StringRequest tracker_srv;
      tracker_srv.value = "mrs_trackers/JoyTracker";

      mrs_msgs::StringResponse response;

      callbackSwitchTracker(tracker_srv, response);
      callbackSwitchController(controller_srv, response);
    }
  }

  //}

  // | -------------------- service callbacks ------------------- |

  /* //{ callbackSwitchTracker() */

  bool ControlManager::callbackSwitchTracker(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

    if (!is_initialized)
      return false;

    char message[200];

    if (!got_odometry) {

      sprintf((char *)&message, "Can't switch tracker, missing odometry!");
      ROS_ERROR("[ControlManager]: %s", message);
      res.success = false;
      res.message = message;
      return true;
    }

    int new_tracker_idx = -1;

    for (unsigned int i = 0; i < tracker_names.size(); i++) {
      if (req.value.compare(tracker_names[i]) == 0) {
        new_tracker_idx = i;
      }
    }

    // check if the tracker exists
    if (new_tracker_idx < 0) {

      sprintf((char *)&message, "The tracker %s does not exist!", req.value.c_str());
      ROS_ERROR("[ControlManager]: %s", message);
      res.success = false;
      res.message = message;
      return true;
    }

    // check if the tracker is already active
    if (new_tracker_idx == active_tracker_idx) {

      sprintf((char *)&message, "The tracker %s is already active!", req.value.c_str());
      ROS_ERROR("[ControlManager]: %s", message);
      res.success = true;
      res.message = message;
      return true;
    }

    {
      std::scoped_lock lock(mutex_last_attitude_cmd, mutex_last_position_cmd, mutex_tracker_list);

      try {

        ROS_INFO("[ControlManager]: Activating tracker %s", tracker_names[new_tracker_idx].c_str());

        if (!tracker_list[new_tracker_idx]->activate(last_position_cmd)) {

          sprintf((char *)&message, "Tracker %s was not activated", req.value.c_str());
          ROS_WARN("[ControlManager]: %s", message);
          res.success = false;

        } else {

          sprintf((char *)&message, "Tracker %s has been activated", req.value.c_str());
          ROS_INFO("[ControlManager]: %s", message);
          res.success = true;

          // super important, switch which the active tracker idx
          try {
            tracker_list[active_tracker_idx]->deactivate();

            // if switching from null tracker, activate the active the controller
            if (tracker_names[active_tracker_idx].compare(null_tracker_name_) == 0) {

              controller_list[active_controller_idx]->activate(last_attitude_cmd);

              // if switching to null tracker, deactivate the active controller
            } else if (tracker_names[new_tracker_idx].compare(null_tracker_name_) == 0) {

              controller_list[active_controller_idx]->deactivate();

            }

            active_tracker_idx = new_tracker_idx;
          }
          catch (std::runtime_error &exrun) {
            ROS_ERROR("[ControlManager]: Could not deactivate tracker %s", tracker_names[active_tracker_idx].c_str());
          }
        }
      }
      catch (std::runtime_error &exrun) {
        ROS_ERROR("[ControlManager]: Error during activation of tracker %s", req.value.c_str());
        ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
      }
    }

    res.message = message;

    return true;
  }

  //}

  /* callbackSwitchController() //{ */

  bool ControlManager::callbackSwitchController(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

    char message[200];

    int new_controller_idx = -1;

    for (unsigned int i = 0; i < controller_names.size(); i++) {
      if (req.value.compare(controller_names[i]) == 0) {
        new_controller_idx = i;
      }
    }

    // check if the controller exists
    if (new_controller_idx < 0) {

      sprintf((char *)&message, "The controller %s does not exist!", req.value.c_str());
      ROS_ERROR("[ControlManager]: %s", message);
      res.success = false;
      res.message = message;
      return true;
    }

    // check if the controller is not active
    if (new_controller_idx == active_controller_idx) {

      sprintf((char *)&message, "The controller %s is already active!", req.value.c_str());
      ROS_ERROR("[ControlManager]: %s", message);
      res.success = true;
      res.message = message;
      return true;
    }

    {
      std::scoped_lock lock(mutex_last_attitude_cmd, mutex_controller_list);

      try {

        ROS_INFO("[ControlManager]: Activating controller %s", controller_names[new_controller_idx].c_str());
        if (!controller_list[new_controller_idx]->activate(last_attitude_cmd)) {

          sprintf((char *)&message, "Controller %s was not activated", req.value.c_str());
          ROS_WARN("[ControlManager]: %s", message);
          res.success = false;

        } else {

          sprintf((char *)&message, "Controller %s has been activated", req.value.c_str());
          ROS_INFO("[ControlManager]: %s", message);
          res.success = true;

          // super important, switch which the active controller idx
          try {

            controller_list[active_controller_idx]->deactivate();
            active_controller_idx = new_controller_idx;
          }
          catch (std::runtime_error &exrun) {
            ROS_ERROR("[ControlManager]: Could not deactivate controller %s", controller_names[active_controller_idx].c_str());
          }
        }
      }
      catch (std::runtime_error &exrun) {
        ROS_ERROR("[ControlManager]: Error during activation of controller %s", req.value.c_str());
        ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
      }
    }

    res.message = message;
    return true;
  }

  //}

  /* //{ callbackEHover() */

  bool ControlManager::callbackEHoverService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    if (!is_initialized)
      return false;

    res.success = ehover(res.message);

    return true;
  }

  //}

  /* //{ callbackMotors() */

  bool ControlManager::callbackMotors(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

    if (!is_initialized)
      return false;

    {
      std::scoped_lock lock(mutex_odometry);

      if (!isPointInSafetyArea2d(odometry.pose.pose.position.x, odometry.pose.pose.position.y)) {
        ROS_ERROR("[ControlManager]: Can't turn motors on, the UAV is outside of the safety area!");
        res.message = "the UAV is outside of the safety area";
        res.success = false;
        return true;
      }
    }

    // set 'enable motors' to the desired value
    motors = req.data;

    // if switching motors off, switch to NullTracker
    if (!motors) {
      
      // request
      mrs_msgs::StringRequest tracker_srv;
      tracker_srv.value = null_tracker_name_;

      // response (not used)
      mrs_msgs::StringResponse response;

      callbackSwitchTracker(tracker_srv, response);
    } 

    char message[200];
    sprintf((char *)&message, "Motors: %s", motors ? "ON" : "OFF");
    res.message = message;
    res.success = true;

    ROS_INFO("[ControlManager]: %s", message);

    return true;
  }

  //}

  /* //{ callbackEnableCallbacks() */

  bool ControlManager::callbackEnableCallbacks(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

    if (!is_initialized)
      return false;

    callbacks_enabled = req.data;

    std_srvs::SetBoolRequest req_goto_out;
    req_goto_out = req;

    std_srvs::SetBoolRequest req_enable_callbacks;
    req_enable_callbacks.data = callbacks_enabled;

    {
      std::scoped_lock lock(mutex_tracker_list);

      // disable callbacks of all trackers
      for (unsigned int i = 0; i < tracker_list.size(); i++) {
        tracker_list[i]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));
      }
    }

    char message[200];
    sprintf((char *)&message, "Callbacks: %s", motors ? "enabled" : "disabled");
    res.message = message;
    res.success = true;

    ROS_INFO("[ControlManager]: %s", message);

    return true;
  }

  //}

  /* callbackSetConstraints() //{ */

  bool ControlManager::callbackSetConstraints(mrs_msgs::TrackerConstraints::Request &req, mrs_msgs::TrackerConstraints::Response &res) {

    if (!is_initialized) {
      res.message = "not initialized";
      res.success = false;
      return true;
    }

    mrs_msgs::TrackerConstraintsResponse::ConstPtr tracker_response;

    mrs_msgs::TrackerConstraintsRequest req_constraints;
    req_constraints = req;

    {
      std::scoped_lock lock(mutex_tracker_list);

      // for each tracker
      for (unsigned int i = 0; i < tracker_list.size(); i++) {

        // if it is the active one, update and retrieve the command
        tracker_response =
            tracker_list[i]->setConstraints(mrs_msgs::TrackerConstraintsRequest::ConstPtr(new mrs_msgs::TrackerConstraintsRequest(req_constraints)));
      }
    }

    res.message = "Setting constraints";
    res.success = true;

    return true;
  }

  //}

  // | -------------- setpoint topics and services -------------- |

  /* //{ callbackGoToService() */

  bool ControlManager::callbackGoToService(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res) {

    if (!is_initialized) {
      res.message = "not initialized";
      res.success = false;
      return true;
    }

    if (!callbacks_enabled) {
      ROS_WARN("[ControlManager]: not passing the goto service through, the callbacks are disabled");
      res.message = "callbacks are disabled";
      res.success = false;
      return true;
    }

    {
      std::scoped_lock lock(mutex_last_position_cmd);

      if (!isPointInSafetyArea3d(req.goal[0], req.goal[1], req.goal[2])) {
        ROS_ERROR("[ControlManager]: 'goto' service failed, the point is outside of the safety area!");
        res.message = "the point is outside of the safety area";
        res.success = false;
        return true;
      }
    }

    mrs_msgs::Vec4Response::ConstPtr tracker_response;
    char                             message[200];

    mrs_msgs::Vec4Request req_goto_out;
    req_goto_out.goal = req.goal;

    {
      std::scoped_lock lock(mutex_tracker_list);

      tracker_response = tracker_list[active_tracker_idx]->goTo(mrs_msgs::Vec4Request::ConstPtr(new mrs_msgs::Vec4Request(req_goto_out)));

      if (tracker_response != mrs_msgs::Vec4Response::Ptr()) {
        res = *tracker_response;
      } else {
        sprintf((char *)&message, "The tracker '%s' does not implement 'goto' service!", tracker_names[active_tracker_idx].c_str());
        res.message = message;
        res.success = false;
      }
    }

    return true;
  }

  //}

  /* //{ callbackGoToFcuService() */

  bool ControlManager::callbackGoToFcuService(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res) {

    if (!is_initialized) {
      res.message = "not initialized";
      res.success = false;
      return true;
    }

    if (!callbacks_enabled) {
      ROS_WARN("[ControlManager]: not passing the goto service through, the callbacks are disabled");
      res.message = "callbacks are disabled";
      res.success = false;
      return true;
    }

    mrs_msgs::Vec4Request request;
    Eigen::Vector2d       des(req.goal[0], req.goal[1]);

    {
      std::scoped_lock lock(mutex_odometry);
      // rotate it from the frame of the drone
      des = rotateVector(des, odometry_yaw);

      request.goal[0] = des[0] + odometry_x;
      request.goal[1] = des[1] + odometry_y;
      request.goal[2] = req.goal[2] + odometry_z;
      request.goal[3] = req.goal[3] + odometry_yaw;
    }

    if (!isPointInSafetyArea3d(request.goal[0], request.goal[1], request.goal[2])) {
      ROS_ERROR("[ControlManager]: 'goto_fcu' service failed, the point is outside of the safety area!");
      res.message = "the point is outside of the safety area";
      res.success = false;
      return true;
    }

    mrs_msgs::Vec4Response::ConstPtr tracker_response;
    char                             message[200];

    {
      std::scoped_lock lock(mutex_tracker_list);

      tracker_response = tracker_list[active_tracker_idx]->goTo(mrs_msgs::Vec4Request::ConstPtr(new mrs_msgs::Vec4Request(request)));

      if (tracker_response != mrs_msgs::Vec4Response::Ptr()) {
        res = *tracker_response;
      } else {
        sprintf((char *)&message, "The tracker '%s' does not implement 'goto' service!", tracker_names[active_tracker_idx].c_str());
        res.message = message;
        res.success = false;
      }
    }

    return true;
  }

  //}

  /* //{ callbackGoToFcuTopic() */

  void ControlManager::callbackGoToFcuTopic(const mrs_msgs::TrackerPointStampedConstPtr &msg) {

    if (!is_initialized) {
      return;
    }

    if (!callbacks_enabled) {
      ROS_WARN("[ControlManager]: not passing the goto topic through, the callbacks are disabled");
      return;
    }

    mrs_msgs::TrackerPointStamped request;
    Eigen::Vector2d               des(msg->position.x, msg->position.y);

    {
      std::scoped_lock lock(mutex_odometry);
      // rotate it from the frame of the drone
      des = rotateVector(des, odometry_yaw);

      request.position.x   = des[0] + odometry_x;
      request.position.y   = des[1] + odometry_y;
      request.position.z   = msg->position.z + odometry_z;
      request.position.yaw = msg->position.yaw + odometry_yaw;
    }

    if (!isPointInSafetyArea3d(request.position.x, request.position.y, request.position.z)) {
      ROS_ERROR("[ControlManager]: 'goto_fcu' topic failed, the point is outside of the safety area!");
      return;
    }

    bool tracker_response;

    {
      std::scoped_lock lock(mutex_tracker_list);

      tracker_response = tracker_list[active_tracker_idx]->goTo(mrs_msgs::TrackerPointStamped::ConstPtr(new mrs_msgs::TrackerPointStamped(request)));

      if (!tracker_response) {
        ROS_ERROR("[ControlManager]: The tracker '%s' does not implement 'goto' topic!", tracker_names[active_tracker_idx].c_str());
      }
    }

    return;
  }

  //}

  /* //{ callbackGoToTopic() */

  void ControlManager::callbackGoToTopic(const mrs_msgs::TrackerPointStampedConstPtr &msg) {

    if (!is_initialized)
      return;

    if (!callbacks_enabled) {
      ROS_WARN("[ControlManager]: not passing the goto topic through, the callbacks are disabled");
      return;
    }

    mrs_msgs::TrackerPointStamped request;

    if (msg->header.frame_id.compare("fcu") == STRING_EQUAL) {

      // rotate it from the frame of the drone
      Eigen::Vector2d des(msg->position.x, msg->position.y);
      des = rotateVector(des, odometry_yaw);

      {
        std::scoped_lock lock(mutex_odometry);

        request.position.x   = des[0] + odometry_x;
        request.position.y   = des[1] + odometry_y;
        request.position.z   = msg->position.z + odometry_z;
        request.position.yaw = msg->position.yaw + odometry_yaw;
      }

    } else {

      request.position.x   = msg->position.x;
      request.position.y   = msg->position.y;
      request.position.z   = msg->position.z;
      request.position.yaw = msg->position.yaw;
    }

    {
      std::scoped_lock lock(mutex_last_position_cmd);

      if (!isPointInSafetyArea3d(request.position.x, request.position.y, request.position.z)) {
        ROS_ERROR("[ControlManager]: 'goto' topic failed, the point is outside of the safety area!");
        return;
      }
    }

    bool tracker_response;

    {
      std::scoped_lock lock(mutex_tracker_list);

      tracker_response = tracker_list[active_tracker_idx]->goTo(mrs_msgs::TrackerPointStamped::ConstPtr(new mrs_msgs::TrackerPointStamped(request)));
    }

    if (!tracker_response) {
      ROS_ERROR("[ControlManager]: The tracker '%s' does not implement 'goto' topic!", tracker_names[active_tracker_idx].c_str());
    }
  }

  //}

  /* //{ callbackGoToRelativeService() */

  bool ControlManager::callbackGoToRelativeService(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res) {

    if (!is_initialized) {
      res.message = "not initialized";
      res.success = false;
      return true;
    }

    if (!callbacks_enabled) {
      ROS_WARN("[ControlManager]: not passing the goto_relative service through, the callbacks are disabled");
      res.message = "callbacks are disabled";
      res.success = false;
      return true;
    }

    {
      std::scoped_lock lock(mutex_last_position_cmd, mutex_odometry);

      if (!isPointInSafetyArea3d(last_position_cmd->position.x + req.goal[0], last_position_cmd->position.y + req.goal[1],
                                 last_position_cmd->position.z + req.goal[2])) {
        ROS_ERROR("[ControlManager]: 'goto_relative' service failed, the point is outside of the safety area!");
        res.message = "the point is outside of the safety area";
        res.success = false;
        return true;
      }
    }

    mrs_msgs::Vec4Response::ConstPtr tracker_response;
    char                             message[200];

    mrs_msgs::Vec4Request req_goto_out;
    req_goto_out.goal = req.goal;

    {
      std::scoped_lock lock(mutex_tracker_list);

      tracker_response = tracker_list[active_tracker_idx]->goToRelative(mrs_msgs::Vec4Request::ConstPtr(new mrs_msgs::Vec4Request(req_goto_out)));

      if (tracker_response != mrs_msgs::Vec4Response::Ptr()) {
        res = *tracker_response;
      } else {
        sprintf((char *)&message, "The tracker '%s' does not implement 'goto_relative' service!", tracker_names[active_tracker_idx].c_str());
        res.message = message;
        res.success = false;
      }
    }

    return true;
  }

  //}

  /* //{ callbackGoToRelativeTopic() */

  void ControlManager::callbackGoToRelativeTopic(const mrs_msgs::TrackerPointStampedConstPtr &msg) {

    if (!is_initialized)
      return;

    if (!callbacks_enabled) {
      ROS_WARN("[ControlManager]: not passing the goto_relative topic through, the callbacks are disabled");
      return;
    }

    mrs_msgs::TrackerPointStamped request;

    if (msg->header.frame_id.compare("fcu") == STRING_EQUAL) {

      // rotate it from the frame of the drone
      Eigen::Vector2d des(msg->position.x, msg->position.y);
      des = rotateVector(des, odometry_yaw);

      {
        std::scoped_lock lock(mutex_last_position_cmd, mutex_odometry);

        request.position.x   = des[0] + odometry_x - last_position_cmd->position.x;
        request.position.y   = des[1] + odometry_y - last_position_cmd->position.y;
        request.position.z   = msg->position.z + odometry_z - last_position_cmd->position.z;
        request.position.yaw = msg->position.yaw + odometry_yaw - last_position_cmd->yaw;
      }

    } else {

      request.position.x   = msg->position.x;
      request.position.y   = msg->position.y;
      request.position.z   = msg->position.z;
      request.position.yaw = msg->position.yaw;
    }

    {
      std::scoped_lock lock(mutex_last_position_cmd, mutex_odometry);

      if (!isPointInSafetyArea3d(last_position_cmd->position.x + request.position.x, last_position_cmd->position.y + request.position.y,
                                 last_position_cmd->position.z + request.position.z)) {
        ROS_ERROR("[ControlManager]: 'goto_relative' topic failed, the point is outside of the safety area!");
        return;
      }
    }

    bool tracker_response;

    {
      std::scoped_lock lock(mutex_tracker_list);

      tracker_response = tracker_list[active_tracker_idx]->goToRelative(mrs_msgs::TrackerPointStamped::ConstPtr(new mrs_msgs::TrackerPointStamped(request)));
    }

    if (!tracker_response) {
      ROS_ERROR("[ControlManager]: The tracker '%s' does not implement 'goto_relative' topic!", tracker_names[active_tracker_idx].c_str());
    }
  }

  //}

  /* //{ callbackGoToAltitudeService() */

  bool ControlManager::callbackGoToAltitudeService(mrs_msgs::Vec1::Request &req, mrs_msgs::Vec1::Response &res) {

    if (!is_initialized) {
      res.message = "not initialized";
      res.success = false;
      return true;
    }

    if (!callbacks_enabled) {
      ROS_WARN("[ControlManager]: not passing the goto_altitude service through, the callbacks are disabled");
      res.message = "callbacks are disabled";
      res.success = false;
      return true;
    }

    {
      std::scoped_lock lock(mutex_last_position_cmd, mutex_odometry);

      if (!isPointInSafetyArea3d(last_position_cmd->position.x, last_position_cmd->position.y, req.goal)) {
        ROS_ERROR("[ControlManager]: 'goto_altitude' service failed, the point is outside of the safety area!");
        res.message = "the point is outside of the safety area";
        res.success = false;
        return true;
      }
    }

    mrs_msgs::Vec1Response::ConstPtr tracker_response;
    char                             message[200];

    mrs_msgs::Vec1Request req_goto_out;
    req_goto_out.goal = req.goal;

    {
      std::scoped_lock lock(mutex_tracker_list);

      tracker_response = tracker_list[active_tracker_idx]->goToAltitude(mrs_msgs::Vec1Request::ConstPtr(new mrs_msgs::Vec1Request(req_goto_out)));

      if (tracker_response != mrs_msgs::Vec1Response::Ptr()) {
        res = *tracker_response;
      } else {
        sprintf((char *)&message, "The tracker '%s' does not implement 'goto_altitude' service!", tracker_names[active_tracker_idx].c_str());
        res.message = message;
        res.success = false;
      }
    }

    return true;
  }

  //}

  /* //{ callbackGoToAltitudeTopic() */

  void ControlManager::callbackGoToAltitudeTopic(const std_msgs::Float64ConstPtr &msg) {

    if (!is_initialized)
      return;

    if (!callbacks_enabled) {
      ROS_WARN("[ControlManager]: not passing the goto_altitude topic through, the callbacks are disabled");
      return;
    }

    {
      std::scoped_lock lock(mutex_last_position_cmd, mutex_odometry);

      if (!isPointInSafetyArea3d(last_position_cmd->position.x, last_position_cmd->position.y, msg->data)) {
        ROS_ERROR("[ControlManager]: 'goto_altitude' topic failed, the point is outside of the safety area!");
        return;
      }
    }

    bool tracker_response;

    {
      std::scoped_lock lock(mutex_tracker_list);

      tracker_response = tracker_list[active_tracker_idx]->goToAltitude(std_msgs::Float64::ConstPtr(new std_msgs::Float64(*msg)));
    }

    if (!tracker_response) {
      ROS_ERROR("[ControlManager]: The tracker '%s' does not implement 'goto_altitude' topic!", tracker_names[active_tracker_idx].c_str());
    }
  }

  //}

  /* //{ callbackSetYawService() */

  bool ControlManager::callbackSetYawService(mrs_msgs::Vec1::Request &req, mrs_msgs::Vec1::Response &res) {

    if (!is_initialized) {
      res.message = "not initialized";
      res.success = false;
      return true;
    }

    if (!callbacks_enabled) {
      ROS_WARN("[ControlManager]: not passing the set_yaw service through, the callbacks are disabled");
      res.message = "callbacks are disabled";
      res.success = false;
      return true;
    }

    mrs_msgs::Vec1Response::ConstPtr tracker_response;
    char                             message[200];

    mrs_msgs::Vec1Request req_goto_out;
    req_goto_out.goal = req.goal;

    {
      std::scoped_lock lock(mutex_tracker_list);

      tracker_response = tracker_list[active_tracker_idx]->setYaw(mrs_msgs::Vec1Request::ConstPtr(new mrs_msgs::Vec1Request(req_goto_out)));

      if (tracker_response != mrs_msgs::Vec1Response::Ptr()) {
        res = *tracker_response;
      } else {
        sprintf((char *)&message, "The tracker '%s' does not implement 'set_yaw' service!", tracker_names[active_tracker_idx].c_str());
        res.message = message;
        res.success = false;
      }
    }

    return true;
  }

  //}

  /* //{ callbackSetYawTopic() */

  void ControlManager::callbackSetYawTopic(const std_msgs::Float64ConstPtr &msg) {

    if (!is_initialized)
      return;

    if (!callbacks_enabled) {
      ROS_WARN("[ControlManager]: not passing the set_yaw topic through, the callbacks are disabled");
      return;
    }

    bool tracker_response;

    {
      std::scoped_lock lock(mutex_tracker_list);

      tracker_response = tracker_list[active_tracker_idx]->setYaw(std_msgs::Float64::ConstPtr(new std_msgs::Float64(*msg)));
    }

    if (!tracker_response) {
      ROS_ERROR("[ControlManager]: The tracker '%s' does not implement 'set_yaw' topic!", tracker_names[active_tracker_idx].c_str());
    }
  }

  //}

  /* //{ callbackSetYawRelativeService() */

  bool ControlManager::callbackSetYawRelativeService(mrs_msgs::Vec1::Request &req, mrs_msgs::Vec1::Response &res) {

    if (!is_initialized) {
      res.message = "not initialized";
      res.success = false;
      return true;
    }

    if (!callbacks_enabled) {
      ROS_WARN("[ControlManager]: not passing the set_yaw_relative service through, the callbacks are disabled");
      res.message = "callbacks are disabled";
      res.success = false;
      return true;
    }

    mrs_msgs::Vec1Response::ConstPtr tracker_response;
    char                             message[200];

    mrs_msgs::Vec1Request req_goto_out;
    req_goto_out.goal = req.goal;

    {
      std::scoped_lock lock(mutex_tracker_list);

      tracker_response = tracker_list[active_tracker_idx]->setYawRelative(mrs_msgs::Vec1Request::ConstPtr(new mrs_msgs::Vec1Request(req_goto_out)));

      if (tracker_response != mrs_msgs::Vec1Response::Ptr()) {
        res = *tracker_response;
      } else {
        sprintf((char *)&message, "The tracker '%s' does not implement 'set_yaw_relative' service!", tracker_names[active_tracker_idx].c_str());
        res.message = message;
        res.success = false;
      }
    }

    return true;
  }

  //}

  /* //{ callbackSetYawRelativeTopic() */

  void ControlManager::callbackSetYawRelativeTopic(const std_msgs::Float64ConstPtr &msg) {

    if (!is_initialized)
      return;

    if (!callbacks_enabled) {
      ROS_WARN("[ControlManager]: not passing the set_yaw_relative topic through, the callbacks are disabled");
      return;
    }

    bool tracker_response;

    {
      std::scoped_lock lock(mutex_tracker_list);

      tracker_response = tracker_list[active_tracker_idx]->setYawRelative(std_msgs::Float64::ConstPtr(new std_msgs::Float64(*msg)));
    }

    if (!tracker_response) {
      ROS_ERROR("[ControlManager]: The tracker '%s' does not implement 'set_yaw_relative' topic!", tracker_names[active_tracker_idx].c_str());
    }
  }

  //}

  /* //{ callbackHoverService() */

  bool ControlManager::callbackHoverService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    if (!is_initialized)
      return false;

    std::scoped_lock lock(mutex_tracker_list);

    std_srvs::TriggerResponse::ConstPtr tracker_response;
    char                                message[200];

    std_srvs::TriggerRequest hover_out;

    tracker_response = tracker_list[active_tracker_idx]->hover(std_srvs::TriggerRequest::ConstPtr(new std_srvs::TriggerRequest(hover_out)));

    if (tracker_response != std_srvs::TriggerResponse::Ptr()) {
      res = *tracker_response;
    } else {
      sprintf((char *)&message, "The tracker '%s' does not implement 'goto' service!", tracker_names[active_tracker_idx].c_str());
      res.message = message;
      res.success = false;
    }

    return true;
  }

  //}

  /* callbackUseJoystick() //{ */

  bool ControlManager::callbackUseJoystick([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    char message[400];

    mrs_msgs::StringRequest controller_srv;
    controller_srv.value = "mrs_controllers/AttitudeController";

    mrs_msgs::StringRequest tracker_srv;
    tracker_srv.value = "mrs_trackers/JoyTracker";

    mrs_msgs::StringResponse response;

    callbackSwitchTracker(tracker_srv, response);

    if (!response.success) {

      sprintf((char *)&message, "Switching to JoyTracker was unsuccssfull: %s", response.message.c_str());
      res.success = false;
      res.message = message;

      ROS_ERROR("[ControlManager]: %s", message);

      return true;
    }

    callbackSwitchController(controller_srv, response);

    if (!response.success) {

      sprintf((char *)&message, "Switching to AttitudeController was unsuccssfull: %s", response.message.c_str());
      res.success = false;
      res.message = message;

      // switch back to hover tracker
      tracker_srv.value = hover_tracker_name_;
      callbackSwitchTracker(tracker_srv, response);

      // switch back to safety controller
      controller_srv.value = failsafe_controller_name_;
      callbackSwitchController(controller_srv, response);

      ROS_ERROR("[ControlManager]: %s", message);

      return true;
    }

    sprintf((char *)&message, "Switched to JoyTracker");
    res.success = true;
    res.message = message;

    ROS_INFO("[ControlManager]: %s", message);

    return true;
  }

  //}

  // | --------------------- other services --------------------- |

  /* //{ isInSafetyArea3d() */

  bool ControlManager::isPointInSafetyArea3d(const double x, const double y, const double z) {

    if (!use_safety_area_) {
      return true;
    }

    std::scoped_lock lock(mutex_max_height, mutex_min_height);

    if (safety_area_polygon->isPointIn(x, y) && z >= min_height && z <= max_height) {
      return true;
    }

    return false;
  }

  //}

  /* //{ isInSafetyArea2d() */

  bool ControlManager::isPointInSafetyArea2d(const double x, const double y) {

    if (!use_safety_area_) {
      return true;
    }

    if (safety_area_polygon->isPointIn(x, y)) {
      return true;
    }

    return false;
  }

  //}

  /* //{ getMaxHeight() */

  double ControlManager::getMaxHeight(void) {

    std::scoped_lock lock(mutex_max_height);

    return max_height;
  }

  //}

  /* //{ getMinHeight() */

  double ControlManager::getMinHeight(void) {

    std::scoped_lock lock(mutex_min_height);

    return min_height;
  }

  //}

  /* //{ callbackEmergencyGoToService() */

  bool ControlManager::callbackEmergencyGoToService(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res) {

    if (!is_initialized)
      return false;

    callbacks_enabled = false;

    mrs_msgs::Vec4Response::ConstPtr tracker_response;
    char                             message[200];

    std_srvs::SetBoolRequest req_enable_callbacks;

    mrs_msgs::Vec4Request req_goto_out;
    req_goto_out.goal = req.goal;

    {
      std::scoped_lock lock(mutex_tracker_list);

      // disable callbacks of all trackers
      req_enable_callbacks.data = false;
      for (unsigned int i = 0; i < tracker_list.size(); i++) {
        tracker_list[i]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));
      }

      // enable the callbacks for the active tracker
      req_enable_callbacks.data = true;
      tracker_list[active_tracker_idx]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));

      // call the goto
      tracker_response = tracker_list[active_tracker_idx]->goTo(mrs_msgs::Vec4Request::ConstPtr(new mrs_msgs::Vec4Request(req_goto_out)));

      // disable the callbacks back again
      req_enable_callbacks.data = false;
      tracker_list[active_tracker_idx]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));

      if (tracker_response != mrs_msgs::Vec4Response::Ptr()) {
        res = *tracker_response;
      } else {
        sprintf((char *)&message, "The tracker '%s' does not implement 'goto' service!", tracker_names[active_tracker_idx].c_str());
        res.message = message;
        res.success = false;
      }
    }

    return true;
  }

  //}

  // --------------------------------------------------------------
  // |                          routines                          |
  // --------------------------------------------------------------

  /* //{ ehover() */

  bool ControlManager::ehover(std::string &message_out) {

    if (!is_initialized)
      return false;

    std::scoped_lock lock(mutex_tracker_list, mutex_last_position_cmd, mutex_last_attitude_cmd, mutex_controller_list);

    char message[200];
    bool success = false;

    try {

      ROS_INFO("[ControlManager]: Activating tracker %s", tracker_names[hover_tracker_idx].c_str());
      tracker_list[hover_tracker_idx]->activate(last_position_cmd);
      sprintf((char *)&message, "Tracker %s has been activated", hover_tracker_name_.c_str());
      ROS_INFO("[ControlManager]: %s", message);

      // super important, switch the active tracker idx
      try {

        tracker_list[active_tracker_idx]->deactivate();
        active_tracker_idx = hover_tracker_idx;

        success = true;
      }
      catch (std::runtime_error &exrun) {

        sprintf((char *)&message, "[ControlManager]: Could not deactivate tracker %s", tracker_names[active_tracker_idx].c_str());
        ROS_ERROR("[ControlManager]: %s", message);

        message_out = std::string(message);
        success     = false;
      }
    }
    catch (std::runtime_error &exrun) {

      sprintf((char *)&message, "[ControlManager]: Error during activation of tracker %s", hover_tracker_name_.c_str());
      ROS_ERROR("[ControlManager]: %s", message);
      ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());

      message_out = std::string(message);
      success     = false;
    }

    try {

      ROS_INFO("[ControlManager]: Activating controller %s", controller_names[0].c_str());
      { controller_list[0]->activate(last_attitude_cmd); }
      sprintf((char *)&message, "Controller %s has been activated", controller_names[0].c_str());
      ROS_INFO("[ControlManager]: %s", message);

      // super important, switch which the active controller idx
      try {

        controller_list[active_controller_idx]->deactivate();
        active_controller_idx = 0;

        success = true;
      }
      catch (std::runtime_error &exrun) {

        sprintf((char *)&message, "[ControlManager]: Could not deactivate controller %s", tracker_names[active_tracker_idx].c_str());
        ROS_ERROR("[ControlManager]: %s", message);

        message_out = std::string(message);
        success     = false;
      }
    }
    catch (std::runtime_error &exrun) {

      sprintf((char *)&message, "[ControlManager]: Error during activation of controller %s", hover_tracker_name_.c_str());
      ROS_ERROR("[ControlManager]: %s", message);
      ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());

      message_out = std::string(message);
      success     = false;
    }

    if (success) {
      sprintf((char *)&message, "[ControlManager]: ehover activated.");
      message_out = std::string(message);
    }

    return success;
  }

  //}

  /* rotateVector() //{ */

  Eigen::Vector2d ControlManager::rotateVector(const Eigen::Vector2d vector_in, double angle) {

    Eigen::Rotation2D<double> rot2(angle);

    return rot2.toRotationMatrix() * vector_in;
  }

  //}

}  // namespace mrs_uav_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_manager::ControlManager, nodelet::Nodelet)
