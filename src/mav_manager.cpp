#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <nav_msgs/Odometry.h>
#include <mrs_msgs/Vec1.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/TrackerStatus.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <mrs_msgs/Float64Stamped.h>

#include <mavros_msgs/AttitudeTarget.h>

#include <mrs_lib/Profiler.h>

#include <tf/transform_datatypes.h>

#include <mrs_lib/ParamLoader.h>

namespace mrs_mav_manager
{

  /* //{ class MavManager */

  // state machine
  typedef enum
  {

    IDLE_STATE,
    FLY_HOME_STATE,
    LANDING_STATE,

  } LandingStates_t;

  const char *state_names[3] = {

      "IDLING", "FLYING HOME", "LANDING"};

  class MavManager : public nodelet::Nodelet {

  private:
    ros::NodeHandle nh_;
    bool            is_initialized = false;

  public:
    virtual void onInit();

    bool callbackTakeoff(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool callbackLand(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool callbackLandHome(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void callbackOdometry(const nav_msgs::OdometryConstPtr &msg);
    void callbackTrackerStatus(const mrs_msgs::TrackerStatusConstPtr &msg);
    void callbackTargetAttitude(const mavros_msgs::AttitudeTargetConstPtr &msg);
    void callbackAttitudeCommand(const mrs_msgs::AttitudeCommandConstPtr &msg);
    void callbackMaxHeight(const mrs_msgs::Float64StampedConstPtr &msg);
    void callbackHeight(const mrs_msgs::Float64StampedConstPtr &msg);

    void changeLandingState(LandingStates_t new_state);

  private:
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

  private:
    ros::Timer      max_height_timer;
    ros::Subscriber subscriber_max_height;
    double          max_height;
    std::mutex      mutex_max_height;
    bool            got_max_height    = false;
    bool            fixing_max_height = false;
    int             max_height_checking_rate_;
    double          max_height_offset_;

  private:
    ros::Subscriber subscriber_height;
    double          height;
    std::mutex      mutex_height;
    bool            got_height = false;

  private:
    ros::Subscriber         subscriber_tracker_status;
    bool                    got_tracker_status = false;
    mrs_msgs::TrackerStatus tracker_status;
    std::mutex              mutex_tracker_status;

  private:
    ros::Subscriber             subscriber_target_attitude;
    bool                        got_target_attitude = false;
    mavros_msgs::AttitudeTarget target_attitude;
    std::mutex                  mutex_target_attitude;

  private:
    ros::Subscriber           subscriber_attitude_command;
    bool                      got_attitude_command = false;
    mrs_msgs::AttitudeCommand attitude_command;
    std::mutex                mutex_attitude_command;

  private:
    ros::ServiceServer service_server_takeoff;
    ros::ServiceServer service_server_land;
    ros::ServiceServer service_server_land_home;

    ros::ServiceClient service_client_takeoff;
    ros::ServiceClient service_client_switch_tracker;
    ros::ServiceClient service_client_land;
    ros::ServiceClient service_client_motors;
    ros::ServiceClient service_client_enabled_callbacks;
    ros::ServiceClient service_client_emergency_goto;

    std::mutex mutex_services;

  private:
    double takeoff_x;
    double takeoff_y;

  private:
    double takeoff_height_;
    double ground_limit_height_;

  private:
    std::string null_tracker_name_;

  private:
    ros::Timer  takeoff_timer;
    double      takeoff_timer_rate_;
    bool        takingoff = false;
    std::string after_takeoff_tracker_name_;
    std::string takeoff_tracker_name_;

  private:
    ros::Timer  landing_timer;
    std::string landing_tracker_name_;
    double      landing_cutoff_height_;
    double      landing_cutoff_mass_factor_;
    double      landing_timer_rate_;
    bool        landing = false;
    double      uav_mass_;
    double      g_;
    double      landing_uav_mass_;
    double      hover_thrust_a_;
    double      hover_thrust_b_;

    LandingStates_t current_state_landing  = IDLE_STATE;
    LandingStates_t previous_state_landing = IDLE_STATE;

    void landingTimer(const ros::TimerEvent &event);
    void takeoffTimer(const ros::TimerEvent &event);
    void maxHeightTimer(const ros::TimerEvent &event);

    // | ------------------------ profiler ------------------------ |
  private:
    mrs_lib::Profiler *profiler;
    bool               profiler_enabled_ = false;
  };

  //}

  /* //{ changeLandingState() */

  void MavManager::changeLandingState(LandingStates_t new_state) {

    previous_state_landing = current_state_landing;
    current_state_landing  = new_state;

    switch (current_state_landing) {

      case IDLE_STATE:
        break;
      case FLY_HOME_STATE:
        break;
      case LANDING_STATE: {

        std::scoped_lock lock(mutex_attitude_command);

        landing_uav_mass_ = uav_mass_ + attitude_command.mass_difference;
      }

      break;
    }

    // just for ROS_INFO
    ROS_INFO("[MavManager]: Switching landing state %s -> %s", state_names[previous_state_landing], state_names[current_state_landing]);
  }

  //}

  /* //{ onInit() */

  void MavManager::onInit() {

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    ros::Time::waitForValid();

    ROS_INFO("[MavManager]: initializing");

    subscriber_odometry         = nh_.subscribe("odometry_in", 1, &MavManager::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
    subscriber_tracker_status   = nh_.subscribe("tracker_status_in", 1, &MavManager::callbackTrackerStatus, this, ros::TransportHints().tcpNoDelay());
    subscriber_target_attitude  = nh_.subscribe("target_attitude_in", 1, &MavManager::callbackTargetAttitude, this, ros::TransportHints().tcpNoDelay());
    subscriber_attitude_command = nh_.subscribe("attitude_command_in", 1, &MavManager::callbackAttitudeCommand, this, ros::TransportHints().tcpNoDelay());
    subscriber_max_height       = nh_.subscribe("max_height_in", 1, &MavManager::callbackMaxHeight, this, ros::TransportHints().tcpNoDelay());
    subscriber_height           = nh_.subscribe("height_in", 1, &MavManager::callbackHeight, this, ros::TransportHints().tcpNoDelay());

    service_server_takeoff   = nh_.advertiseService("takeoff_in", &MavManager::callbackTakeoff, this);
    service_server_land      = nh_.advertiseService("land_in", &MavManager::callbackLand, this);
    service_server_land_home = nh_.advertiseService("land_home_in", &MavManager::callbackLandHome, this);

    service_client_takeoff           = nh_.serviceClient<mrs_msgs::Vec1>("takeoff_out");
    service_client_land              = nh_.serviceClient<std_srvs::Trigger>("land_out");
    service_client_switch_tracker    = nh_.serviceClient<mrs_msgs::String>("switch_tracker_out");
    service_client_motors            = nh_.serviceClient<std_srvs::SetBool>("motors_out");
    service_client_emergency_goto    = nh_.serviceClient<mrs_msgs::Vec4>("emergency_goto_out");
    service_client_enabled_callbacks = nh_.serviceClient<std_srvs::SetBool>("enable_callbacks_out");

    mrs_lib::ParamLoader param_loader(nh_, "MavManager");

    param_loader.load_param("enable_profiler", profiler_enabled_);

    param_loader.load_param("null_tracker", null_tracker_name_);

    param_loader.load_param("takeoff/rate", takeoff_timer_rate_);
    param_loader.load_param("takeoff/after_takeoff_tracker", after_takeoff_tracker_name_);
    param_loader.load_param("takeoff/takeoff_tracker", takeoff_tracker_name_);
    param_loader.load_param("takeoff/takeoff_height", takeoff_height_);
    param_loader.load_param("takeoff/ground_limit_height", ground_limit_height_);

    param_loader.load_param("landing/rate", landing_timer_rate_);
    param_loader.load_param("landing/landing_tracker", landing_tracker_name_);
    param_loader.load_param("landing/landing_cutoff_height", landing_cutoff_height_);
    param_loader.load_param("landing/landing_cutoff_mass_factor", landing_cutoff_mass_factor_);

    param_loader.load_param("uav_mass", uav_mass_);
    param_loader.load_param("g", g_);

    param_loader.load_param("hover_thrust/a", hover_thrust_a_);
    param_loader.load_param("hover_thrust/b", hover_thrust_b_);

    param_loader.load_param("max_height_checking/rate", max_height_checking_rate_);
    param_loader.load_param("max_height_checking/safety_height_offset", max_height_offset_);
    param_loader.load_param("safety_area/max_height", max_height);

    // --------------------------------------------------------------
    // |                    landing state machine                   |
    // --------------------------------------------------------------

    changeLandingState(IDLE_STATE);

    // --------------------------------------------------------------
    // |                          profiler                          |
    // --------------------------------------------------------------

    profiler = new mrs_lib::Profiler(nh_, "MavManager", profiler_enabled_);

    // --------------------------------------------------------------
    // |                           timers                           |
    // --------------------------------------------------------------

    landing_timer    = nh_.createTimer(ros::Rate(landing_timer_rate_), &MavManager::landingTimer, this, false, false);
    takeoff_timer    = nh_.createTimer(ros::Rate(takeoff_timer_rate_), &MavManager::takeoffTimer, this, false, false);
    max_height_timer = nh_.createTimer(ros::Rate(max_height_checking_rate_), &MavManager::maxHeightTimer, this);

    // | ----------------------- finish init ---------------------- |

    if (!param_loader.loaded_successfully()) {
      ROS_ERROR("[MavManager]: Could not load all parameters!");
      ros::shutdown();
    }

    is_initialized = true;

    ROS_INFO("[MavManager]: initilized");
  }

  //}

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  /* //{ landingTimer() */

  void MavManager::landingTimer(const ros::TimerEvent &event) {

    if (!is_initialized)
      return;

    mrs_lib::Routine profiler_routine = profiler->createRoutine("landingTimer", landing_timer_rate_, 0.01, event);

    if (current_state_landing == IDLE_STATE) {

      return;

    } else if (current_state_landing == FLY_HOME_STATE) {

      double temp_odom_x, temp_odom_y;

      {
        std::scoped_lock lock(mutex_odometry);

        temp_odom_x = odometry_x;
        temp_odom_y = odometry_y;
      }

      if (sqrt(pow(temp_odom_x - takeoff_x, 2) + pow(temp_odom_y - takeoff_y, 2)) < 0.5) {

        ros::Duration wait(5.0);
        wait.sleep();

        ROS_INFO("[MavManager]: landing");

        mrs_msgs::String switch_tracker_out;
        switch_tracker_out.request.value = landing_tracker_name_;
        service_client_switch_tracker.call(switch_tracker_out);

        std_srvs::Trigger land_out;
        if (switch_tracker_out.response.success == true) {

          service_client_land.call(land_out);

          ros::Duration wait(1.0);
          wait.sleep();

          changeLandingState(LANDING_STATE);

        } else {

          changeLandingState(IDLE_STATE);
        }
      }

    } else if (current_state_landing == LANDING_STATE) {


      if (landing_tracker_name_.compare(tracker_status.tracker) == 0) {

        {
          std::scoped_lock lock(mutex_height);

          // recalculate the mass based on the thrust
          double thrust_mass_estimate = pow((target_attitude.thrust - hover_thrust_b_) / hover_thrust_a_, 2) / g_;
          ROS_INFO("[MavManager]: landing_uav_mass_: %f thrust_mass_estimate: %f", landing_uav_mass_, thrust_mass_estimate);

          if ((height < landing_cutoff_height_) &&
              ((thrust_mass_estimate < landing_cutoff_mass_factor_ * landing_uav_mass_) || target_attitude.thrust < 0.01)) {

            mrs_msgs::String switch_tracker_out;
            switch_tracker_out.request.value = null_tracker_name_;
            service_client_switch_tracker.call(switch_tracker_out);

            std_srvs::SetBool enable_callbacks_out;
            enable_callbacks_out.request.data = true;
            service_client_enabled_callbacks.call(enable_callbacks_out);

            changeLandingState(IDLE_STATE);

            ROS_INFO("[MavManager]: landing finished");

            landing_timer.stop();
          }
        }

      } else {

        ROS_ERROR("[MavManager]: incorrect tracker detected during landing!");
        /* changeLandingState(IDLE_STATE); */
      }
    }
  }

  //}

  /* //{ takeoffTimer() */

  void MavManager::takeoffTimer(const ros::TimerEvent &event) {

    if (!is_initialized)
      return;

    mrs_lib::Routine profiler_routine = profiler->createRoutine("takeoffTimer", takeoff_timer_rate_, 0.004, event);

    if (takingoff) {
      {
        std::scoped_lock lock(mutex_odometry);

        if (fabs(takeoff_height_ - odometry_z) < 0.2) {
          ROS_INFO("[MavManager]: take off finished, switching to %s", after_takeoff_tracker_name_.c_str());

          mrs_msgs::String switch_tracker_out;
          switch_tracker_out.request.value = after_takeoff_tracker_name_;
          service_client_switch_tracker.call(switch_tracker_out);

          if (switch_tracker_out.response.success == true) {

            ROS_INFO("[MavManager]: switched to %s", after_takeoff_tracker_name_.c_str());

          } else {

            ROS_ERROR("[MavManager]: could not switch to %s: %s", after_takeoff_tracker_name_.c_str(), switch_tracker_out.response.message.c_str());
          }

          takeoff_timer.stop();
        }
      }
    }
  }

  //}

  /* //{ maxHeightTimer() */

  void MavManager::maxHeightTimer(const ros::TimerEvent &event) {

    if (!is_initialized)
      return;

    mrs_lib::Routine profiler_routine = profiler->createRoutine("maxHeightTimer", max_height_checking_rate_, 0.004, event);

    if (!got_max_height || !got_odometry) {
      return;
    }

    {
      std::scoped_lock lock(mutex_max_height, mutex_odometry);

      if (!fixing_max_height) {

        if (odometry_z > max_height + 0.25) {

          ROS_WARN("[MavManager]: max height exceeded: %f >  %f, triggering safety goto", odometry_z, max_height);

          // get the current odometry
          double current_horizontal_speed = sqrt(pow(odometry.twist.twist.linear.x, 2) + pow(odometry.twist.twist.linear.y, 2));
          double current_heading          = atan2(odometry.twist.twist.linear.y, odometry.twist.twist.linear.y);

          double horizontal_t_stop    = current_horizontal_speed / 1.0;
          double horizontal_stop_dist = (horizontal_t_stop * current_horizontal_speed) / 2;
          double stop_dist_x          = cos(current_heading) * horizontal_stop_dist;
          double stop_dist_y          = sin(current_heading) * horizontal_stop_dist;

          mrs_msgs::Vec4 goto_out;
          goto_out.request.goal[0] = odometry_x + stop_dist_x;
          goto_out.request.goal[1] = odometry_y + stop_dist_y;
          goto_out.request.goal[2] = max_height - fabs(max_height_offset_);
          goto_out.request.goal[3] = odometry_yaw;

          {
            std::scoped_lock lock(mutex_services);

            service_client_emergency_goto.call(goto_out);
          }

          if (goto_out.response.success == true) {

            ROS_INFO("[MavManager]: descending");

            fixing_max_height = true;

          } else {

            ROS_ERROR("[MavManager]: goto failed: %s", goto_out.response.message.c_str());
          }
        }

      } else {

        if (odometry_z < max_height) {

          std_srvs::SetBool enable_callbacks_out;
          enable_callbacks_out.request.data = true;
          service_client_enabled_callbacks.call(enable_callbacks_out);

          ROS_WARN("[MavManager]: safety height reached, enabling callbacks");

          fixing_max_height = false;
        }
      }
    }
  }

  //}

  // --------------------------------------------------------------
  // |                          callbacks                         |
  // --------------------------------------------------------------

  // | --------------------- topic callbacks -------------------- |

  /* //{ callbackTrackerStatus() */

  void MavManager::callbackTrackerStatus(const mrs_msgs::TrackerStatusConstPtr &msg) {

    if (!is_initialized)
      return;

    mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackTrackerStatus");

    {
      std::scoped_lock lock(mutex_tracker_status);
      tracker_status = *msg;
    }

    got_tracker_status = true;
  }

  //}

  /* //{ callbackTargetAttitude() */

  void MavManager::callbackTargetAttitude(const mavros_msgs::AttitudeTargetConstPtr &msg) {

    if (!is_initialized)
      return;

    mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackTargetAttitude");

    {
      std::scoped_lock lock(mutex_tracker_status);
      target_attitude = *msg;
    }

    got_target_attitude = true;
  }

  //}

  /* //{ callbackAttitudeCommand() */

  void MavManager::callbackAttitudeCommand(const mrs_msgs::AttitudeCommandConstPtr &msg) {

    if (!is_initialized)
      return;

    mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackAttitudeCommand");

    {
      std::scoped_lock lock(mutex_attitude_command);
      attitude_command = *msg;
    }

    got_attitude_command = true;
  }

  //}

  /* //{ callbackMaxHeight() */

  void MavManager::callbackMaxHeight(const mrs_msgs::Float64StampedConstPtr &msg) {

    if (!is_initialized)
      return;

    mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackMaxHeight");

    {
      std::scoped_lock lock(mutex_max_height);
      max_height = msg->value;
    }

    got_max_height = true;
  }

  //}

  /* //{ callbackHeight() */

  void MavManager::callbackHeight(const mrs_msgs::Float64StampedConstPtr &msg) {

    if (!is_initialized)
      return;

    mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackHeight");

    {
      std::scoped_lock lock(mutex_height);
      height = msg->value;
    }

    got_height = true;
  }

  //}

  /* //{ callbackOdometry() */

  void MavManager::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

    if (!is_initialized)
      return;

    mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackOdometry");

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
  }

  //}

  // | -------------------- service callbacks ------------------- |

  /* //{ callbackTakeoff() */

  bool MavManager::callbackTakeoff([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    if (!is_initialized)
      return false;

    char message[100];

    if (!got_odometry) {
      sprintf((char *)&message, "Can't takeoff, missing odometry!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[MavManager]: %s", message);
      return true;
    }

    if (!got_tracker_status) {
      sprintf((char *)&message, "Can't takeoff, missing tracker status!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[MavManager]: %s", message);
      return true;
    }

    if (!got_target_attitude) {
      sprintf((char *)&message, "Can't takeoff, missing target attitude!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[MavManager]: %s", message);
      return true;
    }

    if (!got_max_height) {
      sprintf((char *)&message, "Can't takeoff, missing max height");
      res.message = message;
      res.success = false;
      ROS_ERROR("[MavManager]: %s", message);
      return true;
    }

    if (!got_height) {
      sprintf((char *)&message, "Can't takeoff, missing height");
      res.message = message;
      res.success = false;
      ROS_ERROR("[MavManager]: %s", message);
      return true;
    }

    if (odometry_z > ground_limit_height_) {
      sprintf((char *)&message, "Can't takeoff, already in the air!");
      res.message = message;
      res.success = false;
      ROS_WARN("[MavManager]: %s", message);
      return true;
    }

    if (null_tracker_name_.compare(tracker_status.tracker) != 0) {
      sprintf((char *)&message, "Can't takeoff, need '%s' to be active!", null_tracker_name_.c_str());
      res.message = message;
      res.success = false;
      ROS_WARN("[MavManager]: %s", message);
      return true;
    }

    ROS_INFO("[MavManager]: taking off");

    mrs_msgs::String switch_tracker_out;
    switch_tracker_out.request.value = takeoff_tracker_name_;
    service_client_switch_tracker.call(switch_tracker_out);

    mrs_msgs::Vec1 takeoff_out;
    takeoff_out.request.goal = takeoff_height_;

    if (switch_tracker_out.response.success == true) {

      service_client_takeoff.call(takeoff_out);

      // if the takeoff was not successful, switch to NullTracker
      if (!takeoff_out.response.success) {

        switch_tracker_out.request.value = null_tracker_name_;
        service_client_switch_tracker.call(switch_tracker_out);

      } else {

        res.success = takeoff_out.response.success;
        res.message = takeoff_out.response.message;

        {
          std::scoped_lock lock(mutex_odometry);

          takeoff_x = odometry_x;
          takeoff_y = odometry_y;
        }

        ROS_INFO("[MavManager]: took off, saving x=%2.2f, y=%2.2f as home position", takeoff_x, takeoff_y);

        takingoff = true;

        takeoff_timer.start();
      }

    } else {

      res.success = switch_tracker_out.response.success;
      res.message = switch_tracker_out.response.message;
    }

    return true;
  }

  //}

  /* //{ callbackLand() */

  bool MavManager::callbackLand([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    if (!is_initialized)
      return false;

    char message[100];

    if (!got_odometry) {
      sprintf((char *)&message, "Can't land, missing odometry!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[MavManager]: %s", message);
      return true;
    }

    if (!got_tracker_status) {
      sprintf((char *)&message, "Can't land, missing tracker status!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[MavManager]: %s", message);
      return true;
    }

    if (!got_attitude_command) {
      sprintf((char *)&message, "Can't land, missing attitude command!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[MavManager]: %s", message);
      return true;
    }

    ROS_INFO("[MavManager]: landing");

    mrs_msgs::String switch_tracker_out;
    switch_tracker_out.request.value = landing_tracker_name_;
    service_client_switch_tracker.call(switch_tracker_out);

    std_srvs::Trigger land_out;
    if (switch_tracker_out.response.success == true) {

      service_client_land.call(land_out);

      res.success = land_out.response.success;
      res.message = land_out.response.message;

      ros::Duration wait(1.0);
      wait.sleep();

      changeLandingState(LANDING_STATE);

      landing_timer.start();

    } else {

      res.success = switch_tracker_out.response.success;
      res.message = switch_tracker_out.response.message;
      changeLandingState(IDLE_STATE);
    }

    return true;
  }

  //}

  /* //{ callbackLandHome() */

  bool MavManager::callbackLandHome([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    if (!is_initialized)
      return false;

    char message[100];

    if (!got_odometry) {
      sprintf((char *)&message, "Can't land, missing odometry!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[MavManager]: %s", message);
      return true;
    }

    if (!got_tracker_status) {
      sprintf((char *)&message, "Can't land, missing tracker status!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[MavManager]: %s", message);
      return true;
    }

    if (!got_attitude_command) {
      sprintf((char *)&message, "Can't land, missing attitude command!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[MavManager]: %s", message);
      return true;
    }

    if (fixing_max_height) {
      sprintf((char *)&message, "Can't land, descedning to safety height!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[MavManager]: %s", message);
      return true;
    }

    ROS_INFO("[MavManager]: landing on home -> x=%2.2f, y=%2.2f", takeoff_x, takeoff_y);

    mrs_msgs::Vec4 goto_out;
    {
      std::scoped_lock lock(mutex_odometry);

      goto_out.request.goal[0] = takeoff_x;
      goto_out.request.goal[1] = takeoff_y;
      goto_out.request.goal[2] = odometry_z;
      goto_out.request.goal[3] = odometry_yaw;
    }

    {
      std::scoped_lock lock(mutex_services);

      service_client_emergency_goto.call(goto_out);
    }

    if (goto_out.response.success == true) {

      res.success = goto_out.response.success;
      res.message = "Flying home for landing";

      ros::Duration wait(1.0);
      wait.sleep();

      changeLandingState(FLY_HOME_STATE);

      landing_timer.start();

    } else {

      res.success = goto_out.response.success;
      res.message = goto_out.response.message;
      changeLandingState(IDLE_STATE);
    }

    return true;
  }

  //}
}  // namespace mrs_mav_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mav_manager::MavManager, nodelet::Nodelet)
