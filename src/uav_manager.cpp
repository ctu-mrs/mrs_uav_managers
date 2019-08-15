/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

#include <nav_msgs/Odometry.h>
#include <mrs_msgs/Vec1.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/TrackerStatus.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/BoolStamped.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/State.h>

#include <mrs_lib/Profiler.h>

#include <tf/transform_datatypes.h>

#include <mrs_lib/ParamLoader.h>

//}

namespace mrs_uav_manager
{

namespace uav_manager
{

/* //{ class UavManager */

// state machine
typedef enum
{

  IDLE_STATE,
  FLY_HOME_STATE,
  LANDING_STATE,

} LandingStates_t;

const char *state_names[4] = {

    "IDLING", "FLYING HOME", "LANDING", "EMERGANCY LANDING"};

class UavManager : public nodelet::Nodelet {

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
  void callbackMavrosState(const mavros_msgs::StateConstPtr &msg);
  void callbackAttitudeCommand(const mrs_msgs::AttitudeCommandConstPtr &msg);
  void callbackMaxHeight(const mrs_msgs::Float64StampedConstPtr &msg);
  void callbackHeight(const mrs_msgs::Float64StampedConstPtr &msg);
  void callbackGains(const std_msgs::StringConstPtr &msg);
  void callbackConstraints(const std_msgs::StringConstPtr &msg);
  void callbackMotors(const mrs_msgs::BoolStampedConstPtr &msg);

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
  ros::Subscriber       subscriber_motors;
  mrs_msgs::BoolStamped motors;
  std::mutex            mutex_motors;
  bool                  got_motors = false;

  // checking whether the gains are being set by the gain manager
private:
  ros::Subscriber subscriber_gains;
  ros::Time       gains_last_time;
  bool            gain_manager_required_;

  // checking whether the constraints are being set by the constraint manager
private:
  ros::Subscriber subscriber_constraints;
  ros::Time       constraints_last_time;
  bool            constraint_manager_required_;

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
  ros::Subscriber    subscriber_mavros_state;
  mavros_msgs::State mavros_state;
  std::mutex         mutex_mavros_state;
  bool               got_mavros_state = false;

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
  ros::ServiceClient service_client_switch_controller;
  ros::ServiceClient service_client_land;
  ros::ServiceClient service_client_eland;
  ros::ServiceClient service_client_motors;
  ros::ServiceClient service_client_enabled_callbacks;
  ros::ServiceClient service_client_emergency_goto;
  ros::ServiceClient service_client_arm;
  ros::ServiceClient service_client_pirouette;

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
  bool        takingoff          = false;
  int         number_of_takeoffs = 0;
  std::string after_takeoff_tracker_name_;
  std::string after_takeoff_controller_name_;
  std::string takeoff_tracker_name_;
  std::string takeoff_controller_name_;
  bool        after_takeoff_pirouette_ = false;

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
  bool        landing_disarm_;
  double      hover_thrust_a_;
  double      hover_thrust_b_;

  LandingStates_t current_state_landing  = IDLE_STATE;
  LandingStates_t previous_state_landing = IDLE_STATE;

  void landingTimer(const ros::TimerEvent &event);
  void takeoffTimer(const ros::TimerEvent &event);
  void maxHeightTimer(const ros::TimerEvent &event);
  void flighttimeTimer(const ros::TimerEvent &event);
  void maxthrustTimer(const ros::TimerEvent &event);

private:
  ros::Timer flighttime_timer;
  double     flighttime_timer_rate_;
  double     flighttime_max_time;
  bool       flighttime_timer_enabled_;
  double     flighttime = 0;
  std::mutex mutex_flightime_timer;

private:
  ros::Timer maxthrust_timer;
  bool       maxthrust_timer_enabled_;
  double     maxthrust_timer_rate_;
  double     maxthrust_max_thrust_;

  // | ------------------------ profiler ------------------------ |
private:
  mrs_lib::Profiler *profiler;
  bool               profiler_enabled_ = false;
};

//}

/* //{ changeLandingState() */

void UavManager::changeLandingState(LandingStates_t new_state) {

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
  ROS_INFO("[UavManager]: Switching landing state %s -> %s", state_names[previous_state_landing], state_names[current_state_landing]);
}

//}

/* //{ onInit() */

void UavManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[UavManager]: initializing");

  subscriber_odometry         = nh_.subscribe("odometry_in", 1, &UavManager::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
  subscriber_tracker_status   = nh_.subscribe("tracker_status_in", 1, &UavManager::callbackTrackerStatus, this, ros::TransportHints().tcpNoDelay());
  subscriber_target_attitude  = nh_.subscribe("target_attitude_in", 1, &UavManager::callbackTargetAttitude, this, ros::TransportHints().tcpNoDelay());
  subscriber_mavros_state     = nh_.subscribe("mavros_state_in", 1, &UavManager::callbackMavrosState, this, ros::TransportHints().tcpNoDelay());
  subscriber_attitude_command = nh_.subscribe("attitude_command_in", 1, &UavManager::callbackAttitudeCommand, this, ros::TransportHints().tcpNoDelay());
  subscriber_max_height       = nh_.subscribe("max_height_in", 1, &UavManager::callbackMaxHeight, this, ros::TransportHints().tcpNoDelay());
  subscriber_height           = nh_.subscribe("height_in", 1, &UavManager::callbackHeight, this, ros::TransportHints().tcpNoDelay());
  subscriber_motors           = nh_.subscribe("motors_in", 1, &UavManager::callbackMotors, this, ros::TransportHints().tcpNoDelay());

  subscriber_gains = nh_.subscribe("gains_in", 1, &UavManager::callbackGains, this, ros::TransportHints().tcpNoDelay());
  gains_last_time  = ros::Time(0);

  subscriber_constraints = nh_.subscribe("constraints_in", 1, &UavManager::callbackConstraints, this, ros::TransportHints().tcpNoDelay());
  constraints_last_time  = ros::Time(0);

  service_server_takeoff   = nh_.advertiseService("takeoff_in", &UavManager::callbackTakeoff, this);
  service_server_land      = nh_.advertiseService("land_in", &UavManager::callbackLand, this);
  service_server_land_home = nh_.advertiseService("land_home_in", &UavManager::callbackLandHome, this);

  service_client_takeoff           = nh_.serviceClient<mrs_msgs::Vec1>("takeoff_out");
  service_client_land              = nh_.serviceClient<std_srvs::Trigger>("land_out");
  service_client_eland             = nh_.serviceClient<std_srvs::Trigger>("eland_out");
  service_client_switch_tracker    = nh_.serviceClient<mrs_msgs::String>("switch_tracker_out");
  service_client_switch_controller = nh_.serviceClient<mrs_msgs::String>("switch_controller_out");
  service_client_motors            = nh_.serviceClient<std_srvs::SetBool>("motors_out");
  service_client_emergency_goto    = nh_.serviceClient<mrs_msgs::Vec4>("emergency_goto_out");
  service_client_enabled_callbacks = nh_.serviceClient<std_srvs::SetBool>("enable_callbacks_out");
  service_client_arm               = nh_.serviceClient<std_srvs::SetBool>("arm_out");
  service_client_pirouette         = nh_.serviceClient<std_srvs::Trigger>("pirouette_out");

  mrs_lib::ParamLoader param_loader(nh_, "UavManager");

  param_loader.load_param("enable_profiler", profiler_enabled_);

  param_loader.load_param("null_tracker", null_tracker_name_);

  param_loader.load_param("takeoff/rate", takeoff_timer_rate_);
  param_loader.load_param("takeoff/after_takeoff/tracker", after_takeoff_tracker_name_);
  param_loader.load_param("takeoff/after_takeoff/controller", after_takeoff_controller_name_);
  param_loader.load_param("takeoff/after_takeoff/pirouette", after_takeoff_pirouette_);
  param_loader.load_param("takeoff/during_takeoff/tracker", takeoff_tracker_name_);
  param_loader.load_param("takeoff/during_takeoff/controller", takeoff_controller_name_);
  param_loader.load_param("takeoff/takeoff_height", takeoff_height_);
  param_loader.load_param("takeoff/ground_limit_height", ground_limit_height_);

  param_loader.load_param("landing/rate", landing_timer_rate_);
  param_loader.load_param("landing/landing_tracker", landing_tracker_name_);
  param_loader.load_param("landing/landing_cutoff_height", landing_cutoff_height_);
  param_loader.load_param("landing/landing_cutoff_mass_factor", landing_cutoff_mass_factor_);
  param_loader.load_param("landing/disarm", landing_disarm_);

  param_loader.load_param("uav_mass", uav_mass_);
  param_loader.load_param("g", g_);

  param_loader.load_param("hover_thrust/a", hover_thrust_a_);
  param_loader.load_param("hover_thrust/b", hover_thrust_b_);

  param_loader.load_param("max_height_checking/rate", max_height_checking_rate_);
  param_loader.load_param("max_height_checking/safety_height_offset", max_height_offset_);
  param_loader.load_param("safety_area/max_height", max_height);

  param_loader.load_param("require_gain_manager", gain_manager_required_);
  param_loader.load_param("require_constraint_manager", constraint_manager_required_);

  param_loader.load_param("flight_timer/enabled", flighttime_timer_enabled_);
  param_loader.load_param("flight_timer/rate", flighttime_timer_rate_);
  param_loader.load_param("flight_timer/max_time", flighttime_max_time);

  param_loader.load_param("max_thrust/enabled", maxthrust_timer_enabled_);
  param_loader.load_param("max_thrust/rate", maxthrust_timer_rate_);
  param_loader.load_param("max_thrust/max_thrust", maxthrust_max_thrust_);

  // --------------------------------------------------------------
  // |                    landing state machine                   |
  // --------------------------------------------------------------

  changeLandingState(IDLE_STATE);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler = new mrs_lib::Profiler(nh_, "UavManager", profiler_enabled_);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  landing_timer    = nh_.createTimer(ros::Rate(landing_timer_rate_), &UavManager::landingTimer, this, false, false);
  takeoff_timer    = nh_.createTimer(ros::Rate(takeoff_timer_rate_), &UavManager::takeoffTimer, this, false, false);
  max_height_timer = nh_.createTimer(ros::Rate(max_height_checking_rate_), &UavManager::maxHeightTimer, this);
  flighttime_timer = nh_.createTimer(ros::Rate(flighttime_timer_rate_), &UavManager::flighttimeTimer, this, false, false);
  maxthrust_timer  = nh_.createTimer(ros::Rate(maxthrust_timer_rate_), &UavManager::maxthrustTimer, this, false, false);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[UavManager]: Could not load all parameters!");
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[UavManager]: initilized");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* //{ landingTimer() */

void UavManager::landingTimer(const ros::TimerEvent &event) {

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

      ROS_INFO("[UavManager]: landing");

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
        ROS_INFO("[UavManager]: landing_uav_mass_: %f thrust_mass_estimate: %f", landing_uav_mass_, thrust_mass_estimate);

        // condition for automatic motor turn off
        if ((height < landing_cutoff_height_) && ((thrust_mass_estimate < landing_cutoff_mass_factor_ * landing_uav_mass_) || target_attitude.thrust < 0.01)) {

          if (current_state_landing == LANDING_STATE) {

            mrs_msgs::String switch_tracker_out;
            switch_tracker_out.request.value = null_tracker_name_;
            service_client_switch_tracker.call(switch_tracker_out);

            std_srvs::SetBool enable_callbacks_out;
            enable_callbacks_out.request.data = true;
            service_client_enabled_callbacks.call(enable_callbacks_out);

            if (landing_disarm_) {

              ROS_INFO("[UavManager]: disarming after landing");

              std_srvs::SetBool arm_out;
              arm_out.request.data = false;
              service_client_arm.call(arm_out);
            }

            changeLandingState(IDLE_STATE);

            ROS_INFO("[UavManager]: landing finished");

          } else {  // emergancy landing

            std_srvs::SetBool arm_out;
            arm_out.request.data = false;
            service_client_arm.call(arm_out);

            std_srvs::SetBool enable_callbacks_out;
            enable_callbacks_out.request.data = true;
            service_client_enabled_callbacks.call(enable_callbacks_out);

            changeLandingState(IDLE_STATE);

            ROS_WARN("[UavManager]: emergancy landing finished");
          }

          landing_timer.stop();
        }
      }

    } else {

      ROS_ERROR("[UavManager]: incorrect tracker detected during landing!");
    }
  }
}

//}

/* //{ takeoffTimer() */

void UavManager::takeoffTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("takeoffTimer", takeoff_timer_rate_, 0.004, event);

  if (takingoff) {
    {
      std::scoped_lock lock(mutex_odometry);

      if (fabs(takeoff_height_ - odometry_z) < 0.2) {

        ROS_INFO("[UavManager]: take off finished, switching to %s", after_takeoff_tracker_name_.c_str());

        // if enabled, start the timer for landing after reaching max thrust
        if (maxthrust_timer_enabled_) {
          maxthrust_timer.start();
        }

        mrs_msgs::String switch_tracker_out;
        switch_tracker_out.request.value = after_takeoff_tracker_name_;
        service_client_switch_tracker.call(switch_tracker_out);

        if (switch_tracker_out.response.success == true) {

          ROS_INFO("[UavManager]: switched to %s", after_takeoff_tracker_name_.c_str());

        } else {

          ROS_ERROR("[UavManager]: could not switch to %s: %s", after_takeoff_tracker_name_.c_str(), switch_tracker_out.response.message.c_str());
        }

        mrs_msgs::String switch_controller_out;
        switch_controller_out.request.value = after_takeoff_controller_name_;
        service_client_switch_controller.call(switch_controller_out);

        if (switch_controller_out.response.success == true) {

          ROS_INFO("[UavManager]: switched to %s", after_takeoff_controller_name_.c_str());

        } else {

          ROS_ERROR("[UavManager]: could not switch to %s: %s", after_takeoff_controller_name_.c_str(), switch_controller_out.response.message.c_str());
        }

        if (after_takeoff_pirouette_) {

          std_srvs::Trigger pirouette_out;
          service_client_pirouette.call(pirouette_out);

          if (pirouette_out.response.success == true) {

            ROS_INFO("[UavManager]: initiated after takeoff pirouette");

          } else {

            ROS_INFO("[UavManager]: pirouette not successfull: %s", pirouette_out.response.message.c_str());
          }
        }

        takeoff_timer.stop();
      }
    }
  }
}

//}

/* //{ maxHeightTimer() */

void UavManager::maxHeightTimer(const ros::TimerEvent &event) {

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

        ROS_WARN("[UavManager]: max height exceeded: %f >  %f, triggering safety goto", odometry_z, max_height);

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

          ROS_INFO("[UavManager]: descending");

          fixing_max_height = true;

        } else {

          ROS_ERROR("[UavManager]: goto failed: %s", goto_out.response.message.c_str());
        }
      }

    } else {

      if (odometry_z < max_height) {

        std_srvs::SetBool enable_callbacks_out;
        enable_callbacks_out.request.data = true;
        service_client_enabled_callbacks.call(enable_callbacks_out);

        ROS_WARN("[UavManager]: safety height reached, enabling callbacks");

        fixing_max_height = false;
      }
    }
  }
}

//}

/* //{ flighttimeTimer() */

void UavManager::flighttimeTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  std::scoped_lock lock(mutex_flightime_timer);

  mrs_lib::Routine profiler_routine = profiler->createRoutine("flighttimeTimer", flighttime_timer_rate_, 0.1, event);

  flighttime += 1.0 / flighttime_timer_rate_;

  if (flighttime > flighttime_max_time) {

    flighttime = 0;
    flighttime_timer.stop();

    ROS_INFO("[UavManager]: max flight time achieved, landing");

    mrs_msgs::String switch_tracker_out;
    switch_tracker_out.request.value = landing_tracker_name_;
    service_client_switch_tracker.call(switch_tracker_out);

    std_srvs::Trigger land_out;
    if (switch_tracker_out.response.success == true) {

      service_client_land.call(land_out);

      ros::Duration wait(1.0);
      wait.sleep();

      changeLandingState(LANDING_STATE);

      landing_timer.start();

    } else {

      changeLandingState(IDLE_STATE);
    }
  }
}

//}

/* //{ maxthrustTimer() */

void UavManager::maxthrustTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("maxthrustTimer", maxthrust_timer_rate_, 0.002, event);

  if (attitude_command.thrust >= maxthrust_max_thrust_) {

    maxthrust_timer.stop();

    ROS_INFO("[UavManager]: detecting maximum allowed thrust in attitude_cmd, landing");

    mrs_msgs::String switch_tracker_out;
    switch_tracker_out.request.value = landing_tracker_name_;
    service_client_switch_tracker.call(switch_tracker_out);

    std_srvs::Trigger land_out;
    if (switch_tracker_out.response.success == true) {

      service_client_land.call(land_out);

      ros::Duration wait(1.0);
      wait.sleep();

      changeLandingState(LANDING_STATE);

      landing_timer.start();

    } else {

      changeLandingState(IDLE_STATE);
    }
  }
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | --------------------- topic callbacks -------------------- |

/* //{ callbackTrackerStatus() */

void UavManager::callbackTrackerStatus(const mrs_msgs::TrackerStatusConstPtr &msg) {

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

void UavManager::callbackTargetAttitude(const mavros_msgs::AttitudeTargetConstPtr &msg) {

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

/* //{ callbackMavrosState() */

void UavManager::callbackMavrosState(const mavros_msgs::StateConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackMavrosState");

  {
    std::scoped_lock lock(mutex_mavros_state);
    mavros_state = *msg;
  }

  got_mavros_state = true;
}

//}

/* //{ callbackAttitudeCommand() */

void UavManager::callbackAttitudeCommand(const mrs_msgs::AttitudeCommandConstPtr &msg) {

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

void UavManager::callbackMaxHeight(const mrs_msgs::Float64StampedConstPtr &msg) {

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

void UavManager::callbackHeight(const mrs_msgs::Float64StampedConstPtr &msg) {

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

/* //{ callbackGains() */

void UavManager::callbackGains([[maybe_unused]] const std_msgs::StringConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackGains");

  gains_last_time = ros::Time::now();
}

//}

/* //{ callbackConstraints() */

void UavManager::callbackConstraints([[maybe_unused]] const std_msgs::StringConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackConstraints");

  constraints_last_time = ros::Time::now();
}

//}

/* //{ callbackOdometry() */

void UavManager::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

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

/* //{ callbackMotors() */

void UavManager::callbackMotors(const mrs_msgs::BoolStampedConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackMotors");

  {
    std::scoped_lock lock(mutex_motors);
    motors = *msg;
  }

  got_motors = true;
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackTakeoff() */

bool UavManager::callbackTakeoff([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized)
    return false;

  char message[200];

  if (!got_odometry) {
    sprintf((char *)&message, "Can't takeoff, missing odometry!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  {
    std::scoped_lock lock(mutex_mavros_state);

    if (!got_mavros_state || (ros::Time::now() - mavros_state.header.stamp).toSec() > 5.0) {
      sprintf((char *)&message, "Can't takeoff, missing mavros state!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[UavManager]: %s", message);
      return true;
    }

    if (!mavros_state.armed) {
      sprintf((char *)&message, "Can't takeoff, UAV not armed!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[UavManager]: %s", message);
      return true;
    }

    if (mavros_state.mode.compare(std::string("OFFBOARD")) != 0) {
      sprintf((char *)&message, "Can't takeoff, UAV not in offboard mode!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[UavManager]: %s", message);
      return true;
    }
  }

  if (!got_tracker_status) {
    sprintf((char *)&message, "Can't takeoff, missing tracker status!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_target_attitude) {
    sprintf((char *)&message, "Can't takeoff, missing target attitude!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_max_height) {
    sprintf((char *)&message, "Can't takeoff, missing max height");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_height) {
    sprintf((char *)&message, "Can't takeoff, missing height");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (odometry_z > ground_limit_height_) {
    sprintf((char *)&message, "Can't takeoff, already in the air!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (null_tracker_name_.compare(tracker_status.tracker) != 0) {
    sprintf((char *)&message, "Can't takeoff, need '%s' to be active!", null_tracker_name_.c_str());
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (gain_manager_required_ && (ros::Time::now() - gains_last_time).toSec() > 5.0) {
    sprintf((char *)&message, "Can't takeoff, GainManager is not running!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (constraint_manager_required_ && (ros::Time::now() - constraints_last_time).toSec() > 5.0) {
    sprintf((char *)&message, "Can't takeoff, ConstraintManager is not running!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  {
    std::scoped_lock lock(mutex_motors);

    if (!got_motors || (ros::Time::now() - motors.stamp).toSec() > 1.0 || !motors.data) {
      sprintf((char *)&message, "Can't takeoff, motors are off!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[UavManager]: %s", message);
      return true;
    }
  }

  if (number_of_takeoffs > 0) {

    if (!got_attitude_command) {

      sprintf((char *)&message, "Can't takeoff, missing attitude command!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[UavManager]: %s", message);
      return true;
    }

    std::scoped_lock lock(mutex_attitude_command);

    if (attitude_command.mass_difference > 0.5) {

      sprintf((char *)&message, "Can't takeoff, estimated mass difference is too large!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[UavManager]: %s", message);
      return true;
    }
  }

  ROS_INFO("[UavManager]: taking off");

  mrs_msgs::String switch_controller_out;
  switch_controller_out.request.value = takeoff_controller_name_;
  service_client_switch_controller.call(switch_controller_out);

  mrs_msgs::String switch_tracker_out;
  switch_tracker_out.request.value = takeoff_tracker_name_;
  service_client_switch_tracker.call(switch_tracker_out);

  mrs_msgs::Vec1 takeoff_out;
  takeoff_out.request.goal = takeoff_height_;

  if (switch_tracker_out.response.success == true && switch_controller_out.response.success == true) {

    service_client_takeoff.call(takeoff_out);

    // if the takeoff was not successful, switch to NullTracker
    if (!takeoff_out.response.success) {

      ROS_ERROR("[UavManager]: takeoff was not successful, calling eland");

      std_srvs::Trigger eland_out;
      service_client_eland.call(eland_out);

    } else {

      res.success = takeoff_out.response.success;
      res.message = takeoff_out.response.message;

      // remember the take off point
      {
        std::scoped_lock lock(mutex_odometry);

        takeoff_x = odometry_x;
        takeoff_y = odometry_y;
      }

      {
        // if enabled, start the timer for measuring the flight time
        if (flighttime_timer_enabled_) {

          std::scoped_lock lock(mutex_flightime_timer);

          flighttime_timer.start();
        }
      }

      ROS_INFO("[UavManager]: took off, saving x=%2.2f, y=%2.2f as home position", takeoff_x, takeoff_y);

      takingoff = true;
      number_of_takeoffs++;

      takeoff_timer.start();
    }

    // could not activate the landoff tracker!
  } else if (!switch_tracker_out.response.success) {

    res.success = switch_tracker_out.response.success;
    res.message = switch_tracker_out.response.message;
    ROS_ERROR("[UavManager]: could not activate %s for takeoff", takeoff_tracker_name_.c_str());

    mrs_msgs::String switch_tracker_out;
    switch_tracker_out.request.value = null_tracker_name_;
    service_client_switch_tracker.call(switch_tracker_out);

  } else if (!switch_controller_out.response.success) {

    res.success = switch_controller_out.response.success;
    res.message = switch_controller_out.response.message;
    ROS_ERROR("[UavManager]: could not activate %s for takeoff", takeoff_controller_name_.c_str());

    mrs_msgs::String switch_tracker_out;
    switch_tracker_out.request.value = null_tracker_name_;
    service_client_switch_tracker.call(switch_tracker_out);
  }

  return true;
}

//}

/* //{ callbackLand() */

bool UavManager::callbackLand([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized)
    return false;

  char message[100];

  if (!got_odometry) {
    sprintf((char *)&message, "Can't land, missing odometry!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_tracker_status) {
    sprintf((char *)&message, "Can't land, missing tracker status!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_attitude_command) {
    sprintf((char *)&message, "Can't land, missing attitude command!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  ROS_INFO("[UavManager]: landing");

  flighttime_timer.stop();

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

bool UavManager::callbackLandHome([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized)
    return false;

  char message[100];

  if (!got_odometry) {
    sprintf((char *)&message, "Can't land, missing odometry!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_tracker_status) {
    sprintf((char *)&message, "Can't land, missing tracker status!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (!got_attitude_command) {
    sprintf((char *)&message, "Can't land, missing attitude command!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  if (fixing_max_height) {
    sprintf((char *)&message, "Can't land, descedning to safety height!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[UavManager]: %s", message);
    return true;
  }

  ROS_INFO("[UavManager]: landing on home -> x=%2.2f, y=%2.2f", takeoff_x, takeoff_y);

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
}  // namespace uav_manager

}  // namespace mrs_uav_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_manager::uav_manager::UavManager, nodelet::Nodelet)
