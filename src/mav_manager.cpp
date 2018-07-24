#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_srvs/SetBool.h>
#include <mrs_msgs/TrackerStatus.h>
#include <mrs_mav_manager/ControlManager.h>

namespace mrs_mav_manager
{

//{ class MavManager

// state machine
typedef enum {

  IDLE_STATE,
  LANDING_STATE,

} States_t;

const char *state_names[3] = {

    "IDLING", "GOING_TO_LAND", "LANDING"};

class MavManager : public nodelet::Nodelet {

public:
  virtual void onInit();
  bool callbackTakeoff(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackLand(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void callbackOdometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackMavrosOdometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackTrackerStatus(const mrs_msgs::TrackerStatusConstPtr &msg);
  void changeLandingState(States_t new_state);

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
  ros::Subscriber    subscriber_mavros_odometry;
  std::mutex         mutex_mavros_odometry;
  nav_msgs::Odometry mavros_odometry;
  double             mavros_odometry_x;
  double             mavros_odometry_y;
  double             mavros_odometry_z;
  double             mavros_odometry_yaw;
  double             mavros_odometry_roll;
  double             mavros_odometry_pitch;
  bool               got_mavros_odometry = false;

private:
  ros::Subscriber         subscriber_tracker_status;
  bool                    got_tracker_status = false;
  mrs_msgs::TrackerStatus tracker_status;
  std::mutex              mutex_tracker_status;

private:
  ros::NodeHandle nh_;

private:
  ros::ServiceServer service_server_takeoff;
  ros::ServiceServer service_server_land;

  ros::ServiceClient service_client_takeoff;
  ros::ServiceClient service_client_switch_tracker;
  ros::ServiceClient service_client_land;
  ros::ServiceClient service_client_motors;

private:
  std::string null_tracker_name_;
  std::string takeoff_tracker_name_;
  std::string landing_tracker_name_;
  double      landing_cutoff_height_;
  double      landing_cutoff_speed_;

private:
  ros::Timer landing_timer;
  bool       landing = false;
  States_t   current_state_landing;
  States_t   previous_state_landing;
  void landingTimer(const ros::TimerEvent &event);
};

//}

//{ changeLandingState()

void MavManager::changeLandingState(States_t new_state) {

  previous_state_landing = current_state_landing;
  current_state_landing  = new_state;

  switch (current_state_landing) {

    case IDLE_STATE:
      break;
    case LANDING_STATE:
      break;
  }

  // just for ROS_INFO
  ROS_DEBUG("[MavManager]: Switching landing state %s -> %s", state_names[previous_state_landing], state_names[current_state_landing]);
}

//}

//{ onInit()

void MavManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  subscriber_odometry        = nh_.subscribe("odometry_in", 1, &MavManager::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
  subscriber_mavros_odometry = nh_.subscribe("mavros_odometry_in", 1, &MavManager::callbackMavrosOdometry, this, ros::TransportHints().tcpNoDelay());
  subscriber_tracker_status  = nh_.subscribe("tracker_status_in", 1, &MavManager::callbackTrackerStatus, this, ros::TransportHints().tcpNoDelay());

  service_server_takeoff = nh_.advertiseService("takeoff_in", &MavManager::callbackTakeoff, this);
  service_server_land    = nh_.advertiseService("land_in", &MavManager::callbackLand, this);

  service_client_takeoff        = nh_.serviceClient<std_srvs::Trigger>("takeoff_out");
  service_client_land           = nh_.serviceClient<std_srvs::Trigger>("land_out");
  service_client_switch_tracker = nh_.serviceClient<mrs_msgs::SwitchTracker>("switch_tracker_out");
  service_client_motors         = nh_.serviceClient<std_srvs::SetBool>("motors_out");

  nh_.getParam("null_tracker", null_tracker_name_);
  nh_.getParam("landoff/landing_tracker", landing_tracker_name_);
  nh_.getParam("landoff/takeoff_tracker", takeoff_tracker_name_);

  nh_.param("landoff/landing_cutoff_height", landing_cutoff_height_, -1.0);
  nh_.param("landoff/landing_cutoff_speed", landing_cutoff_speed_, -1.0);

  if (landing_cutoff_height_ < 0) {
    ROS_ERROR("[MavManager]: landoff/landing_cutoff_height was not specified!");
    ros::shutdown();
  }

  if (landing_cutoff_speed_ < 0) {
    ROS_ERROR("[MavManager]: landoff/landing_cutoff_speed was not specified!");
    ros::shutdown();
  }

  // --------------------------------------------------------------
  // |                    landing state machine                   |
  // --------------------------------------------------------------

  changeLandingState(IDLE_STATE);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  landing_timer = nh_.createTimer(ros::Rate(10), &MavManager::landingTimer, this);

  ROS_INFO("[MavManager]: initilized");
}

//}

//{ landingTimer()

void MavManager::landingTimer(const ros::TimerEvent &event) {

  if (current_state_landing == IDLE_STATE) {
    return;
  } else if (current_state_landing == LANDING_STATE) {
    if (landing_tracker_name_.compare(tracker_status.tracker) == 0) {

      mutex_odometry.lock();
      mutex_mavros_odometry.lock();
      {
        if ((odometry_z < landing_cutoff_height_) && (mavros_odometry.twist.twist.linear.z > -landing_cutoff_speed_)) {

          std_srvs::SetBool motors_out;
          motors_out.request.data = false;
          service_client_motors.call(motors_out);

          mrs_msgs::SwitchTracker switch_tracker_out;
          switch_tracker_out.request.tracker = null_tracker_name_;
          service_client_switch_tracker.call(switch_tracker_out);

          changeLandingState(IDLE_STATE);

          ROS_INFO("[MavManager]: landing finished, switching motors off");
        }
      }
      mutex_mavros_odometry.unlock();
      mutex_odometry.unlock();

    } else {
      changeLandingState(IDLE_STATE);
    }
  }
}

//}

//{ callbackTrackerStatus()

void MavManager::callbackTrackerStatus(const mrs_msgs::TrackerStatusConstPtr &msg) {

  mutex_tracker_status.lock();
  { tracker_status = *msg; }
  mutex_tracker_status.unlock();

  got_tracker_status = true;
}

//}

//{ callbackOdometry()

void MavManager::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

  mutex_odometry.lock();
  {
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
  mutex_odometry.unlock();

  got_odometry = true;
}

//}

//{ callbackMavrosOdometry()

void MavManager::callbackMavrosOdometry(const nav_msgs::OdometryConstPtr &msg) {

  mutex_mavros_odometry.lock();
  {
    mavros_odometry = *msg;

    mavros_odometry_x = mavros_odometry.pose.pose.position.x;
    mavros_odometry_y = mavros_odometry.pose.pose.position.y;
    mavros_odometry_z = mavros_odometry.pose.pose.position.z;

    // calculate the euler angles
    tf::Quaternion quaternion_odometry;
    quaternionMsgToTF(mavros_odometry.pose.pose.orientation, quaternion_odometry);
    tf::Matrix3x3 m(quaternion_odometry);
    m.getRPY(odometry_roll, odometry_pitch, odometry_yaw);
  }
  mutex_mavros_odometry.unlock();

  got_mavros_odometry = true;
}

//}

//{ callbackTakeoff()

bool MavManager::callbackTakeoff(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  char message[100];

  if (!(got_odometry && got_mavros_odometry)) {
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

  if (odometry_z > 0.5) {
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

  mrs_msgs::SwitchTracker switch_tracker_out;
  switch_tracker_out.request.tracker = takeoff_tracker_name_;
  service_client_switch_tracker.call(switch_tracker_out);

  std_srvs::Trigger takeoff_out;

  if (switch_tracker_out.response.success == true) {

    service_client_takeoff.call(takeoff_out);

    res.success = takeoff_out.response.success;
    res.message = takeoff_out.response.message;

  } else {

    res.success = switch_tracker_out.response.success;
    res.message = switch_tracker_out.response.message;
  }

  return true;
}

//}

//{ callbackLand()

bool MavManager::callbackLand(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  char message[100];

  if (!(got_odometry && got_mavros_odometry)) {
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

  ROS_INFO("[MavManager]: landing");

  mrs_msgs::SwitchTracker switch_tracker_out;
  switch_tracker_out.request.tracker = landing_tracker_name_;
  service_client_switch_tracker.call(switch_tracker_out);

  std_srvs::Trigger land_out;
  if (switch_tracker_out.response.success == true) {

    service_client_land.call(land_out);

    res.success = land_out.response.success;
    res.message = land_out.response.message;

    changeLandingState(LANDING_STATE);
  } else {

    res.success = switch_tracker_out.response.success;
    res.message = switch_tracker_out.response.message;
    changeLandingState(IDLE_STATE);
  }

  return true;
}

//}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mav_manager::MavManager, nodelet::Nodelet)
