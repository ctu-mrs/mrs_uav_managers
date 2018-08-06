#include <ros/ros.h>

#include <mrs_mav_manager/Tracker.h>

namespace mrs_mav_manager
{

//{ class NullTracker

class NullTracker : public mrs_mav_manager::Tracker {

public:
  NullTracker(void);

  virtual void initialize(const ros::NodeHandle &parent_nh);
  virtual bool activate(const mrs_msgs::PositionCommand::ConstPtr &cmd);
  virtual void deactivate(void);

  virtual const mrs_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  virtual const mrs_msgs::TrackerStatus::Ptr        getStatus();
  virtual const std_srvs::SetBoolResponse::ConstPtr enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd);

  virtual const mrs_msgs::Vec4Response::ConstPtr goTo(const mrs_msgs::Vec4Request::ConstPtr &cmd);
  virtual const mrs_msgs::Vec4Response::ConstPtr goToRelative(const mrs_msgs::Vec4Request::ConstPtr &cmd);
  virtual const mrs_msgs::Vec1Response::ConstPtr goToAltitude(const mrs_msgs::Vec1Request::ConstPtr &cmd);
  virtual const mrs_msgs::Vec1Response::ConstPtr setYaw(const mrs_msgs::Vec1Request::ConstPtr &cmd);
  virtual const mrs_msgs::Vec1Response::ConstPtr setYawRelative(const mrs_msgs::Vec1Request::ConstPtr &cmd);

  virtual bool goTo(const mrs_msgs::TrackerPointStampedConstPtr &msg);
  virtual bool goToRelative(const mrs_msgs::TrackerPointStampedConstPtr &msg);
  virtual bool goToAltitude(const std_msgs::Float64ConstPtr &msg);
  virtual bool setYaw(const std_msgs::Float64ConstPtr &msg);
  virtual bool setYawRelative(const std_msgs::Float64ConstPtr &msg);

  virtual const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd);

private:
  ros::NodeHandle nh_;
  bool            is_active      = false;
  bool            is_initialized = false;
  bool            callbacks_enabled = false;
};

NullTracker::NullTracker(void) {
}

//}

// | ------------------- trackers interface ------------------- |

//{ initialize()

void NullTracker::initialize(const ros::NodeHandle &parent_nh) {

  ros::NodeHandle nh_(parent_nh, "null_tracker");

  ros::Time::waitForValid();

  is_initialized = true;

  ROS_INFO("[NullTracker]: initialized");
}

//}

//{ activate()

bool NullTracker::activate(const mrs_msgs::PositionCommand::ConstPtr &cmd) {

  ROS_INFO("[NullTracker]: activated");
  is_active = true;
  return true;
}

//}

//{ deactivate()

void NullTracker::deactivate(void) {

  ROS_INFO("[NullTracker]: deactivated");
  is_active = false;
}

//}

//{ update()

const mrs_msgs::PositionCommand::ConstPtr NullTracker::update(const nav_msgs::Odometry::ConstPtr &msg) {

  return mrs_msgs::PositionCommand::Ptr();
}

//}

//{ getStatus()

const mrs_msgs::TrackerStatus::Ptr NullTracker::getStatus() {

  if (is_initialized) {

    mrs_msgs::TrackerStatus::Ptr tracker_status(new mrs_msgs::TrackerStatus);

    if (is_active) {
      tracker_status->active = mrs_msgs::TrackerStatus::ACTIVE;
    } else {
      tracker_status->active = mrs_msgs::TrackerStatus::NONACTIVE;
    }

    return tracker_status;
  } else {

    return mrs_msgs::TrackerStatus::Ptr();
  }
}

//}

//{ enableCallbacks()

const std_srvs::SetBoolResponse::ConstPtr NullTracker::enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd) {

  char                      message[100];
  std_srvs::SetBoolResponse res;

  if (cmd->data != callbacks_enabled) {

    callbacks_enabled = cmd->data;

    sprintf((char *)&message, "Callbacks %s", callbacks_enabled ? "enabled" : "disabled");

    ROS_INFO("[MpcTracker]: %s", message);

  } else {

    sprintf((char *)&message, "Callbacks were already %s", callbacks_enabled ? "enabled" : "disabled");
  }

  res.message = message;
  res.success = true;

  return std_srvs::SetBoolResponse::ConstPtr(new std_srvs::SetBoolResponse(res));
}

//}

// | -------------- setpoint topics and services -------------- |

//{ goTo() service

const mrs_msgs::Vec4Response::ConstPtr NullTracker::goTo(const mrs_msgs::Vec4Request::ConstPtr &cmd) {
  return mrs_msgs::Vec4Response::Ptr();
}

//}

//{ goTo() topic

bool NullTracker::goTo(const mrs_msgs::TrackerPointStampedConstPtr &msg) {
  return false;
}

//}

//{ goToRelative() topic

const mrs_msgs::Vec4Response::ConstPtr NullTracker::goToRelative(const mrs_msgs::Vec4Request::ConstPtr &cmd) {
  return mrs_msgs::Vec4Response::Ptr();
}

//}

//{ goToRelative() topic

bool NullTracker::goToRelative(const mrs_msgs::TrackerPointStampedConstPtr &msg) {
  return false;
}

//}

//{ goToAltitude() service

const mrs_msgs::Vec1Response::ConstPtr NullTracker::goToAltitude(const mrs_msgs::Vec1Request::ConstPtr &cmd) {
  return mrs_msgs::Vec1Response::Ptr();
}

//}

//{ goToAltitude() topic

bool NullTracker::goToAltitude(const std_msgs::Float64ConstPtr &msg) {
  return false;
}

//}

//{ setYaw() service

const mrs_msgs::Vec1Response::ConstPtr NullTracker::setYaw(const mrs_msgs::Vec1Request::ConstPtr &cmd) {
  return mrs_msgs::Vec1Response::Ptr();
}

//}

//{ setYaw() topic

bool NullTracker::setYaw(const std_msgs::Float64ConstPtr &msg) {
  return false;
}

//}

//{ setYawRelative() service

const mrs_msgs::Vec1Response::ConstPtr NullTracker::setYawRelative(const mrs_msgs::Vec1Request::ConstPtr &cmd) {
  return mrs_msgs::Vec1Response::Ptr();
}

//}

//{ setYawRelative() topic

bool NullTracker::setYawRelative(const std_msgs::Float64ConstPtr &msg) {
  return false;
}

//}

//{ hover() hover

const std_srvs::TriggerResponse::ConstPtr NullTracker::hover(const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mav_manager::NullTracker, mrs_mav_manager::Tracker)
