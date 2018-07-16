#include <ros/ros.h>

#include <mrs_mav_manager/Tracker.h>

namespace mrs_mav_manager
{

class NullTracker : public mrs_mav_manager::Tracker {

public:
  NullTracker(void);

  void initialize(const ros::NodeHandle &parent_nh);
  bool activate(const mrs_msgs::PositionCommand::ConstPtr &cmd);
  void deactivate(void);

  const mrs_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  const mrs_msgs::TrackerStatus::Ptr status();

  virtual const mrs_msgs::Vec4Response::ConstPtr goTo(const mrs_msgs::Vec4Request::ConstPtr &cmd);
  virtual const mrs_msgs::Vec4Response::ConstPtr goToRelative(const mrs_msgs::Vec4Request::ConstPtr &cmd);

  virtual const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd);

private:
  ros::NodeHandle nh_;
  bool            is_active      = false;
  bool            is_initialized = false;
};

NullTracker::NullTracker(void) {
}

// called once at the very beginning
void NullTracker::initialize(const ros::NodeHandle &parent_nh) {

  ros::NodeHandle priv_nh(parent_nh, "null_tracker");

  ros::Time::waitForValid();

  is_initialized = true;

  ROS_INFO("[NullTracker]: initialized");
}

bool NullTracker::activate(const mrs_msgs::PositionCommand::ConstPtr &cmd) {

  ROS_INFO("[NullTracker]: activated");
  is_active = true;
  return true;
}

void NullTracker::deactivate(void) {

  ROS_INFO("[NullTracker]: deactivated");
  is_active = false;
}

const mrs_msgs::PositionCommand::ConstPtr NullTracker::update(const nav_msgs::Odometry::ConstPtr &msg) {

  return mrs_msgs::PositionCommand::Ptr();
}

const mrs_msgs::TrackerStatus::Ptr NullTracker::status() {

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

const mrs_msgs::Vec4Response::ConstPtr NullTracker::goTo(const mrs_msgs::Vec4Request::ConstPtr &cmd) {
  return mrs_msgs::Vec4Response::Ptr();
}

const mrs_msgs::Vec4Response::ConstPtr NullTracker::goToRelative(const mrs_msgs::Vec4Request::ConstPtr &cmd) {
  return mrs_msgs::Vec4Response::Ptr();
}

const std_srvs::TriggerResponse::ConstPtr NullTracker::hover(const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mav_manager::NullTracker, mrs_mav_manager::Tracker)
