#include <ros/ros.h>

#include <mrs_mav_manager/Tracker.h>
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/TrackerStatus.h>

namespace mrs_mav_manager
{

class NullTracker : public mrs_mav_manager::Tracker {

public:
  NullTracker(void);

  void Initialize(const ros::NodeHandle &parent_nh);
  bool Activate(const mrs_msgs::PositionCommand::ConstPtr &cmd);
  void                                      Deactivate(void);
  const mrs_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  const mrs_msgs::TrackerStatus::Ptr status();

private:
  ros::NodeHandle nh_;
  bool            is_active      = false;
  bool            is_initialized = false;
};

NullTracker::NullTracker(void) {
}

// called once at the very beginning
void NullTracker::Initialize(const ros::NodeHandle &parent_nh) {

  ros::NodeHandle priv_nh(parent_nh, "null_tracker");

  ros::Time::waitForValid();

  is_initialized = true;

  ROS_INFO("[NullTracker]: initialized");
}

bool NullTracker::Activate(const mrs_msgs::PositionCommand::ConstPtr &cmd) {

  ROS_INFO("[NullTracker]: activated");
  is_active = true;
  return true;
}

void NullTracker::Deactivate(void) {

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
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mav_manager::NullTracker, mrs_mav_manager::Tracker)
