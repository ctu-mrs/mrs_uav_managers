#ifndef TRACKERS_MANAGER_TRACKER_H_
#define TRACKERS_MANAGER_TRACKER_H_

#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/TrackerStatus.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace mrs_mav_manager
{
class Tracker {
public:
  virtual ~Tracker(void) {
  }

  virtual void Initialize(const ros::NodeHandle &parent_nh) = 0;
  virtual bool Activate(const mrs_msgs::PositionCommand::ConstPtr &cmd) = 0;
  virtual void Deactivate(void)                                         = 0;

  virtual const mrs_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg) = 0;
  virtual const mrs_msgs::TrackerStatus::Ptr status()                                               = 0;
};
}

#endif
