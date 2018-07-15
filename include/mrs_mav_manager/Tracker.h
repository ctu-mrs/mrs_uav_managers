#ifndef control_manager_TRACKER_H_
#define control_manager_TRACKER_H_

#include <ros/ros.h>
#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/TrackerStatus.h>
#include <mrs_msgs/Vec4Request.h>
#include <mrs_msgs/Vec4Response.h>
#include <nav_msgs/Odometry.h>

namespace mrs_mav_manager
{
class Tracker {
public:
  virtual ~Tracker(void) {
  }

  virtual void initialize(const ros::NodeHandle &parent_nh)             = 0;
  virtual bool activate(const mrs_msgs::PositionCommand::ConstPtr &cmd) = 0;
  virtual void deactivate(void)                                         = 0;

  virtual const mrs_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg) = 0;
  virtual const mrs_msgs::TrackerStatus::Ptr status()                                               = 0;

  virtual const mrs_msgs::Vec4Response::ConstPtr goTo(const mrs_msgs::Vec4Request::ConstPtr &cmd)         = 0;
  virtual const mrs_msgs::Vec4Response::ConstPtr goToRelative(const mrs_msgs::Vec4Request::ConstPtr &cmd) = 0;
};
}

#endif
