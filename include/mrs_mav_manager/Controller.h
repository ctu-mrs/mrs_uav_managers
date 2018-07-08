#ifndef control_manager_CONTROLLER_H_
#define control_manager_CONTROLLER_H_

#include <mrs_msgs/AttitudeCommand.h>
#include <mrs_msgs/ControllerStatus.h>
#include <mrs_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace mrs_mav_manager
{
class Controller {
public:
  virtual ~Controller(void) {
  }

  virtual void Initialize(const ros::NodeHandle &parent_nh) = 0;
  virtual bool Activate(void)   = 0;
  virtual void Deactivate(void) = 0;

  virtual const mrs_msgs::AttitudeCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &odometry, const mrs_msgs::PositionCommand::ConstPtr &reference) = 0;
  virtual const mrs_msgs::ControllerStatus::Ptr status() = 0;
};
}

#endif
