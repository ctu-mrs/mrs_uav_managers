#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <ros/ros.h>

#include <mrs_msgs/AttitudeCommand.h>
#include <mrs_msgs/ControllerStatus.h>
#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <nav_msgs/Odometry.h>

namespace mrs_uav_manager
{

struct MotorParams
{
  double hover_thrust_a;
  double hover_thrust_b;
};

class Controller {
public:
  virtual ~Controller(void) {
  }

  virtual void initialize(const ros::NodeHandle &parent_nh, mrs_uav_manager::MotorParams motor_params) = 0;
  virtual bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd)                                = 0;
  virtual void deactivate(void)                                                                        = 0;

  virtual const mrs_msgs::AttitudeCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &       odometry,
                                                           const mrs_msgs::PositionCommand::ConstPtr &reference) = 0;
  virtual const mrs_msgs::ControllerStatus::Ptr     getStatus()                                                  = 0;
};
}  // namespace mrs_uav_manager

#endif