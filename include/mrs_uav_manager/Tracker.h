#ifndef TRACKER_H_
#define TRACKER_H_

/* includes //{ */

#include <ros/ros.h>

#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/TrackerStatus.h>
#include <mrs_msgs/UavState.h>

#include <mrs_msgs/Float64Srv.h>
#include <mrs_msgs/Float64SrvRequest.h>
#include <mrs_msgs/Float64SrvResponse.h>

#include <mrs_msgs/ReferenceSrv.h>
#include <mrs_msgs/ReferenceSrvRequest.h>
#include <mrs_msgs/ReferenceSrvResponse.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/SetBoolRequest.h>
#include <std_srvs/SetBoolResponse.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <mrs_msgs/TrackerConstraintsSrv.h>
#include <mrs_msgs/TrackerConstraintsSrvRequest.h>
#include <mrs_msgs/TrackerConstraintsSrvResponse.h>

#include <mrs_msgs/Reference.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/Float64.h>

#include <mrs_msgs/AttitudeCommand.h>

#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Eigen>

#include <mrs_lib/transformer.h>

#include <mrs_uav_manager/common_handlers.h>

//}

namespace mrs_uav_manager
{

class Tracker {

public:
  virtual ~Tracker(void) {
  }

  virtual void initialize(const ros::NodeHandle &parent_nh, const std::string uav_name, std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers) = 0;
  virtual bool activate(const mrs_msgs::PositionCommand::ConstPtr &cmd)                                                                                     = 0;
  virtual void deactivate(void)                                                                                                                             = 0;
  virtual void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &msg)                                                                                = 0;
  virtual bool resetStatic(void)                                                                                                                            = 0;

  virtual const mrs_msgs::PositionCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &msg, const mrs_msgs::AttitudeCommand::ConstPtr &cmd) = 0;
  virtual const mrs_msgs::TrackerStatus             getStatus()                                                                                     = 0;

  virtual const mrs_msgs::ReferenceSrvResponse::ConstPtr goTo(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd)         = 0;
  virtual const mrs_msgs::ReferenceSrvResponse::ConstPtr goToRelative(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd) = 0;
  virtual const mrs_msgs::Float64SrvResponse::ConstPtr   goToAltitude(const mrs_msgs::Float64SrvRequest::ConstPtr &cmd)   = 0;
  virtual const mrs_msgs::Float64SrvResponse::ConstPtr   setYaw(const mrs_msgs::Float64SrvRequest::ConstPtr &cmd)         = 0;
  virtual const mrs_msgs::Float64SrvResponse::ConstPtr   setYawRelative(const mrs_msgs::Float64SrvRequest::ConstPtr &cmd) = 0;

  virtual const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd)           = 0;
  virtual const std_srvs::SetBoolResponse::ConstPtr enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd) = 0;

  virtual bool goTo(const mrs_msgs::ReferenceConstPtr &msg) = 0;

  virtual const mrs_msgs::TrackerConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::TrackerConstraintsSrvRequest::ConstPtr &constraints) = 0;
};
}  // namespace mrs_uav_manager

#endif
