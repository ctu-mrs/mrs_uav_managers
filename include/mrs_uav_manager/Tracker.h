#ifndef TRACKER_H_
#define TRACKER_H_

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

#include <mrs_msgs/TrackerConstraints.h>
#include <mrs_msgs/TrackerConstraintsRequest.h>
#include <mrs_msgs/TrackerConstraintsResponse.h>

#include <mrs_msgs/Reference.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/Float64.h>

#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Eigen>

namespace mrs_uav_manager
{

typedef boost::function<bool(const double x, const double y, const double z)> isPointInSafetyArea3d_t;
typedef boost::function<bool(const double x, const double y)>                 isPointInSafetyArea2d_t;
typedef boost::function<double(void)>                                         getMaxHeight_t;
typedef boost::function<double(void)>                                         getMinHeight_t;

typedef boost::function<bool(const std::string from_frame, const std::string to_frame, const double timeout, mrs_msgs::ReferenceStamped &ref)>
                                                                                                          transformReferenceSingle_t;
typedef boost::function<bool(const geometry_msgs::TransformStamped &tf, mrs_msgs::ReferenceStamped &ref)> transformReference_t;
typedef boost::function<bool(const std::string from_frame, const std::string to_frame, const ros::Time time_stamp, const double timeout,
                             geometry_msgs::TransformStamped &tf)>
    getTransform_t;

struct SafetyArea_t
{
  mrs_uav_manager::isPointInSafetyArea3d_t isPointInSafetyArea3d;
  mrs_uav_manager::isPointInSafetyArea2d_t isPointInSafetyArea2d;
  mrs_uav_manager::getMaxHeight_t          getMaxHeight;
  mrs_uav_manager::getMinHeight_t          getMinHeight;
  bool                                     use_safety_area;
};

struct Transformer_t
{
  mrs_uav_manager::transformReference_t       transformReference;
  mrs_uav_manager::transformReferenceSingle_t transformReferenceSingle;
  mrs_uav_manager::getTransform_t             getTransform;
};

class Tracker {

public:
  virtual ~Tracker(void) {
  }

  virtual void initialize(const ros::NodeHandle &parent_nh, mrs_uav_manager::SafetyArea_t const *safety_area,
                          mrs_uav_manager::Transformer_t const *transformer) = 0;
  virtual bool activate(const mrs_msgs::PositionCommand::ConstPtr &cmd)      = 0;
  virtual void deactivate(void)                                              = 0;
  virtual void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &msg) = 0;

  virtual const mrs_msgs::PositionCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &msg) = 0;
  virtual const mrs_msgs::TrackerStatus             getStatus()                                     = 0;

  virtual const mrs_msgs::ReferenceSrvResponse::ConstPtr goTo(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd)         = 0;
  virtual const mrs_msgs::ReferenceSrvResponse::ConstPtr goToRelative(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd) = 0;
  virtual const mrs_msgs::Float64SrvResponse::ConstPtr   goToAltitude(const mrs_msgs::Float64SrvRequest::ConstPtr &cmd)   = 0;
  virtual const mrs_msgs::Float64SrvResponse::ConstPtr   setYaw(const mrs_msgs::Float64SrvRequest::ConstPtr &cmd)         = 0;
  virtual const mrs_msgs::Float64SrvResponse::ConstPtr   setYawRelative(const mrs_msgs::Float64SrvRequest::ConstPtr &cmd) = 0;

  virtual const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd)           = 0;
  virtual const std_srvs::SetBoolResponse::ConstPtr enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd) = 0;

  virtual bool goTo(const mrs_msgs::ReferenceConstPtr &msg) = 0;

  virtual const mrs_msgs::TrackerConstraintsResponse::ConstPtr setConstraints(const mrs_msgs::TrackerConstraintsRequest::ConstPtr &constraints) = 0;
};
}  // namespace mrs_uav_manager

#endif
