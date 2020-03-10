#include <ros/ros.h>

#include <mrs_uav_manager/Tracker.h>

namespace mrs_uav_manager
{

/* //{ class NullTracker */

class NullTracker : public mrs_uav_manager::Tracker {

public:
  virtual void initialize(const ros::NodeHandle &parent_nh, const std::string uav_name, std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers);
  virtual bool activate(const mrs_msgs::PositionCommand::ConstPtr &cmd);
  virtual void deactivate(void);
  virtual bool resetStatic(void);

  virtual const mrs_msgs::PositionCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &msg, const mrs_msgs::AttitudeCommand::ConstPtr &cmd);
  virtual const mrs_msgs::TrackerStatus             getStatus();
  virtual const std_srvs::SetBoolResponse::ConstPtr enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd);
  virtual void                                      switchOdometrySource(const mrs_msgs::UavState::ConstPtr &msg);

  virtual const mrs_msgs::ReferenceSrvResponse::ConstPtr goTo(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd);
  virtual const mrs_msgs::ReferenceSrvResponse::ConstPtr goToRelative(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd);
  virtual const mrs_msgs::Float64SrvResponse::ConstPtr   goToAltitude(const mrs_msgs::Float64SrvRequest::ConstPtr &cmd);
  virtual const mrs_msgs::Float64SrvResponse::ConstPtr   setYaw(const mrs_msgs::Float64SrvRequest::ConstPtr &cmd);
  virtual const mrs_msgs::Float64SrvResponse::ConstPtr   setYawRelative(const mrs_msgs::Float64SrvRequest::ConstPtr &cmd);

  virtual bool goTo(const mrs_msgs::ReferenceConstPtr &msg);
  virtual bool goToRelative(const mrs_msgs::ReferenceConstPtr &msg);
  virtual bool goToAltitude(const mrs_msgs::Float64ConstPtr &msg);
  virtual bool setYaw(const mrs_msgs::Float64ConstPtr &msg);
  virtual bool setYawRelative(const mrs_msgs::Float64ConstPtr &msg);

  virtual const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd);

  virtual const mrs_msgs::TrackerConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::TrackerConstraintsSrvRequest::ConstPtr &cmd);

private:
  ros::NodeHandle nh_;
  bool            is_active         = false;
  bool            is_initialized    = false;
  bool            callbacks_enabled = false;

private:
  std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers;
};

//}

// | ------------------- trackers interface ------------------- |

/* //{ initialize() */

void NullTracker::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string uav_name,
                             [[maybe_unused]] std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, "null_tracker");

  ros::Time::waitForValid();

  is_initialized = true;

  this->common_handlers = common_handlers;

  ROS_INFO("[NullTracker]: initialized");
}

//}

/* //{ activate() */

bool NullTracker::activate([[maybe_unused]] const mrs_msgs::PositionCommand::ConstPtr &cmd) {

  ROS_INFO("[NullTracker]: activated");
  is_active = true;
  return true;
}

//}

/* //{ deactivate() */

void NullTracker::deactivate(void) {

  ROS_INFO("[NullTracker]: deactivated");
  is_active = false;
}

//}

/* //{ resetStatic() */

bool NullTracker::resetStatic(void) {
  return false;
}

//}

/* switchOdometrySource() //{ */

void NullTracker::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState::ConstPtr &msg) {
}

//}

/* //{ update() */

const mrs_msgs::PositionCommand::ConstPtr NullTracker::update([[maybe_unused]] const mrs_msgs::UavState::ConstPtr &       msg,
                                                              [[maybe_unused]] const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

  return mrs_msgs::PositionCommand::Ptr();
}

//}

/* //{ getStatus() */

const mrs_msgs::TrackerStatus NullTracker::getStatus() {

  mrs_msgs::TrackerStatus tracker_status;

  tracker_status.active            = is_active;
  tracker_status.callbacks_enabled = callbacks_enabled;

  return tracker_status;
}

//}

/* //{ enableCallbacks() */

const std_srvs::SetBoolResponse::ConstPtr NullTracker::enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd) {

  std_srvs::SetBoolResponse res;

  std::stringstream ss;

  if (cmd->data != callbacks_enabled) {

    callbacks_enabled = cmd->data;

    ss << "callbacks " << (callbacks_enabled ? "enabled" : "disabled");

    ROS_DEBUG_STREAM("[NullTracker]: " << ss.str());

  } else {

    ss << "callbacks were already " << (callbacks_enabled ? "enabled" : "disabled");
  }

  res.message = ss.str();
  res.success = true;

  return std_srvs::SetBoolResponse::ConstPtr(std::make_unique<std_srvs::SetBoolResponse>(res));
}

//}

// | -------------- setpoint topics and services -------------- |

/* //{ goTo() service */

const mrs_msgs::ReferenceSrvResponse::ConstPtr NullTracker::goTo([[maybe_unused]] const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd) {
  return mrs_msgs::ReferenceSrvResponse::Ptr();
}

//}

/* //{ goTo() topic */

bool NullTracker::goTo([[maybe_unused]] const mrs_msgs::ReferenceConstPtr &msg) {
  return false;
}

//}

/* //{ goToRelative() topic */

const mrs_msgs::ReferenceSrvResponse::ConstPtr NullTracker::goToRelative([[maybe_unused]] const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd) {
  return mrs_msgs::ReferenceSrvResponse::Ptr();
}

//}

/* //{ goToRelative() topic */

bool NullTracker::goToRelative([[maybe_unused]] const mrs_msgs::ReferenceConstPtr &msg) {
  return false;
}

//}

/* //{ goToAltitude() service */

const mrs_msgs::Float64SrvResponse::ConstPtr NullTracker::goToAltitude([[maybe_unused]] const mrs_msgs::Float64SrvRequest::ConstPtr &cmd) {
  return mrs_msgs::Float64SrvResponse::Ptr();
}

//}

/* //{ goToAltitude() topic */

bool NullTracker::goToAltitude([[maybe_unused]] const mrs_msgs::Float64ConstPtr &msg) {
  return false;
}

//}

/* //{ setYaw() service */

const mrs_msgs::Float64SrvResponse::ConstPtr NullTracker::setYaw([[maybe_unused]] const mrs_msgs::Float64SrvRequest::ConstPtr &cmd) {
  return mrs_msgs::Float64SrvResponse::Ptr();
}

//}

/* //{ setYaw() topic */

bool NullTracker::setYaw([[maybe_unused]] const mrs_msgs::Float64ConstPtr &msg) {
  return false;
}

//}

/* //{ setYawRelative() service */

const mrs_msgs::Float64SrvResponse::ConstPtr NullTracker::setYawRelative([[maybe_unused]] const mrs_msgs::Float64SrvRequest::ConstPtr &cmd) {
  return mrs_msgs::Float64SrvResponse::Ptr();
}

//}

/* //{ setYawRelative() topic */

bool NullTracker::setYawRelative([[maybe_unused]] const mrs_msgs::Float64ConstPtr &msg) {
  return false;
}

//}

/* //{ hover() service */

const std_srvs::TriggerResponse::ConstPtr NullTracker::hover([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ setConstraints() service */

const mrs_msgs::TrackerConstraintsSrvResponse::ConstPtr NullTracker::setConstraints([
    [maybe_unused]] const mrs_msgs::TrackerConstraintsSrvRequest::ConstPtr &cmd) {

  return mrs_msgs::TrackerConstraintsSrvResponse::Ptr();
}

//}

}  // namespace mrs_uav_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_manager::NullTracker, mrs_uav_manager::Tracker)
