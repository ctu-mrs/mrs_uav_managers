#include <ros/ros.h>

#include <mrs_uav_managers/tracker.h>

namespace mrs_uav_managers
{

/* //{ class NullTracker */

class NullTracker : public mrs_uav_managers::Tracker {

public:
  ~NullTracker(){};

  void initialize(const ros::NodeHandle &parent_nh, const std::string uav_name, std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers);
  std::tuple<bool, std::string> activate([[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand> &last_tracker_cmd);
  void                          deactivate(void);
  bool                          resetStatic(void);

  std::optional<mrs_msgs::TrackerCommand>   update(const mrs_msgs::UavState &uav_state, const Controller::ControlOutput &last_control_output);
  const mrs_msgs::TrackerStatus             getStatus();
  const std_srvs::SetBoolResponse::ConstPtr enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr switchOdometrySource(const mrs_msgs::UavState &new_uav_state);

  const mrs_msgs::ReferenceSrvResponse::ConstPtr           setReference(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd);
  const mrs_msgs::VelocityReferenceSrvResponse::ConstPtr   setVelocityReference(const mrs_msgs::VelocityReferenceSrvRequest::ConstPtr &cmd);
  const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr setTrajectoryReference(const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd);

  const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr startTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr stopTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr resumeTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr gotoTrajectoryStart(const std_srvs::TriggerRequest::ConstPtr &cmd);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

private:
  ros::NodeHandle nh_;
  bool            is_active         = false;
  bool            is_initialized    = false;
  bool            callbacks_enabled = false;

  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers;
};

//}

// | ------------------- trackers interface ------------------- |

/* //{ initialize() */

void NullTracker::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string uav_name,
                             [[maybe_unused]] std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, "null_tracker");

  ros::Time::waitForValid();

  is_initialized = true;

  this->common_handlers = common_handlers;

  ROS_INFO("[NullTracker]: initialized");
}

//}

/* //{ activate() */

std::tuple<bool, std::string> NullTracker::activate([[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand> &last_tracker_cmd) {

  std::stringstream ss;
  ss << "activated";

  ROS_INFO_STREAM("[NullTracker]: " << ss.str());
  is_active = true;

  return std::tuple(true, ss.str());
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

const std_srvs::TriggerResponse::ConstPtr NullTracker::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState &new_uav_state) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ update() */

std::optional<mrs_msgs::TrackerCommand> NullTracker::update([[maybe_unused]] const mrs_msgs::UavState &       uav_state,
                                                            [[maybe_unused]] const Controller::ControlOutput &last_control_output) {

  return {};
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

/* //{ setReference() */

const mrs_msgs::ReferenceSrvResponse::ConstPtr NullTracker::setReference([[maybe_unused]] const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd) {
  return mrs_msgs::ReferenceSrvResponse::Ptr();
}

//}

/* //{ setVelocityReference() */

const mrs_msgs::VelocityReferenceSrvResponse::ConstPtr NullTracker::setVelocityReference([
    [maybe_unused]] const mrs_msgs::VelocityReferenceSrvRequest::ConstPtr &cmd) {
  return mrs_msgs::VelocityReferenceSrvResponse::Ptr();
}

//}

/* //{ setTrajectoryReference() */

const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr NullTracker::setTrajectoryReference([
    [maybe_unused]] const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd) {
  return mrs_msgs::TrajectoryReferenceSrvResponse::Ptr();
}

//}

// | --------------------- other services --------------------- |

/* //{ hover() */

const std_srvs::TriggerResponse::ConstPtr NullTracker::hover([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ startTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr NullTracker::startTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ stopTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr NullTracker::stopTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ resumeTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr NullTracker::resumeTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ gotoTrajectoryStart() */

const std_srvs::TriggerResponse::ConstPtr NullTracker::gotoTrajectoryStart([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ setConstraints() */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr NullTracker::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd) {

  return mrs_msgs::DynamicsConstraintsSrvResponse::Ptr();
}

//}

}  // namespace mrs_uav_managers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::NullTracker, mrs_uav_managers::Tracker)
