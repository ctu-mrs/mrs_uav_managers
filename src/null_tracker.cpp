#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_managers/tracker.h>

namespace mrs_uav_managers
{

/* //{ class NullTracker */

class NullTracker : public mrs_uav_managers::Tracker {

public:
  bool initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  void destroy();

  std::tuple<bool, std::string> activate([[maybe_unused]] const std::optional<mrs_msgs::msg::TrackerCommand> &last_tracker_cmd);
  void                          deactivate(void);
  bool                          resetStatic(void);

  std::optional<mrs_msgs::msg::TrackerCommand>            update(const mrs_msgs::msg::UavState                     &uav_state,
                                                                 const mrs_uav_managers::Controller::ControlOutput &last_control_output);
  const mrs_msgs::msg::TrackerStatus                      getStatus();
  const std::shared_ptr<std_srvs::srv::SetBool::Response> enableCallbacks(const std::shared_ptr<std_srvs::srv::SetBool::Request> &request);
  const std::shared_ptr<std_srvs::srv::Trigger::Response> switchOdometrySource(const mrs_msgs::msg::UavState &new_uav_state);

  const std::shared_ptr<mrs_msgs::srv::ReferenceSrv::Response> setReference(const std::shared_ptr<mrs_msgs::srv::ReferenceSrv::Request> &request);
  const std::shared_ptr<mrs_msgs::srv::VelocityReferenceSrv::Response>
  setVelocityReference(const std::shared_ptr<mrs_msgs::srv::VelocityReferenceSrv::Request> &request);
  const std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Response>
  setTrajectoryReference(const std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Request> &request);

  const std::shared_ptr<std_srvs::srv::Trigger::Response> hover(const std::shared_ptr<std_srvs::srv::Trigger::Request> &request);
  const std::shared_ptr<std_srvs::srv::Trigger::Response> startTrajectoryTracking(const std::shared_ptr<std_srvs::srv::Trigger::Request> &request);
  const std::shared_ptr<std_srvs::srv::Trigger::Response> stopTrajectoryTracking(const std::shared_ptr<std_srvs::srv::Trigger::Request> &request);
  const std::shared_ptr<std_srvs::srv::Trigger::Response> resumeTrajectoryTracking(const std::shared_ptr<std_srvs::srv::Trigger::Request> &request);
  const std::shared_ptr<std_srvs::srv::Trigger::Response> gotoTrajectoryStart(const std::shared_ptr<std_srvs::srv::Trigger::Request> &request);

  const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>
  setConstraints(const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> &request);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  bool is_active_         = false;
  bool is_initialized_    = false;
  bool callbacks_enabled_ = false;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;
};

//}

// | ------------------- trackers interface ------------------- |

/* //{ initialize() */

bool NullTracker::initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                             std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  is_initialized_ = true;

  this->common_handlers_  = common_handlers;
  this->private_handlers_ = private_handlers;

  node_  = node;
  clock_ = node->get_clock();

  RCLCPP_INFO(node_->get_logger(), "[NullTracker]: initialized");

  return true;
}

//}

/* destroy() //{ */

void NullTracker::destroy() {
}

//}

/* //{ activate() */

std::tuple<bool, std::string> NullTracker::activate([[maybe_unused]] const std::optional<mrs_msgs::msg::TrackerCommand> &last_tracker_cmd) {

  std::stringstream ss;
  ss << "activated";

  RCLCPP_INFO_STREAM(node_->get_logger(), "[NullTracker]: " << ss.str());
  is_active_ = true;

  return std::tuple(true, ss.str());
}

//}

/* //{ deactivate() */

void NullTracker::deactivate(void) {

  RCLCPP_INFO(node_->get_logger(), "[NullTracker]: deactivated");
  is_active_ = false;
}

//}

/* //{ resetStatic() */

bool NullTracker::resetStatic(void) {
  return false;
}

//}

/* switchOdometrySource() //{ */

const std::shared_ptr<std_srvs::srv::Trigger::Response> NullTracker::switchOdometrySource([[maybe_unused]] const mrs_msgs::msg::UavState &new_uav_state) {
  return nullptr;
}

//}

/* //{ update() */

std::optional<mrs_msgs::msg::TrackerCommand> NullTracker::update([[maybe_unused]] const mrs_msgs::msg::UavState                     &uav_state,
                                                                 [[maybe_unused]] const mrs_uav_managers::Controller::ControlOutput &last_control_output) {

  return {};
}

//}

/* //{ getStatus() */

const mrs_msgs::msg::TrackerStatus NullTracker::getStatus() {

  mrs_msgs::msg::TrackerStatus tracker_status;

  tracker_status.active            = is_active_;
  tracker_status.callbacks_enabled = callbacks_enabled_;

  return tracker_status;
}

//}

/* //{ enableCallbacks() */

const std::shared_ptr<std_srvs::srv::SetBool::Response> NullTracker::enableCallbacks(const std::shared_ptr<std_srvs::srv::SetBool::Request> &request) {

  std::shared_ptr<std_srvs::srv::SetBool::Response> response = std::make_shared<std_srvs::srv::SetBool::Response>();

  std::stringstream ss;

  if (request->data != callbacks_enabled_) {

    callbacks_enabled_ = request->data;

    ss << "callbacks " << (callbacks_enabled_ ? "enabled" : "disabled");
    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "[NullTracker]: " << ss.str());

  } else {

    ss << "callbacks were already " << (callbacks_enabled_ ? "enabled" : "disabled");
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "[NullTracker]: " << ss.str());
  }

  response->message = ss.str();
  response->success = true;

  return response;
}

//}

/* //{ setReference() */

const std::shared_ptr<mrs_msgs::srv::ReferenceSrv::Response>
NullTracker::setReference([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::ReferenceSrv::Request> &request) {

  return nullptr;
}

//}

/* //{ setVelocityReference() */

const std::shared_ptr<mrs_msgs::srv::VelocityReferenceSrv::Response>
NullTracker::setVelocityReference([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::VelocityReferenceSrv::Request> &request) {

  return nullptr;
}

//}

/* //{ setTrajectoryReference() */

const std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Response>
NullTracker::setTrajectoryReference([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Request> &request) {

  return nullptr;
}

//}

// | --------------------- other services --------------------- |

/* //{ hover() */

const std::shared_ptr<std_srvs::srv::Trigger::Response> NullTracker::hover([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> &request) {
  return nullptr;
}

//}

/* //{ startTrajectoryTracking() */

const std::shared_ptr<std_srvs::srv::Trigger::Response>
NullTracker::startTrajectoryTracking([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> &request) {

  return nullptr;
}

//}

/* //{ stopTrajectoryTracking() */

const std::shared_ptr<std_srvs::srv::Trigger::Response>
NullTracker::stopTrajectoryTracking([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> &request) {

  return nullptr;
}

//}

/* //{ resumeTrajectoryTracking() */

const std::shared_ptr<std_srvs::srv::Trigger::Response>
NullTracker::resumeTrajectoryTracking([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> &request) {

  return nullptr;
}

//}

/* //{ gotoTrajectoryStart() */

const std::shared_ptr<std_srvs::srv::Trigger::Response>
NullTracker::gotoTrajectoryStart([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> &request) {

  return nullptr;
}

//}

/* //{ setConstraints() */

const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>
NullTracker::setConstraints([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> &request) {

  return nullptr;
}

//}

} // namespace mrs_uav_managers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::NullTracker, mrs_uav_managers::Tracker)
