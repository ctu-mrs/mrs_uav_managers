#ifndef MRS_UAV_TRACKER_H
#define MRS_UAV_TRACKER_H

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_managers/control_manager/common_handlers.h>
#include <mrs_uav_managers/control_manager/private_handlers.h>

#include <mrs_msgs/msg/tracker_command.hpp>
#include <mrs_msgs/msg/tracker_status.hpp>
#include <mrs_msgs/msg/uav_state.hpp>
#include <mrs_msgs/msg/float64.hpp>

#include <mrs_msgs/srv/float64_srv.hpp>
#include <mrs_msgs/srv/reference_srv.hpp>
#include <mrs_msgs/srv/velocity_reference_srv.hpp>
#include <mrs_msgs/srv/trajectory_reference_srv.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <mrs_msgs/srv/dynamics_constraints_srv.hpp>

#include <mrs_uav_managers/controller.h>

//}

namespace mrs_uav_managers
{

class Tracker {

public:
  /**
   * @brief It is called once for every tracker. The runtime is not limited.
   *
   * @param node the node handle of the ControlManager
   * @param uav_name the UAV name (e.g., "uav1")
   * @param common_handlers handlers shared between trackers and controllers
   * @param private_handlers handlers provided individually to each tracker
   *
   * @return true if success
   */
  virtual bool initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers, std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) = 0;

  virtual void destroy(void) = 0;

  /**
   * @brief It is called before the trackers output will be required and used. Should not take much time (within miliseconds).
   *
   * @param last_tracker_cmd the last command produced by the last active tracker. Should be used as an initial condition to maintain a smooth reference.
   *
   * @return true if success and message
   */
  virtual std::tuple<bool, std::string> activate(const std::optional<mrs_msgs::msg::TrackerCommand> &last_tracker_cmd) = 0;

  /**
   * @brief is called when this trackers output is no longer needed. However, it can be activated later.
   */
  virtual void deactivate(void) = 0;

  /**
   * @brief It is called during every switch of reference frames of the UAV state estimate.
   * The tracker should recalculate its internal states from old the frame to the new one.
   *
   * @param new_uav_state the new UavState which will come in the next update()
   *
   * @return a service response
   */
  virtual const std::shared_ptr<std_srvs::srv::Trigger::Response> switchOdometrySource(const mrs_msgs::msg::UavState &new_uav_state) = 0;

  /**
   * @brief Request for reseting the tracker's states given the UAV is static.
   *
   * @return true if success
   */
  virtual bool resetStatic(void) = 0;

  /**
   * @brief The most important routine. It is called with every state estimator update and it should produce a new reference for the controllers.
   *        The run time should be as short as possible (<= 1 ms).
   *
   * @param uav_state the latest UAV state estimate
   * @param last_attitude_cmd the last controller's output command (may be useful)
   *
   * @return the new reference for the controllers
   */
  virtual std::optional<mrs_msgs::msg::TrackerCommand> update(const mrs_msgs::msg::UavState &uav_state, const Controller::ControlOutput &last_control_output) = 0;

  /**
   * @brief A request for the tracker's status.
   *
   * @return the tracker's status
   */
  virtual const mrs_msgs::msg::TrackerStatus getStatus() = 0;

  /**
   * @brief Request for a flight to a given coordinates.
   *
   * @param request the reference
   *
   * @return a service response
   */
  virtual const std::shared_ptr<mrs_msgs::srv::ReferenceSrv::Response> setReference(const std::shared_ptr<mrs_msgs::srv::ReferenceSrv::Request> &request) = 0;

  /**
   * @brief Request for desired velocity reference
   *
   * @param request the reference
   *
   * @return a service response
   */
  virtual const std::shared_ptr<mrs_msgs::srv::VelocityReferenceSrv::Response> setVelocityReference(const std::shared_ptr<mrs_msgs::srv::VelocityReferenceSrv::Request> &request) = 0;

  /**
   * @brief Request for a flight along a given trajectory
   *
   * @param request the reference trajectory
   *
   * @return a service response
   */
  virtual const std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Response> setTrajectoryReference(const std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Request> &request) = 0;

  /**
   * @brief Request for stopping the motion of the UAV.
   *
   * @param trigger service request
   *
   * @return a service response
   */
  virtual const std::shared_ptr<std_srvs::srv::Trigger::Response> hover(const std::shared_ptr<std_srvs::srv::Trigger::Request> &request) = 0;

  /**
   * @brief Request to goto to the first trajectory coordinate.
   *
   * @param trigger service request
   *
   * @return a service response
   */
  virtual const std::shared_ptr<std_srvs::srv::Trigger::Response> gotoTrajectoryStart(const std::shared_ptr<std_srvs::srv::Trigger::Request> &request) = 0;

  /**
   * @brief Request to start tracking of the pre-loaded trajectory
   *
   * @param trigger service request
   *
   * @return a service response
   */
  virtual const std::shared_ptr<std_srvs::srv::Trigger::Response> startTrajectoryTracking(const std::shared_ptr<std_srvs::srv::Trigger::Request> &request) = 0;

  /**
   * @brief Request to stop tracking of the pre-loaded trajectory. The hover() routine will be engaged, thus it should be implemented by the tracker.
   *
   * @param trigger service request
   *
   * @return a service response
   */
  virtual const std::shared_ptr<std_srvs::srv::Trigger::Response> stopTrajectoryTracking(const std::shared_ptr<std_srvs::srv::Trigger::Request> &request) = 0;

  /**
   * @brief Request to resume the previously stopped trajectory tracking.
   *
   * @param trigger service request
   *
   * @return a service response
   */
  virtual const std::shared_ptr<std_srvs::srv::Trigger::Response> resumeTrajectoryTracking(const std::shared_ptr<std_srvs::srv::Trigger::Request> &request) = 0;

  /**
   * @brief Request for enabling/disabling callbacks.
   *
   * @param request service request
   *
   * @return a service response
   */
  virtual const std::shared_ptr<std_srvs::srv::SetBool::Response> enableCallbacks(const std::shared_ptr<std_srvs::srv::SetBool::Request> &request) = 0;

  /**
   * @brief Request for setting new constraints.
   *
   * @param constraints to be set
   *
   * @return a service response
   */
  virtual const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response> setConstraints(const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> &constraints) = 0;

  virtual ~Tracker() = default;
};

}  // namespace mrs_uav_managers

#endif
