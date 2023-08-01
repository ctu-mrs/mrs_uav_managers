#ifndef MRS_UAV_CONTROLLER_H
#define MRS_UAV_CONTROLLER_H

/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_managers/common_handlers.h>

#include <mrs_msgs/AttitudeCommand.h>
#include <mrs_msgs/ControllerStatus.h>
#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/UavState.h>

#include <mrs_msgs/DynamicsConstraintsSrv.h>
#include <mrs_msgs/DynamicsConstraintsSrvRequest.h>
#include <mrs_msgs/DynamicsConstraintsSrvResponse.h>

//}

namespace mrs_uav_managers
{

// parameters of the propulsion thrust curve
// T = A*sqrt(F) + B
// T is within [0, 1]
// F is in Newtons
struct MotorParams
{
  double A;
  double B;
};

class Controller {
public:
  virtual ~Controller() = default;

  /**
   * @brief Initializes the controller. It is called once for every controller. The runtime is not limited.
   *
   * @param parent_nh the node handle of the ControlManager
   * @param name of the controller for distinguishing multiple running instances of the same code
   * @param name_space the parameter namespace of the controller, can be used during initialization of the private node handle
   * @param uav_mass the net mass of the UAV
   * @param common_handlers handlers shared between trackers and controllers
   */
  virtual void initialize(const ros::NodeHandle &parent_nh, const std::string name, const std::string name_space, const double uav_mass,
                          std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) = 0;

  /**
   * @brief It is called before the controller output will be required and used. Should not take much time (within miliseconds).
   *
   * @param last_attitude_cmd the last command produced by the last active controller. Should be used as an initial condition, e.g., for re-initializing
   * integrators and estimators.
   *
   * @return true if success
   */
  virtual bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd) = 0;

  /**
   * @brief is called when this controller's output is no longer needed. However, it can be activated later.
   */
  virtual void deactivate(void) = 0;

  /**
   * @brief It may be called to reset the controllers disturbance estimators.
   */
  virtual void resetDisturbanceEstimators(void) = 0;

  /**
   * @brief The most important routine. It is called with every odometry update and it should produce a new control command.
   *
   * @param uav_state the latest UAV state estimate
   * @param last_position_cmd the last controller's output command (may be useful)
   *
   * @return the new reference for the controllers
   */
  virtual const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &       uav_state,
                                                           const mrs_msgs::PositionCommand::ConstPtr &last_position_cmd) = 0;

  /**
   * @brief A request for the controller's status.
   *
   * @return the controller's status
   */
  virtual const mrs_msgs::ControllerStatus getStatus() = 0;

  /**
   * @brief It is called during every switch of reference frames of the UAV state estimate.
   * The controller should recalculate its internal states from old the frame to the new one.
   *
   * @param new_uav_state the new UavState which will come in the next update()
   */
  virtual void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &new_uav_state) = 0;

  /**
   * @brief Request for setting new constraints.
   *
   * @param constraints to be set
   *
   * @return a service response
   */
  virtual const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &constraints) = 0;
};

}  // namespace mrs_uav_managers

#endif
