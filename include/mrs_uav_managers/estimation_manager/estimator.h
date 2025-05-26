#ifndef MRS_UAV_MANAGERS_ESTIMATION_MANAGER_ESTIMATOR_H
#define MRS_UAV_MANAGERS_ESTIMATION_MANAGER_ESTIMATOR_H

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>

#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <mrs_msgs/msg/uav_state.hpp>
#include <mrs_msgs/msg/estimator_input.hpp>
#include <mrs_msgs/msg/estimator_diagnostics.hpp>

#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>


#include <mrs_uav_managers/estimation_manager/types.h>
#include <mrs_uav_managers/estimation_manager/support.h>
#include <mrs_uav_managers/estimation_manager/common_handlers.h>
#include <mrs_uav_managers/estimation_manager/private_handlers.h>

//}

namespace mrs_uav_managers
{

/* using namespace estimation_manager; */

using namespace mrs_uav_managers::estimation_manager;

class Estimator {

protected:
  mutable mrs_lib::PublisherHandler<mrs_msgs::msg::EstimatorDiagnostics> ph_diagnostics_;

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  const std::string type_;
  const std::string name_;
  const std::string package_name_;

  std::string frame_id_;  // cannot be constant - must remain overridable by loaded parameter
  std::string ns_frame_id_;

  std::shared_ptr<CommonHandlers_t>  ch_;
  std::shared_ptr<PrivateHandlers_t> ph_;

  double max_flight_z_ = -1.0;

  std::atomic_bool is_mitigating_jump_ = false;

private:
  SMStates_t         previous_sm_state_ = SMStates_t::UNINITIALIZED_STATE;
  SMStates_t         current_sm_state_  = SMStates_t::UNINITIALIZED_STATE;
  mutable std::mutex mutex_current_state_;

public:
  Estimator(const std::string &type, const std::string &name, const std::string &frame_id) : type_(type), name_(name), frame_id_(frame_id) {
  }

  virtual ~Estimator() = default;

public:
  // virtual methods
  virtual void initialize(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) = 0;
  virtual bool start(void)                                                                                                                                = 0;
  virtual bool pause(void)                                                                                                                                = 0;
  virtual bool reset(void)                                                                                                                                = 0;

  // implemented methods
  // access methods
  std::string getName(void) const;
  std::string getPrintName(void) const;
  std::string getType(void) const;
  std::string getFrameId(void) const;
  double      getMaxFlightZ(void) const;
  std::string getSmStateString(const SMStates_t &state) const;
  std::string getCurrentSmStateString(void) const;
  SMStates_t  getCurrentSmState() const;

  void setCurrentSmState(const SMStates_t &new_state);

  bool isMitigatingJump();

  // state machine methods
  bool changeState(SMStates_t new_state);
  bool isInState(const SMStates_t &state_in) const;
  bool isInitialized() const;
  bool isReady() const;
  bool isStarted() const;
  bool isRunning() const;
  bool isStopped() const;
  bool isError() const;

  void publishDiagnostics() const;

  tf2::Vector3          getAccGlobal(const sensor_msgs::msg::Imu::ConstSharedPtr &input_msg, const double hdg);
  tf2::Vector3          getAccGlobal(const mrs_msgs::msg::EstimatorInput::ConstSharedPtr &input_msg, const double hdg);
  tf2::Vector3          getAccGlobal(const geometry_msgs::msg::Vector3Stamped &acc_stamped, const double hdg);
  std::optional<double> getHeadingRate(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg);
};

}  // namespace mrs_uav_managers

#endif  // MRS_UAV_MANAGERS_ESTIMATION_MANAGER_ESTIMATOR_H
