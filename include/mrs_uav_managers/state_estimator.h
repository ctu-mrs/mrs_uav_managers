#ifndef MRS_UAV_MANAGERS_STATE_ESTIMATOR_H
#define MRS_UAV_MANAGERS_STATE_ESTIMATOR_H

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>

#include <nav_msgs/msg/odometry.hpp>

#include <mrs_msgs/msg/uav_state.hpp>
#include <mrs_msgs/msg/float64_array_stamped.hpp>
#include <mrs_msgs/msg/hw_api_capabilities.hpp>

#include <mrs_uav_managers/estimation_manager/estimator.h>

//}

namespace mrs_uav_managers
{

namespace state
{
const char type[] = "STATE";
}

using namespace estimation_manager;

class StateEstimator : public Estimator {

protected:
  const std::string package_name_ = "mrs_uav_state_estimators";

  mrs_msgs::msg::UavState uav_state_;
  mrs_msgs::msg::UavState uav_state_init_;
  mutable std::mutex      mtx_uav_state_;

  nav_msgs::msg::Odometry odom_;
  mutable std::mutex      mtx_odom_;

  nav_msgs::msg::Odometry innovation_;
  nav_msgs::msg::Odometry innovation_init_;
  mutable std::mutex      mtx_innovation_;

  mrs_msgs::msg::Float64ArrayStamped pose_covariance_, twist_covariance_;
  mutable std::mutex                 mtx_covariance_;

  bool is_override_frame_id_ = false;

  bool is_active_ = false;

protected:
  mutable mrs_lib::PublisherHandler<mrs_msgs::msg::UavState>               ph_uav_state_;
  mutable mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>               ph_odom_;
  mutable mrs_lib::PublisherHandler<mrs_msgs::msg::Float64ArrayStamped>    ph_pose_covariance_, ph_twist_covariance_;
  mutable mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>               ph_innovation_;
  mutable mrs_lib::PublisherHandler<geometry_msgs::msg::QuaternionStamped> ph_attitude_;

public:
  StateEstimator(const std::string &name, const std::string &frame_id, const std::string &package_name)
      : Estimator(state::type, name, frame_id), package_name_(package_name) {
  }

  virtual ~StateEstimator() = default;

  virtual bool setUavState(const mrs_msgs::msg::UavState &uav_state) = 0;

  virtual void updateUavState() = 0;

  // implemented methods
  std::optional<mrs_msgs::msg::UavState>        getUavState();
  nav_msgs::msg::Odometry                       getInnovation() const;
  std::vector<double>                           getPoseCovariance() const;
  std::vector<double>                           getTwistCovariance() const;
  void setActive(const bool active);
  void                                          publishUavState() const;
  void                                          publishOdom() const;
  void                                          publishCovariance() const;
  void                                          publishInnovation() const;
  std::optional<geometry_msgs::msg::Quaternion> rotateQuaternionByHeading(const geometry_msgs::msg::Quaternion &q, const double hdg) const;
  bool                                          isCompatibleWithHwApi(const mrs_msgs::msg::HwApiCapabilities::ConstSharedPtr &hw_api_capabilities) const;
};

}  // namespace mrs_uav_managers

#endif  // MRS_UAV_MANAGERS_STATE_ESTIMATOR_H
