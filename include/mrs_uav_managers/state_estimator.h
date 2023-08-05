#ifndef MRS_UAV_MANAGERS_STATE_ESTIMATOR_H
#define MRS_UAV_MANAGERS_STATE_ESTIMATOR_H

/* includes //{ */

#include <ros/ros.h>

#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>

#include <mrs_msgs/UavState.h>
#include <mrs_msgs/Float64ArrayStamped.h>
#include <mrs_msgs/HwApiCapabilities.h>

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

  ros::NodeHandle nh_;

  mrs_msgs::UavState uav_state_;
  mrs_msgs::UavState uav_state_init_;
  mutable std::mutex mtx_uav_state_;

  nav_msgs::Odometry odom_;
  mutable std::mutex mtx_odom_;

  nav_msgs::Odometry innovation_;
  nav_msgs::Odometry innovation_init_;
  mutable std::mutex mtx_innovation_;

  mrs_msgs::Float64ArrayStamped pose_covariance_, twist_covariance_;
  mutable std::mutex            mtx_covariance_;

  bool is_override_frame_id_ = false;

protected:
  mutable mrs_lib::PublisherHandler<mrs_msgs::UavState>               ph_uav_state_;
  mutable mrs_lib::PublisherHandler<nav_msgs::Odometry>               ph_odom_;
  mutable mrs_lib::PublisherHandler<mrs_msgs::Float64ArrayStamped>    ph_pose_covariance_, ph_twist_covariance_;
  mutable mrs_lib::PublisherHandler<nav_msgs::Odometry>               ph_innovation_;
  mutable mrs_lib::PublisherHandler<geometry_msgs::QuaternionStamped> ph_attitude_;

public:
  StateEstimator(const std::string &name, const std::string &frame_id, const std::string &package_name)
      : Estimator(state::type, name, frame_id), package_name_(package_name) {
  }

  virtual ~StateEstimator(void) {
  }

  virtual bool setUavState(const mrs_msgs::UavState &uav_state) = 0;

  // implemented methods
  std::optional<mrs_msgs::UavState>        getUavState();
  nav_msgs::Odometry                       getInnovation() const;
  std::vector<double>                      getPoseCovariance() const;
  std::vector<double>                      getTwistCovariance() const;
  void                                     publishUavState() const;
  void                                     publishOdom() const;
  void                                     publishCovariance() const;
  void                                     publishInnovation() const;
  std::optional<geometry_msgs::Quaternion> rotateQuaternionByHeading(const geometry_msgs::Quaternion &q, const double hdg) const;
  bool                                     isCompatibleWithHwApi(const mrs_msgs::HwApiCapabilitiesConstPtr &hw_api_capabilities) const;
};

}  // namespace mrs_uav_managers

#endif  // MRS_UAV_MANAGERS_STATE_ESTIMATOR_H
