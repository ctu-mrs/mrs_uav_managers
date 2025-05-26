#ifndef MRS_UAV_MANAGERS_AGL_ESTIMATOR_H
#define MRS_UAV_MANAGERS_AGL_ESTIMATOR_H

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>

#include <nav_msgs/msg/odometry.hpp>

#include <mrs_msgs/msg/uav_state.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>
#include <mrs_msgs/msg/float64_array_stamped.hpp>
#include <mrs_msgs/msg/hw_api_capabilities.hpp>

#include <mrs_uav_managers/estimation_manager/estimator.h>
#include <mrs_uav_managers/estimation_manager/support.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

//}

namespace mrs_uav_managers
{

namespace agl
{
const char type[] = "AGL";
}

using namespace estimation_manager;

class AglEstimator : public Estimator {

protected:
  const std::string package_name_ = "mrs_uav_state_estimators";

  mrs_msgs::msg::Float64Stamped agl_height_;
  mrs_msgs::msg::Float64Stamped agl_height_init_;
  mutable std::mutex            mtx_agl_height_;

  mrs_msgs::msg::Float64ArrayStamped agl_height_cov_;
  mrs_msgs::msg::Float64ArrayStamped agl_height_cov_init_;
  mutable std::mutex                 mtx_agl_height_cov_;

  bool is_override_frame_id_ = false;

protected:
  mutable mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>      ph_agl_height_;
  mutable mrs_lib::PublisherHandler<mrs_msgs::msg::Float64ArrayStamped> ph_agl_height_cov_;

public:
  AglEstimator(const std::string &name, const std::string &frame_id, const std::string &package_name)
      : Estimator(agl::type, name, frame_id), package_name_(package_name) {
  }

  // virtual methods
  virtual mrs_msgs::msg::Float64Stamped getUavAglHeight() const     = 0;
  virtual std::vector<double>           getHeightCovariance() const = 0;

  // implemented methods
  void publishAglHeight() const;
  void publishCovariance() const;
  bool isCompatibleWithHwApi(const mrs_msgs::msg::HwApiCapabilities::ConstSharedPtr &hw_api_capabilities) const;
};

}  // namespace mrs_uav_managers

#endif  // MRS_UAV_MANAGERS_AGL_ESTIMATOR_H
