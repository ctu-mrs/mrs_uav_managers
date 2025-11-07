#include <mrs_uav_managers/state_estimator.h>

namespace mrs_uav_managers
{

/*//{ getUavState() */
std::optional<mrs_msgs::msg::UavState> StateEstimator::getUavState() {

  if (!isRunning()) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: getUavState() was called while estimator is not running", getPrintName().c_str());
    return {};
  }

  updateUavState();

  return mrs_lib::get_mutexed(mtx_uav_state_, uav_state_);
}
/*//}*/

/*//{ getInnovation() */
nav_msgs::msg::Odometry StateEstimator::getInnovation() const {
  return mrs_lib::get_mutexed(mtx_innovation_, innovation_);
}
/*//}*/

/*//{ getPoseCovariance() */
std::vector<double> StateEstimator::getPoseCovariance() const {
  return mrs_lib::get_mutexed(mtx_covariance_, pose_covariance_.values);
}
/*//}*/

/*//{ getTwistCovariance() */
std::vector<double> StateEstimator::getTwistCovariance() const {
  return mrs_lib::get_mutexed(mtx_covariance_, twist_covariance_.values);
}
/*//}*/

/*//{ setActive() */
void StateEstimator::setActive(const bool active) {
  is_active_ = active;
}
/*//}*/

/*//{ publishUavState() */
void StateEstimator::publishUavState() const {

  if (!ch_->debug_topics.state) {
    return;
  }

  auto uav_state = mrs_lib::get_mutexed(mtx_uav_state_, uav_state_);
  ph_uav_state_.publish(uav_state);
}
/*//}*/

/*//{ publishOdom() */
void StateEstimator::publishOdom() const {

  auto odom = mrs_lib::get_mutexed(mtx_odom_, odom_);
  ph_odom_.publish(odom);
}
/*//}*/

/*//{ publishCovariance() */
void StateEstimator::publishCovariance() const {

  if (!ch_->debug_topics.covariance) {
    return;
  }

  auto pose_cov  = mrs_lib::get_mutexed(mtx_covariance_, pose_covariance_);
  auto twist_cov = mrs_lib::get_mutexed(mtx_covariance_, twist_covariance_);
  ph_pose_covariance_.publish(pose_cov);
  ph_twist_covariance_.publish(twist_cov);
}
/*//}*/

/*//{ publishInnovation() */
void StateEstimator::publishInnovation() const {

  if (!ch_->debug_topics.innovation) {
    return;
  }

  auto innovation = mrs_lib::get_mutexed(mtx_innovation_, innovation_);
  ph_innovation_.publish(innovation);
}
/*//}*/

/*//{ rotateQuaternionByHeading() */
std::optional<geometry_msgs::msg::Quaternion> StateEstimator::rotateQuaternionByHeading(const geometry_msgs::msg::Quaternion &q, const double hdg) const {

  try {
    tf2::Quaternion tf2_q = mrs_lib::AttitudeConverter(q);

    // Obtain heading from quaternion
    double q_hdg = 0;
    q_hdg        = mrs_lib::AttitudeConverter(q).getHeading();

    // Build rotation matrix from difference between new heading and quaternion heading
    tf2::Matrix3x3 rot_mat = mrs_lib::AttitudeConverter(Eigen::AngleAxisd(hdg - q_hdg, Eigen::Vector3d::UnitZ()));

    // Transform the quaternion orientation by the rotation matrix
    geometry_msgs::msg::Quaternion q_new = mrs_lib::AttitudeConverter(tf2::Transform(rot_mat) * tf2_q);
    return q_new;
  }
  catch (...) {
    RCLCPP_WARN(node_->get_logger(), "[rotateQuaternionByHeading()]: failed to rotate quaternion by heading");
    return {};
  }
}
/*//}*/

/*//{ isCompatibleWithHwApi() */
bool StateEstimator::isCompatibleWithHwApi(const mrs_msgs::msg::HwApiCapabilities::ConstSharedPtr &hw_api_capabilities) const {

  bool requires_gnss, requires_imu, requires_distance_sensor, requires_altitude, requires_magnetometer_heading, requires_position, requires_orientation,
      requires_velocity, requires_angular_velocity;

  ph_->param_loader->loadParam("requires/gnss", requires_gnss);
  ph_->param_loader->loadParam("requires/imu", requires_imu);
  ph_->param_loader->loadParam("requires/distance_sensor", requires_distance_sensor);
  ph_->param_loader->loadParam("requires/altitude", requires_altitude);
  ph_->param_loader->loadParam("requires/magnetometer_heading", requires_magnetometer_heading);
  ph_->param_loader->loadParam("requires/position", requires_position);
  ph_->param_loader->loadParam("requires/orientation", requires_orientation);
  ph_->param_loader->loadParam("requires/velocity", requires_velocity);
  ph_->param_loader->loadParam("requires/angular_velocity", requires_angular_velocity);

  if (!ph_->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Could not load all non-optional hw_api compatibility parameters. Shutting down.", getPrintName().c_str());
    rclcpp::shutdown();
  }

  if (requires_gnss && !hw_api_capabilities->produces_gnss) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: requires gnss but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  if (requires_imu && !hw_api_capabilities->produces_imu) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: requires imu but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  if (requires_distance_sensor && !hw_api_capabilities->produces_distance_sensor) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: requires distance_sensor but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  if (requires_altitude && !hw_api_capabilities->produces_altitude) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: requires altitude but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  if (requires_magnetometer_heading && !hw_api_capabilities->produces_magnetometer_heading) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: requires magnetometer_heading but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  if (requires_position && !hw_api_capabilities->produces_position) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: requires position but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  if (requires_orientation && !hw_api_capabilities->produces_orientation) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: requires orientation but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  if (requires_velocity && !hw_api_capabilities->produces_velocity) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: requires velocity but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  if (requires_angular_velocity && !hw_api_capabilities->produces_angular_velocity) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: requires angular_velocity but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  return true;
}
/*//}*/

} // namespace mrs_uav_managers
