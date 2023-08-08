#include <mrs_uav_managers/state_estimator.h>

namespace mrs_uav_managers
{


/*//{ getUavState() */
std::optional<mrs_msgs::UavState> StateEstimator::getUavState() {

  if (!isRunning()) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: getUavState() was called while estimator is not running", getPrintName().c_str());
    return {};
  }

  return mrs_lib::get_mutexed(mtx_uav_state_, uav_state_);
}
/*//}*/

/*//{ getInnovation() */
nav_msgs::Odometry StateEstimator::getInnovation() const {
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
std::optional<geometry_msgs::Quaternion> StateEstimator::rotateQuaternionByHeading(const geometry_msgs::Quaternion& q, const double hdg) const {

  try {
    tf2::Quaternion tf2_q = mrs_lib::AttitudeConverter(q);

    // Obtain heading from quaternion
    double q_hdg = 0;
    q_hdg        = mrs_lib::AttitudeConverter(q).getHeading();

    // Build rotation matrix from difference between new heading and quaternion heading
    tf2::Matrix3x3 rot_mat = mrs_lib::AttitudeConverter(Eigen::AngleAxisd(hdg - q_hdg, Eigen::Vector3d::UnitZ()));

    // Transform the quaternion orientation by the rotation matrix
    geometry_msgs::Quaternion q_new = mrs_lib::AttitudeConverter(tf2::Transform(rot_mat) * tf2_q);
    return q_new;
  }
  catch (...) {
    ROS_WARN("[rotateQuaternionByHeading()]: failed to rotate quaternion by heading");
    return {};
  }
}
/*//}*/

/*//{ isCompatibleWithHwApi() */
bool StateEstimator::isCompatibleWithHwApi(const mrs_msgs::HwApiCapabilitiesConstPtr& hw_api_capabilities) const {

  bool success = true;

  success *= ph_->loadConfigFile(ros::package::getPath(package_name_) + "/config/estimators/" + getName() + "/" + getName() + ".yaml");

  if (!success) {
    ROS_ERROR("[%s]: could not load config file", getPrintName().c_str());
    ros::shutdown();
  }

  mrs_lib::ParamLoader param_loader(nh_, getPrintName());
  param_loader.setPrefix(Support::toSnakeCase(ch_->nodelet_name) + "/" + ch_->package_name + "/" + Support::toSnakeCase(ch_->nodelet_name) + "/" + getName() + "/");

  bool requires_gnss, requires_imu, requires_distance_sensor, requires_altitude, requires_magnetometer_heading, requires_position, requires_orientation,
      requires_velocity, requires_angular_velocity;
  param_loader.loadParam("requires/gnss", requires_gnss);
  param_loader.loadParam("requires/imu", requires_imu);
  param_loader.loadParam("requires/distance_sensor", requires_distance_sensor);
  param_loader.loadParam("requires/altitude", requires_altitude);
  param_loader.loadParam("requires/magnetometer_heading", requires_magnetometer_heading);
  param_loader.loadParam("requires/position", requires_position);
  param_loader.loadParam("requires/orientation", requires_orientation);
  param_loader.loadParam("requires/velocity", requires_velocity);
  param_loader.loadParam("requires/angular_velocity", requires_angular_velocity);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional hw_api compatibility parameters. Shutting down.", getPrintName().c_str());
    ros::shutdown();
  }

  if (requires_gnss && !hw_api_capabilities->produces_gnss) {
    ROS_ERROR("[%s]: requires gnss but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  if (requires_imu && !hw_api_capabilities->produces_imu) {
    ROS_ERROR("[%s]: requires imu but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  if (requires_distance_sensor && !hw_api_capabilities->produces_distance_sensor) {
    ROS_ERROR("[%s]: requires distance_sensor but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  if (requires_altitude && !hw_api_capabilities->produces_altitude) {
    ROS_ERROR("[%s]: requires altitude but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  if (requires_magnetometer_heading && !hw_api_capabilities->produces_magnetometer_heading) {
    ROS_ERROR("[%s]: requires magnetometer_heading but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  if (requires_position && !hw_api_capabilities->produces_position) {
    ROS_ERROR("[%s]: requires position but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  if (requires_orientation && !hw_api_capabilities->produces_orientation) {
    ROS_ERROR("[%s]: requires orientation but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  if (requires_velocity && !hw_api_capabilities->produces_velocity) {
    ROS_ERROR("[%s]: requires velocity but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  if (requires_angular_velocity && !hw_api_capabilities->produces_angular_velocity) {
    ROS_ERROR("[%s]: requires angular_velocity but hw api does not provide it.", getPrintName().c_str());
    return false;
  }

  return true;
}
/*//}*/


}  // namespace mrs_uav_managers

