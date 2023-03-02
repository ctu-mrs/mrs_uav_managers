#include "estimators/state/state_estimator.h"

namespace mrs_uav_state_estimators
{

/*//{ publishUavState() */
void StateEstimator::publishUavState() const {
  std::scoped_lock lock(mtx_uav_state_);
  ph_uav_state_.publish(uav_state_);
}
/*//}*/

/*//{ publishOdom() */
void StateEstimator::publishOdom() const {

  std::scoped_lock lock(mtx_odom_);
  ph_odom_.publish(odom_);
}
/*//}*/

/*//{ publishCovariance() */
void StateEstimator::publishCovariance() const {
  std::scoped_lock lock(mtx_covariance_);
  ph_pose_covariance_.publish(pose_covariance_);
  ph_twist_covariance_.publish(twist_covariance_);
}
/*//}*/

/*//{ publishInnovation() */
void StateEstimator::publishInnovation() const {

  std::scoped_lock lock(mtx_innovation_);
  ph_innovation_.publish(innovation_);
}
/*//}*/

/*//{ rotateQuaternionByHeading() */
geometry_msgs::Quaternion StateEstimator::rotateQuaternionByHeading(const geometry_msgs::Quaternion& q, const double hdg) const {

  tf2::Quaternion tf2_q = mrs_lib::AttitudeConverter(q);

  // Obtain heading from quaternion
  double q_hdg = 0;
  try {
    q_hdg = mrs_lib::AttitudeConverter(q).getHeading();
  }
  catch (...) {
    ROS_WARN("[rotateQuaternionByHeading()]: failed to getHeading() from quaternion");
  }

  // Build rotation matrix from difference between new heading and quaternion heading
  tf2::Matrix3x3 rot_mat = mrs_lib::AttitudeConverter(Eigen::AngleAxisd(hdg - q_hdg, Eigen::Vector3d::UnitZ()));

  // Transform the quaternion orientation by the rotation matrix
  geometry_msgs::Quaternion q_new = mrs_lib::AttitudeConverter(tf2::Transform(rot_mat) * tf2_q);

  return q_new;
}
/*//}*/

/*//{ isCompatibleWithHwApi() */
bool StateEstimator::isCompatibleWithHwApi(const mrs_msgs::HwApiCapabilitiesConstPtr& hw_api_capabilities) const {

  Support::loadParamFile(ros::package::getPath(ch_->package_name) + "/config/estimators/" + getName() + ".yaml", nh_.getNamespace());

  mrs_lib::ParamLoader param_loader(nh, getPrintName());
  param_loader.setPrefix(getNamespacedName() + "/");

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

}  // namespace mrs_uav_state_estimators

