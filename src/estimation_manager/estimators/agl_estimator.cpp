#include <mrs_uav_managers/agl_estimator.h>

namespace mrs_uav_managers
{

/*//{ publishAglHeight() */
void AglEstimator::publishAglHeight() const {
  ph_agl_height_.publish(mrs_lib::get_mutexed(mtx_agl_height_, agl_height_));
}
/*//}*/

/*//{ publishCovariance() */
void AglEstimator::publishCovariance() const {
  
  if (!ch_->debug_topics.covariance) {
    return;
  }

  ph_agl_height_cov_.publish(mrs_lib::get_mutexed(mtx_agl_height_, agl_height_cov_));
}
/*//}*/

/*//{ isCompatibleWithHwApi() */
bool AglEstimator::isCompatibleWithHwApi(const mrs_msgs::HwApiCapabilitiesConstPtr& hw_api_capabilities) const {

  bool success = true;

  success *= ph_->loadConfigFile(ros::package::getPath(package_name_) + "/config/estimators/" + getName() + "/" + getName() + ".yaml");

  if (!success) {
    ROS_ERROR("[%s]: could not load config file", getPrintName().c_str());
    ros::shutdown();
  }

  mrs_lib::ParamLoader param_loader(nh_, getPrintName());
  param_loader.setPrefix(ch_->package_name + "/" + Support::toSnakeCase(ch_->nodelet_name) + "/" + getName() + "/");

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
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
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

