#include <mrs_uav_managers/agl_estimator.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

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
bool AglEstimator::isCompatibleWithHwApi(const mrs_msgs::msg::HwApiCapabilities::ConstSharedPtr &hw_api_capabilities) const {

  ph_->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/private/" + getName() + "/" + getName() + ".yaml");
  ph_->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/public/" + getName() + "/" + getName() + ".yaml");

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
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
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
