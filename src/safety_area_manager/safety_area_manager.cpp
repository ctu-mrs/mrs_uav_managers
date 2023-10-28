#include "mrs_uav_managers/safety_area_manager/safety_area_manager.h"

// TODO: 
//    initialize atributes:
//      mutex_safety_area_min_z_
//      safety_area_min_z_
//      _safety_area_frame_
//      safety_zone_                        !!!!!
//      transformer_

namespace mrs_uav_managers
{

namespace safety_area_manager 
{

void SafetyAreaManager::onInit(){
  ROS_INFO("SafetyAreaManager: onInit called");
  publisher = nh.advertise<std_msgs::String>("chatter", 1000);
  ROS_INFO("SafetyAreaManager: topic inited");
  std_msgs::String msg;
  msg.data = "hello world";
  publisher.publish(msg);
  ROS_INFO("SafetyAreaManager: msg published");
}

bool SafetyAreaManager::isPointInSafetyArea3d(const mrs_msgs::ReferenceStamped point) {
  // copy member variables
  auto min_z = mrs_lib::get_mutexed(mutex_safety_area_min_z_, safety_area_min_z_);

  auto ret = transformer_->transformSingle(point, _safety_area_frame_);

  if (!ret) {
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: SafetyArea: Could not transform the point to the safety area frame");
      return false;
  }

  mrs_msgs::ReferenceStamped point_transformed = ret.value();

  if (safety_zone_->isPointValid3d(point_transformed.reference.position.x, point_transformed.reference.position.y, point_transformed.reference.position.z) &&
      point_transformed.reference.position.z >= min_z && point_transformed.reference.position.z <= getMaxZ(_safety_area_frame_)) {
      return true;
  }

  return false;
}

bool SafetyAreaManager::isPointInSafetyArea2d(const mrs_msgs::ReferenceStamped point) {

  auto ret = transformer_->transformSingle(point, _safety_area_frame_);

  if (!ret) {

    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: SafetyArea: Could not transform reference to the safety area frame");

    return false;
  }

  mrs_msgs::ReferenceStamped point_transformed = ret.value();

  return safety_zone_->isPointValid2d(point_transformed.reference.position.x, point_transformed.reference.position.y);
}

bool SafetyAreaManager::isPathToPointInSafetyArea3d(const mrs_msgs::ReferenceStamped start, const mrs_msgs::ReferenceStamped end) {
  mrs_msgs::ReferenceStamped start_transformed, end_transformed;

  {
    auto ret = transformer_->transformSingle(start, _safety_area_frame_);

    if (!ret) {

      ROS_ERROR("[ControlManager]: SafetyArea: Could not transform the first point in the path");

      return false;
    }

    start_transformed = ret.value();
  }

  {
    auto ret = transformer_->transformSingle(end, _safety_area_frame_);

    if (!ret) {

      ROS_ERROR("[ControlManager]: SafetyArea: Could not transform the first point in the path");

      return false;
    }

    end_transformed = ret.value();
  }

  return safety_zone_->isPathValid3d(start_transformed.reference.position.x, start_transformed.reference.position.y, start_transformed.reference.position.z,
                                    end_transformed.reference.position.x, end_transformed.reference.position.y, end_transformed.reference.position.z);
}


bool SafetyAreaManager::isPathToPointInSafetyArea2d(const mrs_msgs::ReferenceStamped start, const mrs_msgs::ReferenceStamped end) {
  mrs_msgs::ReferenceStamped start_transformed, end_transformed;

  {
    auto ret = transformer_->transformSingle(start, _safety_area_frame_);

    if (!ret) {

      ROS_ERROR("[ControlManager]: SafetyArea: Could not transform the first point in the path");

      return false;
    }

    start_transformed = ret.value();
  }

  {
    auto ret = transformer_->transformSingle(end, _safety_area_frame_);

    if (!ret) {

      ROS_ERROR("[ControlManager]: SafetyArea: Could not transform the first point in the path");

      return false;
    }

    end_transformed = ret.value();
  }

  return safety_zone_->isPathValid2d(start_transformed.reference.position.x, start_transformed.reference.position.y, end_transformed.reference.position.x,
                                    end_transformed.reference.position.y);
}

} // namespace safety_area_manager

} // namespace mrs_uav_managers


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::safety_area_manager::SafetyAreaManager, nodelet::Nodelet)
