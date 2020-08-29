
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_broadcaster.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>

#include <string>

namespace mrs_uav_tf_manager
{

class TfManager : public nodelet::Nodelet {

public:
  virtual void onInit();
  bool         is_initialized_ = false;

  void            callbackMavrosOdometry(const nav_msgs::OdometryConstPtr& msg);
  ros::Subscriber sub_odom_mavros_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  mrs_lib::Transformer                           transformer_;

  // frame names
  std::string fcu_frame_id_, fcu_untilted_frame_id_;

  // profiler
  mrs_lib::Profiler profiler_;

  // support functions
  bool noNans(const geometry_msgs::TransformStamped& tf);
};

}  // namespace mrs_uav_tf_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_tf_manager::TfManager, nodelet::Nodelet)
