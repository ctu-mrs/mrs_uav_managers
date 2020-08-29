#include "tf_manager.h"

namespace mrs_uav_tf_manager
{

/* //{ onInit() */

void TfManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[TfManager]: TfManager node initializing");

  // --------------------------------------------------------------
  // |                         parameters                         |
  // --------------------------------------------------------------

  /* parameters //{ */

  ROS_INFO("[TfManager]: Loading parameters");

  mrs_lib::ParamLoader param_loader(nh_, "TfManager");

  param_loader.loadParam("frames/fcu_frame_id", fcu_frame_id_);
  param_loader.loadParam("frames/fcu_untilted_frame_id", fcu_untilted_frame_id_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[TfManager]: Could not load all non-optional parameters. Shutting down.");
    ros::shutdown();
  }
  //}

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------
  /* subscribers //{ */

  // subscribe to mavros odometry
  sub_odom_mavros_ = nh_.subscribe("odom_mavros_in", 1, &TfManager::callbackMavrosOdometry, this, ros::TransportHints().tcpNoDelay());


  // --------------------------------------------------------------
  // |                             TF                             |
  // --------------------------------------------------------------

  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

  //}

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[TfManager]: initialized");
}

//}

/* //{ callbackMavrosOdometry() */
void TfManager::callbackMavrosOdometry(const nav_msgs::OdometryConstPtr &msg) {


  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackMavrosOdometry");

  // Obtain heading from UAV orientation
  double heading;
  try {
    heading = mrs_lib::AttitudeConverter(msg->pose.pose.orientation).getHeading();
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during getting heading");
    return;
  }

  // We need to undo the heading
  Eigen::Matrix3d odom_pixhawk_R = mrs_lib::AttitudeConverter(msg->pose.pose.orientation);
  Eigen::Matrix3d undo_heading_R = mrs_lib::AttitudeConverter(Eigen::AngleAxis(-heading, Eigen::Vector3d(0, 0, 1)));

  tf2::Quaternion q;
  q = mrs_lib::AttitudeConverter(undo_heading_R * odom_pixhawk_R);
  q = q.inverse();

  // Fill tf msg
  geometry_msgs::TransformStamped tf;
  tf.header.stamp            = ros::Time::now();
  tf.header.frame_id         = fcu_frame_id_;
  tf.child_frame_id          = fcu_untilted_frame_id_;
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation      = mrs_lib::AttitudeConverter(q);

  // Publish tf
  if (noNans(tf)) {
    try {
      broadcaster_->sendTransform(tf);
    }
    catch (...) {
      ROS_ERROR("[TfManager]: Exception caught during publishing TF: %s - %s.", tf.child_frame_id.c_str(), tf.header.frame_id.c_str());
    }
  } else {
    ROS_ERROR("[TfManager]: NaN detected in tf. Not publishing.");
  }
}

//}

/* noNans() //{ */
bool TfManager::noNans(const geometry_msgs::TransformStamped& tf) {

  return (std::isfinite(tf.transform.rotation.x) && std::isfinite(tf.transform.rotation.y) && std::isfinite(tf.transform.rotation.z) && std::isfinite(tf.transform.rotation.w));
}

}  // namespace mrs_uav_tf_manager
