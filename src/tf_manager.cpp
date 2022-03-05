#define VERSION "1.0.2.0"

/*//{ includes */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <tf2_ros/transform_broadcaster.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/mutex.h>

#include <string>

/*//}*/

namespace mrs_uav_managers
{

namespace tf_manager
{

/*//{ class TfManager */
class TfManager : public nodelet::Nodelet {

public:
  virtual void onInit();
  bool         is_initialized_ = false;

  void            callbackMavrosOdometry(const nav_msgs::OdometryConstPtr& msg);
  ros::Subscriber sub_odom_mavros_;

  void            callbackImu(const sensor_msgs::ImuConstPtr& msg);
  ros::Subscriber sub_imu_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  // frame names
  std::string fcu_frame_id_, fcu_untilted_frame_id_;

  // profiler
  mrs_lib::Profiler profiler_;

  // scope timer logger
  bool                                       scope_timer_enabled_ = false;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;

  // support functions
  void publishTf(const geometry_msgs::Quaternion& q_in);

  bool noNans(const geometry_msgs::TransformStamped& tf);
};

/*//}*/

/* //{ onInit() */

void TfManager::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[TfManager]: initializing");

  // --------------------------------------------------------------
  // |                         parameters                         |
  // --------------------------------------------------------------

  /* parameters //{ */

  ROS_INFO("[TfManager]: Loading parameters");

  mrs_lib::ParamLoader param_loader(nh, "TfManager");

  param_loader.loadParam("frames/fcu_frame_id", fcu_frame_id_);
  param_loader.loadParam("frames/fcu_untilted_frame_id", fcu_untilted_frame_id_);

  bool imu_mode = false;
  param_loader.loadParam("imu_mode", imu_mode);

  // | ------------------- scope timer logger ------------------- |

  param_loader.loadParam("scope_timer/enabled", scope_timer_enabled_);
  const std::string scope_timer_log_filename = param_loader.loadParam2("scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<mrs_lib::ScopeTimerLogger>(scope_timer_log_filename, scope_timer_enabled_);

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

  if (imu_mode) {

    ROS_INFO("[TfManager]: using IMU data for publishing of untilted frame");
    sub_imu_ = nh.subscribe("imu_in", 1, &TfManager::callbackImu, this, ros::TransportHints().tcpNoDelay());

  } else {

    ROS_INFO("[TfManager]: using odometry data for publishing of untilted frame");
    sub_odom_mavros_ = nh.subscribe("odom_mavros_in", 1, &TfManager::callbackMavrosOdometry, this, ros::TransportHints().tcpNoDelay());
  }


  // --------------------------------------------------------------
  // |                             TF                             |
  // --------------------------------------------------------------

  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

  //}

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[TfManager]: initialized, version %s", VERSION);

  ROS_DEBUG("[TfManager]: debug output is enabled");
}

//}

// | ------------------------ callbacks ----------------------- |

/* //{ callbackMavrosOdometry() */
void TfManager::callbackMavrosOdometry(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackMavrosOdometry");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("TfManager::callbackMavrosOdometry", scope_timer_logger_, scope_timer_enabled_);

  publishTf(msg->pose.pose.orientation);
}

//}

/* //{ callbackImu() */
void TfManager::callbackImu(const sensor_msgs::ImuConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackImu");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("TfManager::callbackImu", scope_timer_logger_, scope_timer_enabled_);

  publishTf(msg->orientation);
}

//}

// | ------------------------ routines ------------------------ |

/* publishTf() //{ */

void TfManager::publishTf(const geometry_msgs::Quaternion& q_in) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("publishTf");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("TfManager::publishTf", scope_timer_logger_, scope_timer_enabled_);

  // Obtain heading from UAV orientation
  double heading;
  try {
    heading = mrs_lib::AttitudeConverter(q_in).getHeading();
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during getting heading");
    return;
  }

  // We need to undo the heading
  Eigen::Matrix3d odom_pixhawk_R = mrs_lib::AttitudeConverter(q_in);
  Eigen::Matrix3d undo_heading_R = mrs_lib::AttitudeConverter(Eigen::AngleAxis(-heading, Eigen::Vector3d(0, 0, 1)));

  tf2::Quaternion q_headingless;
  q_headingless = mrs_lib::AttitudeConverter(undo_heading_R * odom_pixhawk_R);
  q_headingless = q_headingless.inverse();

  // Fill tf msg
  geometry_msgs::TransformStamped tf;
  tf.header.stamp            = ros::Time::now();
  tf.header.frame_id         = fcu_frame_id_;
  tf.child_frame_id          = fcu_untilted_frame_id_;
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation      = mrs_lib::AttitudeConverter(q_headingless);

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

  ROS_INFO_ONCE("[TfManager]: published first TF: %s -> %s", fcu_frame_id_.c_str(), fcu_untilted_frame_id_.c_str());
}

//}

/* noNans() //{ */
bool TfManager::noNans(const geometry_msgs::TransformStamped& tf) {

  return (std::isfinite(tf.transform.rotation.x) && std::isfinite(tf.transform.rotation.y) && std::isfinite(tf.transform.rotation.z) &&
          std::isfinite(tf.transform.rotation.w));
}

//}

}  // namespace tf_manager

}  // namespace mrs_uav_managers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::tf_manager::TfManager, nodelet::Nodelet)
