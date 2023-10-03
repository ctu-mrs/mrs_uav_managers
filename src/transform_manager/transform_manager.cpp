/* //{ includes */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/transform_broadcaster.h>
#include <mrs_lib/gps_conversions.h>

#include <mrs_msgs/UavState.h>
#include <mrs_msgs/Float64Stamped.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/NavSatFix.h>

#include <nav_msgs/Odometry.h>

#include <memory>
#include <string>

#include <mrs_uav_managers/estimation_manager/support.h>
#include <mrs_uav_managers/estimation_manager/common_handlers.h>
#include <transform_manager/tf_source.h>

/*//}*/

namespace mrs_uav_managers
{

namespace transform_manager
{

/*//{ class TransformManager */
class TransformManager : public nodelet::Nodelet {

  using Support = estimation_manager::Support;

public:
  TransformManager() {
    ch_ = std::make_shared<estimation_manager::CommonHandlers_t>();

    ch_->nodelet_name = nodelet_name_;
    ch_->package_name = package_name_;
  }

  void onInit();
  bool is_initialized_ = false;

  std::string getName() const;

  std::string getPrintName() const;

private:
  const std::string package_name_ = "mrs_uav_managers";
  const std::string nodelet_name_ = "TransformManager";
  const std::string name_         = "transform_manager";

  bool publish_fcu_untilted_tf_;

  std::string ns_local_origin_parent_frame_id_;
  std::string ns_local_origin_child_frame_id_;
  bool        publish_local_origin_tf_;

  std::string ns_stable_origin_parent_frame_id_;
  std::string ns_stable_origin_child_frame_id_;
  bool        publish_stable_origin_tf_;

  std::string         ns_fixed_origin_parent_frame_id_;
  std::string         ns_fixed_origin_child_frame_id_;
  bool                publish_fixed_origin_tf_;
  geometry_msgs::Pose pose_fixed_;
  geometry_msgs::Pose pose_fixed_diff_;

  std::string          world_origin_units_;
  geometry_msgs::Point world_origin_;

  std::vector<std::string>               tf_source_names_, estimator_names_;
  std::vector<std::unique_ptr<TfSource>> tf_sources_;

  std::vector<std::string> utm_source_priority_list_;
  std::string              utm_source_name;

  std::mutex mtx_broadcast_utm_origin_;
  std::mutex mtx_broadcast_world_origin_;


  ros::NodeHandle nh_;

  std::shared_ptr<estimation_manager::CommonHandlers_t> ch_;

  std::shared_ptr<mrs_lib::TransformBroadcaster> broadcaster_;

  void timeoutCallback(const std::string& topic, const ros::Time& last_msg);

  mrs_lib::SubscribeHandler<mrs_msgs::UavState> sh_uav_state_;
  void                                          callbackUavState(const mrs_msgs::UavState::ConstPtr msg);
  std::string                                   first_frame_id_;
  std::string                                   last_frame_id_;
  bool                                          is_first_frame_id_set_ = false;

  mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped> sh_height_agl_;
  void                                                callbackHeightAgl(const mrs_msgs::Float64Stamped::ConstPtr msg);

  mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped> sh_hw_api_orientation_;
  void                                                        callbackHwApiOrientation(const geometry_msgs::QuaternionStamped::ConstPtr msg);

  mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> sh_gnss_;
  void                                              callbackGnss(const sensor_msgs::NavSatFix::ConstPtr msg);
  std::atomic<bool>                                 got_utm_offset_ = false;


  bool        slam_mapping_origin_tf_enabled_;
  bool        slam_mapping_origin_tf_inverted_;
  bool        slam_mapping_origin_tf_custom_frame_id_enabled_;
  std::string slam_mapping_origin_tf_custom_frame_id_;
  double      slam_mapping_origin_tf_cache_duration_;

  std::string                                   slam_mapping_origin_tf_lateral_topic_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_mapping_odom_lat_;
  void                                          callbackMappingOdomLat(const nav_msgs::Odometry::ConstPtr msg);
  std::vector<nav_msgs::Odometry>               vec_mapping_odom_lat_;

  std::string                                                 slam_mapping_origin_tf_orientation_topic_;
  mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped> sh_mapping_odom_rot_;
  void                                                        callbackMappingOdomRot(const geometry_msgs::QuaternionStamped::ConstPtr msg);
  std::vector<geometry_msgs::QuaternionStamped>               vec_mapping_odom_rot_;
  std::mutex                                                  mtx_mapping_odom_rot_;
  bool                                                        got_mapping_odom_rot_ = false;

  std::string                                   slam_mapping_origin_tf_altitude_topic_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_mapping_odom_alt_;
  void                                          callbackMappingOdomAlt(const nav_msgs::Odometry::ConstPtr msg);
  std::vector<nav_msgs::Odometry>               vec_mapping_odom_alt_;
  std::mutex                                    mtx_mapping_odom_alt_;
  bool                                          got_mapping_odom_alt_ = false;

  void publishFcuUntiltedTf(const geometry_msgs::QuaternionStampedConstPtr& msg);
};
/*//}*/


/*//{ onInit() */
void TransformManager::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[%s]: initializing", getPrintName().c_str());

  broadcaster_ = std::make_shared<mrs_lib::TransformBroadcaster>();

  ch_->transformer = std::make_shared<mrs_lib::Transformer>(nh_, getPrintName());
  ch_->transformer->retryLookupNewest(true);

  mrs_lib::ParamLoader param_loader(nh_, getPrintName());

  const std::string yaml_prefix = "mrs_uav_managers/transform_manager/";

  param_loader.loadParam("uav_name", ch_->uav_name);

  /*//{ initialize scope timer */
  param_loader.loadParam(yaml_prefix + "scope_timer/enabled", ch_->scope_timer.enabled);
  std::string       filepath;
  const std::string time_logger_filepath = ros::package::getPath(package_name_) + "/scope_timer/transform_manager_scope_timer.txt";
  ch_->scope_timer.logger                = std::make_shared<mrs_lib::ScopeTimerLogger>(time_logger_filepath, ch_->scope_timer.enabled);
  /*//}*/

  /*//{ load world_origin parameters */

  bool   is_origin_param_ok = true;
  double world_origin_x     = 0;
  double world_origin_y     = 0;

  param_loader.loadParam("world_origin_units", world_origin_units_);

  if (Support::toLowercase(world_origin_units_) == "utm") {
    ROS_INFO("[%s]: Loading world origin in UTM units.", getPrintName().c_str());
    is_origin_param_ok &= param_loader.loadParam("world_origin_x", world_origin_x);
    is_origin_param_ok &= param_loader.loadParam("world_origin_y", world_origin_y);

  } else if (Support::toLowercase(world_origin_units_) == "latlon") {
    double lat, lon;
    ROS_INFO("[%s]: Loading world origin in LatLon units.", getPrintName().c_str());
    is_origin_param_ok &= param_loader.loadParam("world_origin_x", lat);
    is_origin_param_ok &= param_loader.loadParam("world_origin_y", lon);
    mrs_lib::UTM(lat, lon, &world_origin_x, &world_origin_y);
    ROS_INFO("[%s]: Converted to UTM x: %f, y: %f.", getPrintName().c_str(), world_origin_x, world_origin_y);

  } else {
    ROS_ERROR("[%s]: world_origin_units must be (\"UTM\"|\"LATLON\"). Got '%s'", getPrintName().c_str(), world_origin_units_.c_str());
    ros::shutdown();
  }

  world_origin_.x = world_origin_x;
  world_origin_.y = world_origin_y;
  world_origin_.z = 0;

  if (!is_origin_param_ok) {
    ROS_ERROR("[%s]: Could not load all mandatory parameters from world file. Please check your world file.", getPrintName().c_str());
    ros::shutdown();
  }
  /*//}*/

  /*//{ load local_origin parameters */
  std::string local_origin_parent_frame_id;
  param_loader.loadParam(yaml_prefix + "local_origin_tf/parent", local_origin_parent_frame_id);
  ns_local_origin_parent_frame_id_ = ch_->uav_name + "/" + local_origin_parent_frame_id;

  std::string local_origin_child_frame_id;
  param_loader.loadParam(yaml_prefix + "local_origin_tf/child", local_origin_child_frame_id);
  ns_local_origin_child_frame_id_ = ch_->uav_name + "/" + local_origin_child_frame_id;

  param_loader.loadParam(yaml_prefix + "local_origin_tf/enabled", publish_local_origin_tf_);
  /*//}*/

  /*//{ load stable_origin parameters */
  std::string stable_origin_parent_frame_id;
  param_loader.loadParam(yaml_prefix + "stable_origin_tf/parent", stable_origin_parent_frame_id);
  ns_stable_origin_parent_frame_id_ = ch_->uav_name + "/" + stable_origin_parent_frame_id;

  std::string stable_origin_child_frame_id;
  param_loader.loadParam(yaml_prefix + "stable_origin_tf/child", stable_origin_child_frame_id);
  ns_stable_origin_child_frame_id_ = ch_->uav_name + "/" + stable_origin_child_frame_id;

  param_loader.loadParam(yaml_prefix + "stable_origin_tf/enabled", publish_stable_origin_tf_);
  /*//}*/

  /*//{ load fixed_origin parameters */
  std::string fixed_origin_parent_frame_id;
  param_loader.loadParam(yaml_prefix + "fixed_origin_tf/parent", fixed_origin_parent_frame_id);
  ns_fixed_origin_parent_frame_id_ = ch_->uav_name + "/" + fixed_origin_parent_frame_id;

  std::string fixed_origin_child_frame_id;
  param_loader.loadParam(yaml_prefix + "fixed_origin_tf/child", fixed_origin_child_frame_id);
  ns_fixed_origin_child_frame_id_ = ch_->uav_name + "/" + fixed_origin_child_frame_id;

  param_loader.loadParam(yaml_prefix + "fixed_origin_tf/enabled", publish_fixed_origin_tf_);
  /*//}*/

  /*//{ load fcu_untilted parameters */
  std::string fcu_frame_id;
  param_loader.loadParam(yaml_prefix + "fcu_untilted_tf/parent", fcu_frame_id);
  ch_->frames.fcu    = fcu_frame_id;
  ch_->frames.ns_fcu = ch_->uav_name + "/" + fcu_frame_id;

  std::string fcu_untilted_frame_id;
  param_loader.loadParam(yaml_prefix + "fcu_untilted_tf/child", fcu_untilted_frame_id);
  ch_->frames.fcu_untilted    = fcu_untilted_frame_id;
  ch_->frames.ns_fcu_untilted = ch_->uav_name + "/" + fcu_untilted_frame_id;

  param_loader.loadParam(yaml_prefix + "fcu_untilted_tf/enabled", publish_fcu_untilted_tf_);
  /*//}*/

  /*//{ load slam mapping origin parameters */
  param_loader.loadParam(yaml_prefix + "/slam_mapping_origin_tf/enabled", slam_mapping_origin_tf_enabled_);
  if (slam_mapping_origin_tf_enabled_) {
    param_loader.loadParam(yaml_prefix + "/slam_mapping_origin_tf/lateral_topic", slam_mapping_origin_tf_lateral_topic_);
    param_loader.loadParam(yaml_prefix + "/slam_mapping_origin_tf/altitude_topic", slam_mapping_origin_tf_altitude_topic_);
    param_loader.loadParam(yaml_prefix + "/slam_mapping_origin_tf/orientation_topic", slam_mapping_origin_tf_orientation_topic_);
    param_loader.loadParam(yaml_prefix + "/slam_mapping_origin_tf/inverted", slam_mapping_origin_tf_inverted_);
    param_loader.loadParam(yaml_prefix + "/slam_mapping_origin_tf/custom_frame_id/enabled", slam_mapping_origin_tf_custom_frame_id_enabled_);
    if (slam_mapping_origin_tf_custom_frame_id_enabled_) {
      param_loader.loadParam(yaml_prefix + "/slam_mapping_origin_tf/custom_frame_id/frame_id", slam_mapping_origin_tf_custom_frame_id_);
    }
  }
  /*//}*/

  param_loader.loadParam("mrs_uav_managers/estimation_manager/state_estimators", estimator_names_);
  param_loader.loadParam(yaml_prefix + "tf_sources", tf_source_names_);

  param_loader.loadParam(yaml_prefix + "utm_source_priority", utm_source_priority_list_);
  for (auto utm_source : utm_source_priority_list_) {
    if (Support::isStringInVector(utm_source, estimator_names_)) {
      ROS_INFO("[%s]: the source for utm_origin and world origin is: %s", getPrintName().c_str(), utm_source.c_str());
      utm_source_name = utm_source;
      break;
    }
  }

  /*//{ initialize tf sources */
  for (size_t i = 0; i < tf_source_names_.size(); i++) {
    const std::string tf_source_name = tf_source_names_[i];
    const bool        is_utm_source  = tf_source_name == utm_source_name;
    ROS_INFO("[%s]: loading tf source: %s", getPrintName().c_str(), tf_source_name.c_str());
    tf_sources_.push_back(std::make_unique<TfSource>(tf_source_name, nh_, broadcaster_, ch_, is_utm_source));
  }

  // additionally publish tf of all available estimators
  for (int i = 0; i < int(estimator_names_.size()); i++) {
    const std::string estimator_name = estimator_names_[i];
    const bool        is_utm_source  = estimator_name == utm_source_name;
    ROS_INFO("[%s]: loading tf source of estimator: %s", getPrintName().c_str(), estimator_name.c_str());
    tf_sources_.push_back(std::make_unique<TfSource>(estimator_name, nh_, broadcaster_, ch_, is_utm_source));
  }
  //}

  /*//{ initialize subscribers */
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = getPrintName();
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_uav_state_ = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in", &TransformManager::callbackUavState, this);

  sh_height_agl_ = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(shopts, "height_agl_in", &TransformManager::callbackHeightAgl, this);

  sh_hw_api_orientation_ =
      mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped>(shopts, "orientation_in", &TransformManager::callbackHwApiOrientation, this);

  sh_gnss_ = mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>(shopts, "gnss_in", &TransformManager::callbackGnss, this);

  if (slam_mapping_origin_tf_enabled_) {
    sh_mapping_odom_lat_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "/" + ch_->uav_name + "/" + slam_mapping_origin_tf_lateral_topic_,
                                                                         &TransformManager::callbackMappingOdomLat, this);

    if (slam_mapping_origin_tf_orientation_topic_ != slam_mapping_origin_tf_lateral_topic_) {
      sh_mapping_odom_rot_ = mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped>(
          shopts, "/" + ch_->uav_name + "/" + slam_mapping_origin_tf_orientation_topic_.c_str(), &TransformManager::callbackMappingOdomRot, this);
    }

    if (slam_mapping_origin_tf_altitude_topic_ != slam_mapping_origin_tf_lateral_topic_) {
      sh_mapping_odom_alt_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "/" + ch_->uav_name + "/" + slam_mapping_origin_tf_altitude_topic_.c_str(),
                                                                           &TransformManager::callbackMappingOdomAlt, this);
    }
  }
  /*//}*/

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
    ros::shutdown();
  }

  is_initialized_ = true;
  ROS_INFO("[%s]: initialized", getPrintName().c_str());
}
/*//}*/

/*//{ callbackUavState() */

void TransformManager::callbackUavState(const mrs_msgs::UavState::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("TransformManager::publishFcuUntilted", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  // obtain first frame_id
  if (!is_first_frame_id_set_) {
    first_frame_id_                = msg->header.frame_id;
    last_frame_id_                 = msg->header.frame_id;
    pose_fixed_                    = msg->pose;
    pose_fixed_diff_.orientation.w = 1;
    is_first_frame_id_set_         = true;
  }

  if (publish_local_origin_tf_) {
    /*//{ publish local_origin tf*/
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp    = msg->header.stamp;
    tf_msg.header.frame_id = ns_local_origin_parent_frame_id_;
    tf_msg.child_frame_id  = ns_local_origin_child_frame_id_;

    // transform pose to first frame_id
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose   = msg->pose;

    if (pose.pose.orientation.w == 0 && pose.pose.orientation.z == 0 && pose.pose.orientation.y == 0 && pose.pose.orientation.x == 0) {
      ROS_WARN_ONCE("[%s]: Uninitialized quaternion detected during publishing stable_origin tf of %s. Setting w=1", getPrintName().c_str(),
                    pose.header.frame_id.c_str());
      pose.pose.orientation.w = 1.0;
    }

    auto res = ch_->transformer->transformSingle(pose, first_frame_id_.substr(0, first_frame_id_.find("_origin")) + "_local_origin");

    if (res) {
      const tf2::Transform      tf       = Support::tf2FromPose(res->pose);
      const tf2::Transform      tf_inv   = tf.inverse();
      const geometry_msgs::Pose pose_inv = Support::poseFromTf2(tf_inv);
      tf_msg.transform.translation       = Support::pointToVector3(pose_inv.position);
      tf_msg.transform.rotation          = pose_inv.orientation;

      if (Support::noNans(tf_msg)) {
        try {
          broadcaster_->sendTransform(tf_msg);
        }
        catch (...) {
          ROS_ERROR("exception caught ");
        }
      } else {
        ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getPrintName().c_str(), tf_msg.header.frame_id.c_str(),
                          tf_msg.child_frame_id.c_str());
      }
      ROS_INFO_ONCE("[%s]: Broadcasting transform from parent frame: %s to child frame: %s", getPrintName().c_str(), tf_msg.header.frame_id.c_str(),
                    tf_msg.child_frame_id.c_str());
    } else {
      ROS_ERROR_THROTTLE(1.0, "[%s]: Could not transform pose to %s. Not publishing local_origin transform.", getPrintName().c_str(), first_frame_id_.c_str());
      return;
    }
    /*//}*/
  }

  if (publish_stable_origin_tf_) {
    /*//{ publish stable_origin tf*/
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp    = msg->header.stamp;
    tf_msg.header.frame_id = ns_stable_origin_parent_frame_id_;
    tf_msg.child_frame_id  = ns_stable_origin_child_frame_id_;

    // transform pose to first frame_id
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose   = msg->pose;
    if (pose.pose.orientation.w == 0 && pose.pose.orientation.z == 0 && pose.pose.orientation.y == 0 && pose.pose.orientation.x == 0) {
      ROS_WARN_ONCE("[%s]: Uninitialized quaternion detected during publishing stable_origin tf of %s. Setting w=1", getPrintName().c_str(),
                    pose.header.frame_id.c_str());
      pose.pose.orientation.w = 1.0;
    }

    auto res = ch_->transformer->transformSingle(pose, first_frame_id_);

    if (res) {
      const tf2::Transform      tf       = Support::tf2FromPose(res->pose);
      const tf2::Transform      tf_inv   = tf.inverse();
      const geometry_msgs::Pose pose_inv = Support::poseFromTf2(tf_inv);
      tf_msg.transform.translation       = Support::pointToVector3(pose_inv.position);
      tf_msg.transform.rotation          = pose_inv.orientation;

      if (Support::noNans(tf_msg)) {
        try {
          broadcaster_->sendTransform(tf_msg);
        }
        catch (...) {
          ROS_ERROR("exception caught ");
        }
      } else {
        ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getPrintName().c_str(), tf_msg.header.frame_id.c_str(),
                          tf_msg.child_frame_id.c_str());
      }
      ROS_INFO_ONCE("[%s]: Broadcasting transform from parent frame: %s to child frame: %s", getPrintName().c_str(), tf_msg.header.frame_id.c_str(),
                    tf_msg.child_frame_id.c_str());
    } else {
      ROS_ERROR_THROTTLE(1.0, "[%s]: Could not transform pose to %s. Not publishing stable_origin transform.", getPrintName().c_str(), first_frame_id_.c_str());
      return;
    }
    /*//}*/
  }

  if (publish_fixed_origin_tf_) {
    /*//{ publish fixed_origin tf*/
    if (msg->header.frame_id != last_frame_id_) {
      ROS_WARN("[%s]: Detected estimator change from %s to %s. Updating offset for fixed origin.", getPrintName().c_str(), last_frame_id_.c_str(),
               msg->header.frame_id.c_str());

      pose_fixed_diff_ = Support::getPoseDiff(msg->pose, pose_fixed_);
    }


    pose_fixed_ = Support::applyPoseDiff(msg->pose, pose_fixed_diff_);

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp    = msg->header.stamp;
    tf_msg.header.frame_id = ns_fixed_origin_parent_frame_id_;
    tf_msg.child_frame_id  = ns_fixed_origin_child_frame_id_;

    const tf2::Transform      tf       = Support::tf2FromPose(pose_fixed_);
    const tf2::Transform      tf_inv   = tf.inverse();
    const geometry_msgs::Pose pose_inv = Support::poseFromTf2(tf_inv);
    tf_msg.transform.translation       = Support::pointToVector3(pose_inv.position);
    tf_msg.transform.rotation          = pose_inv.orientation;

    if (Support::noNans(tf_msg)) {
      try {
        broadcaster_->sendTransform(tf_msg);
      }
      catch (...) {
        ROS_ERROR("exception caught ");
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getPrintName().c_str(), tf_msg.header.frame_id.c_str(),
                        tf_msg.child_frame_id.c_str());
    }
    ROS_INFO_ONCE("[%s]: Broadcasting transform from parent frame: %s to child frame: %s", getPrintName().c_str(), tf_msg.header.frame_id.c_str(),
                  tf_msg.child_frame_id.c_str());
    /*//}*/
  }

  /*//{ choose another source of utm and world tfs after estimator switch */
  if (msg->header.frame_id != last_frame_id_) {
    const std::string last_estimator_name    = Support::frameIdToEstimatorName(last_frame_id_);
    const std::string current_estimator_name = Support::frameIdToEstimatorName(msg->header.frame_id);

    ROS_INFO("[%s]: Detected estimator switch: %s -> %s", getPrintName().c_str(), last_estimator_name.c_str(), current_estimator_name.c_str());

    bool   valid_utm_source_found = false;
    size_t potential_utm_source_index;

    for (size_t i = 0; i < tf_sources_.size(); i++) {

      // first check if tf source can publish utm origin and is not the switched from estimator
      if (tf_sources_.at(i)->getIsUtmBased() && tf_sources_.at(i)->getName() != last_estimator_name) {

        valid_utm_source_found     = true;
        potential_utm_source_index = i;

        // check if switched to estimator is utm_based, if so, use it
        if (tf_sources_.at(i)->getIsUtmBased() && tf_sources_.at(i)->getName() == current_estimator_name) {
          potential_utm_source_index = i;
          break;
        }
      }
    }


    // if we found a valid utm source, use it, otherwise stay with the switched from estimator
    if (valid_utm_source_found) {

      tf_sources_.at(potential_utm_source_index)->setIsUtmSource(true);
      ROS_INFO("[%s]: setting is_utm_source of estimator %s to true", getPrintName().c_str(), current_estimator_name.c_str());

      // stop previous estimator from publishing utm source
      for (size_t i = 0; i < tf_sources_.size(); i++) {
        if (tf_sources_.at(i)->getName() == last_estimator_name) {
          tf_sources_.at(i)->setIsUtmSource(false);
          ROS_INFO("[%s]: setting is_utm_source of estimator %s to false", getPrintName().c_str(), last_estimator_name.c_str());
        }
      }
    }
  }
  /*//}*/

  last_frame_id_ = msg->header.frame_id;
}
/*//}*/

/*//{ callbackHeightAgl() */

void TransformManager::callbackHeightAgl(const mrs_msgs::Float64Stamped::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("TransformManager::publish>HeightAgl", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp    = msg->header.stamp;
  tf_msg.header.frame_id = ch_->frames.ns_fcu_untilted;
  tf_msg.child_frame_id  = msg->header.frame_id;

  tf_msg.transform.translation.x = 0;
  tf_msg.transform.translation.y = 0;
  tf_msg.transform.translation.z = -msg->value;
  tf_msg.transform.rotation.x    = 0;
  tf_msg.transform.rotation.y    = 0;
  tf_msg.transform.rotation.z    = 0;
  tf_msg.transform.rotation.w    = 1;

  if (Support::noNans(tf_msg)) {
    try {
      broadcaster_->sendTransform(tf_msg);
    }
    catch (...) {
      ROS_ERROR("exception caught ");
    }
  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getPrintName().c_str(), tf_msg.header.frame_id.c_str(),
                      tf_msg.child_frame_id.c_str());
  }
  ROS_INFO_ONCE("[%s]: Broadcasting transform from parent frame: %s to child frame: %s", getPrintName().c_str(), tf_msg.header.frame_id.c_str(),
                tf_msg.child_frame_id.c_str());
}
/*//}*/

/*//{ callbackHwApiOrientation() */
void TransformManager::callbackHwApiOrientation(const geometry_msgs::QuaternionStamped::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("TransformManager::publishFcuUntilted", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  if (publish_fcu_untilted_tf_) {
    publishFcuUntiltedTf(msg);
  }
}
/*//}*/

/*//{ callbackGnss() */
void TransformManager::callbackGnss(const sensor_msgs::NavSatFix::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("TransformManager::callbackGnss", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  if (!got_utm_offset_) {

    double out_x;
    double out_y;

    mrs_lib::UTM(msg->latitude, msg->longitude, &out_x, &out_y);

    if (!std::isfinite(out_x)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in UTM variable \"out_x\"!!!");
      return;
    }

    if (!std::isfinite(out_y)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in UTM variable \"out_y\"!!!");
      return;
    }

    geometry_msgs::Point utm_origin;
    utm_origin.x = out_x;
    utm_origin.y = out_y;
    utm_origin.z = msg->altitude;

    ROS_INFO("[%s]: utm_origin position calculated as: x: %.2f, y: %.2f, z: %.2f", getPrintName().c_str(), utm_origin.x, utm_origin.y, utm_origin.z);

    for (size_t i = 0; i < tf_sources_.size(); i++) {
      tf_sources_[i]->setUtmOrigin(utm_origin);
      tf_sources_[i]->setWorldOrigin(world_origin_);
    }
    got_utm_offset_ = true;
  }
}
/*//}*/

/*//{ callbackMappingOdomLat() */

void TransformManager::callbackMappingOdomLat(const nav_msgs::Odometry::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  if (!got_mapping_odom_rot_ && slam_mapping_origin_tf_orientation_topic_ == slam_mapping_origin_tf_lateral_topic_) {
    got_mapping_odom_rot_ = true;
  }

  if (!got_mapping_odom_alt_ && slam_mapping_origin_tf_altitude_topic_ == slam_mapping_origin_tf_lateral_topic_) {
    got_mapping_odom_alt_ = true;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("TransformManager::callbackMappingOdomLat", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  const double hdg_mapping_old = mrs_lib::AttitudeConverter(msg->pose.pose.orientation).getHeading();

  /* publish aloam mapping origin tf //{ */

  bool clear_needed = false;

  if (got_mapping_odom_rot_ && got_mapping_odom_alt_) {
    std::scoped_lock lock(mtx_mapping_odom_rot_);

    // Copy mapping odometry
    nav_msgs::Odometry mapping_odom;
    mapping_odom = *msg;

    // Find corresponding orientation
    tf2::Quaternion                  tf2_rot;
    geometry_msgs::QuaternionStamped rot_tmp = *sh_mapping_odom_rot_.getMsg();  // start with newest msg

    for (size_t i = 0; i < vec_mapping_odom_rot_.size(); i++) {
      if (mapping_odom.header.stamp < vec_mapping_odom_rot_.at(i).header.stamp) {

        // Choose an orientation with closest timestamp
        double time_diff      = std::fabs(vec_mapping_odom_rot_.at(i).header.stamp.toSec() - mapping_odom.header.stamp.toSec());
        double time_diff_prev = std::numeric_limits<double>::max();
        if (i > 0) {
          time_diff_prev = std::fabs(vec_mapping_odom_rot_.at(i - 1).header.stamp.toSec() - mapping_odom.header.stamp.toSec());
        }
        if (time_diff_prev < time_diff && i > 0) {
          i = i - 1;
        }

        // Cache is too small if it is full and its oldest element is used
        if (clear_needed && i == 0) {
          ROS_WARN_THROTTLE(1.0, "[%s] Mapping orientation cache is too small.", getPrintName().c_str());
        }
        rot_tmp = vec_mapping_odom_rot_.at(i);
        break;
      }
    }

    tf2_rot = mrs_lib::AttitudeConverter(rot_tmp.quaternion);

    // Obtain heading from orientation
    double hdg = 0;
    try {
      hdg = mrs_lib::AttitudeConverter(rot_tmp.quaternion).getHeading();
    }
    catch (...) {
      ROS_WARN("[%s]: failed to getHeading() from rot_tmp", getPrintName().c_str());
    }

    // Build rotation matrix from difference between new heading and old heading
    tf2::Matrix3x3 rot_mat = mrs_lib::AttitudeConverter(Eigen::AngleAxisd(hdg_mapping_old - hdg, Eigen::Vector3d::UnitZ()));

    // Transform the mavros orientation by the rotation matrix
    geometry_msgs::Quaternion new_orientation = mrs_lib::AttitudeConverter(tf2::Transform(rot_mat) * tf2_rot);

    // Set new orientation
    mapping_odom.pose.pose.orientation = new_orientation;


    // Find corresponding local odom
    double odom_alt = msg->pose.pose.position.z;
    for (size_t i = 0; i < vec_mapping_odom_alt_.size(); i++) {
      if (mapping_odom.header.stamp < vec_mapping_odom_alt_.at(i).header.stamp) {

        // Choose orientation with closest timestamp
        double time_diff      = std::fabs(vec_mapping_odom_alt_.at(i).header.stamp.toSec() - mapping_odom.header.stamp.toSec());
        double time_diff_prev = std::numeric_limits<double>::max();
        if (i > 0) {
          time_diff_prev = std::fabs(vec_mapping_odom_alt_.at(i - 1).header.stamp.toSec() - mapping_odom.header.stamp.toSec());
        }
        if (time_diff_prev < time_diff && i > 0) {
          i = i - 1;
        }
        // Cache is too small if it is full and its oldest element is used
        if (clear_needed && i == 0) {
          ROS_WARN_THROTTLE(1.0, "[%s] mapping orientation cache (for slam mapping tf) is too small.", getPrintName().c_str());
        }
        odom_alt = vec_mapping_odom_alt_.at(i).pose.pose.position.z;
        break;
      }
    }

    // Set altitude
    mapping_odom.pose.pose.position.z = odom_alt;

    // Get inverse transform
    tf2::Transform      tf_mapping_inv   = Support::tf2FromPose(mapping_odom.pose.pose).inverse();
    geometry_msgs::Pose pose_mapping_inv = Support::poseFromTf2(tf_mapping_inv);

    geometry_msgs::TransformStamped tf_mapping;
    tf_mapping.header.stamp    = mapping_odom.header.stamp;
    tf_mapping.header.frame_id = ch_->frames.ns_fcu;
    if (slam_mapping_origin_tf_custom_frame_id_enabled_) {
      tf_mapping.child_frame_id = ch_->uav_name + "/" + slam_mapping_origin_tf_custom_frame_id_;
    } else {
      tf_mapping.child_frame_id = mapping_odom.header.frame_id;
    }
    tf_mapping.transform.translation = Support::pointToVector3(pose_mapping_inv.position);
    tf_mapping.transform.rotation    = pose_mapping_inv.orientation;

    if (Support::noNans(tf_mapping)) {
      try {
        broadcaster_->sendTransform(tf_mapping);
      }
      catch (...) {
        ROS_ERROR("[%s]: Exception caught during publishing TF: %s - %s.", getPrintName().c_str(), tf_mapping.child_frame_id.c_str(),
                  tf_mapping.header.frame_id.c_str());
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getPrintName().c_str(), tf_mapping.header.frame_id.c_str(),
                        tf_mapping.child_frame_id.c_str());
    }
  }

  //}
}
/*//}*/

/*//{ callbackMappingOdomRot() */
void TransformManager::callbackMappingOdomRot(const geometry_msgs::QuaternionStamped::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }
  std::scoped_lock lock(mtx_mapping_odom_rot_);

  const ros::Time time_now = ros::Time::now();

  // Add new data
  vec_mapping_odom_rot_.push_back(*msg);

  // Delete old data
  size_t index_delete = 0;
  bool   clear_needed = false;
  for (size_t i = 0; i < vec_mapping_odom_rot_.size(); i++) {
    if (time_now - vec_mapping_odom_rot_.at(i).header.stamp > ros::Duration(slam_mapping_origin_tf_cache_duration_)) {
      index_delete = i;
      clear_needed = true;
    } else {
      break;
    }
  }
  if (clear_needed) {
    for (int i = (int)index_delete; i >= 0; i--) {
      vec_mapping_odom_rot_.erase(vec_mapping_odom_rot_.begin() + i);
    }
    clear_needed = false;
  }

  if (!got_mapping_odom_rot_) {
    got_mapping_odom_rot_ = true;
  }
}
/*//}*/

/*//{ callbackMappingOdomAlt() */
void TransformManager::callbackMappingOdomAlt(const nav_msgs::Odometry::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }
  
  std::scoped_lock lock(mtx_mapping_odom_alt_);

  const ros::Time time_now = ros::Time::now();

  // Add new data
  vec_mapping_odom_alt_.push_back(*msg);

  // Delete old data
  size_t index_delete = 0;
  bool   clear_needed = false;
  for (size_t i = 0; i < vec_mapping_odom_alt_.size(); i++) {
    if (time_now - vec_mapping_odom_alt_.at(i).header.stamp > ros::Duration(slam_mapping_origin_tf_cache_duration_)) {
      index_delete = i;
      clear_needed = true;
    } else {
      break;
    }
  }
  if (clear_needed) {
    for (int i = (int)index_delete; i >= 0; i--) {
      vec_mapping_odom_alt_.erase(vec_mapping_odom_alt_.begin() + i);
    }
    clear_needed = false;
  }

  if (!got_mapping_odom_alt_) {
    got_mapping_odom_alt_ = true;
  }
}
/*//}*/

/*//{ publishFcuUntiltedTf() */
void TransformManager::publishFcuUntiltedTf(const geometry_msgs::QuaternionStampedConstPtr& msg) {

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("TransformManager::publishFcuUntilted", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  double heading;

  try {
    heading = mrs_lib::AttitudeConverter(msg->quaternion).getHeading();
  }
  catch (...) {
    ROS_ERROR("[%s]: Exception caught during getting heading", getPrintName().c_str());
    return;
  }
  scope_timer.checkpoint("heading");

  const Eigen::Matrix3d odom_pixhawk_R = mrs_lib::AttitudeConverter(msg->quaternion);
  const Eigen::Matrix3d undo_heading_R = mrs_lib::AttitudeConverter(Eigen::AngleAxis(-heading, Eigen::Vector3d(0, 0, 1)));

  const tf2::Quaternion q     = mrs_lib::AttitudeConverter(undo_heading_R * odom_pixhawk_R);
  const tf2::Quaternion q_inv = q.inverse();

  scope_timer.checkpoint("q inverse");

  geometry_msgs::TransformStamped tf;
  tf.header.stamp            = msg->header.stamp;  // TODO(petrlmat) ros::Time::now()?
  tf.header.frame_id         = ch_->frames.ns_fcu;
  tf.child_frame_id          = ch_->frames.ns_fcu_untilted;
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation      = mrs_lib::AttitudeConverter(q_inv);

  scope_timer.checkpoint("tf fill");
  if (Support::noNans(tf)) {
    broadcaster_->sendTransform(tf);
  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN encountered in fcu_untilted tf", getPrintName().c_str());
  }
  scope_timer.checkpoint("tf pub");
}
/*//}*/

/*//{ getName() */
std::string TransformManager::getName() const {
  return name_;
}
/*//}*/

/*//{ getPrintName() */
std::string TransformManager::getPrintName() const {
  return nodelet_name_;
}
/*//}*/

}  // namespace transform_manager

}  // namespace mrs_uav_managers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::transform_manager::TransformManager, nodelet::Nodelet)
