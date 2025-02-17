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
#include <mrs_msgs/HwApiAltitude.h>
#include <mrs_msgs/RtkGps.h>
#include <mrs_msgs/ReferenceStampedSrv.h>

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
#include <transform_manager/tf_mapping_origin.h>

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
  std::string _custom_config_;
  std::string _platform_config_;
  std::string _world_config_;

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
  geometry_msgs::PoseStamped pose_first_;
  geometry_msgs::Pose pose_fixed_;
  geometry_msgs::Pose pose_fixed_diff_;

  std::string ns_amsl_origin_parent_frame_id_;
  std::string ns_amsl_origin_child_frame_id_;
  bool        publish_amsl_origin_tf_;

  std::string          world_origin_units_;
  geometry_msgs::Point world_origin_;

  ros::ServiceServer srvs_set_world_origin_;
  bool               callbackSetWorldOrigin(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);

  std::vector<std::string>               tf_source_names_, estimator_names_;
  std::vector<std::unique_ptr<TfSource>> tf_sources_;

  std::vector<std::string> utm_source_priority_list_;
  std::string              utm_source_name_;

  std::mutex mtx_broadcast_utm_origin_;
  std::mutex mtx_broadcast_world_origin_;

  ros::NodeHandle nh_;

  std::shared_ptr<estimation_manager::CommonHandlers_t> ch_;

  std::shared_ptr<mrs_lib::TransformBroadcaster> broadcaster_;
  tf2_ros::StaticTransformBroadcaster            static_broadcaster_;

  std::unique_ptr<TfMappingOrigin> tf_mapping_origin_;

  void timeoutCallback(const std::string& topic, const ros::Time& last_msg);

  mrs_lib::SubscribeHandler<mrs_msgs::UavState> sh_uav_state_;
  void                                          callbackUavState(const mrs_msgs::UavState::ConstPtr msg);
  std::string                                   first_frame_id_;
  std::string                                   last_frame_id_;
  bool                                          is_first_frame_id_set_        = false;
  bool                                          is_local_static_tf_published_ = false;

  mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped> sh_height_agl_;
  void                                                callbackHeightAgl(const mrs_msgs::Float64Stamped::ConstPtr msg);

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAltitude> sh_altitude_amsl_;
  void                                               callbackAltitudeAmsl(const mrs_msgs::HwApiAltitude::ConstPtr msg);

  mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped> sh_hw_api_orientation_;
  void                                                        callbackHwApiOrientation(const geometry_msgs::QuaternionStamped::ConstPtr msg);

  mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> sh_gnss_;
  void                                              callbackGnss(const sensor_msgs::NavSatFix::ConstPtr msg);
  std::atomic<bool>                                 got_utm_offset_ = false;

  mrs_lib::SubscribeHandler<mrs_msgs::RtkGps> sh_rtk_gps_;
  void                                        callbackRtkGps(const mrs_msgs::RtkGps::ConstPtr msg);

  std::optional<geometry_msgs::Pose> transformRtkToFcu(const geometry_msgs::PoseStamped& pose_in) const;

  void publishFcuUntiltedTf(const geometry_msgs::QuaternionStampedConstPtr& msg);

  void publishLocalTf();
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

  param_loader.loadParam("custom_config", _custom_config_);
  param_loader.loadParam("platform_config", _platform_config_);
  param_loader.loadParam("world_config", _world_config_);

  if (_custom_config_ != "") {
    param_loader.addYamlFile(_custom_config_);
  }

  if (_platform_config_ != "") {
    param_loader.addYamlFile(_platform_config_);
  }

  if (_world_config_ != "") {
    param_loader.addYamlFile(_world_config_);
  }

  param_loader.addYamlFileFromParam("private_config");
  param_loader.addYamlFileFromParam("public_config");
  param_loader.addYamlFileFromParam("estimators_config");

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

  param_loader.loadParam("world_origin/units", world_origin_units_);

  if (Support::toLowercase(world_origin_units_) == "utm") {

    ROS_INFO("[%s]: Loading world origin in UTM units.", getPrintName().c_str());

    is_origin_param_ok &= param_loader.loadParam("world_origin/origin_x", world_origin_x);
    is_origin_param_ok &= param_loader.loadParam("world_origin/origin_y", world_origin_y);

  } else if (Support::toLowercase(world_origin_units_) == "latlon") {

    ROS_INFO("[%s]: Loading world origin in LatLon units.", getPrintName().c_str());

    double lat, lon;
    is_origin_param_ok &= param_loader.loadParam("world_origin/origin_x", lat);
    is_origin_param_ok &= param_loader.loadParam("world_origin/origin_y", lon);

    mrs_lib::UTM(lat, lon, &world_origin_x, &world_origin_y);

    ROS_INFO("[%s]: Converted to UTM x: %f, y: %f.", getPrintName().c_str(), world_origin_x, world_origin_y);

  } else {
    ROS_ERROR("[%s]: world_origin/units must be (\"UTM\"|\"LATLON\"). Got '%s'", getPrintName().c_str(), world_origin_units_.c_str());
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

  /*//{ load amsl_origin parameters*/
  std::string amsl_parent_frame_id, amsl_child_frame_id;
  param_loader.loadParam(yaml_prefix + "altitude_amsl_tf/enabled", publish_amsl_origin_tf_);
  param_loader.loadParam(yaml_prefix + "altitude_amsl_tf/parent", amsl_parent_frame_id);
  param_loader.loadParam(yaml_prefix + "altitude_amsl_tf/child", amsl_child_frame_id);
  ch_->frames.amsl                = amsl_child_frame_id;
  ch_->frames.ns_amsl             = ch_->uav_name + "/" + amsl_child_frame_id;
  ns_amsl_origin_parent_frame_id_ = ch_->uav_name + "/" + amsl_parent_frame_id;
  ns_amsl_origin_child_frame_id_  = ch_->uav_name + "/" + amsl_child_frame_id;

  /*//}*/

  param_loader.loadParam(yaml_prefix + "rtk_antenna/frame_id", ch_->frames.rtk_antenna);
  ch_->frames.ns_rtk_antenna = ch_->uav_name + "/" + ch_->frames.rtk_antenna;

  param_loader.loadParam("mrs_uav_managers/estimation_manager/state_estimators", estimator_names_);
  param_loader.loadParam(yaml_prefix + "tf_sources", tf_source_names_);

  param_loader.loadParam(yaml_prefix + "utm_source_priority", utm_source_priority_list_);
  for (auto utm_source : utm_source_priority_list_) {
    if (Support::isStringInVector(utm_source, estimator_names_)) {
      ROS_INFO("[%s]: the source for utm_origin and world origin is: %s", getPrintName().c_str(), utm_source.c_str());
      utm_source_name_ = utm_source;
      break;
    }
  }

  /*//{ initialize tf sources */
  for (size_t i = 0; i < tf_source_names_.size(); i++) {

    const std::string tf_source_name = tf_source_names_[i];
    const bool        is_utm_source  = tf_source_name == utm_source_name_;

    ROS_INFO("[%s]: loading tf source: %s", getPrintName().c_str(), tf_source_name.c_str());

    auto source_param_loader = std::make_shared<mrs_lib::ParamLoader>(nh_, "TransformManager/" + tf_source_name);

    if (_custom_config_ != "") {
      source_param_loader->addYamlFile(_custom_config_);
    }

    if (_platform_config_ != "") {
      source_param_loader->addYamlFile(_platform_config_);
    }

    if (_world_config_ != "") {
      source_param_loader->addYamlFile(_world_config_);
    }

    source_param_loader->addYamlFileFromParam("private_config");
    source_param_loader->addYamlFileFromParam("public_config");
    source_param_loader->addYamlFileFromParam("estimators_config");

    tf_sources_.push_back(std::make_unique<TfSource>(tf_source_name, nh_, source_param_loader, broadcaster_, ch_, is_utm_source));
  }

  // additionally publish tf of all available estimators
  for (int i = 0; i < int(estimator_names_.size()); i++) {

    const std::string estimator_name = estimator_names_[i];
    const bool        is_utm_source  = estimator_name == utm_source_name_;
    ROS_INFO("[%s]: loading tf source of estimator: %s", getPrintName().c_str(), estimator_name.c_str());

    auto estimator_param_loader = std::make_shared<mrs_lib::ParamLoader>(nh_, "TransformManager/" + estimator_name);

    if (_custom_config_ != "") {
      estimator_param_loader->addYamlFile(_custom_config_);
    }

    if (_platform_config_ != "") {
      estimator_param_loader->addYamlFile(_platform_config_);
    }

    if (_world_config_ != "") {
      estimator_param_loader->addYamlFile(_world_config_);
    }

    estimator_param_loader->addYamlFileFromParam("private_config");
    estimator_param_loader->addYamlFileFromParam("public_config");
    estimator_param_loader->addYamlFileFromParam("estimators_config");

    tf_sources_.push_back(std::make_unique<TfSource>(estimator_name, nh_, estimator_param_loader, broadcaster_, ch_, is_utm_source));
  }

  // initialize mapping_origin tf
  bool mapping_origin_tf_enabled;
  param_loader.loadParam(yaml_prefix + "mapping_origin_tf/enabled", mapping_origin_tf_enabled, false);

  if (mapping_origin_tf_enabled) {

    auto mapping_param_loader = std::make_shared<mrs_lib::ParamLoader>(nh_, "TransformManager/mapping_origin_tf");

    if (_custom_config_ != "") {
      mapping_param_loader->addYamlFile(_custom_config_);
    }

    if (_platform_config_ != "") {
      mapping_param_loader->addYamlFile(_platform_config_);
    }

    if (_world_config_ != "") {
      mapping_param_loader->addYamlFile(_world_config_);
    }

    mapping_param_loader->addYamlFileFromParam("private_config");
    mapping_param_loader->addYamlFileFromParam("public_config");
    mapping_param_loader->addYamlFileFromParam("estimators_config");

    tf_mapping_origin_ = std::make_unique<TfMappingOrigin>(nh_, mapping_param_loader, broadcaster_, ch_);
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

  sh_altitude_amsl_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiAltitude>(shopts, "altitude_amsl_in", &TransformManager::callbackAltitudeAmsl, this);

  sh_hw_api_orientation_ =
      mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped>(shopts, "orientation_in", &TransformManager::callbackHwApiOrientation, this);

  if (utm_source_name_ == "rtk" || utm_source_name_ == "rtk_garmin") {
    sh_rtk_gps_ = mrs_lib::SubscribeHandler<mrs_msgs::RtkGps>(shopts, "rtk_gps_in", &TransformManager::callbackRtkGps, this);
  } else {
    sh_gnss_ = mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>(shopts, "gnss_in", &TransformManager::callbackGnss, this);
  }
  /*//}*/

/*//{ initialize service servers*/
  srvs_set_world_origin_ = nh_.advertiseService("set_world_origin_in", &TransformManager::callbackSetWorldOrigin, this);
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
    pose_first_.pose               = msg->pose;
    pose_first_.header             = msg->header;
    pose_fixed_diff_.orientation.w = 1;

    // we don't want vins_kickoff to be our first estimator
    if (msg->header.frame_id != ch_->uav_name + "/vins_kickoff_origin") {
      is_first_frame_id_set_ = true;
    }
  }

  // publish static tf from fixed_origin to local_origin based on the first message
  if (publish_local_origin_tf_ && !is_local_static_tf_published_) {
    publishLocalTf();
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

      // first check if tf source can publish utm origin and is not the switched-from estimator
      if (tf_sources_.at(i)->getIsUtmBased() && tf_sources_.at(i)->getName() != last_estimator_name) {

        valid_utm_source_found     = true;
        potential_utm_source_index = i;

        // check if switched-to estimator is utm_based, if so, use it
        if (tf_sources_.at(i)->getIsUtmBased() && tf_sources_.at(i)->getName() == current_estimator_name) {
          potential_utm_source_index = i;
          break;
        }
      }
    }


    // if we found a valid utm source, use it, otherwise stay with the switched-from estimator
    if (valid_utm_source_found) {

      // stop all estimators from publishing utm source
      for (size_t i = 0; i < tf_sources_.size(); i++) {
        if (tf_sources_.at(i)->getIsUtmSource()) {
          tf_sources_.at(i)->setIsUtmSource(false);
          ROS_INFO("[%s]: setting is_utm_source of estimator %s to false", getPrintName().c_str(), last_estimator_name.c_str());
        }
      }

      tf_sources_.at(potential_utm_source_index)->setIsUtmSource(true);
      ROS_INFO("[%s]: setting is_utm_source of estimator %s to true", getPrintName().c_str(), current_estimator_name.c_str());
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

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("TransformManager::callbackHeightAgl", ch_->scope_timer.logger, ch_->scope_timer.enabled);

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

/*//{ callbackAltitudeAmsl() */

void TransformManager::callbackAltitudeAmsl(const mrs_msgs::HwApiAltitude::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("TransformManager::callbackAltitudeAmsl", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp    = msg->stamp;
  tf_msg.header.frame_id = ch_->frames.ns_fcu_untilted;
  tf_msg.child_frame_id  = ch_->frames.ns_amsl;

  tf_msg.transform.translation.x = 0;
  tf_msg.transform.translation.y = 0;
  tf_msg.transform.translation.z = -msg->amsl;
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

  if (got_utm_offset_) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("TransformManager::callbackGnss", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  double out_x;
  double out_y;

  mrs_lib::UTM(msg->latitude, msg->longitude, &out_x, &out_y);

  if (!std::isfinite(out_x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in UTM variable \"out_x\"!!!", getPrintName().c_str());
    return;
  }

  if (!std::isfinite(out_y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in UTM variable \"out_y\"!!!", getPrintName().c_str());
    return;
  }

  geometry_msgs::Point utm_origin;
  utm_origin.x = out_x;
  utm_origin.y = out_y;
  utm_origin.z = msg->altitude;

  ROS_INFO("[%s]: utm_origin position calculated as: x: %.2f, y: %.2f, z: %.2f from GNSS", getPrintName().c_str(), utm_origin.x, utm_origin.y, utm_origin.z);

  for (size_t i = 0; i < tf_sources_.size(); i++) {
    tf_sources_[i]->setUtmOrigin(utm_origin);
    tf_sources_[i]->setWorldOrigin(world_origin_);
  }
  got_utm_offset_ = true;
}
/*//}*/

/*//{ callbackRtkGps() */
void TransformManager::callbackRtkGps(const mrs_msgs::RtkGps::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  if (got_utm_offset_) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("TransformManager::callbackRtkGps", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  double out_x;
  double out_y;

  geometry_msgs::PoseStamped rtk_pos;

  if (!std::isfinite(msg->gps.latitude)) {
    ROS_ERROR_THROTTLE(1.0, "[%s] NaN detected in RTK variable \"msg->latitude\"!!!", getPrintName().c_str());
    return;
  }

  if (!std::isfinite(msg->gps.longitude)) {
    ROS_ERROR_THROTTLE(1.0, "[%s] NaN detected in RTK variable \"msg->longitude\"!!!", getPrintName().c_str());
    return;
  }

  if (!std::isfinite(msg->gps.altitude)) {
    ROS_ERROR_THROTTLE(1.0, "[%s] NaN detected in RTK variable \"msg->altitude\"!!!", getPrintName().c_str());
    return;
  }

  if (msg->fix_type.fix_type != mrs_msgs::RtkFixType::RTK_FIX) {
    ROS_INFO_THROTTLE(1.0, "[%s] %s RTK FIX", getPrintName().c_str(), Support::waiting_for_string.c_str());
    return;
  }

  rtk_pos.header = msg->header;
  mrs_lib::UTM(msg->gps.latitude, msg->gps.longitude, &rtk_pos.pose.position.x, &rtk_pos.pose.position.y);
  rtk_pos.pose.position.z  = msg->gps.altitude;
  rtk_pos.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

  rtk_pos.pose.position.x -= ch_->world_origin.x;
  rtk_pos.pose.position.y -= ch_->world_origin.y;


  // transform the RTK position from antenna to FCU
  auto res = transformRtkToFcu(rtk_pos);
  if (res) {
    rtk_pos.pose = res.value();
  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: transform to fcu failed", getPrintName().c_str());
    return;
  }

  mrs_lib::UTM(msg->gps.latitude, msg->gps.longitude, &out_x, &out_y);

  if (!std::isfinite(out_x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in UTM variable \"out_x\"!!!", getPrintName().c_str());
    return;
  }

  if (!std::isfinite(out_y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in UTM variable \"out_y\"!!!", getPrintName().c_str());
    return;
  }

  geometry_msgs::Point utm_origin;
  utm_origin.x = out_x;
  utm_origin.y = out_y;
  utm_origin.z = msg->gps.altitude;

  ROS_INFO("[%s]: utm_origin position calculated as: x: %.2f, y: %.2f, z: %.2f from RTK msg", getPrintName().c_str(), utm_origin.x, utm_origin.y, utm_origin.z);

  for (size_t i = 0; i < tf_sources_.size(); i++) {
    tf_sources_[i]->setUtmOrigin(utm_origin);
    tf_sources_[i]->setWorldOrigin(world_origin_);
  }
  got_utm_offset_ = true;
}
/*//}*/

/*//{ callbackSetWorldOrigin() */
bool TransformManager::callbackSetWorldOrigin(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res) {

  if (!is_initialized_) {
    res.success = false;
    res.message = "Could not set world origin";
    return true;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("TransformManager::callbackSetWorldOrigin", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  geometry_msgs::Point world_origin;

  if (req.header.frame_id.find("latlon_origin") != std::string::npos) {
    const double lat = req.reference.position.x;
    const double lon = req.reference.position.y;
    mrs_lib::UTM(lat, lon, &world_origin.x, &world_origin.y);
    ROS_INFO("[TransformManager]: Setting world origin to lat: %.6f lon: %.6f", req.reference.position.x, req.reference.position.y);
  } else if (req.header.frame_id.find("utm_origin") != std::string::npos) {
    world_origin.x = req.reference.position.x;
    world_origin.y = req.reference.position.y;
    ROS_INFO("[TransformManager]: Setting world origin to x: %.2f y: %.2f UTM", req.reference.position.x, req.reference.position.y);
  } else {
    ROS_ERROR("[TransformManager]: Requested unsupported frame_id: \"%s\" in set_world_origin service. Supported are: latlon_origin, utm_origin",
              req.header.frame_id.c_str());
    res.success = false;
    res.message = "Requested unsupported frame_id. Supported are: latlon_origin, utm_origin";
    return true;
  }

  for (size_t i = 0; i < tf_sources_.size(); i++) {
    tf_sources_[i]->setWorldOrigin(world_origin);
  }

  res.success = true;
  res.message = "World origin set successfully";

  return true;
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

/* publishLocalTf() //{*/
void TransformManager::publishLocalTf() {

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer(getPrintName() + "::publishLocalTf", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = pose_first_.header.stamp;

  tf_msg.header.frame_id       = ns_fixed_origin_child_frame_id_;
  tf_msg.child_frame_id        = ns_local_origin_child_frame_id_;
  tf_msg.transform.translation = Support::pointToVector3(pose_first_.pose.position);
  tf_msg.transform.rotation    = pose_first_.pose.orientation;

  if (Support::noNans(tf_msg)) {

    try {
      static_broadcaster_.sendTransform(tf_msg);
    }
    catch (...) {
      ROS_ERROR("[%s]: exception caught while publishing tf from %s to %s", getPrintName().c_str(), tf_msg.header.frame_id.c_str(),
                tf_msg.child_frame_id.c_str());
    }

  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getPrintName().c_str(), tf_msg.header.frame_id.c_str(),
                      tf_msg.child_frame_id.c_str());
  }
  ROS_INFO_ONCE("[%s]: Broadcasting transform from parent frame: %s to child frame: %s", getPrintName().c_str(), tf_msg.header.frame_id.c_str(),
                tf_msg.child_frame_id.c_str());
  is_local_static_tf_published_ = true;
}
/*//}*/

/*//{ transformRtkToFcu() */
std::optional<geometry_msgs::Pose> TransformManager::transformRtkToFcu(const geometry_msgs::PoseStamped& pose_in) const {

  geometry_msgs::PoseStamped pose_tmp = pose_in;

  // inject current orientation into rtk pose
  auto res1 = ch_->transformer->getTransform(ch_->frames.ns_fcu_untilted, ch_->frames.ns_fcu, ros::Time::now());
  if (res1) {
    pose_tmp.pose.orientation = res1.value().transform.rotation;
  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Could not obtain transform from %s to %s.", getPrintName().c_str(), ch_->frames.ns_fcu_untilted.c_str(),
                       ch_->frames.ns_fcu.c_str());
    return {};
  }

  // invert tf
  tf2::Transform             tf_utm_to_antenna = Support::tf2FromPose(pose_tmp.pose);
  geometry_msgs::PoseStamped utm_in_antenna;
  utm_in_antenna.pose            = Support::poseFromTf2(tf_utm_to_antenna.inverse());
  utm_in_antenna.header.stamp    = pose_in.header.stamp;
  utm_in_antenna.header.frame_id = ch_->frames.ns_rtk_antenna;

  // transform to fcu
  geometry_msgs::PoseStamped utm_in_fcu;
  utm_in_fcu.header.frame_id = ch_->frames.ns_fcu;
  utm_in_fcu.header.stamp    = pose_in.header.stamp;
  auto res2                  = ch_->transformer->transformSingle(utm_in_antenna, ch_->frames.ns_fcu);

  if (res2) {
    utm_in_fcu = res2.value();
  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Could not transform RTK pose from %s to %s.", getPrintName().c_str(), utm_in_antenna.header.frame_id.c_str(),
                       ch_->frames.ns_fcu.c_str());
    return {};
  }

  // invert tf
  tf2::Transform      tf_fcu_to_utm = Support::tf2FromPose(utm_in_fcu.pose);
  geometry_msgs::Pose fcu_in_utm    = Support::poseFromTf2(tf_fcu_to_utm.inverse());

  return fcu_in_utm;
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
