#pragma once
#ifndef TF_SOURCE_H
#define TF_SOURCE_H

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/transform_broadcaster.h>
#include <mrs_lib/gps_conversions.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>

#include <mrs_uav_managers/estimation_manager/support.h>
#include <mrs_uav_managers/estimation_manager/common_handlers.h>

namespace mrs_uav_managers
{

/*//{ class TfSource */
class TfSource {

  using Support = estimation_manager::Support;

public:
  /*//{ constructor */
  TfSource(const std::string& name, const rclcpp::Node::SharedPtr& node, std::shared_ptr<mrs_lib::ParamLoader> param_loader, const std::shared_ptr<mrs_lib::TransformBroadcaster>& broadcaster, const std::shared_ptr<estimation_manager::CommonHandlers_t> ch, const bool is_utm_source) : name_(name), broadcaster_(broadcaster), ch_(ch), is_utm_source_(is_utm_source) {

    node_  = node;
    clock_ = node->get_clock();

    RCLCPP_INFO(node_->get_logger(), "[%s]: initializing", getPrintName().c_str());

    /*//{ load parameters */

    const std::string yaml_prefix = "mrs_uav_managers/transform_manager/";

    std::string odom_topic, attitude_topic, ns;

    param_loader->loadParam(yaml_prefix + getName() + "/odom_topic", odom_topic);

    param_loader->loadParam(yaml_prefix + getName() + "/custom_frame_id/enabled", custom_frame_id_enabled_, false);
    if (custom_frame_id_enabled_) {
      param_loader->loadParam(yaml_prefix + getName() + "/custom_frame_id/frame_id", custom_frame_id_);
    }

    param_loader->loadParam(yaml_prefix + getName() + "/custom_child_frame_id/enabled", custom_child_frame_id_enabled_, false);
    if (custom_child_frame_id_enabled_) {
      param_loader->loadParam(yaml_prefix + getName() + "/custom_child_frame_id/frame_id", custom_child_frame_id_);
    }

    param_loader->loadParam(yaml_prefix + getName() + "/tf_from_attitude/enabled", tf_from_attitude_enabled_);
    if (tf_from_attitude_enabled_) {
      param_loader->loadParam(yaml_prefix + getName() + "/tf_from_attitude/attitude_topic", attitude_topic);
    }

    param_loader->loadParam(yaml_prefix + getName() + "/namespace", ns);
    full_topic_odom_     = "/" + ch_->uav_name + "/" + ns + "/" + odom_topic;
    full_topic_attitude_ = "/" + ch_->uav_name + "/" + ns + "/" + attitude_topic;
    param_loader->loadParam(yaml_prefix + getName() + "/inverted", is_inverted_);
    param_loader->loadParam(yaml_prefix + getName() + "/republish_in_frames", republish_in_frames_);

    /* coordinate frames origins //{ */
    param_loader->loadParam(yaml_prefix + getName() + "/utm_based", is_utm_based_);
    /* param_loader->loadParam(yaml_prefix + getName() + "/publish_local_tf", publish_local_tf_); */

    /*//{ utm source */
    if (is_utm_based_) {
      std::string utm_origin_parent_frame_id;
      param_loader->loadParam(yaml_prefix + "utm_origin_tf/parent", utm_origin_parent_frame_id);
      ns_utm_origin_parent_frame_id_ = ch_->uav_name + "/" + utm_origin_parent_frame_id;

      std::string utm_origin_child_frame_id;
      param_loader->loadParam(yaml_prefix + "utm_origin_tf/child", utm_origin_child_frame_id);
      ns_utm_origin_child_frame_id_ = ch_->uav_name + "/" + utm_origin_child_frame_id;
    }
    /*//}*/

    /*//{ world source */
    if (is_utm_based_) {
      std::string world_origin_parent_frame_id;
      param_loader->loadParam(yaml_prefix + "world_origin_tf/parent", world_origin_parent_frame_id);
      ns_world_origin_parent_frame_id_ = ch_->uav_name + "/" + world_origin_parent_frame_id;

      std::string world_origin_child_frame_id;
      param_loader->loadParam(yaml_prefix + "world_origin_tf/child", world_origin_child_frame_id);
      ns_world_origin_child_frame_id_ = ch_->uav_name + "/" + world_origin_child_frame_id;
    }
    /*//}*/

    //}

    if (!param_loader->loadedSuccessfully()) {
      RCLCPP_ERROR(node_->get_logger(), "[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
      rclcpp::shutdown();
    }

    /*//}*/

    /*//{ initialize subscribers */
    mrs_lib::SubscriberHandlerOptions shopts;

    shopts.node               = node_;
    shopts.node_name          = getPrintName();
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;

    sh_tf_source_odom_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, full_topic_odom_, &TfSource::callbackTfSourceOdom, this);

    if (tf_from_attitude_enabled_) {
      sh_tf_source_att_ = mrs_lib::SubscriberHandler<geometry_msgs::msg::QuaternionStamped>(shopts, full_topic_attitude_, &TfSource::callbackTfSourceAtt, this);
    }

    /* } */

    for (auto frame_id : republish_in_frames_) {
      republishers_.push_back(std::make_pair(frame_id, mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(node_, full_topic_odom_ + "/" + frame_id.substr(0, frame_id.find("_origin")))));
    }
    is_initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s]: initialized", getPrintName().c_str());
  }
  /*//}*/

  /*//{ getName() */
  std::string getName() {
    return name_;
  }
  /*//}*/

  /*//{ getPrintName() */
  std::string getPrintName() {
    return ch_->nodelet_name + "/" + name_;
  }
  /*//}*/

  /*//{ getIsUtmBased() */
  bool getIsUtmBased() {
    return is_utm_based_;
  }
  /*//}*/

  /*//{ getIsUtmSource() */
  bool getIsUtmSource() {
    return is_utm_source_;
  }
  /*//}*/

  /*//{ setIsUtmSource() */
  void setIsUtmSource(const bool is_utm_source) {

    // TODO during ROS2 refactoring, this seemed redundant
    /* if (is_utm_source) { */

    /*   mrs_lib::SubscriberHandlerOptions shopts; */

    /*   shopts.node               = node_; */
    /*   shopts.node_name          = getPrintName(); */
    /*   shopts.no_message_timeout = mrs_lib::no_timeout; */
    /*   shopts.threadsafe         = true; */
    /*   shopts.autostart          = true; */

    /* } */

    is_utm_source_ = is_utm_source;
  }
  /*//}*/

  /*//{ setUtmOrigin() */
  void setUtmOrigin(const geometry_msgs::msg::Point& pt) {

    if (is_utm_based_ && !is_utm_origin_set_) {
      utm_origin_        = pt;
      is_utm_origin_set_ = true;
    }
  }
  /*//}*/

  /*//{ setWorldOrigin() */
  void setWorldOrigin(const geometry_msgs::msg::Point& pt) {

    if (is_utm_based_) {
      world_origin_        = pt;
      is_world_origin_set_ = true;
    }
  }
  /*//}*/

private:
  const std::string name_;
  const std::string ns_fcu_frame_id_;

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<mrs_lib::TransformBroadcaster>       broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  std::shared_ptr<estimation_manager::CommonHandlers_t> ch_;

  bool is_inverted_;

  bool             is_utm_based_;
  bool             publish_local_tf_ = false;
  std::atomic_bool is_utm_source_    = false;
  std::string      ns_utm_origin_parent_frame_id_;
  std::string      ns_utm_origin_child_frame_id_;

  bool                      is_utm_origin_set_ = false;
  geometry_msgs::msg::Point utm_origin_;

  std::string ns_world_origin_parent_frame_id_;
  std::string ns_world_origin_child_frame_id_;

  bool                      is_world_origin_set_ = false;
  geometry_msgs::msg::Point world_origin_;

  std::string full_topic_odom_;
  std::string full_topic_attitude_;
  bool        tf_from_attitude_enabled_ = false;

  bool        custom_frame_id_enabled_;
  std::string custom_frame_id_;

  bool        custom_child_frame_id_enabled_;
  std::string custom_child_frame_id_;

  std::atomic_bool is_initialized_               = false;
  std::atomic_bool is_local_static_tf_published_ = false;
  std::atomic_bool is_utm_static_tf_published_   = false;
  std::atomic_bool is_world_static_tf_published_ = false;

  std::vector<std::string> republish_in_frames_;

  std::vector<std::pair<std::string, mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>>> republishers_;

  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>               sh_tf_source_odom_;
  mrs_lib::SubscriberHandler<geometry_msgs::msg::QuaternionStamped> sh_tf_source_att_;
  nav_msgs::msg::Odometry::ConstSharedPtr                           first_msg_;
  bool                                                              got_first_msg_ = false;


  /*//{ callbackTfSourceOdom()*/
  void callbackTfSourceOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

    if (!is_initialized_) {
      return;
    }

    mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer(node_, getPrintName() + "::callbackTfSourceOdom", ch_->scope_timer.logger, ch_->scope_timer.enabled);

    scope_timer.checkpoint("get msg");

    if (!got_first_msg_) {
      first_msg_     = msg;
      got_first_msg_ = true;
    }

    publishTfFromOdom(msg);
    scope_timer.checkpoint("pub tf");

    if (publish_local_tf_ && !is_local_static_tf_published_) {
      publishLocalTf(msg->header.frame_id);
      scope_timer.checkpoint("pub local tf");
    }

    /* if (publish_utm_tf_ && is_utm_based_ && is_utm_origin_set_ && !is_utm_static_tf_published_) { */
    /*   publishUtmTf(msg->header.frame_id); */
    /*   scope_timer.checkpoint("pub utm tf"); */
    /* } */

    /* if (publish_world_tf_ && is_utm_based_ && is_world_origin_set_ && !is_world_static_tf_published_) { */
    /*   publishWorldTf(msg->header.frame_id); */
    /*   scope_timer.checkpoint("pub world tf"); */
    /* } */

    for (auto republisher : republishers_) {
      republishInFrame(msg, ch_->uav_name + "/" + republisher.first, republisher.second);
    }
  }
  /*//}*/

  /*//{ callbackTfSourceAtt()*/
  void callbackTfSourceAtt(const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr msg) {

    if (!is_initialized_) {
      return;
    }

    mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer(node_, getPrintName() + "::callbackTfSourceAtt", ch_->scope_timer.logger, ch_->scope_timer.enabled);

    scope_timer.checkpoint("get msg");
    publishTfFromAtt(msg);
  }
  /*//}*/

  /* publishTfFromOdom() //{*/
  void publishTfFromOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& odom) {

    mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer(node_, getPrintName() + "::publishTfFromOdom", ch_->scope_timer.logger, ch_->scope_timer.enabled);

    const tf2::Transform           tf       = Support::tf2FromPose(odom->pose.pose);
    const tf2::Transform           tf_inv   = tf.inverse();
    const geometry_msgs::msg::Pose pose_inv = Support::poseFromTf2(tf_inv);

    /*//{ tf source origin */
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp         = odom->header.stamp;
    std::string origin_frame_id = custom_frame_id_enabled_ ? ch_->uav_name + "/" + custom_frame_id_ : odom->header.frame_id;
    std::string child_frame_id  = custom_child_frame_id_enabled_ ? ch_->uav_name + "/" + custom_child_frame_id_ : ch_->frames.ns_fcu;
    if (is_inverted_) {

      tf_msg.header.frame_id       = child_frame_id;
      tf_msg.child_frame_id        = origin_frame_id;
      tf_msg.transform.translation = Support::pointToVector3(pose_inv.position);
      tf_msg.transform.rotation    = pose_inv.orientation;

    } else {
      tf_msg.header.frame_id       = origin_frame_id;
      tf_msg.child_frame_id        = child_frame_id;
      tf_msg.transform.translation = Support::pointToVector3(odom->pose.pose.position);
      tf_msg.transform.rotation    = odom->pose.pose.orientation;
    }

    if (Support::noNans(tf_msg)) {
      try {
        broadcaster_->sendTransform(tf_msg);
      }
      catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "exception caught ");
      }

      if (tf_from_attitude_enabled_) {
        if (is_inverted_) {
          tf_msg.child_frame_id += "_att_only";
        } else {
          tf_msg.header.frame_id += "_att_only";
        }
        try {
          broadcaster_->sendTransform(tf_msg);
        }
        catch (...) {
          RCLCPP_ERROR(node_->get_logger(), "exception caught ");
        }
      }
      /*//}*/

      /*//{ tf utm origin */

      geometry_msgs::msg::TransformStamped tf_utm_msg;
      if (is_utm_source_) {

        if (!is_utm_origin_set_) {
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 5000, "[%s]: %s utm_origin initialization", getPrintName().c_str(), Support::waiting_for_string.c_str());
          return;
        }

        geometry_msgs::msg::Pose pose_utm = odom->pose.pose;
        pose_utm.position.x += utm_origin_.x - first_msg_->pose.pose.position.x;
        pose_utm.position.y += utm_origin_.y - first_msg_->pose.pose.position.y;
        pose_utm.position.z += utm_origin_.z - first_msg_->pose.pose.position.z;

        tf_utm_msg.header.stamp    = odom->header.stamp;
        tf_utm_msg.header.frame_id = ns_utm_origin_parent_frame_id_;
        tf_utm_msg.child_frame_id  = ns_utm_origin_child_frame_id_;

        tf2::Transform tf_utm;
        if (is_inverted_) {
          tf_utm = Support::tf2FromPose(pose_utm).inverse();
        } else {
          tf_utm = Support::tf2FromPose(pose_utm);
        }
        tf_utm_msg.transform = Support::msgFromTf2(tf_utm);

        try {
          broadcaster_->sendTransform(tf_utm_msg);
          RCLCPP_INFO_ONCE(node_->get_logger(), "[%s]: publishing utm_origin tf", getPrintName().c_str());
        }
        catch (...) {
          RCLCPP_ERROR(node_->get_logger(), "exception caught ");
        }
      }
      /*//}*/

      /*//{ tf world origin */
      if (is_utm_source_) {
        geometry_msgs::msg::TransformStamped tf_world_msg;
        tf_world_msg.header.stamp    = odom->header.stamp;
        tf_world_msg.header.frame_id = ns_world_origin_parent_frame_id_;
        tf_world_msg.child_frame_id  = ns_world_origin_child_frame_id_;

        tf2::Transform tf_world;
        tf_world.setOrigin(tf2::Vector3(world_origin_.x, world_origin_.y, world_origin_.z));
        tf_world.setRotation(tf2::Quaternion(0, 0, 0, 1));

        geometry_msgs::msg::Pose pose_utm = odom->pose.pose;
        pose_utm.position.x += utm_origin_.x - first_msg_->pose.pose.position.x;
        pose_utm.position.y += utm_origin_.y - first_msg_->pose.pose.position.y;

        tf2::Transform tf_utm;
        if (is_inverted_) {
          tf_utm = Support::tf2FromPose(pose_utm).inverse();
        } else {
          tf_utm = Support::tf2FromPose(pose_utm);
        }

        tf_world_msg.transform          = Support::msgFromTf2(tf_utm * tf_world);
        tf_world_msg.transform.rotation = pose_inv.orientation;

        try {
          broadcaster_->sendTransform(tf_world_msg);
          RCLCPP_INFO_ONCE(node_->get_logger(), "[%s]: publishing world_origin tf", getPrintName().c_str());
        }
        catch (...) {
          RCLCPP_ERROR(node_->get_logger(), "exception caught ");
        }
      }

      /*//}*/

    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getPrintName().c_str(), tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str());
    }
    RCLCPP_INFO_ONCE(node_->get_logger(), "[%s]: Broadcasting transform from parent frame: %s to child frame: %s based on topic: %s", getPrintName().c_str(), tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str(), full_topic_odom_.c_str());
  }
  /*//}*/

  /* publishTfFromAtt() //{*/
  void publishTfFromAtt(const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& msg) {

    mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer(node_, getPrintName() + "::publishTfFromAtt", ch_->scope_timer.logger, ch_->scope_timer.enabled);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;

    geometry_msgs::msg::Pose pose;
    pose.position.x  = 0.0;
    pose.position.y  = 0.0;
    pose.position.z  = 0.0;
    pose.orientation = msg->quaternion;
    if (is_inverted_) {

      const tf2::Transform           tf       = Support::tf2FromPose(pose);
      const tf2::Transform           tf_inv   = tf.inverse();
      const geometry_msgs::msg::Pose pose_inv = Support::poseFromTf2(tf_inv);

      tf_msg.header.frame_id       = ch_->frames.ns_fcu;
      tf_msg.child_frame_id        = msg->header.frame_id;
      tf_msg.transform.translation = Support::pointToVector3(pose_inv.position);
      tf_msg.transform.rotation    = pose_inv.orientation;

    } else {
      tf_msg.header.frame_id       = msg->header.frame_id;
      tf_msg.child_frame_id        = ch_->frames.ns_fcu;
      tf_msg.transform.translation = Support::pointToVector3(pose.position);
      tf_msg.transform.rotation    = pose.orientation;
    }

    if (Support::noNans(tf_msg)) {
      try {
        scope_timer.checkpoint("before pub");
        broadcaster_->sendTransform(tf_msg);
        scope_timer.checkpoint("after pub");
      }
      catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "exception caught ");
      }
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getPrintName().c_str(), tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str());
    }
    RCLCPP_INFO_ONCE(node_->get_logger(), "[%s]: Broadcasting transform from parent frame: %s to child frame: %s based on topic: %s", getPrintName().c_str(), tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str(), full_topic_attitude_.c_str());
  }
  /*//}*/

  /* publishLocalTf() //{*/
  void publishLocalTf(const std::string& frame_id) {

    mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer(node_, getPrintName() + "::publishLocalTf", ch_->scope_timer.logger, ch_->scope_timer.enabled);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = clock_->now();

    tf_msg.header.frame_id       = frame_id;
    tf_msg.child_frame_id        = frame_id.substr(0, frame_id.find("_origin")) + "_local_origin";
    tf_msg.transform.translation = Support::pointToVector3(first_msg_->pose.pose.position);
    tf_msg.transform.rotation    = first_msg_->pose.pose.orientation;

    if (Support::noNans(tf_msg)) {
      try {
        static_broadcaster_->sendTransform(tf_msg);
      }
      catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "exception caught ");
      }
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getPrintName().c_str(), tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str());
    }
    RCLCPP_INFO_ONCE(node_->get_logger(), "[%s]: Broadcasting transform from parent frame: %s to child frame: %s based on first message of: %s", getPrintName().c_str(), tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str(), full_topic_odom_.c_str());
    is_local_static_tf_published_ = true;
  }
  /*//}*/

  /* publishUtmTf() //{*/
  void publishUtmTf(const std::string& frame_id) {

    mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer(node_, getPrintName() + "::publishUtmTf", ch_->scope_timer.logger, ch_->scope_timer.enabled);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = clock_->now();

    tf_msg.header.frame_id         = frame_id;
    tf_msg.child_frame_id          = frame_id.substr(0, frame_id.find("_origin")) + "_utm_origin";
    tf_msg.transform.translation.x = -utm_origin_.x;  // minus because inverse tf tree
    tf_msg.transform.translation.y = -utm_origin_.y;  // minus because inverse tf tree
    tf_msg.transform.translation.z = -utm_origin_.z;  // minus because inverse tf tree
    tf_msg.transform.rotation.x    = 0;
    tf_msg.transform.rotation.y    = 0;
    tf_msg.transform.rotation.z    = 0;
    tf_msg.transform.rotation.w    = 1;

    if (Support::noNans(tf_msg)) {
      try {
        static_broadcaster_->sendTransform(tf_msg);
      }
      catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "exception caught ");
      }
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getPrintName().c_str(), tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str());
    }
    RCLCPP_INFO_ONCE(node_->get_logger(), "[%s]: Broadcasting transform from parent frame: %s to child frame: %s based on first message of: %s", getPrintName().c_str(), tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str(), full_topic_odom_.c_str());
    is_utm_static_tf_published_ = true;
  }
  /*//}*/

  /* publishWorldTf() //{*/
  void publishWorldTf(const std::string& frame_id) {

    mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer(node_, getPrintName() + "::publishWorldTf", ch_->scope_timer.logger, ch_->scope_timer.enabled);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = clock_->now();

    tf_msg.header.frame_id         = frame_id;
    tf_msg.child_frame_id          = frame_id.substr(0, frame_id.find("_origin")) + "_world_origin";
    tf_msg.transform.translation.x = -(utm_origin_.x - world_origin_.x);  // minus because inverse tf tree
    tf_msg.transform.translation.y = -(utm_origin_.y - world_origin_.y);  // minus because inverse tf tree
    /* tf_msg.transform.translation.z = -(utm_origin_.z);                    // minus because inverse tf tree */
    tf_msg.transform.translation.z = 0;
    tf_msg.transform.rotation.x    = 0;
    tf_msg.transform.rotation.y    = 0;
    tf_msg.transform.rotation.z    = 0;
    tf_msg.transform.rotation.w    = 1;

    if (Support::noNans(tf_msg)) {
      try {
        static_broadcaster_->sendTransform(tf_msg);
      }
      catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "exception caught ");
      }
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getPrintName().c_str(), tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str());
    }
    RCLCPP_INFO_ONCE(node_->get_logger(), "[%s]: Broadcasting transform from parent frame: %s to child frame: %s based on first message of: %s", getPrintName().c_str(), tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str(), full_topic_odom_.c_str());
    is_world_static_tf_published_ = true;
  }
  /*//}*/

  /* republishInFrame() //{*/
  void republishInFrame(const nav_msgs::msg::Odometry::ConstSharedPtr& msg, const std::string& frame_id, mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>& ph) {

    mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer(node_, getPrintName() + "::republishInFrame", ch_->scope_timer.logger, ch_->scope_timer.enabled);

    nav_msgs::msg::Odometry msg_out = *msg;
    msg_out.header.frame_id         = frame_id;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose   = msg->pose.pose;

    auto res = ch_->transformer->transformSingle(pose, frame_id);
    if (res) {
      msg_out.pose.pose = res->pose;
      ph.publish(msg_out);
    } else {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Could not transform pose to %s. Not republishing odom in this frame.", getPrintName().c_str(), frame_id.c_str());
      return;
    }
  }

  /*//}*/
};

/*//}*/

}  // namespace mrs_uav_managers

#endif
