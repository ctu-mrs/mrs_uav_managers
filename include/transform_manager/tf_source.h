#pragma once
#ifndef TF_SOURCE_H
#define TF_SOURCE_H

#include <ros/ros.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/transform_broadcaster.h>
#include <mrs_lib/gps_conversions.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>

#include <memory>
#include <string>

#include "estimation_manager/support.h"
#include "estimation_manager/common_handlers.h"

namespace mrs_uav_managers
{

/*//{ class TfSource */
class TfSource {

  using Support = estimation_manager::Support;

public:
  /*//{ constructor */
  TfSource(const std::string& name, ros::NodeHandle nh, const std::shared_ptr<mrs_lib::TransformBroadcaster>& broadcaster,
           const std::shared_ptr<estimation_manager::CommonHandlers_t> ch)
      : name_(name), nh_(nh), broadcaster_(broadcaster), ch_(ch) {

    ROS_INFO("[%s]: initializing", getPrintName().c_str());

    /*//{ load parameters */
    mrs_lib::ParamLoader param_loader(nh_, getPrintName());
    std::string          odom_topic, attitude_topic, ns;
    param_loader.loadParam(getName() + "/odom_topic", odom_topic);
    param_loader.loadParam(getName() + "/custom_frame_id/enabled", custom_frame_id_enabled_, false);
    if (custom_frame_id_enabled_) {
      param_loader.loadParam(getName() + "/custom_frame_id/frame_id", custom_frame_id_);
    }
    param_loader.loadParam(getName() + "/tf_from_attitude/enabled", tf_from_attitude_enabled_);
    if (tf_from_attitude_enabled_) {
      param_loader.loadParam(getName() + "/tf_from_attitude/attitude_topic", attitude_topic);
    }
    param_loader.loadParam(getName() + "/namespace", ns);
    full_topic_odom_     = "/" + ch_->uav_name + "/" + ns + "/" + odom_topic;
    full_topic_attitude_ = "/" + ch_->uav_name + "/" + ns + "/" + attitude_topic;
    param_loader.loadParam(getName() + "/inverted", is_inverted_);
    param_loader.loadParam(getName() + "/republish_in_frames", republish_in_frames_);

    /* coordinate frames origins //{ */
    param_loader.loadParam(getName() + "/utm_based", is_utm_based_);
    if (is_utm_based_) {
      param_loader.loadParam(getName() + "/in_utm", is_in_utm_);
    }

    // set initial UTM coordinates to zero for tf sources already in UTM frame
    if (is_in_utm_) {
      geometry_msgs::Point origin_pt;
      origin_pt.x = 0;
      origin_pt.y = 0;
      origin_pt.z = 0;
      setUtmOrigin(origin_pt);
    }
    /*//{ utm source */
    std::string utm_origin_source;
    param_loader.loadParam("utm_origin_tf/source", utm_origin_source);
    if (utm_origin_source == getName()) {

      if (!is_utm_based_) {
        ROS_ERROR("[%s]: is utm_origin source but is not utm-based. Check your config!", getPrintName().c_str());
        ros::shutdown();
      }
      is_utm_source_ = true;

      std::string utm_origin_parent_frame_id;
      param_loader.loadParam("utm_origin_tf/parent", utm_origin_parent_frame_id);
      ns_utm_origin_parent_frame_id_ = ch_->uav_name + "/" + utm_origin_parent_frame_id;

      std::string utm_origin_child_frame_id;
      param_loader.loadParam("utm_origin_tf/child", utm_origin_child_frame_id);
      ns_utm_origin_child_frame_id_ = ch_->uav_name + "/" + utm_origin_child_frame_id;
    }
    /*//}*/

    /*//{ world source */
    std::string world_origin_source;
    param_loader.loadParam("world_origin_tf/source", world_origin_source);
    if (world_origin_source == getName()) {

      if (!is_utm_based_) {
        ROS_ERROR("[%s]: is world_origin source but is not utm-based. Check your config!", getPrintName().c_str());
        ros::shutdown();
      }
      is_world_source_ = true;

      std::string world_origin_parent_frame_id;
      param_loader.loadParam("world_origin_tf/parent", world_origin_parent_frame_id);
      ns_world_origin_parent_frame_id_ = ch_->uav_name + "/" + world_origin_parent_frame_id;

      std::string world_origin_child_frame_id;
      param_loader.loadParam("world_origin_tf/child", world_origin_child_frame_id);
      ns_world_origin_child_frame_id_ = ch_->uav_name + "/" + world_origin_child_frame_id;
    }
    /*//}*/

    //}

    if (!param_loader.loadedSuccessfully()) {
      ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
      ros::shutdown();
    }

    /*//}*/

    /*//{ initialize subscribers */
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_;
    shopts.node_name          = getPrintName();
    shopts.no_message_timeout = ros::Duration(0.5);
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sh_tf_source_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, full_topic_odom_, &TfSource::callbackTfSourceOdom, this);
    if (tf_from_attitude_enabled_) {
      sh_tf_source_att_ = mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped>(shopts, full_topic_attitude_, &TfSource::callbackTfSourceAtt, this);
    }
    /*//}*/

    for (auto frame_id : republish_in_frames_) {
      republishers_.push_back(
          std::make_pair(frame_id, mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, full_topic_odom_ + "/" + frame_id.substr(0, frame_id.find("_origin")))));
    }
    is_initialized_ = true;
    ROS_INFO("[%s]: initialized", getPrintName().c_str());
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

  /*//{ setUtmOrigin() */
  void setUtmOrigin(const geometry_msgs::Point& pt) {

    if (is_utm_based_ && !is_utm_origin_set_) {
      utm_origin_        = pt;
      is_utm_origin_set_ = true;
    }
  }
  /*//}*/

  /*//{ setWorldOrigin() */
  void setWorldOrigin(const geometry_msgs::Point& pt) {

    if (is_utm_based_ && !is_world_origin_set_) {
      world_origin_        = pt;
      is_world_origin_set_ = true;
    }
  }
  /*//}*/

private:
  const std::string name_;
  const std::string ns_fcu_frame_id_;

  ros::NodeHandle nh_;

  std::shared_ptr<mrs_lib::TransformBroadcaster> broadcaster_;
  tf2_ros::StaticTransformBroadcaster            static_broadcaster_;

  std::shared_ptr<estimation_manager::CommonHandlers_t> ch_;

  bool is_inverted_;

  bool        is_utm_based_;
  bool        is_in_utm_     = false;
  bool        is_utm_source_ = false;
  std::string ns_utm_origin_parent_frame_id_;
  std::string ns_utm_origin_child_frame_id_;

  bool                 is_utm_origin_set_ = false;
  geometry_msgs::Point utm_origin_;

  bool        is_world_source_ = false;
  std::string ns_world_origin_parent_frame_id_;
  std::string ns_world_origin_child_frame_id_;

  bool                 is_world_origin_set_ = false;
  geometry_msgs::Point world_origin_;

  std::string full_topic_odom_;
  std::string full_topic_attitude_;
  bool        tf_from_attitude_enabled_ = false;

  bool        custom_frame_id_enabled_;
  std::string custom_frame_id_;

  std::atomic_bool is_initialized_               = false;
  std::atomic_bool is_local_static_tf_published_ = false;
  std::atomic_bool is_utm_static_tf_published_   = false;
  std::atomic_bool is_world_static_tf_published_ = false;

  std::vector<std::string> republish_in_frames_;

  std::vector<std::pair<std::string, mrs_lib::PublisherHandler<nav_msgs::Odometry>>> republishers_;

  mrs_lib::SubscribeHandler<nav_msgs::Odometry>               sh_tf_source_odom_;
  mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped> sh_tf_source_att_;
  nav_msgs::OdometryConstPtr                                  first_msg_;

  /*//{ callbackTfSourceOdom()*/
  void callbackTfSourceOdom(mrs_lib::SubscribeHandler<nav_msgs::Odometry>& wrp) {

    if (!is_initialized_) {
      return;
    }

    nav_msgs::OdometryConstPtr msg = wrp.getMsg();
    first_msg_                     = msg;
    publishTfFromOdom(msg);

    if (!is_local_static_tf_published_) {
      publishLocalTf(msg->header.frame_id);
    }

    if (is_utm_based_ && is_utm_origin_set_ && !is_utm_static_tf_published_) {
      publishUtmTf(msg->header.frame_id);
    }

    if (is_utm_based_ && is_world_origin_set_ && !is_world_static_tf_published_) {
      publishWorldTf(msg->header.frame_id);
    }

    for (auto republisher : republishers_) {
      republishInFrame(msg, ch_->uav_name + "/" + republisher.first, republisher.second);
    }
  }
  /*//}*/

  /*//{ callbackTfSourceAtt()*/
  void callbackTfSourceAtt(mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped>& wrp) {

    if (!is_initialized_) {
      return;
    }

    geometry_msgs::QuaternionStampedConstPtr msg = wrp.getMsg();
    publishTfFromAtt(msg);
  }
  /*//}*/

  /* publishTfFromOdom() //{*/
  void publishTfFromOdom(const nav_msgs::OdometryConstPtr& odom) {


    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = odom->header.stamp;

    std::string origin_frame_id = custom_frame_id_enabled_ ? ch_->uav_name + custom_frame_id_ : odom->header.frame_id;

    const tf2::Transform      tf       = Support::tf2FromPose(odom->pose.pose);
    const tf2::Transform      tf_inv   = tf.inverse();
    const geometry_msgs::Pose pose_inv = Support::poseFromTf2(tf_inv);

    /*//{ tf source origin */
    if (is_inverted_) {

      tf_msg.header.frame_id       = ch_->frames.ns_fcu;
      tf_msg.child_frame_id        = origin_frame_id;
      tf_msg.transform.translation = Support::pointToVector3(pose_inv.position);
      tf_msg.transform.rotation    = pose_inv.orientation;

    } else {
      tf_msg.header.frame_id       = origin_frame_id;
      tf_msg.child_frame_id        = ch_->frames.ns_fcu;
      tf_msg.transform.translation = Support::pointToVector3(odom->pose.pose.position);
      tf_msg.transform.rotation    = odom->pose.pose.orientation;
    }

    if (Support::noNans(tf_msg)) {
      try {
        broadcaster_->sendTransform(tf_msg);
      }
      catch (...) {
        ROS_ERROR("exception caught ");
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
          ROS_ERROR("exception caught ");
        }
      }
      /*//}*/

      /*//{ tf utm origin */
      if (is_utm_source_) {
        geometry_msgs::TransformStamped tf_utm_msg;
        tf_utm_msg.header.stamp    = odom->header.stamp;
        tf_utm_msg.header.frame_id = ns_utm_origin_parent_frame_id_;
        tf_utm_msg.child_frame_id  = ns_utm_origin_child_frame_id_;

        if (is_inverted_) {
          tf2::Transform tf_utm;
          tf_utm.setRotation(tf_inv.getRotation());
          tf2::Vector3 utm_origin_pt(utm_origin_.x, utm_origin_.y, utm_origin_.z);
          utm_origin_pt                    = tf_utm * utm_origin_pt;  // transform the utm_origin coordinated to fcu frame
          tf_utm_msg.transform.translation = Support::pointToVector3(pose_inv.position);
          tf_utm_msg.transform.translation.x -= utm_origin_pt.x();
          tf_utm_msg.transform.translation.y -= utm_origin_pt.y();
          tf_utm_msg.transform.translation.z -= utm_origin_pt.z();
          tf_utm_msg.transform.rotation = pose_inv.orientation;

        } else {
          tf_utm_msg.transform.translation = Support::pointToVector3(odom->pose.pose.position);
          tf_utm_msg.transform.translation.x += utm_origin_.x;
          tf_utm_msg.transform.translation.y += utm_origin_.y;
          tf_utm_msg.transform.translation.z += utm_origin_.z;
          tf_utm_msg.transform.rotation = odom->pose.pose.orientation;
        }
        try {
          broadcaster_->sendTransform(tf_utm_msg);
        }
        catch (...) {
          ROS_ERROR("exception caught ");
        }
      }
      /*//}*/

      /*//{ tf world origin*/
      if (is_world_source_) {
        geometry_msgs::TransformStamped tf_world_msg;
        tf_world_msg.header.stamp    = odom->header.stamp;
        tf_world_msg.header.frame_id = ns_world_origin_parent_frame_id_;
        tf_world_msg.child_frame_id  = ns_world_origin_child_frame_id_;

        if (is_inverted_) {
          tf2::Transform tf_world;
          tf_world.setRotation(tf_inv.getRotation());

          tf2::Vector3 world_origin_pt(utm_origin_.x - world_origin_.x, utm_origin_.y - world_origin_.y, world_origin_.z);
          world_origin_pt                    = tf_world * world_origin_pt;  // transform the world_origin coordinated to fcu frame
          tf_world_msg.transform.translation = Support::pointToVector3(pose_inv.position);
          tf_world_msg.transform.translation.x -= world_origin_pt.x();
          tf_world_msg.transform.translation.y -= world_origin_pt.y();
          tf_world_msg.transform.translation.z -= world_origin_pt.z();
          tf_world_msg.transform.rotation = pose_inv.orientation;

        } else {
          tf_world_msg.transform.translation = Support::pointToVector3(odom->pose.pose.position);
          tf_world_msg.transform.translation.x += (world_origin_.x - utm_origin_.x);
          tf_world_msg.transform.translation.y += (world_origin_.y - utm_origin_.y);
          tf_world_msg.transform.translation.z += (world_origin_.z - utm_origin_.z);
          tf_world_msg.transform.rotation = odom->pose.pose.orientation;
        }
        try {
          broadcaster_->sendTransform(tf_world_msg);
        }
        catch (...) {
          ROS_ERROR("exception caught ");
        }
      }

      /*//}*/

    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getPrintName().c_str(), tf_msg.header.frame_id.c_str(),
                        tf_msg.child_frame_id.c_str());
    }
    ROS_INFO_ONCE("[%s]: Broadcasting transform from parent frame: %s to child frame: %s based on topic: %s", getPrintName().c_str(),
                  tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str(), full_topic_odom_.c_str());
  }
  /*//}*/

  /* publishTfFromAtt() //{*/
  void publishTfFromAtt(const geometry_msgs::QuaternionStampedConstPtr& msg) {

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;

    geometry_msgs::Pose pose;
    pose.orientation = msg->quaternion;
    if (is_inverted_) {

      const tf2::Transform      tf       = Support::tf2FromPose(pose);
      const tf2::Transform      tf_inv   = tf.inverse();
      const geometry_msgs::Pose pose_inv = Support::poseFromTf2(tf_inv);

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
        broadcaster_->sendTransform(tf_msg);
      }
      catch (...) {
        ROS_ERROR("exception caught ");
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getPrintName().c_str(), tf_msg.header.frame_id.c_str(),
                        tf_msg.child_frame_id.c_str());
    }
    ROS_INFO_ONCE("[%s]: Broadcasting transform from parent frame: %s to child frame: %s based on topic: %s", getPrintName().c_str(),
                  tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str(), full_topic_attitude_.c_str());
  }
  /*//}*/

  /* publishLocalTf() //{*/
  void publishLocalTf(const std::string& frame_id) {


    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();

    tf_msg.header.frame_id       = frame_id;
    tf_msg.child_frame_id        = frame_id.substr(0, frame_id.find("_origin")) + "_local_origin";
    tf_msg.transform.translation = Support::pointToVector3(first_msg_->pose.pose.position);
    tf_msg.transform.rotation    = first_msg_->pose.pose.orientation;

    if (Support::noNans(tf_msg)) {
      try {
        static_broadcaster_.sendTransform(tf_msg);
      }
      catch (...) {
        ROS_ERROR("exception caught ");
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getPrintName().c_str(), tf_msg.header.frame_id.c_str(),
                        tf_msg.child_frame_id.c_str());
    }
    ROS_INFO_ONCE("[%s]: Broadcasting transform from parent frame: %s to child frame: %s based on first message of: %s", getPrintName().c_str(),
                  tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str(), full_topic_odom_.c_str());
    is_local_static_tf_published_ = true;
  }
  /*//}*/

  /* publishUtmTf() //{*/
  void publishUtmTf(const std::string& frame_id) {


    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();

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
        static_broadcaster_.sendTransform(tf_msg);
      }
      catch (...) {
        ROS_ERROR("exception caught ");
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getPrintName().c_str(), tf_msg.header.frame_id.c_str(),
                        tf_msg.child_frame_id.c_str());
    }
    ROS_INFO_ONCE("[%s]: Broadcasting transform from parent frame: %s to child frame: %s based on first message of: %s", getPrintName().c_str(),
                  tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str(), full_topic_odom_.c_str());
    is_utm_static_tf_published_ = true;
  }
  /*//}*/

  /* publishWorldTf() //{*/
  void publishWorldTf(const std::string& frame_id) {


    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();

    tf_msg.header.frame_id         = frame_id;
    tf_msg.child_frame_id          = frame_id.substr(0, frame_id.find("_origin")) + "_world_origin";
    tf_msg.transform.translation.x = -(utm_origin_.x - world_origin_.x);  // minus because inverse tf tree
    tf_msg.transform.translation.y = -(utm_origin_.y - world_origin_.y);  // minus because inverse tf tree
    tf_msg.transform.translation.z = -(utm_origin_.z);                    // minus because inverse tf tree
    tf_msg.transform.rotation.x    = 0;
    tf_msg.transform.rotation.y    = 0;
    tf_msg.transform.rotation.z    = 0;
    tf_msg.transform.rotation.w    = 1;

    if (Support::noNans(tf_msg)) {
      try {
        static_broadcaster_.sendTransform(tf_msg);
      }
      catch (...) {
        ROS_ERROR("exception caught ");
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getPrintName().c_str(), tf_msg.header.frame_id.c_str(),
                        tf_msg.child_frame_id.c_str());
    }
    ROS_INFO_ONCE("[%s]: Broadcasting transform from parent frame: %s to child frame: %s based on first message of: %s", getPrintName().c_str(),
                  tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str(), full_topic_odom_.c_str());
    is_world_static_tf_published_ = true;
  }
  /*//}*/

  /* republishInFrame() //{*/
  void republishInFrame(const nav_msgs::OdometryConstPtr& msg, const std::string& frame_id, mrs_lib::PublisherHandler<nav_msgs::Odometry>& ph) {

    nav_msgs::Odometry msg_out = *msg;
    msg_out.header.frame_id    = frame_id;

    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose   = msg->pose.pose;

    auto res = ch_->transformer->transformSingle(pose, frame_id);
    if (res) {
      msg_out.pose.pose = res->pose;
      ph.publish(msg_out);
    } else {
      ROS_ERROR_THROTTLE(1.0, "[%s]: Could not transform pose to %s. Not republishing odom in this frame.", getPrintName().c_str(), frame_id.c_str());
      return;
    }
  }

  /*//}*/
};

/*//}*/

}  // namespace mrs_uav_managers

#endif
