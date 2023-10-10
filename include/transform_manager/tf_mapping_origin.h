#pragma once
#ifndef TF_MAPPING_ORIGIN_H
#define TF_MAPPING_ORIGIN_H

#include <ros/ros.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/transform_broadcaster.h>
#include <mrs_lib/attitude_converter.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <mrs_uav_managers/estimation_manager/support.h>
#include <mrs_uav_managers/estimation_manager/common_handlers.h>

namespace mrs_uav_managers
{

/*//{ class TfMappingOrigin */
class TfMappingOrigin {

  using Support = estimation_manager::Support;

public:
  /*//{ constructor */
  TfMappingOrigin(ros::NodeHandle nh, const std::shared_ptr<mrs_lib::TransformBroadcaster>& broadcaster,
                  const std::shared_ptr<estimation_manager::CommonHandlers_t> ch)
      : nh_(nh), broadcaster_(broadcaster), ch_(ch) {

    ROS_INFO("[%s]: initializing", getPrintName().c_str());


    mrs_lib::ParamLoader param_loader(nh_, getPrintName());

    const std::string yaml_prefix = "mrs_uav_managers/transform_manager/mapping_origin_tf/";

    /*//{ load mapping origin parameters */
    param_loader.loadParam(yaml_prefix + "debug_prints", debug_prints_);
    param_loader.loadParam(yaml_prefix + "lateral_topic", lateral_topic_);
    param_loader.loadParam(yaml_prefix + "altitude_topic", altitude_topic_);
    param_loader.loadParam(yaml_prefix + "orientation_topic", orientation_topic_);
    param_loader.loadParam(yaml_prefix + "inverted", tf_inverted_);
    param_loader.loadParam(yaml_prefix + "custom_frame_id/enabled", custom_frame_id_enabled_);
    if (custom_frame_id_enabled_) {
      param_loader.loadParam(yaml_prefix + "custom_frame_id/frame_id", custom_frame_id_);
    }

    if (!param_loader.loadedSuccessfully()) {
      ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
      ros::shutdown();
    }

    /*//}*/

    /*//{ initialize subscribers */
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_;
    shopts.node_name          = getPrintName();
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sh_mapping_odom_lat_ =
        mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "/" + ch_->uav_name + "/" + lateral_topic_, &TfMappingOrigin::callbackMappingOdomLat, this);

    if (orientation_topic_ != lateral_topic_) {
      sh_mapping_odom_rot_ = mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped>(shopts, "/" + ch_->uav_name + "/" + orientation_topic_.c_str(),
                                                                                         &TfMappingOrigin::callbackMappingOdomRot, this);
    }

    if (altitude_topic_ != lateral_topic_) {
      sh_mapping_odom_alt_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "/" + ch_->uav_name + "/" + altitude_topic_.c_str(),
                                                                           &TfMappingOrigin::callbackMappingOdomAlt, this);
    }
    /*//}*/

    ph_map_delay_ = mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>(nh_, "map_delay_out", 10);

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

private:
  const std::string name_ = "TfMappingOrigin";

  ros::NodeHandle nh_;

  std::shared_ptr<mrs_lib::TransformBroadcaster> broadcaster_;

  std::shared_ptr<estimation_manager::CommonHandlers_t> ch_;

  std::atomic_bool is_initialized_ = false;

  mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped> ph_map_delay_;

  bool        debug_prints_;
  bool        tf_inverted_;
  bool        custom_frame_id_enabled_;
  std::string custom_frame_id_;
  double      cache_duration_;

  std::string                                   lateral_topic_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_mapping_odom_lat_;
  std::vector<nav_msgs::Odometry>               vec_mapping_odom_lat_;
  std::atomic_bool                              got_mapping_odom_lat_ = false;
  std::atomic<double>                           timestamp_lat_;

  /*//{ callbackMappingOdomLat() */

  void callbackMappingOdomLat(const nav_msgs::Odometry::ConstPtr msg) {

    if (!is_initialized_) {
      return;
    }

    timestamp_lat_ = msg->header.stamp.toSec();

    if (!got_mapping_odom_lat_) {
      got_mapping_odom_lat_ = true;
    }

    if (!got_mapping_odom_rot_ && orientation_topic_ == lateral_topic_) {
      got_mapping_odom_rot_ = true;
    }

    if (!got_mapping_odom_alt_ && altitude_topic_ == lateral_topic_) {
      got_mapping_odom_alt_ = true;
    }

    mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("TfMappingOrigin::callbackMappingOdomLat", ch_->scope_timer.logger, ch_->scope_timer.enabled);

    const double hdg_mapping_old = mrs_lib::AttitudeConverter(msg->pose.pose.orientation).getHeading();

    /* publish aloam mapping origin tf //{ */

    bool clear_needed = false;

    if (got_mapping_odom_rot_ && got_mapping_odom_alt_) {
      std::scoped_lock lock(mtx_mapping_odom_rot_);

      // Copy mapping odometry
      nav_msgs::Odometry mapping_odom;
      mapping_odom = *msg;

      // Find corresponding orientation
      geometry_msgs::QuaternionStamped rot_tmp           = *sh_mapping_odom_rot_.getMsg();  // start with newest msg
      ros::Time                        dbg_timestamp_rot = rot_tmp.header.stamp;
      tf2::Quaternion                  tf2_rot;

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
          rot_tmp           = vec_mapping_odom_rot_.at(i);
          dbg_timestamp_rot = vec_mapping_odom_rot_.at(i).header.stamp;
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
      double    odom_alt          = msg->pose.pose.position.z;  // start with newest msg
      ros::Time dbg_timestamp_alt = msg->header.stamp;
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
            ROS_WARN_THROTTLE(1.0, "[%s] mapping orientation cache (for mapping tf) is too small.", getPrintName().c_str());
          }
          odom_alt          = vec_mapping_odom_alt_.at(i).pose.pose.position.z;
          dbg_timestamp_alt = vec_mapping_odom_alt_.at(i).header.stamp;
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
      if (custom_frame_id_enabled_) {
        tf_mapping.child_frame_id = ch_->uav_name + "/" + custom_frame_id_;
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

      // debug timestamps: rot and alt timestamps should be as close as possible to lat_timestamp, the delay is the difference between current time and the time
      // of acquisition of scan that was used to calculate the pose estimate
      if (debug_prints_) {
        ROS_INFO("[%s] lat timestamp: %.6f, rot stamp: %.6f (diff: %.6f),  alt stamp: %.6f (diff: %.6f), delay: %.6f", getPrintName().c_str(),
                 mapping_odom.header.stamp.toSec(), dbg_timestamp_rot.toSec(), dbg_timestamp_rot.toSec() - mapping_odom.header.stamp.toSec(),
                 dbg_timestamp_alt.toSec(), dbg_timestamp_alt.toSec() - mapping_odom.header.stamp.toSec(),
                 ros::Time::now().toSec() - mapping_odom.header.stamp.toSec());
      }

      mrs_msgs::Float64Stamped map_delay_msg;
      map_delay_msg.header.stamp = ros::Time::now();
      map_delay_msg.value        = ros::Time::now().toSec() - mapping_odom.header.stamp.toSec();
      ph_map_delay_.publish(map_delay_msg);
    }

    //}
  }
  /*//}*/

  std::string                                                 orientation_topic_;
  mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped> sh_mapping_odom_rot_;
  std::vector<geometry_msgs::QuaternionStamped>               vec_mapping_odom_rot_;
  std::mutex                                                  mtx_mapping_odom_rot_;
  std::atomic_bool                                            got_mapping_odom_rot_ = false;

  /*//{ callbackMappingOdomRot() */
  void callbackMappingOdomRot(const geometry_msgs::QuaternionStamped::ConstPtr msg) {

    if (!is_initialized_) {
      return;
    }

    if (!got_mapping_odom_lat_) {
      return;
    }

    std::scoped_lock lock(mtx_mapping_odom_rot_);

    // Add new data
    vec_mapping_odom_rot_.push_back(*msg);

    // Delete old data
    size_t index_delete = 0;
    bool   clear_needed = false;
    for (size_t i = 0; i < vec_mapping_odom_rot_.size(); i++) {
      if (timestamp_lat_ - vec_mapping_odom_rot_.at(i).header.stamp.toSec() > cache_duration_) {
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

    if (debug_prints_) {
      ROS_INFO_THROTTLE(1.0, "[%s]: mapping odom rot cache size: %lu", getPrintName().c_str(), vec_mapping_odom_rot_.size());
    }
  }
  /*//}*/

  std::string                                   altitude_topic_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_mapping_odom_alt_;
  std::vector<nav_msgs::Odometry>               vec_mapping_odom_alt_;
  std::mutex                                    mtx_mapping_odom_alt_;
  std::atomic_bool                              got_mapping_odom_alt_ = false;

  /*//{ callbackMappingOdomAlt() */
  void callbackMappingOdomAlt(const nav_msgs::Odometry::ConstPtr msg) {

    if (!is_initialized_) {
      return;
    }

    if (!got_mapping_odom_lat_) {
      return;
    }

    std::scoped_lock lock(mtx_mapping_odom_alt_);

    // Add new data
    vec_mapping_odom_alt_.push_back(*msg);

    // Delete old data
    size_t index_delete = 0;
    bool   clear_needed = false;
    for (size_t i = 0; i < vec_mapping_odom_alt_.size(); i++) {
      if (timestamp_lat_ - vec_mapping_odom_alt_.at(i).header.stamp.toSec() > cache_duration_) {
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

    if (debug_prints_) {
      ROS_INFO_THROTTLE(1.0, "[%s]: mapping odom alt cache size: %lu", getPrintName().c_str(), vec_mapping_odom_alt_.size());
    }
  }
  /*//}*/
};
/*//}*/

}  // namespace mrs_uav_managers

#endif
