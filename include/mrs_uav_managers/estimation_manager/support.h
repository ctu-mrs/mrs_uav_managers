#pragma once
#ifndef ESTIMATION_MANAGER_SUPPORT_H
#define ESTIMATION_MANAGER_SUPPORT_H

/* includes //{ */

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <mrs_msgs/msg/uav_state.hpp>

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>

//}

namespace mrs_uav_managers
{

namespace estimation_manager
{

/*//{ class Support */
class Support {

public:
  const static inline std::string waiting_for_string = "\033[0;36mWAITING FOR:\033[0m";

  /*//{ toSnakeCase() */

  static std::string toSnakeCase(const std::string& str_in) {

    std::string str(1, tolower(str_in[0]));

    for (auto it = str_in.begin() + 1; it != str_in.end(); ++it) {
      if (isupper(*it) && *(it - 1) != '_' && islower(*(it - 1))) {
        str += "_";
      }
      str += *it;
    }

    std::transform(str.begin(), str.end(), str.begin(), ::tolower);

    return str;
  }

  /*//}*/

  /* toLowercase() //{ */

  static std::string toLowercase(const std::string str_in) {

    std::string str_out = str_in;
    std::transform(str_out.begin(), str_out.end(), str_out.begin(), ::tolower);

    return str_out;
  }

  //}

  /* toUppercase() //{ */

  static std::string toUppercase(const std::string str_in) {

    std::string str_out = str_in;
    std::transform(str_out.begin(), str_out.end(), str_out.begin(), ::toupper);

    return str_out;
  }

  //}

  /*//{ stateCovToString() */

  template <typename StateCov>
  static std::string stateCovToString(const StateCov& sc) {

    std::stringstream ss;
    ss << "State:\n";

    for (int i = 0; i < sc.x.rows(); i++) {
      for (int j = 0; j < sc.x.cols(); j++) {
        ss << sc.x(i, j) << " ";
      }
      ss << "\n";
    }

    ss << "Cov:\n";

    for (int i = 0; i < sc.P.rows(); i++) {
      for (int j = 0; j < sc.P.cols(); j++) {
        ss << sc.P(i, j) << " ";
      }
      ss << "\n";
    }

    return ss.str();
  }

  /*//}*/

  /* //{ rotateVecByHdg() */

  static tf2::Vector3 rotateVecByHdg(const geometry_msgs::msg::Vector3& vec_in, const double hdg_in) {

    const tf2::Quaternion q_hdg = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(hdg_in);

    const tf2::Vector3 vec_tf2(vec_in.x, vec_in.y, vec_in.z);

    const tf2::Vector3 vec_rotated = tf2::quatRotate(q_hdg, vec_tf2);

    return vec_rotated;
  }

  //}

  /* noNans() //{ */
  static bool noNans(const geometry_msgs::msg::TransformStamped& tf) {

    return (std::isfinite(tf.transform.rotation.x) && std::isfinite(tf.transform.rotation.y) && std::isfinite(tf.transform.rotation.z) &&
            std::isfinite(tf.transform.rotation.w) && std::isfinite(tf.transform.translation.x) && std::isfinite(tf.transform.translation.y) &&
            std::isfinite(tf.transform.translation.z));
  }
  //}

  /* noNans() //{ */

  static bool noNans(const geometry_msgs::msg::Quaternion& q) {

    return (std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w));
  }

  //}

  /* isZeroQuaternion() //{ */

  static bool isZeroQuaternion(const geometry_msgs::msg::Quaternion& q) {

    return (q.x == 0 && q.y == 0 && q.z == 0 && q.w == 0);
  }

  //}

  /* tf2FromPose() //{ */

  static tf2::Transform tf2FromPose(const geometry_msgs::msg::Pose& pose_in) {

    tf2::Vector3 position(pose_in.position.x, pose_in.position.y, pose_in.position.z);

    tf2::Quaternion q;
    tf2::fromMsg(pose_in.orientation, q);

    tf2::Transform tf_out(q, position);

    return tf_out;
  }

  //}

  /* poseFromTf2() //{ */

  static geometry_msgs::msg::Pose poseFromTf2(const tf2::Transform& tf_in) {

    geometry_msgs::msg::Pose pose_out;
    pose_out.position.x = tf_in.getOrigin().getX();
    pose_out.position.y = tf_in.getOrigin().getY();
    pose_out.position.z = tf_in.getOrigin().getZ();

    pose_out.orientation = tf2::toMsg(tf_in.getRotation());

    return pose_out;
  }

  //}

  /* msgFromTf2() //{ */

  static geometry_msgs::msg::Transform msgFromTf2(const tf2::Transform& tf_in) {

    geometry_msgs::msg::Transform tf_out;
    tf_out.translation.x = tf_in.getOrigin().getX();
    tf_out.translation.y = tf_in.getOrigin().getY();
    tf_out.translation.z = tf_in.getOrigin().getZ();
    tf_out.rotation      = tf2::toMsg(tf_in.getRotation());

    return tf_out;
  }

  //}

  /* tf2FromMsg() //{ */

  static tf2::Transform tf2FromMsg(const geometry_msgs::msg::Transform& tf_in) {

    tf2::Transform tf_out;
    tf_out.setOrigin(tf2::Vector3(tf_in.translation.x, tf_in.translation.y, tf_in.translation.z));
    tf_out.setRotation(tf2::Quaternion(tf_in.rotation.x, tf_in.rotation.y, tf_in.rotation.z, tf_in.rotation.w));

    return tf_out;
  }

  //}

  /* pointToVector3() //{ */

  static geometry_msgs::msg::Vector3 pointToVector3(const geometry_msgs::msg::Point& point_in) {

    geometry_msgs::msg::Vector3 vec_out;

    vec_out.x = point_in.x;
    vec_out.y = point_in.y;
    vec_out.z = point_in.z;

    return vec_out;
  }

  //}

  /*//{ rotateVector() */

  static geometry_msgs::msg::Vector3 rotateVector(const geometry_msgs::msg::Vector3& vec_in, const geometry_msgs::msg::Quaternion& q_in) {

    try {
      Eigen::Matrix3d R = mrs_lib::AttitudeConverter(q_in);
      Eigen::Vector3d vec_in_eigen(vec_in.x, vec_in.y, vec_in.z);
      Eigen::Vector3d vec_eigen_rotated = R * vec_in_eigen;

      geometry_msgs::msg::Vector3 vec_out;

      vec_out.x = vec_eigen_rotated[0];
      vec_out.y = vec_eigen_rotated[1];
      vec_out.z = vec_eigen_rotated[2];

      return vec_out;
    }
    catch (...) {
      std::cout << "[EstimationManager::Support::rotateVector()]: failed" << std::endl;
      return vec_in;
    }
  }

  /*//}*/

  /*//{ uavStateToOdom() */

  static nav_msgs::msg::Odometry uavStateToOdom(const mrs_msgs::msg::UavState& uav_state) {

    nav_msgs::msg::Odometry odom;

    odom.header              = uav_state.header;
    odom.child_frame_id      = uav_state.child_frame_id;
    odom.pose.pose           = uav_state.pose;
    odom.twist.twist.angular = uav_state.velocity.angular;

    tf2::Quaternion q;
    tf2::fromMsg(odom.pose.pose.orientation, q);
    odom.twist.twist.linear = Support::rotateVector(uav_state.velocity.linear, mrs_lib::AttitudeConverter(q.inverse()));

    return odom;
  }

  /*//}*/

  /*//{ getPoseDiff() */

  static geometry_msgs::msg::Pose getPoseDiff(const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Pose& p2) {

    tf2::Vector3 v1, v2;
    tf2::fromMsg(p1.position, v1);
    tf2::fromMsg(p2.position, v2);
    const tf2::Vector3 v3 = v1 - v2;

    tf2::Quaternion q1, q2;
    tf2::fromMsg(p1.orientation, q1);
    tf2::fromMsg(p2.orientation, q2);
    tf2::Quaternion q3 = q2 * q1.inverse();
    q3.normalize();

    geometry_msgs::msg::Pose pose_diff;
    tf2::toMsg(v3, pose_diff.position);
    pose_diff.orientation = tf2::toMsg(q3);

    return pose_diff;
  }

  /*//}*/

  /*//{ applyPoseDiff() */

  static geometry_msgs::msg::Pose applyPoseDiff(const geometry_msgs::msg::Pose& pose_in, const geometry_msgs::msg::Pose& pose_diff) {

    tf2::Vector3    pos_in;
    tf2::Quaternion q_in;
    tf2::fromMsg(pose_in.position, pos_in);
    tf2::fromMsg(pose_in.orientation, q_in);

    tf2::Vector3    pos_diff;
    tf2::Quaternion q_diff;
    tf2::fromMsg(pose_diff.position, pos_diff);
    tf2::fromMsg(pose_diff.orientation, q_diff);

    const tf2::Vector3    pos_out = tf2::quatRotate(q_diff.inverse(), (pos_in - pos_diff));
    const tf2::Quaternion q_out   = q_diff.inverse() * q_in;

    geometry_msgs::msg::Pose pose_out;
    tf2::toMsg(pos_out, pose_out.position);
    pose_out.orientation = tf2::toMsg(q_out);

    return pose_out;
  }

  //}

  /*//{ loadParamFile() */

  static void loadParamFile(const rclcpp::Node::SharedPtr& node, const std::string& file_path, const std::string& ns = "") {

    std::string command = "rosparam load " + file_path + " " + ns;
    int         result  = std::system(command.c_str());

    if (result != 0) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Could not set config file " << file_path << " to the parameter server.");
    }
  }

  /*//}*/

  /*//{ isStringInVector() */

  static bool isStringInVector(const std::string& value, const std::vector<std::string>& str_vec) {
    return std::find(str_vec.begin(), str_vec.end(), value) != str_vec.end();
  }

  /*//}*/

  /*//{ frameIdToEstimatorName() */

  static std::string frameIdToEstimatorName(const std::string& str_in) {
    const std::string str_tmp = str_in.substr(str_in.find("/") + 1, str_in.size());
    return str_tmp.substr(0, str_tmp.find("_origin"));
  }

  /*//}*/

private:
  Support() {
  }
};

/*//}*/

}  // namespace estimation_manager

}  // namespace mrs_uav_managers

#endif  // ESTIMATION_MANAGER_SUPPORT_H
