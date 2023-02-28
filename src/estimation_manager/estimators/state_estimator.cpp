#include "estimators/state/state_estimator.h"

namespace mrs_uav_state_estimation
{

/*//{ publishUavState() */
void StateEstimator::publishUavState() const {
  std::scoped_lock lock(mtx_uav_state_);
  ph_uav_state_.publish(uav_state_);
}
/*//}*/

/*//{ publishOdom() */
void StateEstimator::publishOdom() const {

  std::scoped_lock lock(mtx_odom_);
  ph_odom_.publish(odom_);
}
/*//}*/

/*//{ publishCovariance() */
void StateEstimator::publishCovariance() const {
  std::scoped_lock lock(mtx_covariance_);
  ph_pose_covariance_.publish(pose_covariance_);
  ph_twist_covariance_.publish(twist_covariance_);
}
/*//}*/

/*//{ publishInnovation() */
void StateEstimator::publishInnovation() const {

  std::scoped_lock lock(mtx_innovation_);
  ph_innovation_.publish(innovation_);
}
/*//}*/

/*//{ rotateQuaternionByHeading() */
geometry_msgs::Quaternion StateEstimator::rotateQuaternionByHeading(const geometry_msgs::Quaternion& q, const double hdg) const {

      tf2::Quaternion tf2_q = mrs_lib::AttitudeConverter(q);

      // Obtain heading from quaternion
      double q_hdg = 0;
      try {
        q_hdg = mrs_lib::AttitudeConverter(q).getHeading();
      }
      catch (...) {
        ROS_WARN("[rotateQuaternionByHeading()]: failed to getHeading() from quaternion");
      }

      // Build rotation matrix from difference between new heading and quaternion heading
      tf2::Matrix3x3 rot_mat = mrs_lib::AttitudeConverter(Eigen::AngleAxisd(hdg - q_hdg, Eigen::Vector3d::UnitZ()));

      // Transform the quaternion orientation by the rotation matrix
      geometry_msgs::Quaternion q_new = mrs_lib::AttitudeConverter(tf2::Transform(rot_mat) * tf2_q);

      return q_new;
}
/*//}*/

}  // namespace mrs_uav_state_estimation

