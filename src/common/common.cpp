#include <common.h>

namespace mrs_uav_managers
{

/* idxInVector() //{ */

std::optional<unsigned int> idxInVector(const std::string& str, const std::vector<std::string>& vec) {

  for (unsigned int i = 0; i < vec.size(); i++) {
    if (str == vec[i]) {
      return {i};
    }
  }

  return false;
}

//}

/* validateTrackerCommand() //{ */

bool validateTrackerCommand(const std::optional<mrs_msgs::TrackerCommand>& msg, const std::string& node_name, const std::string& var_name) {

  if (!msg) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: the optional variable '%s' is not set!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check positions

  if (!std::isfinite(msg->position.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->position.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->position.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->position.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->position.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->position.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check velocities

  if (!std::isfinite(msg->velocity.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->velocity.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->velocity.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->velocity.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->velocity.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->velocity.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check accelerations

  if (!std::isfinite(msg->acceleration.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->acceleration.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->acceleration.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->acceleration.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->acceleration.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->acceleration.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check jerk

  if (!std::isfinite(msg->jerk.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->jerk.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->jerk.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->jerk.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->jerk.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->jerk.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check snap

  if (!std::isfinite(msg->snap.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->snap.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->snap.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->snap.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->snap.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->snap.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check attitude rate

  if (!std::isfinite(msg->attitude_rate.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->attitude_rate.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->attitude_rate.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->attitude_rate.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->attitude_rate.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->attitude_rate.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check heading

  if (!std::isfinite(msg->heading)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->heading'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->heading_rate)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->heading_rate'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check thrust

  if (!std::isfinite(msg->thrust)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->thrust'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateOdometry() //{ */

bool validateOdometry(const nav_msgs::Odometry& msg, const std::string& node_name, const std::string& var_name) {

  // check position

  if (!std::isfinite(msg.pose.pose.position.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.pose.pose.position.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.pose.position.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.pose.pose.position.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.pose.position.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.pose.pose.position.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check orientation

  if (!std::isfinite(msg.pose.pose.orientation.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.pose.pose.orientation.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.pose.orientation.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.pose.pose.orientation.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.pose.orientation.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.pose.pose.orientation.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.pose.orientation.w)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.pose.pose.orientation.w'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check velocity

  if (!std::isfinite(msg.twist.twist.linear.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.twist.twist.linear.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.twist.twist.linear.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.twist.twist.linear.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.twist.twist.linear.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.twist.twist.linear.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateVelocityReference() //{ */

bool validateVelocityReference(const mrs_msgs::VelocityReference& msg, const std::string& node_name, const std::string& var_name) {

  // check velocity

  if (!std::isfinite(msg.velocity.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.velocity.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.velocity.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.velocity.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.altitude)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.altitude'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.heading)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.heading'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.heading_rate)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.heading_rate'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateUavState() //{ */

bool validateUavState(const mrs_msgs::UavState& msg, const std::string& node_name, const std::string& var_name) {

  // check position

  if (!std::isfinite(msg.pose.position.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.pose.position.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.position.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.pose.position.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.position.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.pose.position.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check orientation

  if (!std::isfinite(msg.pose.orientation.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.pose.orientation.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.orientation.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.pose.orientation.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.orientation.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.pose.orientation.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.orientation.w)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.pose.orientation.w'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check linear velocity

  if (!std::isfinite(msg.velocity.linear.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.velocity.linear.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.linear.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.velocity.linear.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.linear.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.velocity.linear.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check angular velocity

  if (!std::isfinite(msg.velocity.angular.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.velocity.angular.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.angular.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.velocity.angular.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.angular.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.velocity.angular.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check linear acceleration

  if (!std::isfinite(msg.acceleration.linear.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration.linear.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration.linear.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration.linear.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration.linear.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration.linear.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check angular acceleration

  if (!std::isfinite(msg.acceleration.angular.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration.angular.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration.angular.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration.angular.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration.angular.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration.angular.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check acceleration angular disturbance

  if (!std::isfinite(msg.acceleration_disturbance.angular.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration_disturbance.angular.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration_disturbance.angular.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration_disturbance.angular.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration_disturbance.angular.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration_disturbance.angular.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check acceleration linear disturbance

  if (!std::isfinite(msg.acceleration_disturbance.linear.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration_disturbance.linear.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration_disturbance.linear.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration_disturbance.linear.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration_disturbance.linear.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration_disturbance.linear.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  return true;
}

//}

/* RCChannelToRange() //{ */

double RCChannelToRange(double rc_value, double range, double deadband) {

  double tmp_neg1_to_1 = (rc_value - 0.5) * 2.0;

  if (tmp_neg1_to_1 > 1.0) {
    tmp_neg1_to_1 = 1.0;
  } else if (tmp_neg1_to_1 < -1.0) {
    tmp_neg1_to_1 = -1.0;
  }

  // check the deadband
  if (tmp_neg1_to_1 < deadband && tmp_neg1_to_1 > -deadband) {
    return 0.0;
  }

  if (tmp_neg1_to_1 > 0) {

    double tmp = (tmp_neg1_to_1 - deadband) / (1.0 - deadband);

    return range * tmp;

  } else {

    double tmp = (-tmp_neg1_to_1 - deadband) / (1.0 - deadband);

    return -range * tmp;
  }

  return 0.0;
}

//}

/* validateControlOutput() //{ */

bool validateControlOutput(const Controller::ControlOutput& control_output, const std::string& node_name, const std::string& var_name) {

  if (!control_output.control_output) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: the optional variable '%s' is not set!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  std::variant<std::string> node_name_var{node_name};
  std::variant<std::string> var_name_var{var_name};

  return std::visit(HwApiCmdValidator(), control_output.control_output.value(), node_name_var, var_name_var);
}

//}

// | -------------- validation of HW api commands ------------- |

/* validateHwApiActuatorCmd() //{ */

bool validateHwApiActuatorCmd(const mrs_msgs::HwApiActuatorCmd& msg, const std::string& node_name, const std::string& var_name) {

  for (size_t i = 0; i < msg.motors.size(); i++) {
    if (!std::isfinite(msg.motors[i])) {
      ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.motors[%d]'!!!", node_name.c_str(), var_name.c_str(), int(i));
      return false;
    }
  }

  return true;
}

//}

/* validateHwApiControlGroupCmd() //{ */

bool validateHwApiControlGroupCmd(const mrs_msgs::HwApiControlGroupCmd& msg, const std::string& node_name, const std::string& var_name) {

  if (!std::isfinite(msg.roll)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.roll'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pitch)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.pitch'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.yaw)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.yaw'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.throttle)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.throttle'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateHwApiAttitudeCmd() //{ */

bool validateHwApiAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd& msg, const std::string& node_name, const std::string& var_name) {

  // check the orientation

  if (!std::isfinite(msg.orientation.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.orientation.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.orientation.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.orientation.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.orientation.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.orientation.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.orientation.w)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.orientation.w'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check the throttle

  if (!std::isfinite(msg.throttle)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.throttle'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateHwApiAttitudeRateCmd() //{ */

bool validateHwApiAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd& msg, const std::string& node_name, const std::string& var_name) {

  // check the body rate

  if (!std::isfinite(msg.body_rate.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.body_rate.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.body_rate.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.body_rate.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.body_rate.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.body_rate.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check the throttle

  if (!std::isfinite(msg.throttle)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.throttle'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateHwApiAccelerationHdgRateCmd() //{ */

bool validateHwApiAccelerationHdgRateCmd(const mrs_msgs::HwApiAccelerationHdgRateCmd& msg, const std::string& node_name, const std::string& var_name) {

  // | ----------------- check the acceleration ----------------- |

  if (!std::isfinite(msg.acceleration.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check the heading rate

  if (!std::isfinite(msg.heading_rate)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.heading_rate'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateHwApiAccelerationHdgCmd() //{ */

bool validateHwApiAccelerationHdgCmd(const mrs_msgs::HwApiAccelerationHdgCmd& msg, const std::string& node_name, const std::string& var_name) {

  // | ----------------- check the acceleration ----------------- |

  if (!std::isfinite(msg.acceleration.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.acceleration.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check the heading

  if (!std::isfinite(msg.heading)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.heading'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateHwApiVelocityHdgRateCmd() //{ */

bool validateHwApiVelocityHdgRateCmd(const mrs_msgs::HwApiVelocityHdgRateCmd& msg, const std::string& node_name, const std::string& var_name) {

  // | ----------------- check the velocity ----------------- |

  if (!std::isfinite(msg.velocity.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.velocity.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.velocity.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.velocity.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check the heading rate

  if (!std::isfinite(msg.heading_rate)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.heading_rate'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateHwApiVelocityHdgCmd() //{ */

bool validateHwApiVelocityHdgCmd(const mrs_msgs::HwApiVelocityHdgCmd& msg, const std::string& node_name, const std::string& var_name) {

  // | ----------------- check the velocity ----------------- |

  if (!std::isfinite(msg.velocity.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.velocity.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.velocity.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.velocity.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check the heading

  if (!std::isfinite(msg.heading)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.heading'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateHwApiPositionHdgCmd() //{ */

bool validateHwApiPositionCmd(const mrs_msgs::HwApiPositionCmd& msg, const std::string& node_name, const std::string& var_name) {

  // | ----------------- check the position ----------------- |

  if (!std::isfinite(msg.position.x)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.position.x'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.position.y)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.position.y'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.position.z)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.position.z'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check the heading

  if (!std::isfinite(msg.heading)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.heading'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  return true;
}

//}

}  // namespace mrs_uav_managers
