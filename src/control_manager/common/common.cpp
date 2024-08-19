#include <mrs_uav_managers/control_manager/common.h>

namespace mrs_uav_managers
{

namespace control_manager
{

/* idxInVector() //{ */

std::optional<unsigned int> idxInVector(const std::string& str, const std::vector<std::string>& vec) {

  for (unsigned int i = 0; i < vec.size(); i++) {
    if (str == vec[i]) {
      return {i};
    }
  }

  return {};
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

  if (!std::isfinite(msg->heading_acceleration)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->heading_acceleration'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->heading_jerk)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->heading_jerk'!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  // check throttle

  if (!std::isfinite(msg->throttle)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s->throttle'!!!", node_name.c_str(), var_name.c_str());
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

/* validateReference() //{ */

bool validateReference(const mrs_msgs::Reference& msg, const std::string& node_name, const std::string& var_name) {

  // check position

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

  // check heading

  if (!std::isfinite(msg.heading)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable '%s.heading'!!!", node_name.c_str(), var_name.c_str());
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

/* loadDetailedUavModelParams() //{ */

std::optional<DetailedModelParams_t> loadDetailedUavModelParams(ros::NodeHandle& nh, const std::string& node_name, const std::string& platform_config,
                                                                const std::string& custom_config) {

  mrs_lib::ParamLoader param_loader(nh, node_name);

  if (custom_config != "") {
    param_loader.addYamlFile(custom_config);
  }

  if (platform_config != "") {
    param_loader.addYamlFile(platform_config);
  }

  double mass;
  double arm_length;
  double body_height;
  double force_constant;
  double torque_constant;
  double prop_radius;
  double rpm_min;
  double rpm_max;

  bool detailed_loaded = true;

  detailed_loaded &= param_loader.loadParam("uav_mass", mass, 0.0);

  detailed_loaded &= param_loader.loadParam("model_params/arm_length", arm_length, 0.0);
  detailed_loaded &= param_loader.loadParam("model_params/body_height", body_height, 0.0);

  detailed_loaded &= param_loader.loadParam("model_params/propulsion/force_constant", force_constant, 0.0);
  detailed_loaded &= param_loader.loadParam("model_params/propulsion/torque_constant", torque_constant, 0.0);
  detailed_loaded &= param_loader.loadParam("model_params/propulsion/prop_radius", prop_radius, 0.0);
  detailed_loaded &= param_loader.loadParam("model_params/propulsion/rpm/min", rpm_min, 0.0);
  detailed_loaded &= param_loader.loadParam("model_params/propulsion/rpm/max", rpm_max, 0.0);

  Eigen::MatrixXd allocation_matrix;

  detailed_loaded &= param_loader.loadMatrixDynamic("model_params/propulsion/allocation_matrix", allocation_matrix, Eigen::Matrix4d::Identity(), 4, -1);

  if (!detailed_loaded) {
    ROS_WARN(
        "[%s]: detailed UAV model params not loaded, missing some parameters. This will not permit operations when ACTUATORS or CONTROL_GROUP control "
        "outputs would be possible.",
        node_name.c_str());
    return {};
  } else {
    ROS_INFO("[%s]: detailed UAV model params successfully loaded.", node_name.c_str());
  }

  int n_motors = allocation_matrix.cols();

  DetailedModelParams_t model_params;

  model_params.arm_length  = arm_length;
  model_params.body_height = body_height;
  model_params.prop_radius = prop_radius;

  Eigen::Matrix3d inertia_matrix;

  bool inertia_loaded = param_loader.loadMatrixStatic("model_params/inertia_matrix", inertia_matrix, Eigen::Matrix3d::Identity());

  if (inertia_loaded) {

    model_params.inertia = inertia_matrix;
    ROS_INFO("[%s]: inertia matrix loaded from config file:", node_name.c_str());
    ROS_INFO_STREAM(model_params.inertia);

  } else {

    ROS_INFO("[%s]: inertia matrix missing in the config file, computing it from the other params.", node_name.c_str());

    // create the inertia matrix
    model_params.inertia       = Eigen::Matrix3d::Zero();
    model_params.inertia(0, 0) = mass * (3.0 * arm_length * arm_length + body_height * body_height) / 12.0;
    model_params.inertia(1, 1) = mass * (3.0 * arm_length * arm_length + body_height * body_height) / 12.0;
    model_params.inertia(2, 2) = (mass * arm_length * arm_length) / 2.0;

    ROS_INFO("[%s]: inertia computed form parameters:", node_name.c_str());
    ROS_INFO_STREAM(model_params.inertia);
  }

  // create the force-torque allocation matrix
  model_params.force_torque_mixer = allocation_matrix;
  model_params.force_torque_mixer.row(0) *= arm_length * force_constant;
  model_params.force_torque_mixer.row(1) *= arm_length * force_constant;
  model_params.force_torque_mixer.row(2) *= torque_constant * (3.0 * prop_radius) * force_constant;
  model_params.force_torque_mixer.row(3) *= force_constant;

  // | ------- create the control group allocation matrix ------- |

  // pseudoinverse of the force-torque matrix (maximum norm)
  Eigen::MatrixXd alloc_tmp =
      model_params.force_torque_mixer.transpose() * (model_params.force_torque_mixer * model_params.force_torque_mixer.transpose()).inverse();

  // | ------------- normalize the allocation matrix ------------ |
  // this will make it match the PX4 control group mixing

  // the first two columns (roll, pitch)
  for (int i = 0; i < n_motors; i++) {
    alloc_tmp.block(i, 0, 1, 2).normalize();
  }

  // the 3rd column (yaw)
  for (int i = 0; i < n_motors; i++) {
    if (alloc_tmp(i, 2) > 1e-2) {
      alloc_tmp(i, 2) = 1.0;
    } else if (alloc_tmp(i, 2) < -1e-2) {
      alloc_tmp(i, 2) = -1.0;
    } else {
      alloc_tmp(i, 2) = 0.0;
    }
  }

  // the 4th column (throttle)
  for (int i = 0; i < n_motors; i++) {
    alloc_tmp(i, 3) = 1.0;
  }

  model_params.control_group_mixer = alloc_tmp;

  return model_params;
}

//}

/* getLowestOutput() //{ */

CONTROL_OUTPUT getLowestOuput(const ControlOutputModalities_t& outputs) {

  if (outputs.actuators) {
    return ACTUATORS_CMD;
  }

  if (outputs.control_group) {
    return CONTROL_GROUP;
  }

  if (outputs.attitude_rate) {
    return ATTITUDE_RATE;
  }

  if (outputs.attitude) {
    return ATTITUDE;
  }

  if (outputs.acceleration_hdg_rate) {
    return ACCELERATION_HDG_RATE;
  }

  if (outputs.acceleration_hdg) {
    return ACCELERATION_HDG;
  }

  if (outputs.velocity_hdg_rate) {
    return VELOCITY_HDG_RATE;
  }

  if (outputs.velocity_hdg) {
    return VELOCITY_HDG;
  }

  return POSITION;
}

//}

/* getHighestOutput() //{ */

CONTROL_OUTPUT getHighestOuput(const ControlOutputModalities_t& outputs) {

  if (outputs.position) {
    return POSITION;
  }

  if (outputs.velocity_hdg) {
    return VELOCITY_HDG;
  }

  if (outputs.velocity_hdg_rate) {
    return VELOCITY_HDG_RATE;
  }

  if (outputs.acceleration_hdg) {
    return ACCELERATION_HDG;
  }

  if (outputs.acceleration_hdg_rate) {
    return ACCELERATION_HDG_RATE;
  }

  if (outputs.attitude) {
    return ATTITUDE;
  }

  if (outputs.attitude_rate) {
    return ATTITUDE_RATE;
  }

  if (outputs.control_group) {
    return CONTROL_GROUP;
  }

  return ACTUATORS_CMD;
}

//}

// | -------- extraction of throttle out of hw api cmds ------- |

/* extractThrottle() //{ */

std::optional<double> extractThrottle(const Controller::ControlOutput& control_output) {

  if (!control_output.control_output) {
    return {};
  }

  return std::visit(HwApiCmdExtractThrottleVisitor(), control_output.control_output.value());
}

//}

// | -------------- validation of HW api commands ------------- |

/* validateControlOutput() //{ */

bool validateControlOutput(const Controller::ControlOutput& control_output, const ControlOutputModalities_t& output_modalities, const std::string& node_name,
                           const std::string& var_name) {

  if (!control_output.control_output) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: the optional variable '%s' is not set!!!", node_name.c_str(), var_name.c_str());
    return false;
  }

  std::variant<ControlOutputModalities_t> output_modalities_var{output_modalities};
  std::variant<std::string>               node_name_var{node_name};
  std::variant<std::string>               var_name_var{var_name};

  return std::visit(HwApiValidateVisitor(), control_output.control_output.value(), output_modalities_var, node_name_var, var_name_var);
}

//}

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

// | ------------ initialization of HW api commands ----------- |

/* initializeDefaultOutput() //{ */

Controller::HwApiOutputVariant initializeDefaultOutput(const ControlOutputModalities_t& possible_outputs, const mrs_msgs::UavState& uav_state,
                                                       const double& min_throttle, const double& n_motors) {

  CONTROL_OUTPUT lowest_output = getLowestOuput(possible_outputs);

  Controller::HwApiOutputVariant output;

  switch (lowest_output) {
    case ACTUATORS_CMD: {
      output = mrs_msgs::HwApiActuatorCmd();
      break;
    }
    case CONTROL_GROUP: {
      output = mrs_msgs::HwApiControlGroupCmd();
      break;
    }
    case ATTITUDE_RATE: {
      output = mrs_msgs::HwApiAttitudeRateCmd();
      break;
    }
    case ATTITUDE: {
      output = mrs_msgs::HwApiAttitudeCmd();
      break;
    }
    case ACCELERATION_HDG_RATE: {
      output = mrs_msgs::HwApiAccelerationHdgRateCmd();
      break;
    }
    case ACCELERATION_HDG: {
      output = mrs_msgs::HwApiAccelerationHdgCmd();
      break;
    }
    case VELOCITY_HDG_RATE: {
      output = mrs_msgs::HwApiVelocityHdgRateCmd();
      break;
    }
    case VELOCITY_HDG: {
      output = mrs_msgs::HwApiVelocityHdgCmd();
      break;
    }
    case POSITION: {
      output = mrs_msgs::HwApiPositionCmd();
      break;
    }
  }

  std::variant<mrs_msgs::UavState> uav_state_var{uav_state};
  std::variant<double>             min_throttle_var{min_throttle};
  std::variant<double>             n_motors_var{n_motors};

  std::visit(HwApiInitializeVisitor(), output, uav_state_var, min_throttle_var, n_motors_var);

  return output;
}

//}

/* initializeHwApiCmd(mrs_msgs::HwApiActuatorCmd& msg) //{ */

void initializeHwApiCmd(mrs_msgs::HwApiActuatorCmd& msg, const double& min_throttle, const double& n_motors) {

  msg.stamp = ros::Time::now();

  for (int i = 0; i < n_motors; i++) {
    msg.motors.push_back(min_throttle);
  }
}

//}

/* initializeHwApiCmd(mrs_msgs::HwApiControlGroupCmd& msg) //{ */

void initializeHwApiCmd(mrs_msgs::HwApiControlGroupCmd& msg, const double& min_throttle) {

  msg.stamp = ros::Time::now();

  msg.roll     = 0;
  msg.pitch    = 0;
  msg.yaw      = 0;
  msg.throttle = min_throttle;
}

//}

/* initializeHwApiCmd(mrs_msgs::HwApiAttitudeRateCmd& msg) //{ */

void initializeHwApiCmd(mrs_msgs::HwApiAttitudeRateCmd& msg, const double& min_throttle) {

  msg.stamp = ros::Time::now();

  msg.body_rate.x = 0;
  msg.body_rate.y = 0;
  msg.body_rate.z = 0;
  msg.throttle    = min_throttle;
}

//}

/* initializeHwApiCmd(mrs_msgs::HwApiAttitudeCmd& msg) //{ */

void initializeHwApiCmd(mrs_msgs::HwApiAttitudeCmd& msg, const mrs_msgs::UavState& uav_state, const double& min_throttle) {

  msg.stamp = ros::Time::now();

  msg.orientation = uav_state.pose.orientation;
  msg.throttle    = min_throttle;
}

//}

/* initializeHwApiCmd(mrs_msgs::HwApiAccelerationHdgRateCmd& msg) //{ */

void initializeHwApiCmd(mrs_msgs::HwApiAccelerationHdgRateCmd& msg, const mrs_msgs::UavState& uav_state) {

  msg.header.frame_id = uav_state.header.frame_id;
  msg.header.stamp    = ros::Time::now();

  msg.acceleration.x = 0;
  msg.acceleration.y = 0;
  msg.acceleration.z = 0;
  msg.heading_rate   = 0;
}

//}

/* initializeHwApiCmd(mrs_msgs::HwApiAccelerationHdgCmd& msg) //{ */

void initializeHwApiCmd(mrs_msgs::HwApiAccelerationHdgCmd& msg, const mrs_msgs::UavState& uav_state) {

  msg.header.frame_id = uav_state.header.frame_id;
  msg.header.stamp    = ros::Time::now();

  msg.acceleration.x = 0;
  msg.acceleration.y = 0;
  msg.acceleration.z = 0;

  try {
    msg.heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (std::runtime_error& exrun) {
    msg.heading = 0;
  }
}

//}

/* initializeHwApiCmd(mrs_msgs::HwApiVelocityHdgRateCmd& msg) //{ */

void initializeHwApiCmd(mrs_msgs::HwApiVelocityHdgRateCmd& msg, const mrs_msgs::UavState& uav_state) {

  msg.header.frame_id = uav_state.header.frame_id;
  msg.header.stamp    = ros::Time::now();

  msg.velocity.x   = 0;
  msg.velocity.y   = 0;
  msg.velocity.z   = 0;
  msg.heading_rate = 0;
}

//}

/* initializeHwApiCmd(mrs_msgs::HwApiVelocityHdgCmd& msg) //{ */

void initializeHwApiCmd(mrs_msgs::HwApiVelocityHdgCmd& msg, const mrs_msgs::UavState& uav_state) {

  msg.header.frame_id = uav_state.header.frame_id;
  msg.header.stamp    = ros::Time::now();

  msg.velocity.x = 0;
  msg.velocity.y = 0;
  msg.velocity.z = 0;

  try {
    msg.heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (std::runtime_error& exrun) {
    msg.heading = 0;
  }
}

//}

/* initializeHwApiCmd(mrs_msgs::HwApiPositionCmd& msg) //{ */

void initializeHwApiCmd(mrs_msgs::HwApiPositionCmd& msg, const mrs_msgs::UavState& uav_state) {

  msg.header.frame_id = uav_state.header.frame_id;
  msg.header.stamp    = ros::Time::now();

  msg.position.x = uav_state.pose.position.x;
  msg.position.y = uav_state.pose.position.y;
  msg.position.z = uav_state.pose.position.z;

  try {
    msg.heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (std::runtime_error& exrun) {
    msg.heading = 0;
  }
}

//}

}  // namespace control_manager

}  // namespace mrs_uav_managers
