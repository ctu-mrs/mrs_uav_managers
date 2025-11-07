#include <mrs_uav_managers/control_manager/common.h>

namespace mrs_uav_managers
{

namespace control_manager
{

/* idxInVector() //{ */

std::optional<unsigned int> idxInVector(const std::string &str, const std::vector<std::string> &vec) {

  for (unsigned int i = 0; i < vec.size(); i++) {
    if (str == vec[i]) {
      return {i};
    }
  }

  return {};
}

//}

/* validateTrackerCommand() //{ */

bool validateTrackerCommand(const rclcpp::Node::SharedPtr &node, const std::optional<mrs_msgs::msg::TrackerCommand> &msg, const std::string &var_name) {

  if (!msg) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "the optional variable '%s' is not set!!!", var_name.c_str());
    return false;
  }

  // check positions

  if (!std::isfinite(msg->position.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->position.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->position.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->position.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->position.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->position.z'!!!", var_name.c_str());
    return false;
  }

  // check velocities

  if (!std::isfinite(msg->velocity.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->velocity.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->velocity.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->velocity.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->velocity.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->velocity.z'!!!", var_name.c_str());
    return false;
  }

  // check accelerations

  if (!std::isfinite(msg->acceleration.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->acceleration.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->acceleration.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->acceleration.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->acceleration.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->acceleration.z'!!!", var_name.c_str());
    return false;
  }

  // check jerk

  if (!std::isfinite(msg->jerk.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->jerk.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->jerk.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->jerk.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->jerk.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->jerk.z'!!!", var_name.c_str());
    return false;
  }

  // check snap

  if (!std::isfinite(msg->snap.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->snap.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->snap.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->snap.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->snap.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->snap.z'!!!", var_name.c_str());
    return false;
  }

  // check attitude rate

  if (!std::isfinite(msg->attitude_rate.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->attitude_rate.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->attitude_rate.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->attitude_rate.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->attitude_rate.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->attitude_rate.z'!!!", var_name.c_str());
    return false;
  }

  // check heading

  if (!std::isfinite(msg->heading)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->heading'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->heading_rate)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->heading_rate'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->heading_acceleration)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->heading_acceleration'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg->heading_jerk)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->heading_jerk'!!!", var_name.c_str());
    return false;
  }

  // check throttle

  if (!std::isfinite(msg->throttle)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s->throttle'!!!", var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateOdometry() //{ */

bool validateOdometry(const rclcpp::Node::SharedPtr &node, const nav_msgs::msg::Odometry &msg, const std::string &var_name) {

  // check position

  if (!std::isfinite(msg.pose.pose.position.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.pose.pose.position.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.pose.position.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.pose.pose.position.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.pose.position.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.pose.pose.position.z'!!!", var_name.c_str());
    return false;
  }

  // check orientation

  if (!std::isfinite(msg.pose.pose.orientation.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.pose.pose.orientation.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.pose.orientation.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.pose.pose.orientation.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.pose.orientation.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.pose.pose.orientation.z'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.pose.orientation.w)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.pose.pose.orientation.w'!!!", var_name.c_str());
    return false;
  }

  // check velocity

  if (!std::isfinite(msg.twist.twist.linear.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.twist.twist.linear.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.twist.twist.linear.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.twist.twist.linear.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.twist.twist.linear.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.twist.twist.linear.z'!!!", var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateVelocityReference() //{ */

bool validateVelocityReference(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::VelocityReference &msg, const std::string &var_name) {

  // check velocity

  if (!std::isfinite(msg.velocity.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.velocity.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.velocity.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.velocity.z'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.altitude)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.altitude'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.heading)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.heading'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.heading_rate)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.heading_rate'!!!", var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateReference() //{ */

bool validateReference(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::Reference &msg, const std::string &var_name) {

  // check position

  if (!std::isfinite(msg.position.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.position.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.position.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.position.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.position.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.position.z'!!!", var_name.c_str());
    return false;
  }

  // check heading

  if (!std::isfinite(msg.heading)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.heading'!!!", var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateUavState() //{ */

bool validateUavState(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::UavState &msg, const std::string &var_name) {

  // check position

  if (!std::isfinite(msg.pose.position.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.pose.position.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.position.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.pose.position.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.position.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.pose.position.z'!!!", var_name.c_str());
    return false;
  }

  // check orientation

  if (!std::isfinite(msg.pose.orientation.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.pose.orientation.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.orientation.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.pose.orientation.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.orientation.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.pose.orientation.z'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pose.orientation.w)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.pose.orientation.w'!!!", var_name.c_str());
    return false;
  }

  // check linear velocity

  if (!std::isfinite(msg.velocity.linear.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.velocity.linear.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.linear.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.velocity.linear.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.linear.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.velocity.linear.z'!!!", var_name.c_str());
    return false;
  }

  // check angular velocity

  if (!std::isfinite(msg.velocity.angular.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.velocity.angular.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.angular.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.velocity.angular.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.angular.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.velocity.angular.z'!!!", var_name.c_str());
    return false;
  }

  // check linear acceleration

  if (!std::isfinite(msg.acceleration.linear.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration.linear.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration.linear.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration.linear.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration.linear.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration.linear.z'!!!", var_name.c_str());
    return false;
  }

  // check angular acceleration

  if (!std::isfinite(msg.acceleration.angular.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration.angular.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration.angular.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration.angular.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration.angular.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration.angular.z'!!!", var_name.c_str());
    return false;
  }

  // check acceleration angular disturbance

  if (!std::isfinite(msg.acceleration_disturbance.angular.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration_disturbance.angular.x'!!!",
                          var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration_disturbance.angular.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration_disturbance.angular.y'!!!",
                          var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration_disturbance.angular.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration_disturbance.angular.z'!!!",
                          var_name.c_str());
    return false;
  }

  // check acceleration linear disturbance

  if (!std::isfinite(msg.acceleration_disturbance.linear.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration_disturbance.linear.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration_disturbance.linear.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration_disturbance.linear.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration_disturbance.linear.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration_disturbance.linear.z'!!!", var_name.c_str());
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

std::optional<DetailedModelParams_t> loadDetailedUavModelParams(const rclcpp::Node::SharedPtr &node, const std::string &platform_config,
                                                                const std::string &custom_config) {

  mrs_lib::ParamLoader param_loader(node);

  if (custom_config != "") {
    param_loader.addYamlFile(custom_config);
  }

  if (platform_config != "") {
    param_loader.addYamlFile(platform_config);
  }

  param_loader.addYamlFileFromParam("detailed_uav_dynamics_params_config");

  double mass;
  double arm_length;
  double body_height;
  double force_constant;
  double torque_constant;
  double prop_radius;
  double rpm_min;
  double rpm_max;

  bool enabled = false;

  param_loader.loadParam("model_params/enabled", enabled);

  Eigen::MatrixXd allocation_matrix;

  bool inertia_enabled = false;

  if (!enabled) {
    RCLCPP_INFO(node->get_logger(), "detailed UAV model params were not provided");
    return {};
  }

  param_loader.loadParam("uav_mass", mass);

  param_loader.loadParam("model_params/arm_length", arm_length);
  param_loader.loadParam("model_params/body_height", body_height);

  param_loader.loadParam("model_params/propulsion/force_constant", force_constant);
  param_loader.loadParam("model_params/propulsion/torque_constant", torque_constant);
  param_loader.loadParam("model_params/propulsion/prop_radius", prop_radius);
  param_loader.loadParam("model_params/propulsion/rpm/min", rpm_min);
  param_loader.loadParam("model_params/propulsion/rpm/max", rpm_max);

  param_loader.loadParam("model_params/inertia_matrix/enabled", inertia_enabled);

  param_loader.loadMatrixDynamic("model_params/propulsion/allocation_matrix", allocation_matrix, Eigen::Matrix4d::Identity(), 4, -1);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_WARN(node->get_logger(),
                "detailed UAV model params not loaded, missing some parameters. This will not permit operations when ACTUATORS or CONTROL_GROUP control "
                "outputs would be possible.");
    return {};
  } else {
    RCLCPP_INFO(node->get_logger(), "detailed UAV model params successfully loaded.");
  }

  int n_motors = allocation_matrix.cols();

  DetailedModelParams_t model_params;

  model_params.arm_length  = arm_length;
  model_params.body_height = body_height;
  model_params.prop_radius = prop_radius;

  Eigen::Matrix3d inertia_matrix;

  if (inertia_enabled) {

    bool inertia_loaded = param_loader.loadMatrixStatic("model_params/inertia_matrix/matrix", inertia_matrix, Eigen::Matrix3d::Identity());

    if (inertia_loaded) {

      model_params.inertia = inertia_matrix;
      RCLCPP_INFO(node->get_logger(), "inertia matrix loaded from config file:");
      RCLCPP_INFO_STREAM(node->get_logger(), model_params.inertia);

    } else {

      RCLCPP_ERROR(node->get_logger(), "inertia matrix should be loaded from yaml, but was not loaded sucessfully!");
      rclcpp::shutdown();
      exit(1);
    }

  } else {

    RCLCPP_INFO(node->get_logger(), "computing inertia it from the UAV detailed model params");

    // create the inertia matrix
    model_params.inertia       = Eigen::Matrix3d::Zero();
    model_params.inertia(0, 0) = mass * (3.0 * arm_length * arm_length + body_height * body_height) / 12.0;
    model_params.inertia(1, 1) = mass * (3.0 * arm_length * arm_length + body_height * body_height) / 12.0;
    model_params.inertia(2, 2) = (mass * arm_length * arm_length) / 2.0;

    RCLCPP_INFO(node->get_logger(), "inertia computed form parameters:");
    RCLCPP_INFO_STREAM(node->get_logger(), model_params.inertia);
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

CONTROL_OUTPUT getLowestOuput(const ControlOutputModalities_t &outputs) {

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

CONTROL_OUTPUT getHighestOuput(const ControlOutputModalities_t &outputs) {

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

std::optional<double> extractThrottle(const Controller::ControlOutput &control_output) {

  if (!control_output.control_output) {
    return {};
  }

  return std::visit(HwApiCmdExtractThrottleVisitor(), control_output.control_output.value());
}

//}

// | -------------- validation of HW api commands ------------- |

/* validateControlOutput() //{ */

bool validateControlOutput(const rclcpp::Node::SharedPtr &node, const Controller::ControlOutput &control_output,
                           const ControlOutputModalities_t &output_modalities, const std::string &var_name) {

  if (!control_output.control_output) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "the optional variable '%s' is not set!!!", var_name.c_str());
    return false;
  }

  std::variant<ControlOutputModalities_t> output_modalities_var{output_modalities};
  std::variant<std::string>               var_name_var{var_name};
  std::variant<rclcpp::Node::SharedPtr>   node_var{node};

  return std::visit(HwApiValidateVisitor(), node_var, control_output.control_output.value(), output_modalities_var, var_name_var);
}

//}

/* validateHwApiActuatorCmd() //{ */

bool validateHwApiActuatorCmd(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::HwApiActuatorCmd &msg, const std::string &var_name) {

  for (size_t i = 0; i < msg.motors.size(); i++) {
    if (!std::isfinite(msg.motors[i])) {
      RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.motors[%d]'!!!", var_name.c_str(), int(i));
      return false;
    }
  }

  return true;
}

//}

/* validateHwApiControlGroupCmd() //{ */

bool validateHwApiControlGroupCmd(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::HwApiControlGroupCmd &msg, const std::string &var_name) {

  if (!std::isfinite(msg.roll)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.roll'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.pitch)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.pitch'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.yaw)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.yaw'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.throttle)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.throttle'!!!", var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateHwApiAttitudeCmd() //{ */

bool validateHwApiAttitudeCmd(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::HwApiAttitudeCmd &msg, const std::string &var_name) {

  // check the orientation

  if (!std::isfinite(msg.orientation.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.orientation.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.orientation.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.orientation.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.orientation.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.orientation.z'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.orientation.w)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.orientation.w'!!!", var_name.c_str());
    return false;
  }

  // check the throttle

  if (!std::isfinite(msg.throttle)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.throttle'!!!", var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateHwApiAttitudeRateCmd() //{ */

bool validateHwApiAttitudeRateCmd(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::HwApiAttitudeRateCmd &msg, const std::string &var_name) {

  // check the body rate

  if (!std::isfinite(msg.body_rate.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.body_rate.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.body_rate.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.body_rate.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.body_rate.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.body_rate.z'!!!", var_name.c_str());
    return false;
  }

  // check the throttle

  if (!std::isfinite(msg.throttle)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.throttle'!!!", var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateHwApiAccelerationHdgRateCmd() //{ */

bool validateHwApiAccelerationHdgRateCmd(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::HwApiAccelerationHdgRateCmd &msg,
                                         const std::string &var_name) {

  // | ----------------- check the acceleration ----------------- |

  if (!std::isfinite(msg.acceleration.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration.z'!!!", var_name.c_str());
    return false;
  }

  // check the heading rate

  if (!std::isfinite(msg.heading_rate)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.heading_rate'!!!", var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateHwApiAccelerationHdgCmd() //{ */

bool validateHwApiAccelerationHdgCmd(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::HwApiAccelerationHdgCmd &msg, const std::string &var_name) {

  // | ----------------- check the acceleration ----------------- |

  if (!std::isfinite(msg.acceleration.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.acceleration.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.acceleration.z'!!!", var_name.c_str());
    return false;
  }

  // check the heading

  if (!std::isfinite(msg.heading)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.heading'!!!", var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateHwApiVelocityHdgRateCmd() //{ */

bool validateHwApiVelocityHdgRateCmd(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::HwApiVelocityHdgRateCmd &msg, const std::string &var_name) {

  // | ----------------- check the velocity ----------------- |

  if (!std::isfinite(msg.velocity.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.velocity.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.velocity.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.velocity.z'!!!", var_name.c_str());
    return false;
  }

  // check the heading rate

  if (!std::isfinite(msg.heading_rate)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.heading_rate'!!!", var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateHwApiVelocityHdgCmd() //{ */

bool validateHwApiVelocityHdgCmd(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::HwApiVelocityHdgCmd &msg, const std::string &var_name) {

  // | ----------------- check the velocity ----------------- |

  if (!std::isfinite(msg.velocity.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.velocity.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.velocity.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.velocity.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.velocity.z'!!!", var_name.c_str());
    return false;
  }

  // check the heading

  if (!std::isfinite(msg.heading)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.heading'!!!", var_name.c_str());
    return false;
  }

  return true;
}

//}

/* validateHwApiPositionHdgCmd() //{ */

bool validateHwApiPositionCmd(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::HwApiPositionCmd &msg, const std::string &var_name) {

  // | ----------------- check the position ----------------- |

  if (!std::isfinite(msg.position.x)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.position.x'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.position.y)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.position.y'!!!", var_name.c_str());
    return false;
  }

  if (!std::isfinite(msg.position.z)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.position.z'!!!", var_name.c_str());
    return false;
  }

  // check the heading

  if (!std::isfinite(msg.heading)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "NaN detected in variable '%s.heading'!!!", var_name.c_str());
    return false;
  }

  return true;
}

//}

// | ------------ initialization of HW api commands ----------- |

/* initializeDefaultOutput() //{ */

Controller::HwApiOutputVariant initializeDefaultOutput(const rclcpp::Node::SharedPtr &node, const ControlOutputModalities_t &possible_outputs,
                                                       const mrs_msgs::msg::UavState &uav_state, const double &min_throttle, const double &n_motors) {

  CONTROL_OUTPUT lowest_output = getLowestOuput(possible_outputs);

  Controller::HwApiOutputVariant output;

  switch (lowest_output) {
  case ACTUATORS_CMD: {
    output = mrs_msgs::msg::HwApiActuatorCmd();
    break;
  }
  case CONTROL_GROUP: {
    output = mrs_msgs::msg::HwApiControlGroupCmd();
    break;
  }
  case ATTITUDE_RATE: {
    output = mrs_msgs::msg::HwApiAttitudeRateCmd();
    break;
  }
  case ATTITUDE: {
    output = mrs_msgs::msg::HwApiAttitudeCmd();
    break;
  }
  case ACCELERATION_HDG_RATE: {
    output = mrs_msgs::msg::HwApiAccelerationHdgRateCmd();
    break;
  }
  case ACCELERATION_HDG: {
    output = mrs_msgs::msg::HwApiAccelerationHdgCmd();
    break;
  }
  case VELOCITY_HDG_RATE: {
    output = mrs_msgs::msg::HwApiVelocityHdgRateCmd();
    break;
  }
  case VELOCITY_HDG: {
    output = mrs_msgs::msg::HwApiVelocityHdgCmd();
    break;
  }
  case POSITION: {
    output = mrs_msgs::msg::HwApiPositionCmd();
    break;
  }
  }

  std::variant<rclcpp::Node::SharedPtr> node_var{node};
  std::variant<mrs_msgs::msg::UavState> uav_state_var{uav_state};
  std::variant<double>                  min_throttle_var{min_throttle};
  std::variant<double>                  n_motors_var{n_motors};

  std::visit(HwApiInitializeVisitor(), node_var, output, uav_state_var, min_throttle_var, n_motors_var);

  return output;
}

//}

/* initializeHwApiCmd(mrs_msgs::msg::HwApiActuatorCmd& msg) //{ */

void initializeHwApiCmd(const rclcpp::Node::SharedPtr &node, mrs_msgs::msg::HwApiActuatorCmd &msg, const double &min_throttle, const double &n_motors) {

  msg.stamp = node->get_clock()->now();

  for (int i = 0; i < n_motors; i++) {
    msg.motors.push_back(min_throttle);
  }
}

//}

/* initializeHwApiCmd(mrs_msgs::msg::HwApiControlGroupCmd& msg) //{ */

void initializeHwApiCmd(const rclcpp::Node::SharedPtr &node, mrs_msgs::msg::HwApiControlGroupCmd &msg, const double &min_throttle) {

  msg.stamp = node->get_clock()->now();

  msg.roll     = 0;
  msg.pitch    = 0;
  msg.yaw      = 0;
  msg.throttle = min_throttle;
}

//}

/* initializeHwApiCmd(mrs_msgs::msg::HwApiAttitudeRateCmd& msg) //{ */

void initializeHwApiCmd(const rclcpp::Node::SharedPtr &node, mrs_msgs::msg::HwApiAttitudeRateCmd &msg, const double &min_throttle) {

  msg.stamp = node->get_clock()->now();

  msg.body_rate.x = 0;
  msg.body_rate.y = 0;
  msg.body_rate.z = 0;
  msg.throttle    = min_throttle;
}

//}

/* initializeHwApiCmd(mrs_msgs::msg::HwApiAttitudeCmd& msg) //{ */

void initializeHwApiCmd(const rclcpp::Node::SharedPtr &node, mrs_msgs::msg::HwApiAttitudeCmd &msg, const mrs_msgs::msg::UavState &uav_state,
                        const double &min_throttle) {

  msg.stamp = node->get_clock()->now();

  msg.orientation = uav_state.pose.orientation;
  msg.throttle    = min_throttle;
}

//}

/* initializeHwApiCmd(mrs_msgs::msg::HwApiAccelerationHdgRateCmd& msg) //{ */

void initializeHwApiCmd(const rclcpp::Node::SharedPtr &node, mrs_msgs::msg::HwApiAccelerationHdgRateCmd &msg, const mrs_msgs::msg::UavState &uav_state) {

  msg.header.frame_id = uav_state.header.frame_id;
  msg.header.stamp    = node->get_clock()->now();

  msg.acceleration.x = 0;
  msg.acceleration.y = 0;
  msg.acceleration.z = 0;
  msg.heading_rate   = 0;
}

//}

/* initializeHwApiCmd(mrs_msgs::msg::HwApiAccelerationHdgCmd& msg) //{ */

void initializeHwApiCmd(const rclcpp::Node::SharedPtr &node, mrs_msgs::msg::HwApiAccelerationHdgCmd &msg, const mrs_msgs::msg::UavState &uav_state) {

  msg.header.frame_id = uav_state.header.frame_id;
  msg.header.stamp    = node->get_clock()->now();

  msg.acceleration.x = 0;
  msg.acceleration.y = 0;
  msg.acceleration.z = 0;

  try {
    msg.heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (std::runtime_error &exrun) {
    msg.heading = 0;
  }
}

//}

/* initializeHwApiCmd(mrs_msgs::msg::HwApiVelocityHdgRateCmd& msg) //{ */

void initializeHwApiCmd(const rclcpp::Node::SharedPtr &node, mrs_msgs::msg::HwApiVelocityHdgRateCmd &msg, const mrs_msgs::msg::UavState &uav_state) {

  msg.header.frame_id = uav_state.header.frame_id;
  msg.header.stamp    = node->get_clock()->now();

  msg.velocity.x   = 0;
  msg.velocity.y   = 0;
  msg.velocity.z   = 0;
  msg.heading_rate = 0;
}

//}

/* initializeHwApiCmd(mrs_msgs::msg::HwApiVelocityHdgCmd& msg) //{ */

void initializeHwApiCmd(const rclcpp::Node::SharedPtr &node, mrs_msgs::msg::HwApiVelocityHdgCmd &msg, const mrs_msgs::msg::UavState &uav_state) {

  msg.header.frame_id = uav_state.header.frame_id;
  msg.header.stamp    = node->get_clock()->now();

  msg.velocity.x = 0;
  msg.velocity.y = 0;
  msg.velocity.z = 0;

  try {
    msg.heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (std::runtime_error &exrun) {
    msg.heading = 0;
  }
}

//}

/* initializeHwApiCmd(mrs_msgs::msg::HwApiPositionCmd& msg) //{ */

void initializeHwApiCmd(const rclcpp::Node::SharedPtr &node, mrs_msgs::msg::HwApiPositionCmd &msg, const mrs_msgs::msg::UavState &uav_state) {

  msg.header.frame_id = uav_state.header.frame_id;
  msg.header.stamp    = node->get_clock()->now();

  msg.position.x = uav_state.pose.position.x;
  msg.position.y = uav_state.pose.position.y;
  msg.position.z = uav_state.pose.position.z;

  try {
    msg.heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (std::runtime_error &exrun) {
    msg.heading = 0;
  }
}

//}

} // namespace control_manager

} // namespace mrs_uav_managers
