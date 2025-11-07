#include <control_manager/output_publisher.h>

namespace mrs_uav_managers
{

namespace control_manager
{

OutputPublisher::OutputPublisher() {
}

OutputPublisher::OutputPublisher(const rclcpp::Node::SharedPtr &node) {

  ph_hw_api_actuator_cmd_              = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiActuatorCmd>(node, "~/hw_api_actuator_cmd_out");
  ph_hw_api_control_group_cmd_         = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiControlGroupCmd>(node, "~/hw_api_control_group_cmd_out");
  ph_hw_api_attitude_rate_cmd_         = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAttitudeRateCmd>(node, "~/hw_api_attitude_rate_cmd_out");
  ph_hw_api_attitude_cmd_              = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAttitudeCmd>(node, "~/hw_api_attitude_cmd_out");
  ph_hw_api_acceleration_hdg_rate_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAccelerationHdgRateCmd>(node, "~/hw_api_acceleration_hdg_rate_cmd_out");
  ph_hw_api_acceleration_hdg_cmd_      = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAccelerationHdgCmd>(node, "~/hw_api_acceleration_hdg_cmd_out");
  ph_hw_api_velocity_hdg_rate_cmd_     = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiVelocityHdgRateCmd>(node, "~/hw_api_velocity_hdg_rate_cmd_out");
  ph_hw_api_velocity_hdg_cmd_          = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiVelocityHdgCmd>(node, "~/hw_api_velocity_hdg_cmd_out");
  ph_hw_api_position_cmd_              = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiPositionCmd>(node, "~/hw_api_position_cmd_out");
}

void OutputPublisher::publish(const Controller::HwApiOutputVariant &control_output) {

  std::visit(OutputPublisher::PublisherVisitor(this), control_output);
}

// | ------------------------- private ------------------------ |

void OutputPublisher::publish(const mrs_msgs::msg::HwApiActuatorCmd &msg) {
  ph_hw_api_actuator_cmd_.publish(msg);
}

void OutputPublisher::publish(const mrs_msgs::msg::HwApiControlGroupCmd &msg) {
  ph_hw_api_control_group_cmd_.publish(msg);
}

void OutputPublisher::publish(const mrs_msgs::msg::HwApiAttitudeRateCmd &msg) {
  ph_hw_api_attitude_rate_cmd_.publish(msg);
}

void OutputPublisher::publish(const mrs_msgs::msg::HwApiAttitudeCmd &msg) {
  ph_hw_api_attitude_cmd_.publish(msg);
}

void OutputPublisher::publish(const mrs_msgs::msg::HwApiAccelerationHdgRateCmd &msg) {
  ph_hw_api_acceleration_hdg_rate_cmd_.publish(msg);
}

void OutputPublisher::publish(const mrs_msgs::msg::HwApiAccelerationHdgCmd &msg) {
  ph_hw_api_acceleration_hdg_cmd_.publish(msg);
}

void OutputPublisher::publish(const mrs_msgs::msg::HwApiVelocityHdgRateCmd &msg) {
  ph_hw_api_velocity_hdg_rate_cmd_.publish(msg);
}

void OutputPublisher::publish(const mrs_msgs::msg::HwApiVelocityHdgCmd &msg) {
  ph_hw_api_velocity_hdg_cmd_.publish(msg);
}

void OutputPublisher::publish(const mrs_msgs::msg::HwApiPositionCmd &msg) {
  ph_hw_api_position_cmd_.publish(msg);
}

} // namespace control_manager

} // namespace mrs_uav_managers
