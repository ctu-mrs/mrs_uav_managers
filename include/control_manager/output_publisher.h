#ifndef CONTROL_MANAGER_OUTPUT_PUBLISHER
#define CONTROL_MANAGER_OUTPUT_PUBLISHER

#include <mrs_lib/publisher_handler.h>

#include <mrs_uav_managers/controller.h>

namespace mrs_uav_managers
{

namespace control_manager
{

class OutputPublisher {

public:
  OutputPublisher();

  OutputPublisher(const rclcpp::Node::SharedPtr& node);

  void publish(const Controller::HwApiOutputVariant& control_output);

private:
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiActuatorCmd>            ph_hw_api_actuator_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiControlGroupCmd>        ph_hw_api_control_group_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAttitudeRateCmd>        ph_hw_api_attitude_rate_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAttitudeCmd>            ph_hw_api_attitude_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAccelerationHdgRateCmd> ph_hw_api_acceleration_hdg_rate_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAccelerationHdgCmd>     ph_hw_api_acceleration_hdg_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiVelocityHdgRateCmd>     ph_hw_api_velocity_hdg_rate_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiVelocityHdgCmd>         ph_hw_api_velocity_hdg_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiPositionCmd>            ph_hw_api_position_cmd_;

  void publish(const mrs_msgs::msg::HwApiActuatorCmd& msg);
  void publish(const mrs_msgs::msg::HwApiControlGroupCmd& msg);
  void publish(const mrs_msgs::msg::HwApiAttitudeRateCmd& msg);
  void publish(const mrs_msgs::msg::HwApiAttitudeCmd& msg);
  void publish(const mrs_msgs::msg::HwApiAccelerationHdgRateCmd& msg);
  void publish(const mrs_msgs::msg::HwApiAccelerationHdgCmd& msg);
  void publish(const mrs_msgs::msg::HwApiVelocityHdgRateCmd& msg);
  void publish(const mrs_msgs::msg::HwApiVelocityHdgCmd& msg);
  void publish(const mrs_msgs::msg::HwApiPositionCmd& msg);

  class PublisherVisitor {

  public:
    PublisherVisitor(OutputPublisher* obj) : obj_(obj){};

    OutputPublisher* obj_;

    template <class T>
    void operator()(const T& msg) {
      obj_->publish(msg);
    }
  };
};

}  // namespace control_manager

}  // namespace mrs_uav_managers

#endif  //  CONTROL_MANAGER_OUTPUT_PUBLISHER
