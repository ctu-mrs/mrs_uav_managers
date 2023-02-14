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

  OutputPublisher(ros::NodeHandle& nh);

  void publish(const Controller::HwApiOutputVariant& control_output);

private:
  mrs_lib::PublisherHandler<mrs_msgs::HwApiActuatorCmd>            ph_hw_api_actuator_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiControlGroupCmd>        ph_hw_api_control_group_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiAttitudeRateCmd>        ph_hw_api_attitude_rate_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiAttitudeCmd>            ph_hw_api_attitude_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiAccelerationHdgRateCmd> ph_hw_api_acceleration_hdg_rate_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiAccelerationHdgCmd>     ph_hw_api_acceleration_hdg_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgRateCmd>     ph_hw_api_velocity_hdg_rate_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgCmd>         ph_hw_api_velocity_hdg_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiPositionCmd>            ph_hw_api_position_cmd_;

  void publish(const mrs_msgs::HwApiActuatorCmd& msg);
  void publish(const mrs_msgs::HwApiControlGroupCmd& msg);
  void publish(const mrs_msgs::HwApiAttitudeRateCmd& msg);
  void publish(const mrs_msgs::HwApiAttitudeCmd& msg);
  void publish(const mrs_msgs::HwApiAccelerationHdgRateCmd& msg);
  void publish(const mrs_msgs::HwApiAccelerationHdgCmd& msg);
  void publish(const mrs_msgs::HwApiVelocityHdgRateCmd& msg);
  void publish(const mrs_msgs::HwApiVelocityHdgCmd& msg);
  void publish(const mrs_msgs::HwApiPositionCmd& msg);

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
