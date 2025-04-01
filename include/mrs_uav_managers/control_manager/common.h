#ifndef CONTROL_MANAGER_COMMON_H
#define CONTROL_MANAGER_COMMON_H

#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <string>
#include <optional>

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/param_loader.h>

#include <mrs_uav_managers/control_manager/common_handlers.h>
#include <mrs_uav_managers/controller.h>

#include <mrs_msgs/msg/tracker_command.hpp>
#include <mrs_msgs/msg/uav_state.hpp>
#include <mrs_msgs/msg/velocity_reference.hpp>

#include <mrs_msgs/msg/hw_api_actuator_cmd.hpp>
#include <mrs_msgs/msg/hw_api_control_group_cmd.hpp>
#include <mrs_msgs/msg/hw_api_attitude_rate_cmd.hpp>
#include <mrs_msgs/msg/hw_api_attitude_cmd.hpp>
#include <mrs_msgs/msg/hw_api_acceleration_hdg_rate_cmd.hpp>
#include <mrs_msgs/msg/hw_api_acceleration_hdg_cmd.hpp>
#include <mrs_msgs/msg/hw_api_velocity_hdg_rate_cmd.hpp>
#include <mrs_msgs/msg/hw_api_velocity_hdg_cmd.hpp>
#include <mrs_msgs/msg/hw_api_position_cmd.hpp>

#include <mrs_msgs/msg/hw_api_capabilities.hpp>

#include <nav_msgs/msg/odometry.hpp>

namespace mrs_uav_managers
{

namespace control_manager
{

enum CONTROL_OUTPUT
{
  ACTUATORS_CMD,
  CONTROL_GROUP,
  ATTITUDE_RATE,
  ATTITUDE,
  ACCELERATION_HDG_RATE,
  ACCELERATION_HDG,
  VELOCITY_HDG_RATE,
  VELOCITY_HDG,
  POSITION
};

CONTROL_OUTPUT getLowestOuput(const ControlOutputModalities_t& outputs);

CONTROL_OUTPUT getHighestOuput(const ControlOutputModalities_t& outputs);

std::optional<unsigned int> idxInVector(const std::string& str, const std::vector<std::string>& vec);

// checks for invalid values in the result from trackers
bool validateTrackerCommand(const rclcpp::Node::SharedPtr& node, const std::optional<mrs_msgs::msg::TrackerCommand>& msg, const std::string& var_name);

// checks for invalid messages in/out
bool validateOdometry(const rclcpp::Node::SharedPtr& node, const nav_msgs::msg::Odometry& msg, const std::string& var_name);
bool validateUavState(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::UavState& msg, const std::string& var_name);
bool validateVelocityReference(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::VelocityReference& msg, const std::string& var_name);
bool validateReference(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::Reference& msg, const std::string& var_name);

std::optional<DetailedModelParams_t> loadDetailedUavModelParams(const rclcpp::Node::SharedPtr& node, const std::string& platform_config, const std::string& custom_config);

// translates the channel values to desired range
double RCChannelToRange(double rc_value, double range, double deadband);

/* throttle extraction //{ */

std::optional<double> extractThrottle(const Controller::ControlOutput& control_output);

struct HwApiCmdExtractThrottleVisitor
{
  std::optional<double> operator()(const mrs_msgs::msg::HwApiActuatorCmd& msg) {

    if (msg.motors.size() == 0) {
      return std::nullopt;
    }

    double throttle = 0;

    for (size_t i = 0; i < msg.motors.size(); i++) {
      throttle += msg.motors.at(i);
    };

    throttle /= msg.motors.size();

    return throttle;
  }
  std::optional<double> operator()(const mrs_msgs::msg::HwApiControlGroupCmd& msg) {
    return msg.throttle;
  }
  std::optional<double> operator()(const mrs_msgs::msg::HwApiAttitudeCmd& msg) {
    return msg.throttle;
  }
  std::optional<double> operator()(const mrs_msgs::msg::HwApiAttitudeRateCmd& msg) {
    return msg.throttle;
  }
  std::optional<double> operator()([[maybe_unused]] const mrs_msgs::msg::HwApiAccelerationHdgRateCmd& msg) {
    return std::nullopt;
  }
  std::optional<double> operator()([[maybe_unused]] const mrs_msgs::msg::HwApiAccelerationHdgCmd& msg) {
    return std::nullopt;
  }
  std::optional<double> operator()([[maybe_unused]] const mrs_msgs::msg::HwApiVelocityHdgRateCmd& msg) {
    return std::nullopt;
  }
  std::optional<double> operator()([[maybe_unused]] const mrs_msgs::msg::HwApiVelocityHdgCmd& msg) {
    return std::nullopt;
  }
  std::optional<double> operator()([[maybe_unused]] const mrs_msgs::msg::HwApiPositionCmd& msg) {
    return std::nullopt;
  }
};

//}

/* control output validation //{ */

bool validateControlOutput(const rclcpp::Node::SharedPtr& node, const Controller::ControlOutput& control_output, const ControlOutputModalities_t& output_modalities, const std::string& var_name);

// validation of hw api messages
bool validateHwApiActuatorCmd(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiActuatorCmd& msg, const std::string& var_name);
bool validateHwApiControlGroupCmd(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiControlGroupCmd& msg, const std::string& var_name);
bool validateHwApiAttitudeCmd(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiAttitudeCmd& msg, const std::string& var_name);
bool validateHwApiAttitudeRateCmd(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiAttitudeRateCmd& msg, const std::string& var_name);
bool validateHwApiAccelerationHdgRateCmd(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiAccelerationHdgRateCmd& msg, const std::string& var_name);
bool validateHwApiAccelerationHdgCmd(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiAccelerationHdgCmd& msg, const std::string& var_name);
bool validateHwApiVelocityHdgRateCmd(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiVelocityHdgRateCmd& msg, const std::string& var_name);
bool validateHwApiVelocityHdgCmd(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiVelocityHdgCmd& msg, const std::string& var_name);
bool validateHwApiPositionCmd(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiPositionCmd& msg, const std::string& var_name);

struct HwApiValidateVisitor
{
  bool operator()(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiActuatorCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& var_name) {

    if (!output_modalities.actuators) {
      RCLCPP_ERROR(node->get_logger(), "The controller returned an output modality (actuator cmd) that is not supported by the hardware API");
      return false;
    }

    return validateHwApiActuatorCmd(node, msg, var_name);
  }

  bool operator()(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiControlGroupCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& var_name) {

    if (!output_modalities.control_group) {
      RCLCPP_ERROR(node->get_logger(), "The controller returned an output modality (control group cmd) that is not supported by the hardware API");
      return false;
    }

    return validateHwApiControlGroupCmd(node, msg, var_name);
  }

  bool operator()(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiAttitudeCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& var_name) {

    if (!output_modalities.attitude) {
      RCLCPP_ERROR(node->get_logger(), "The controller returned an output modality (attitude cmd) that is not supported by the hardware API");
      return false;
    }

    return validateHwApiAttitudeCmd(node, msg, var_name);
  }

  bool operator()(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiAttitudeRateCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& var_name) {

    if (!output_modalities.attitude_rate) {
      RCLCPP_ERROR(node->get_logger(), "The controller returned an output modality (attitude rate cmd) that is not supported by the hardware API");
      return false;
    }

    return validateHwApiAttitudeRateCmd(node, msg, var_name);
  }

  bool operator()(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiAccelerationHdgRateCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& var_name) {

    if (!output_modalities.acceleration_hdg_rate) {
      RCLCPP_ERROR(node->get_logger(), "The controller returned an output modality (acceleration+hdg rate cmd) that is not supported by the hardware API");
      return false;
    }

    return validateHwApiAccelerationHdgRateCmd(node, msg, var_name);
  }

  bool operator()(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiAccelerationHdgCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& var_name) {

    if (!output_modalities.acceleration_hdg) {
      RCLCPP_ERROR(node->get_logger(), "The controller returned an output modality (acceleration+hdg cmd) that is not supported by the hardware API");
      return false;
    }

    return validateHwApiAccelerationHdgCmd(node, msg, var_name);
  }

  bool operator()(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiVelocityHdgRateCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& var_name) {

    if (!output_modalities.velocity_hdg_rate) {
      RCLCPP_ERROR(node->get_logger(), "The controller returned an output modality (velocity+hdg rate cmd) that is not supported by the hardware API");
      return false;
    }

    return validateHwApiVelocityHdgRateCmd(node, msg, var_name);
  }

  bool operator()(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiVelocityHdgCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& var_name) {

    if (!output_modalities.velocity_hdg) {
      RCLCPP_ERROR(node->get_logger(), "The controller returned an output modality (velocity+hdg cmd) that is not supported by the hardware API");
      return false;
    }

    return validateHwApiVelocityHdgCmd(node, msg, var_name);
  }

  bool operator()(const rclcpp::Node::SharedPtr& node, const mrs_msgs::msg::HwApiPositionCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& var_name) {

    if (!output_modalities.position) {
      RCLCPP_ERROR(node->get_logger(), "The controller returned an output modality (position cmd) that is not supported by the hardware API");
      return false;
    }

    return validateHwApiPositionCmd(node, msg, var_name);
  }
};

//}

/* control output initialization //{ */

Controller::HwApiOutputVariant initializeDefaultOutput(const rclcpp::Node::SharedPtr& node, const ControlOutputModalities_t& possible_outputs, const mrs_msgs::msg::UavState& uav_state, const double& min_throttle, const double& n_motors);

void initializeHwApiCmd(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiActuatorCmd& msg, const double& min_throttle, const double& n_motors);
void initializeHwApiCmd(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiControlGroupCmd& msg, const double& min_throttle);
void initializeHwApiCmd(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiAttitudeRateCmd& msg, const double& min_throttle);
void initializeHwApiCmd(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiAttitudeCmd& msg, const mrs_msgs::msg::UavState& uav_state, const double& min_throttle);
void initializeHwApiCmd(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiAccelerationHdgRateCmd& msg, const mrs_msgs::msg::UavState& uav_state);
void initializeHwApiCmd(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiAccelerationHdgCmd& msg, const mrs_msgs::msg::UavState& uav_state);
void initializeHwApiCmd(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiVelocityHdgRateCmd& msg, const mrs_msgs::msg::UavState& uav_state);
void initializeHwApiCmd(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiVelocityHdgCmd& msg, const mrs_msgs::msg::UavState& uav_state);
void initializeHwApiCmd(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiPositionCmd& msg, const mrs_msgs::msg::UavState& uav_state);

struct HwApiInitializeVisitor
{
  void operator()(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiActuatorCmd& msg, [[maybe_unused]] const mrs_msgs::msg::UavState& uav_state, const double& min_throttle, const double& n_motors) {
    initializeHwApiCmd(node, msg, min_throttle, n_motors);
  }
  void operator()(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiControlGroupCmd& msg, [[maybe_unused]] const mrs_msgs::msg::UavState& uav_state, const double& min_throttle, [[maybe_unused]] const double& n_motors) {
    initializeHwApiCmd(node, msg, min_throttle);
  }
  void operator()(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiAttitudeCmd& msg, const mrs_msgs::msg::UavState& uav_state, const double& min_throttle, [[maybe_unused]] const double& n_motors) {
    initializeHwApiCmd(node, msg, uav_state, min_throttle);
  }
  void operator()(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiAttitudeRateCmd& msg, [[maybe_unused]] const mrs_msgs::msg::UavState& uav_state, const double& min_throttle, [[maybe_unused]] const double& n_motors) {
    initializeHwApiCmd(node, msg, min_throttle);
  }
  void operator()(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiAccelerationHdgRateCmd& msg, const mrs_msgs::msg::UavState& uav_state, [[maybe_unused]] const double& min_throttle, [[maybe_unused]] const double& n_motors) {
    initializeHwApiCmd(node, msg, uav_state);
  }
  void operator()(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiAccelerationHdgCmd& msg, const mrs_msgs::msg::UavState& uav_state, [[maybe_unused]] const double& min_throttle, [[maybe_unused]] const double& n_motors) {
    initializeHwApiCmd(node, msg, uav_state);
  }
  void operator()(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiVelocityHdgRateCmd& msg, const mrs_msgs::msg::UavState& uav_state, [[maybe_unused]] const double& min_throttle, [[maybe_unused]] const double& n_motors) {
    initializeHwApiCmd(node, msg, uav_state);
  }
  void operator()(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiVelocityHdgCmd& msg, const mrs_msgs::msg::UavState& uav_state, [[maybe_unused]] const double& min_throttle, [[maybe_unused]] const double& n_motors) {
    initializeHwApiCmd(node, msg, uav_state);
  }
  void operator()(const rclcpp::Node::SharedPtr& node, mrs_msgs::msg::HwApiPositionCmd& msg, const mrs_msgs::msg::UavState& uav_state, [[maybe_unused]] const double& min_throttle, [[maybe_unused]] const double& n_motors) {
    initializeHwApiCmd(node, msg, uav_state);
  }
};

//}

}  // namespace control_manager

}  // namespace mrs_uav_managers

#endif  // CONTROL_MANAGER_COMMON_H
