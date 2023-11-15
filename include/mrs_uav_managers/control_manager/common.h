#ifndef CONTROL_MANAGER_COMMON_H
#define CONTROL_MANAGER_COMMON_H

#include <ros/ros.h>

#include <vector>
#include <string>
#include <optional>
#include <variant>

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/param_loader.h>

#include <mrs_uav_managers/control_manager/common_handlers.h>
#include <mrs_uav_managers/controller.h>

#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/VelocityReference.h>

#include <mrs_msgs/HwApiActuatorCmd.h>
#include <mrs_msgs/HwApiControlGroupCmd.h>
#include <mrs_msgs/HwApiAttitudeRateCmd.h>
#include <mrs_msgs/HwApiAttitudeCmd.h>
#include <mrs_msgs/HwApiAccelerationHdgRateCmd.h>
#include <mrs_msgs/HwApiAccelerationHdgCmd.h>
#include <mrs_msgs/HwApiVelocityHdgRateCmd.h>
#include <mrs_msgs/HwApiVelocityHdgCmd.h>
#include <mrs_msgs/HwApiPositionCmd.h>

#include <mrs_msgs/HwApiCapabilities.h>

#include <nav_msgs/Odometry.h>

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
bool validateTrackerCommand(const std::optional<mrs_msgs::TrackerCommand>& msg, const std::string& node_name, const std::string& var_name);

// checks for invalid messages in/out
bool validateOdometry(const nav_msgs::Odometry& msg, const std::string& node_name, const std::string& var_name);
bool validateUavState(const mrs_msgs::UavState& msg, const std::string& node_name, const std::string& var_nam);
bool validateVelocityReference(const mrs_msgs::VelocityReference& msg, const std::string& node_name, const std::string& var_name);
bool validateReference(const mrs_msgs::Reference& msg, const std::string& node_name, const std::string& var_name);

std::optional<DetailedModelParams_t> loadDetailedUavModelParams(ros::NodeHandle& nh, const std::string& node_name, const std::string& platform_config,
                                                                const std::string& custom_config);

// translates the channel values to desired range
double RCChannelToRange(double rc_value, double range, double deadband);

/* throttle extraction //{ */

std::optional<double> extractThrottle(const Controller::ControlOutput& control_output);

struct HwApiCmdExtractThrottleVisitor
{
  std::optional<double> operator()(const mrs_msgs::HwApiActuatorCmd& msg) {

    if (msg.motors.size() == 0) {
      return std::nullopt;
    }

    double throttle = 0;

    for (size_t i = 0; i < msg.motors.size(); i++) {
      throttle += msg.motors[i];
    };

    throttle /= msg.motors.size();

    return throttle;
  }
  std::optional<double> operator()(const mrs_msgs::HwApiControlGroupCmd& msg) {
    return msg.throttle;
  }
  std::optional<double> operator()(const mrs_msgs::HwApiAttitudeCmd& msg) {
    return msg.throttle;
  }
  std::optional<double> operator()(const mrs_msgs::HwApiAttitudeRateCmd& msg) {
    return msg.throttle;
  }
  std::optional<double> operator()([[maybe_unused]] const mrs_msgs::HwApiAccelerationHdgRateCmd& msg) {
    return std::nullopt;
  }
  std::optional<double> operator()([[maybe_unused]] const mrs_msgs::HwApiAccelerationHdgCmd& msg) {
    return std::nullopt;
  }
  std::optional<double> operator()([[maybe_unused]] const mrs_msgs::HwApiVelocityHdgRateCmd& msg) {
    return std::nullopt;
  }
  std::optional<double> operator()([[maybe_unused]] const mrs_msgs::HwApiVelocityHdgCmd& msg) {
    return std::nullopt;
  }
  std::optional<double> operator()([[maybe_unused]] const mrs_msgs::HwApiPositionCmd& msg) {
    return std::nullopt;
  }
};

//}

/* control output validation //{ */

bool validateControlOutput(const Controller::ControlOutput& control_output, const ControlOutputModalities_t& output_modalities, const std::string& node_name,
                           const std::string& var_name);

// validation of hw api messages
bool validateHwApiActuatorCmd(const mrs_msgs::HwApiActuatorCmd& msg, const std::string& node_name, const std::string& var_name);
bool validateHwApiControlGroupCmd(const mrs_msgs::HwApiControlGroupCmd& msg, const std::string& node_name, const std::string& var_name);
bool validateHwApiAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd& msg, const std::string& node_name, const std::string& var_name);
bool validateHwApiAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd& msg, const std::string& node_name, const std::string& var_name);
bool validateHwApiAccelerationHdgRateCmd(const mrs_msgs::HwApiAccelerationHdgRateCmd& msg, const std::string& node_name, const std::string& var_name);
bool validateHwApiAccelerationHdgCmd(const mrs_msgs::HwApiAccelerationHdgCmd& msg, const std::string& node_name, const std::string& var_name);
bool validateHwApiVelocityHdgRateCmd(const mrs_msgs::HwApiVelocityHdgRateCmd& msg, const std::string& node_name, const std::string& var_name);
bool validateHwApiVelocityHdgCmd(const mrs_msgs::HwApiVelocityHdgCmd& msg, const std::string& node_name, const std::string& var_name);
bool validateHwApiPositionCmd(const mrs_msgs::HwApiPositionCmd& msg, const std::string& node_name, const std::string& var_name);

struct HwApiValidateVisitor
{
  bool operator()(const mrs_msgs::HwApiActuatorCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& node_name,
                  const std::string& var_name) {

    if (!output_modalities.actuators) {
      ROS_ERROR("[%s]: The controller returned an output modality (actuator cmd) that is not supported by the hardware API", node_name.c_str());
      return false;
    }

    return validateHwApiActuatorCmd(msg, node_name, var_name);
  }
  bool operator()(const mrs_msgs::HwApiControlGroupCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& node_name,
                  const std::string& var_name) {

    if (!output_modalities.control_group) {
      ROS_ERROR("[%s]: The controller returned an output modality (control group cmd) that is not supported by the hardware API", node_name.c_str());
      return false;
    }

    return validateHwApiControlGroupCmd(msg, node_name, var_name);
  }
  bool operator()(const mrs_msgs::HwApiAttitudeCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& node_name,
                  const std::string& var_name) {

    if (!output_modalities.attitude) {
      ROS_ERROR("[%s]: The controller returned an output modality (attitude cmd) that is not supported by the hardware API", node_name.c_str());
      return false;
    }

    return validateHwApiAttitudeCmd(msg, node_name, var_name);
  }
  bool operator()(const mrs_msgs::HwApiAttitudeRateCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& node_name,
                  const std::string& var_name) {

    if (!output_modalities.attitude_rate) {
      ROS_ERROR("[%s]: The controller returned an output modality (attitude rate cmd) that is not supported by the hardware API", node_name.c_str());
      return false;
    }

    return validateHwApiAttitudeRateCmd(msg, node_name, var_name);
  }
  bool operator()(const mrs_msgs::HwApiAccelerationHdgRateCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& node_name,
                  const std::string& var_name) {

    if (!output_modalities.acceleration_hdg_rate) {
      ROS_ERROR("[%s]: The controller returned an output modality (acceleration+hdg rate cmd) that is not supported by the hardware API", node_name.c_str());
      return false;
    }

    return validateHwApiAccelerationHdgRateCmd(msg, node_name, var_name);
  }
  bool operator()(const mrs_msgs::HwApiAccelerationHdgCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& node_name,
                  const std::string& var_name) {

    if (!output_modalities.acceleration_hdg) {
      ROS_ERROR("[%s]: The controller returned an output modality (acceleration+hdg cmd) that is not supported by the hardware API", node_name.c_str());
      return false;
    }

    return validateHwApiAccelerationHdgCmd(msg, node_name, var_name);
  }
  bool operator()(const mrs_msgs::HwApiVelocityHdgRateCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& node_name,
                  const std::string& var_name) {

    if (!output_modalities.velocity_hdg_rate) {
      ROS_ERROR("[%s]: The controller returned an output modality (velocity+hdg rate cmd) that is not supported by the hardware API", node_name.c_str());
      return false;
    }

    return validateHwApiVelocityHdgRateCmd(msg, node_name, var_name);
  }
  bool operator()(const mrs_msgs::HwApiVelocityHdgCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& node_name,
                  const std::string& var_name) {

    if (!output_modalities.velocity_hdg) {
      ROS_ERROR("[%s]: The controller returned an output modality (velocity+hdg cmd) that is not supported by the hardware API", node_name.c_str());
      return false;
    }

    return validateHwApiVelocityHdgCmd(msg, node_name, var_name);
  }
  bool operator()(const mrs_msgs::HwApiPositionCmd& msg, const ControlOutputModalities_t& output_modalities, const std::string& node_name,
                  const std::string& var_name) {

    if (!output_modalities.position) {
      ROS_ERROR("[%s]: The controller returned an output modality (position cmd) that is not supported by the hardware API", node_name.c_str());
      return false;
    }

    return validateHwApiPositionCmd(msg, node_name, var_name);
  }
};

//}

/* control output initialization //{ */

Controller::HwApiOutputVariant initializeDefaultOutput(const ControlOutputModalities_t& possible_outputs, const mrs_msgs::UavState& uav_state,
                                                       const double& min_throttle, const double& n_motors);

void initializeHwApiCmd(mrs_msgs::HwApiActuatorCmd& msg, const double& min_throttle, const double& n_motors);
void initializeHwApiCmd(mrs_msgs::HwApiControlGroupCmd& msg, const double& min_throttle);
void initializeHwApiCmd(mrs_msgs::HwApiAttitudeRateCmd& msg, const double& min_throttle);
void initializeHwApiCmd(mrs_msgs::HwApiAttitudeCmd& msg, const mrs_msgs::UavState& uav_state, const double& min_throttle);
void initializeHwApiCmd(mrs_msgs::HwApiAccelerationHdgRateCmd& msg, const mrs_msgs::UavState& uav_state);
void initializeHwApiCmd(mrs_msgs::HwApiAccelerationHdgCmd& msg, const mrs_msgs::UavState& uav_state);
void initializeHwApiCmd(mrs_msgs::HwApiVelocityHdgRateCmd& msg, const mrs_msgs::UavState& uav_state);
void initializeHwApiCmd(mrs_msgs::HwApiVelocityHdgCmd& msg, const mrs_msgs::UavState& uav_state);
void initializeHwApiCmd(mrs_msgs::HwApiPositionCmd& msg, const mrs_msgs::UavState& uav_state);

struct HwApiInitializeVisitor
{
  void operator()(mrs_msgs::HwApiActuatorCmd& msg, [[maybe_unused]] const mrs_msgs::UavState& uav_state, const double& min_throttle, const double& n_motors) {
    initializeHwApiCmd(msg, min_throttle, n_motors);
  }
  void operator()(mrs_msgs::HwApiControlGroupCmd& msg, [[maybe_unused]] const mrs_msgs::UavState& uav_state, const double& min_throttle,
                  [[maybe_unused]] const double& n_motors) {
    initializeHwApiCmd(msg, min_throttle);
  }
  void operator()(mrs_msgs::HwApiAttitudeCmd& msg, const mrs_msgs::UavState& uav_state, const double& min_throttle, [[maybe_unused]] const double& n_motors) {
    initializeHwApiCmd(msg, uav_state, min_throttle);
  }
  void operator()(mrs_msgs::HwApiAttitudeRateCmd& msg, [[maybe_unused]] const mrs_msgs::UavState& uav_state, const double& min_throttle,
                  [[maybe_unused]] const double& n_motors) {
    initializeHwApiCmd(msg, min_throttle);
  }
  void operator()(mrs_msgs::HwApiAccelerationHdgRateCmd& msg, const mrs_msgs::UavState& uav_state, [[maybe_unused]] const double& min_throttle,
                  [[maybe_unused]] const double& n_motors) {
    initializeHwApiCmd(msg, uav_state);
  }
  void operator()(mrs_msgs::HwApiAccelerationHdgCmd& msg, const mrs_msgs::UavState& uav_state, [[maybe_unused]] const double& min_throttle,
                  [[maybe_unused]] const double& n_motors) {
    initializeHwApiCmd(msg, uav_state);
  }
  void operator()(mrs_msgs::HwApiVelocityHdgRateCmd& msg, const mrs_msgs::UavState& uav_state, [[maybe_unused]] const double& min_throttle,
                  [[maybe_unused]] const double& n_motors) {
    initializeHwApiCmd(msg, uav_state);
  }
  void operator()(mrs_msgs::HwApiVelocityHdgCmd& msg, const mrs_msgs::UavState& uav_state, [[maybe_unused]] const double& min_throttle,
                  [[maybe_unused]] const double& n_motors) {
    initializeHwApiCmd(msg, uav_state);
  }
  void operator()(mrs_msgs::HwApiPositionCmd& msg, const mrs_msgs::UavState& uav_state, [[maybe_unused]] const double& min_throttle,
                  [[maybe_unused]] const double& n_motors) {
    initializeHwApiCmd(msg, uav_state);
  }
};

//}

}  // namespace control_manager

}  // namespace mrs_uav_managers

#endif  // CONTROL_MANAGER_COMMON_H
