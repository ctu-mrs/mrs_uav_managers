#ifndef MRS_UAV_MANAGERS_COMMON_H
#define MRS_UAV_MANAGERS_COMMON_H

#include <ros/ros.h>

#include <vector>
#include <string>
#include <optional>
#include <variant>

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

#include <nav_msgs/Odometry.h>

namespace mrs_uav_managers
{

std::optional<unsigned int> idxInVector(const std::string& str, const std::vector<std::string>& vec);

// checks for invalid values in the result from trackers
bool validateTrackerCommand(const std::optional<mrs_msgs::TrackerCommand>& msg, const std::string& node_name, const std::string& var_name);

// checks for invalid messages in/out
bool validateOdometry(const nav_msgs::Odometry& msg, const std::string& node_name, const std::string& var_name);
bool validateUavState(const mrs_msgs::UavState& msg, const std::string& node_name, const std::string& var_nam);
bool validateVelocityReference(const mrs_msgs::VelocityReference& msg, const std::string& node_name, const std::string& var_name);
bool validateControlOutput(const Controller::ControlOutput& control_output, const std::string& node_name, const std::string& var_name);

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

struct HwApiCmdValidator
{
  bool operator()(const mrs_msgs::HwApiActuatorCmd& msg, const std::string& node_name, const std::string& var_name) {
    return validateHwApiActuatorCmd(msg, node_name, var_name);
  }
  bool operator()(const mrs_msgs::HwApiControlGroupCmd& msg, const std::string& node_name, const std::string& var_name) {
    return validateHwApiControlGroupCmd(msg, node_name, var_name);
  }
  bool operator()(const mrs_msgs::HwApiAttitudeCmd& msg, const std::string& node_name, const std::string& var_name) {
    return validateHwApiAttitudeCmd(msg, node_name, var_name);
  }
  bool operator()(const mrs_msgs::HwApiAttitudeRateCmd& msg, const std::string& node_name, const std::string& var_name) {
    return validateHwApiAttitudeRateCmd(msg, node_name, var_name);
  }
  bool operator()(const mrs_msgs::HwApiAccelerationHdgRateCmd& msg, const std::string& node_name, const std::string& var_name) {
    return validateHwApiAccelerationHdgRateCmd(msg, node_name, var_name);
  }
  bool operator()(const mrs_msgs::HwApiAccelerationHdgCmd& msg, const std::string& node_name, const std::string& var_name) {
    return validateHwApiAccelerationHdgCmd(msg, node_name, var_name);
  }
  bool operator()(const mrs_msgs::HwApiVelocityHdgRateCmd& msg, const std::string& node_name, const std::string& var_name) {
    return validateHwApiVelocityHdgRateCmd(msg, node_name, var_name);
  }
  bool operator()(const mrs_msgs::HwApiVelocityHdgCmd& msg, const std::string& node_name, const std::string& var_name) {
    return validateHwApiVelocityHdgCmd(msg, node_name, var_name);
  }
  bool operator()(const mrs_msgs::HwApiPositionCmd& msg, const std::string& node_name, const std::string& var_name) {
    return validateHwApiPositionCmd(msg, node_name, var_name);
  }
};

// translates the channel values to desired range
double RCChannelToRange(double rc_value, double range, double deadband);

}  // namespace mrs_uav_managers

#endif  // MRS_UAV_MANAGERS_COMMON_H
