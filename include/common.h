#ifndef MRS_UAV_MANAGERS_COMMON_H
#define MRS_UAV_MANAGERS_COMMON_H

#include <ros/ros.h>

#include <vector>
#include <string>
#include <optional>

#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/HwApiActuatorCmd.h>
#include <mrs_msgs/HwApiAttitudeRateCmd.h>
#include <mrs_msgs/HwApiAttitudeCmd.h>
#include <mrs_msgs/VelocityReference.h>

#include <nav_msgs/Odometry.h>

namespace mrs_uav_managers
{

std::optional<unsigned int> idxInVector(const std::string& str, const std::vector<std::string>& vec);

// checks for invalid values in the result from trackers
bool validateTrackerCommand(const mrs_msgs::TrackerCommand::ConstPtr tracker_command, const std::string& node_name, const std::string& var_name);
bool validateAttitudeCommand(const mrs_msgs::AttitudeCommand::ConstPtr attitude_command, const std::string& node_name, const std::string& var_name);

// checks for invalid messages in/out
bool validateOdometry(const nav_msgs::Odometry& msg, const std::string& node_name, const std::string& var_name);
bool validateUavState(const mrs_msgs::UavState& msg, const std::string& node_name, const std::string& var_nam);
bool validateHwApiAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd& msg, const std::string& node_name, const std::string& var_name);
bool validateHwApiAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd& msg, const std::string& node_name, const std::string& var_name);
bool validateVelocityReference(const mrs_msgs::VelocityReference& msg, const std::string& node_name, const std::string& var_name);

// translates the channel values to desired range
double RCChannelToRange(double rc_value, double range, double deadband);

}  // namespace mrs_uav_managers

#endif  // MRS_UAV_MANAGERS_COMMON_H
