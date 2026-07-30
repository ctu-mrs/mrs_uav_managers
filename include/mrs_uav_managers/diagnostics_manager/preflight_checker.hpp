#pragma once

/* includes //{ */

#include <cmath>
#include <geometry_msgs/msg/vector3.hpp>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/estimation_diagnostics.hpp>
#include <mrs_msgs/msg/hw_api_capabilities.hpp>
#include <mrs_msgs/msg/safety_area_manager_diagnostics.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <string>
#include <vector>

//}

namespace mrs_uav_managers::diagnostics_manager
{

class PreflightChecker

{
public:
  PreflightChecker(rclcpp::Node::SharedPtr node, const std::string &robot_name);

  /** @brief Result of running the full preflight check suite. */
  struct PreflightResult
  {
    bool                     speed_ok        = true;
    bool                     height_ok       = true;
    bool                     gyro_ok         = true;
    bool                     topics_ok       = true;
    bool                     position_valid  = true;
    bool                     control_enabled = true;
    bool                     can_takeoff     = false; ///< AND of all individual checks
    std::vector<std::string> violations;              ///< human-readable failure reasons
  };

  struct PreflightInputs
  {
    std::optional<geometry_msgs::msg::Vector3> velocity;
    std::optional<sensor_msgs::msg::Range>     distance_sensor_range;
    std::optional<geometry_msgs::msg::Vector3> angular_rate;
    bool                                       has_distance_sensor = false;
    bool                                       has_imu             = false;
    bool                                       position_valid      = false; // from safety area manager diagnostics
  };

  /** @brief Run checks with data collected from the various subscribed topics. */
  // This is the main entry point for using the PreflightChecker in state monitor
  PreflightResult runPreflightChecks(void);

  /** @brief Run speed / height / gyro / topic / position checks; updates debounce timestamps. */
  //  Useful for testing individual check logic with custom inputs, without needing to publish to all the relevant topics.
  //  Overloads the above method that collects data from topics and then calls this one.
  PreflightResult runPreflightChecks(const PreflightInputs &inputs);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;
  std::string              robot_name_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_; ///< callback group for subscribers

  void initialize(void);

  /** @brief Individual per-check helpers. */
  std::optional<std::string>              preflightCheckSpeed(const std::optional<geometry_msgs::msg::Vector3> &velocity);
  std::optional<std::string>              preflightCheckHeight(const std::optional<sensor_msgs::msg::Range> &distance_sensor_range, bool has_distance_sensor);
  std::optional<std::string>              preflightCheckGyro(const std::optional<geometry_msgs::msg::Vector3> &angular_rate, bool has_imu);
  std::optional<std::vector<std::string>> preflightCheckTopics(void);

  /** @brief Generic callback that marks a heartbeat topic as seen. */
  void genericTopicCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg, size_t id);

  /** @brief Static configuration for the preflight check suite. */
  struct PreflightConfig
  {
    bool   enabled               = false;
    double time_window           = 5.0;
    double not_reporting_timeout = 3.0;

    bool   speed_check_enabled = false;
    double speed_check_max     = 0.0;

    bool   height_check_enabled = false;
    double height_check_max     = 0.0;

    bool   gyro_check_enabled = false;
    double gyro_check_max     = 0.0;

    bool                     topic_check_enabled = false;
    double                   topic_check_timeout = 0.0;
    std::vector<std::string> topic_check_topics; // "name:type" entries
  };

  PreflightConfig preflight_cfg_;

  /** @brief Per-check timestamp of the last observed violation. */
  rclcpp::Time speed_check_violated_time_;
  rclcpp::Time height_check_violated_time_;
  rclcpp::Time gyro_check_violated_time_;

  bool speed_violation_seen_  = false;
  bool height_violation_seen_ = false;
  bool gyro_violation_seen_   = false;

  /** @brief Tracks last-message time for one topic in the generic topic_check. */
  struct TopicHeartbeat
  {
    std::string  name;
    rclcpp::Time last_msg_time;
    bool         ever_seen;
  };
  std::mutex                                          topic_heartbeats_mutex_;
  std::vector<TopicHeartbeat>                         topic_heartbeats_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> topic_check_subs_;

  /** @brief Gather all inputs for the preflight checks from the various subscribed topics. */
  PreflightInputs collectPreflightData(void);

  // | ---------------------- ROS subscribers --------------------- |
  std::shared_ptr<mrs_lib::TimeoutManager>                                tim_mgr_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>        sh_estimation_diagnostics_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiCapabilities>            sh_hw_api_capabilities_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Range>                     sh_hw_api_distance_sensor_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>                       sh_hw_api_imu_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::SafetyAreaManagerDiagnostics> sh_safety_area_manager_diagnostics_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>    sh_control_manager_diagnostics_;
};

} // namespace mrs_uav_managers::diagnostics_manager
