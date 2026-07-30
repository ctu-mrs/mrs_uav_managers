#pragma once

/* includes //{ */

#include <diagnostic_msgs/msg/key_value.hpp>
#include <mrs_lib/errorgraph/error_publisher.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_msgs/msg/sensor_status.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_managers/diagnostics_manager/utils/rate_tracker.hpp>

//}

namespace mrs_uav_managers::diagnostics_manager
{

class DiagnosticsSensorHandler {
public:
  bool initialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, const std::string &name_space,
                  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr);

  virtual mrs_msgs::msg::SensorStatus updateStatus();

  virtual ~DiagnosticsSensorHandler() = default;

  std::string name_;

protected:
  // Hook for derived classes to do additional initialization (e.g. create subscribers) after the base class has loaded parameters and set up rate monitoring.
  // config_key is the YAML key (e.g. "GPS") used for loading plugin-specific parameters.
  // plugin_config_path is the (optional, may be empty) value of this instance's "plugin_config" field from the central config -- derived classes that need
  // their own fields should fall back to a package-owned default file (e.g. keyed by config_key) when it's empty, rather than requiring the central config
  // to declare plugin-specific fields.
  virtual bool onInitialize([[maybe_unused]] rclcpp::Node::SharedPtr &node, [[maybe_unused]] const std::string &config_key,
                            [[maybe_unused]] const std::string &name_space, [[maybe_unused]] const std::string &plugin_config_path,
                            [[maybe_unused]] rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr);

  // Hook for derived classes to provide additional details in the SensorStatus message
  // By default, returns an empty JSON object, but derived classes can override this to include custom details about the sensor status (e.g. last message
  // timestamp, error counts, etc.)
  virtual std::vector<diagnostic_msgs::msg::KeyValue> fill_details();

  std::string topic_;
  uint8_t     sensor_type_uint_ = 0;
  bool        is_initialized_   = false;

  // Rate monitoring
  std::string expected_publisher_node_;
  std::string expected_publisher_component_;
  double      expected_rate_  = 0.0;
  double      rate_tolerance_ = 0.3;

  // Grace period before reporting rate errors
  static constexpr double GRACE_PERIOD_S = 5.0;
  rclcpp::Time            init_time_;

  // Runtime state updated from subscriber callbacks
  struct RuntimeState
  {
    rclcpp::Time last_msg_wall_time{0, 0, RCL_STEADY_TIME};
    uint64_t     msg_count{0};
  };
  mutable std::mutex mutex_state_;
  RuntimeState       state_;

  // Sliding-window rate tracker (internally synchronised)
  utils::RateTracker rate_tracker_;

  mrs_lib::SubscriberHandlerOptions shopts_;
  rclcpp::QoS                       qos_profile_{10};

  // Error publisher for reporting detailed errors (optional, can be used by derived classes)
  std::shared_ptr<mrs_lib::errorgraph::ErrorPublisher> error_publisher_;

  // | -------------------- sensor-health helpers ------------------- |

  /** @brief Current measured rate from the sliding-window tracker. */
  double getMeasuredRate() const;

  /** @brief True while the node is still within the startup grace period. */
  bool isInGracePeriod(const rclcpp::Time &now) const;

  /** @brief True if @p last_msg is recent enough (within 3× the expected period). */
  bool isTopicFresh(const rclcpp::Time &now, const rclcpp::Time &last_msg) const;

  // | -------------------- support functions ------------------- |
  uint8_t         mapSensorType(const std::string &type_str);
  Eigen::Matrix3d cov2eigen(const std::array<double, 9> &msg_cov);

  template <typename MessageType>
  mrs_lib::SubscriberHandler<MessageType> create_main_subscriber(rclcpp::Node::SharedPtr &node, const std::string &topic_name,
                                                                 rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr,
                                                                 const rclcpp::Duration          &timeout     = mrs_lib::no_timeout);
};

} // namespace mrs_uav_managers::diagnostics_manager

#include <mrs_uav_managers/diagnostics_manager/create_main_subscriber.tpp>
