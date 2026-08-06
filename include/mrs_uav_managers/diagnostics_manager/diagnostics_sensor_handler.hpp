#pragma once

/* includes //{ */

#include <diagnostic_msgs/msg/key_value.hpp>
#include <mrs_lib/errorgraph/error_publisher.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_msgs/msg/sensor_status.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include <mrs_uav_managers/diagnostics_manager/enums/sensor_type.hpp>
#include <mrs_uav_managers/diagnostics_manager/utils/rate_tracker.hpp>

//}

namespace mrs_uav_managers::diagnostics_manager
{

/** @brief Values shared by every DiagnosticsSensorHandler instance (transformer, body frame). */
struct DiagnosticsCommonHandlers_t
{
  std::shared_ptr<mrs_lib::Transformer> transformer;
  std::string                           body_frame;
};

class DiagnosticsSensorHandler {
public:
  // Called once by DiagnosticsManager to load parameters, set up rate monitoring, and invoke onInitialize().
  // Returns false on failure (e.g. onInitialize() returning false or a config error).
  bool initialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, const std::string &name_space,
                  const DiagnosticsCommonHandlers_t &common_handlers, rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr);

  // Called periodically by DiagnosticsManager to get this handler's current health (rate/staleness plus fill_details()).
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

  // Hook for derived classes to provide additional details in the SensorStatus message, as flat string-valued key/value pairs
  // (e.g. last message timestamp, error counts, etc.). By default, returns an empty vector.
  // If a sensor's details are inherently structured/nested rather than flat key/value pairs, a plugin may instead publish its
  // own mrs_msgs::msg::SensorInfo message (see CameraSensorHandler for a worked example) and leave this at its default.
  virtual std::vector<diagnostic_msgs::msg::KeyValue> fill_details();

  std::string topic_;
  uint8_t     sensor_type_uint_ = 0;
  bool        is_initialized_   = false;

  // Rate monitoring
  std::string expected_publisher_node_;
  std::string expected_publisher_component_;
  double      expected_rate_       = 0.0;
  double      rate_tolerance_      = 0.3;
  bool        no_upper_rate_limit_ = false; // set when expected_rate config used the "N+" (floor-only) syntax

  // Grace period before reporting rate errors
  static constexpr double GRACE_PERIOD_S = 5.0;
  rclcpp::Time            init_time_;

  // Runtime state updated from subscriber callbacks
  struct RuntimeState
  {
    rclcpp::Time               last_msg_wall_time{0, 0, RCL_STEADY_TIME};
    uint64_t                   msg_count{0};
    std::optional<std::string> last_frame_id; // set by recordMessageReceived(frame_id)
  };
  mutable std::mutex mutex_state_;
  RuntimeState       state_;

  // Sliding-window rate tracker (internally synchronised)
  utils::RateTracker rate_tracker_;

  mrs_lib::SubscriberHandlerOptions shopts_;
  rclcpp::QoS                       qos_profile_{10};

  // Error publisher for reporting detailed errors (optional, can be used by derived classes)
  std::shared_ptr<mrs_lib::errorgraph::ErrorPublisher> error_publisher_;

  // Frame transform check (check_frame_transform_ is a per-sensor opt-out)
  std::shared_ptr<mrs_lib::Transformer> transformer_;
  std::string                           body_frame_;
  bool                                  check_frame_transform_ = true;

  // | -------------------- sensor-health helpers ------------------- |

  /** @brief Current measured rate from the sliding-window tracker. */
  double getMeasuredRate() const;

  /** @brief True while the node is still within the startup grace period. */
  bool isInGracePeriod(const rclcpp::Time &now) const;

  /** @brief True if @p last_msg is recent enough (within 3× the expected period). */
  bool isTopicFresh(const rclcpp::Time &now, const rclcpp::Time &last_msg) const;

  /** @brief Records rate/staleness bookkeeping for the arrival of one message. Call once per message from every subscription callback --
   * create_main_subscriber()'s SubscriberHandler<T> callback does this automatically; a plugin using a generic/type-erased subscription
   * (which can't use create_main_subscriber()) must call this itself from its own callback. */
  void recordMessageReceived();

  /** @brief Also records the message's frame_id, for updateStatus()'s frame-transformability check. */
  void recordMessageReceived(const std::string &frame_id);

  // | -------------------- support functions ------------------- |
  /** @brief Maps a config type: string to its mrs_msgs::msg::SensorStatus::TYPE_* constant, via sensor_type_t (enums/sensor_type.hpp) --
   * X_ENUM_SEQ there is the single source of truth for the set of valid strings, so a genuinely new sensor type that's missing its
   * mrs_msgs::msg::SensorStatus::TYPE_* constant fails to compile rather than silently degrading at runtime. Returns std::nullopt for
   * an unmapped string (a typo in config, since the compile-time gap above is already ruled out) -- the caller treats this as fatal,
   * since silently degrading to TYPE_UNKNOWN would hide the mistake instead of making the operator fix it. */
  std::optional<uint8_t> mapSensorType(const std::string &type_str);
  Eigen::Matrix3d        cov2eigen(const std::array<double, 9> &msg_cov);

  /** @brief Scalar "spread" of a covariance matrix, in the same units as the underlying quantity: pow(det(cov), 1/6). The determinant of a 3x3
   * covariance is units^2, so the 6th root (not the more common cube root) is needed to bring it back to units^1. */
  double covUncertainty(const std::array<double, 9> &msg_cov);

  /** @brief Parses expected_rate: a number checks both bounds; a trailing '+' ("10+") checks only the floor.
   * std::nullopt means malformed -- the caller treats that as fatal. */
  std::optional<std::pair<double, bool>> parseExpectedRate(const std::string &raw);

  /** @brief Formats a value with 1 decimal place for SensorStatus.message (std::to_string() always prints 6). */
  std::string formatDecimal(double value);

  template <typename MessageType>
  mrs_lib::SubscriberHandler<MessageType> create_main_subscriber(rclcpp::Node::SharedPtr &node, const std::string &topic_name,
                                                                 rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr,
                                                                 const rclcpp::Duration          &timeout     = mrs_lib::no_timeout);
};

} // namespace mrs_uav_managers::diagnostics_manager

#include <mrs_uav_managers/diagnostics_manager/create_main_subscriber.tpp>
