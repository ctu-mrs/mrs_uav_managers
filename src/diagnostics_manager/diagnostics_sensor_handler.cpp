#include <mrs_uav_managers/diagnostics_manager/diagnostics_sensor_handler.hpp>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace mrs_uav_managers::diagnostics_manager
{

/* initialize() //{ */

bool DiagnosticsSensorHandler::initialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, const std::string &name_space,
                                          const DiagnosticsCommonHandlers_t &common_handlers, rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {

  transformer_ = common_handlers.transformer;
  body_frame_  = common_handlers.body_frame;

  mrs_lib::ParamLoader param_loader(node, "DiagnosticsSensorHandler");

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path, std::string(""));
  if (!custom_config_path.empty()) {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("public_config");
  param_loader.addYamlFileFromParam("public_sensor_handlers");
  param_loader.setPrefix("mrs_uav_managers/diagnostics_manager/sensor_handlers/");

  name_ = config_key; // default name is the config key
  // Load all common parameters using the YAML key (config_key)
  std::string sensor_type_str;
  param_loader.loadParam(config_key + "/topic", topic_);
  param_loader.loadParam(config_key + "/type", sensor_type_str);
  param_loader.loadParam(config_key + "/expected_publisher/node", expected_publisher_node_, std::string("HwApiManager"));
  param_loader.loadParam(config_key + "/expected_publisher/component", expected_publisher_component_, std::string("main"));
  std::string expected_rate_str;
  param_loader.loadParam(config_key + "/expected_rate", expected_rate_str);
  param_loader.loadParam(config_key + "/rate_tolerance", rate_tolerance_, 0.3);
  param_loader.loadParam(config_key + "/check_frame_transform", check_frame_transform_, true);

  std::string qos_reliability;
  param_loader.loadParam(config_key + "/qos_reliability", qos_reliability, std::string("reliable"));

  std::string plugin_config_path;
  param_loader.loadParam(config_key + "/plugin_config", plugin_config_path, std::string(""));

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node->get_logger(), "[%s]: failed to load config, not initializing", config_key.c_str());
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "[%s]: loaded config (topic: '%s')", name_.c_str(), topic_.c_str());

  std::string handler_instance = name_ + " handler (" + topic_ + ")";
  error_publisher_             = std::make_shared<mrs_lib::errorgraph::ErrorPublisher>(node, node->get_clock(), "DiagnosticsManager", handler_instance);

  const auto sensor_type = mapSensorType(sensor_type_str);
  if (!sensor_type.has_value()) {
    RCLCPP_ERROR(node->get_logger(), "[%s]: unknown sensor type '%s'", name_.c_str(), sensor_type_str.c_str());
    return false;
  }
  sensor_type_uint_ = sensor_type.value();

  const auto parsed_rate = parseExpectedRate(expected_rate_str);
  if (!parsed_rate.has_value()) {
    RCLCPP_ERROR(node->get_logger(), "[%s]: invalid expected_rate '%s' (expected a positive number, optionally suffixed with '+')", name_.c_str(),
                 expected_rate_str.c_str());
    return false;
  }
  expected_rate_       = parsed_rate->first;
  no_upper_rate_limit_ = parsed_rate->second;

  // Create QoS profile based on config
  qos_profile_ = rclcpp::QoS(10);
  if (qos_reliability == "best_effort") {
    qos_profile_.best_effort();
  } else {
    qos_profile_.reliable();
  }
  qos_profile_.durability_volatile();

  // Initialize timing
  init_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();
  {
    std::scoped_lock lock(mutex_state_);
    state_ = RuntimeState{};
  }
  rate_tracker_.clear();


  const bool initialized = onInitialize(node, config_key, name_space, plugin_config_path, cbkgrp_subs);
  is_initialized_        = initialized;

  return initialized;
}

//}

/* updateStatus() //{ */

mrs_msgs::msg::SensorStatus DiagnosticsSensorHandler::updateStatus() {
  mrs_msgs::msg::SensorStatus ss;
  ss.name  = name_;
  ss.type  = sensor_type_uint_;
  ss.topic = topic_;

  if (!is_initialized_) {
    ss.ready   = false;
    ss.rate    = -1.0;
    ss.message = "Not initialized";
    ss.level   = mrs_msgs::msg::SensorStatus::ERROR;
    ss.details = fill_details();
    return ss;
  }

  const rclcpp::Time now = rclcpp::Clock(RCL_STEADY_TIME).now();

  // Snapshot mutable runtime state once
  RuntimeState snapshot;
  {
    std::scoped_lock lock(mutex_state_);
    snapshot = state_;
  }
  const double measured_rate = getMeasuredRate();

  // Grace period — don't report rate errors right after startup
  if (isInGracePeriod(now)) {
    ss.ready   = false;
    ss.rate    = measured_rate;
    ss.message = "Initializing (grace period)";
    ss.level   = mrs_msgs::msg::SensorStatus::STALE;
    ss.details = fill_details();
    return ss;
  }

  // No messages ever received
  if (snapshot.msg_count == 0) {
    const double elapsed_since_init = (now - init_time_).seconds();
    ss.ready                        = false;
    ss.rate                         = 0.0;
    ss.message                      = "No messages received for " + std::to_string(elapsed_since_init) + " seconds since startup";
    ss.level                        = mrs_msgs::msg::SensorStatus::ERROR;
    mrs_lib::errorgraph::node_id_t source_node;
    source_node.node      = expected_publisher_node_;
    source_node.component = expected_publisher_component_;
    error_publisher_->addWaitingForTopicError(topic_, source_node);
    ss.details = fill_details();
    return ss;
  }

  // Topic gone silent — no message for 3x the expected period
  if (!isTopicFresh(now, snapshot.last_msg_wall_time)) {
    const double time_since_last = (now - snapshot.last_msg_wall_time).seconds();
    ss.ready                     = false;
    ss.rate                      = 0.0;
    ss.message                   = "No messages received for " + std::to_string(time_since_last) + " seconds";
    ss.level                     = mrs_msgs::msg::SensorStatus::ERROR;
    mrs_lib::errorgraph::node_id_t source_node;
    source_node.node      = expected_publisher_node_;
    source_node.component = expected_publisher_component_;
    error_publisher_->addWaitingForTopicError(topic_, source_node);
    ss.details = fill_details();
    return ss;
  }

  // Rate comparison
  ss.rate = measured_rate;

  const double lower_bound = expected_rate_ * (1.0 - rate_tolerance_);
  const double upper_bound = expected_rate_ * (1.0 + rate_tolerance_);

  const bool within_upper = no_upper_rate_limit_ || measured_rate <= upper_bound;

  if (measured_rate >= lower_bound && within_upper) {
    ss.ready   = true;
    ss.level   = mrs_msgs::msg::SensorStatus::OK;
    ss.message = "Rate within expected range";

    if (check_frame_transform_ && snapshot.last_frame_id.has_value() && !transformer_->getTransform(*snapshot.last_frame_id, body_frame_).has_value()) {
      ss.ready   = false;
      ss.level   = mrs_msgs::msg::SensorStatus::WARN;
      ss.message = "Frame '" + *snapshot.last_frame_id + "' cannot be transformed to '" + body_frame_ + "'";
    }
  } else if (measured_rate < lower_bound) {
    ss.ready   = false;
    ss.level   = mrs_msgs::msg::SensorStatus::WARN;
    ss.message = "Rate too low: expected " + formatHz(expected_rate_) + " Hz, got " + formatHz(measured_rate) + " Hz";
  } else {
    ss.ready   = true;
    ss.level   = mrs_msgs::msg::SensorStatus::WARN;
    ss.message = "Rate too high: expected " + formatHz(expected_rate_);
  }

  ss.details = fill_details();

  return ss;
}

//}

/* onInitialize() //{ */

bool DiagnosticsSensorHandler::onInitialize([[maybe_unused]] rclcpp::Node::SharedPtr &node, [[maybe_unused]] const std::string &config_key,
                                            [[maybe_unused]] const std::string &name_space, [[maybe_unused]] const std::string &plugin_config_path,
                                            [[maybe_unused]] rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {
  return true;
}

//}

/* fill_details() //{ */

std::vector<diagnostic_msgs::msg::KeyValue> DiagnosticsSensorHandler::fill_details() {
  return {};
}

//}

// | -------------------- sensor-health helpers ------------------- |

/* getMeasuredRate() //{ */

double DiagnosticsSensorHandler::getMeasuredRate() const {
  return rate_tracker_.rate();
}

//}

/* isInGracePeriod() //{ */

bool DiagnosticsSensorHandler::isInGracePeriod(const rclcpp::Time &now) const {
  return (now - init_time_).seconds() < GRACE_PERIOD_S;
}

//}

/* isTopicFresh() //{ */

bool DiagnosticsSensorHandler::isTopicFresh(const rclcpp::Time &now, const rclcpp::Time &last_msg) const {
  return (now - last_msg).seconds() <= (1.0 / expected_rate_) * 3.0;
}

//}

/* recordMessageReceived() //{ */

void DiagnosticsSensorHandler::recordMessageReceived() {
  const rclcpp::Time now = rclcpp::Clock(RCL_STEADY_TIME).now();
  rate_tracker_.record(now);
  std::scoped_lock lock(mutex_state_);
  state_.msg_count++;
  state_.last_msg_wall_time = now;
}

void DiagnosticsSensorHandler::recordMessageReceived(const std::string &frame_id) {
  recordMessageReceived();
  std::scoped_lock lock(mutex_state_);
  state_.last_frame_id = frame_id;
}

//}

// | -------------------- support functions ------------------- |

/* mapSensorType() //{ */

std::optional<uint8_t> DiagnosticsSensorHandler::mapSensorType(const std::string &type_str) {
  const sensor_type_t sensor_type = from_string<sensor_type_t>(type_str);
  if (sensor_type == sensor_type_t::UNKNOWN) {
    return std::nullopt;
  }

  return to_ros(sensor_type);
}

//}

/* cov2eigen() //{ */

Eigen::Matrix3d DiagnosticsSensorHandler::cov2eigen(const std::array<double, 9> &msg_cov) {
  Eigen::Matrix3d cov;
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      cov(r, c) = msg_cov.at(r + 3 * c);
  return cov;
}

//}

/* covUncertainty() //{ */

double DiagnosticsSensorHandler::covUncertainty(const std::array<double, 9> &msg_cov) {
  return std::pow(cov2eigen(msg_cov).determinant(), 1.0 / 6.0);
}

//}

/* parseExpectedRate() //{ */

std::optional<std::pair<double, bool>> DiagnosticsSensorHandler::parseExpectedRate(const std::string &raw) {
  const bool        floor_only = !raw.empty() && raw.back() == '+';
  const std::string numeric    = floor_only ? raw.substr(0, raw.size() - 1) : raw;

  if (numeric.empty()) {
    return std::nullopt;
  }

  try {
    std::size_t  consumed = 0;
    const double value    = std::stod(numeric, &consumed);
    if (consumed != numeric.size() || value <= 0.0) {
      return std::nullopt;
    }
    return std::make_pair(value, floor_only);
  }
  catch (const std::exception &) {
    return std::nullopt;
  }
}

//}

/* formatHz() //{ */

std::string DiagnosticsSensorHandler::formatHz(double value) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.1f", value);
  return std::string(buf);
}

//}

} // namespace mrs_uav_managers::diagnostics_manager
