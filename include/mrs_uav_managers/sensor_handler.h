#pragma once
#include <rclcpp/rclcpp.hpp>
#include <mrs_msgs/msg/sensor_status.hpp>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <nlohmann/json.hpp>


namespace mrs_uav_managers
{

using json = nlohmann::json;

class SensorHandler {
public:
  virtual bool initialize(rclcpp::Node::SharedPtr &node, const std::string &name, const std::string &name_space, const std::string &topic) = 0;

  virtual mrs_msgs::msg::SensorStatus updateStatus() = 0;

  virtual ~SensorHandler() = default;

protected:
  rclcpp::Time last_msg_time_;
  double current_rate_;

  double calculateRate(const rclcpp::Time &current_msg_time) {
    if (last_msg_time_.seconds() == 0.0) {
      last_msg_time_ = current_msg_time;
      return -1.0;
    }

    double dt      = (current_msg_time - last_msg_time_).seconds();
    last_msg_time_ = current_msg_time;

    return (dt > 0.0) ? (1.0 / dt) : -1.0;
  }

  // Helper function to create a subscriber with rate calculation of the main topic
  template <typename MessageType>
  mrs_lib::SubscriberHandler<MessageType> createSubscriber(rclcpp::Node::SharedPtr &node, const std::string &topic_name,
                                                           const rclcpp::Duration &timeout = mrs_lib::no_timeout, rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr) {

    mrs_lib::SubscriberHandlerOptions shopts;
    shopts.node               = node;
    shopts.node_name          = "DiagnosticManager";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.subscription_options.callback_group = cbkgrp_subs;

    auto callback = [this](const typename MessageType::ConstPtr &msg) { current_rate_ = calculateRate(msg->header.stamp); };

    return mrs_lib::SubscriberHandler<MessageType>(shopts, topic_name, callback);
  }
};

} // namespace mrs_uav_managers
