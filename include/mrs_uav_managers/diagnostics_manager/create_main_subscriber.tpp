#pragma once

namespace mrs_uav_managers
{

template <typename MessageType>
mrs_lib::SubscriberHandler<MessageType> DiagnosticsSensorHandler::create_main_subscriber(rclcpp::Node::SharedPtr &node, const std::string &topic_name,
                                                                                         rclcpp::CallbackGroup::SharedPtr cbkgrp_subs,
                                                                                         const rclcpp::Duration          &timeout) {

  shopts_.node                                = node;
  shopts_.node_name                           = "DiagnosticsManager";
  shopts_.no_message_timeout                  = timeout;
  shopts_.threadsafe                          = true;
  shopts_.autostart                           = true;
  shopts_.subscription_options.callback_group = cbkgrp_subs;
  shopts_.qos                                 = qos_profile_;

  auto callback = [this]([[maybe_unused]] const typename MessageType::ConstPtr &msg) {
    const rclcpp::Time now = rclcpp::Clock(RCL_STEADY_TIME).now();
    rate_tracker_.record(now);
    {
      std::scoped_lock lock(mutex_state_);
      state_.msg_count++;
      state_.last_msg_wall_time = now;
    }
  };

  return mrs_lib::SubscriberHandler<MessageType>(shopts_, topic_name, callback);
}

} // namespace mrs_uav_managers
