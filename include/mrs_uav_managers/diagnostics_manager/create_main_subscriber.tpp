#pragma once

namespace mrs_uav_managers::diagnostics_manager
{

/* create_main_subscriber() //{ */

template <typename MessageType>
mrs_lib::SubscriberHandler<MessageType> DiagnosticsSensorHandler::create_main_subscriber(rclcpp::Node::SharedPtr &node, const std::string &topic_name,
                                                                                         rclcpp::CallbackGroup::SharedPtr cbkgrp_subs,
                                                                                         const rclcpp::Duration          &timeout) {

  shopts_.node                                = node;
  shopts_.node_name                           = "DiagnosticsManager";
  shopts_.no_message_timeout                  = timeout;
  shopts_.timeout_manager                     = timeout_manager_;
  shopts_.threadsafe                          = true;
  shopts_.autostart                           = true;
  shopts_.subscription_options.callback_group = cbkgrp_subs;
  shopts_.qos                                 = qos_profile_;

  auto callback = [this](const typename MessageType::ConstPtr &msg) {
    if constexpr (requires { msg->header.frame_id; }) {
      recordMessageReceived(msg->header.frame_id);
    } else {
      recordMessageReceived();
    }
  };

  return mrs_lib::SubscriberHandler<MessageType>(shopts_, topic_name, callback);
}

//}

} // namespace mrs_uav_managers::diagnostics_manager
