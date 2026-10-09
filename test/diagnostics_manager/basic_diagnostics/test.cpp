#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_msgs/msg/uav_info.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/errorgraph_element.hpp>
#include <mrs_msgs/msg/errorgraph_error.hpp>

#include <mrs_uav_testing/test_generic.h>

#include <mutex>
#include <optional>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {
    sub_errors_ = node_->create_subscription<mrs_msgs::msg::ErrorgraphElement>(
        "/uav1/errors", 100, [this](const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg) { errorsCallback(msg); });
  }

  bool test(void);

private:
  rclcpp::Subscription<mrs_msgs::msg::ErrorgraphElement>::SharedPtr sub_errors_;
  std::mutex                                                        errors_mtx_;
  std::optional<mrs_msgs::msg::ErrorgraphElement>                   last_diagnostics_manager_msg_;
  bool                                                              saw_any_waiting_error_ = false;

  void errorsCallback(const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg);
};

void Tester::errorsCallback(const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg) {
  std::scoped_lock lck(errors_mtx_);
  if (msg->source_node.node == "DiagnosticsManager" && msg->source_node.component == "main") {
    last_diagnostics_manager_msg_ = *msg;

    for (const auto &error : msg->errors) {
      if (error.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_NODE) {
        saw_any_waiting_error_ = true;
        break;
      }
    }
  }
}

bool Tester::test(void) {

  const std::string uav_name = "uav1";

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
      return false;
    }
  }

  std::shared_ptr<mrs_msgs::msg::UavInfo> received_msg;

  auto sub = node_->create_subscription<mrs_msgs::msg::UavInfo>("/" + uav_name + "/diagnostics_manager/uav_info", rclcpp::SystemDefaultsQoS(),
                                                                [&received_msg](const mrs_msgs::msg::UavInfo::SharedPtr msg) { received_msg = msg; });

  const auto deadline = node_->get_clock()->now() + rclcpp::Duration(20s);

  while (rclcpp::ok() && !received_msg && node_->get_clock()->now() < deadline) {
    sleep(0.1);
  }

  if (!received_msg) {
    RCLCPP_ERROR(node_->get_logger(),
                 "no message received on 'diagnostics_manager/uav_info' within 20s — DiagnosticsManager is not running as part of the core stack");
    return false;
  }

  std::shared_ptr<mrs_msgs::msg::GeneralRobotInfo> general_robot_info_msg;

  auto sub2 = node_->create_subscription<mrs_msgs::msg::GeneralRobotInfo>(
      "/" + uav_name + "/diagnostics_manager/general_robot_info", rclcpp::SystemDefaultsQoS(),
      [&general_robot_info_msg](const mrs_msgs::msg::GeneralRobotInfo::SharedPtr msg) { general_robot_info_msg = msg; });

  auto all_ok = [](const mrs_msgs::msg::PreflightStatus &s) { return s.speed_ok && s.height_ok && s.gyro_ok && s.topics_ok && s.position_valid; };

  const auto preflight_deadline = node_->get_clock()->now() + rclcpp::Duration(20s);

  while (rclcpp::ok() && node_->get_clock()->now() < preflight_deadline) {

    if (general_robot_info_msg && all_ok(general_robot_info_msg->preflight_status)) {
      break;
    }

    sleep(0.1);
  }

  if (!general_robot_info_msg) {
    RCLCPP_ERROR(node_->get_logger(), "no message received on 'diagnostics_manager/general_robot_info' within 20s");
    return false;
  }

  if (!all_ok(general_robot_info_msg->preflight_status)) {
    RCLCPP_ERROR(node_->get_logger(), "general_robot_info.preflight_status did not become fully ok within 20s on a healthy sim stack");
    return false;
  }

  // DiagnosticsManager reports waiting_for_node for its delegated managers, then stops once healthy.
  const int required_consecutive_clean = 3;
  int       consecutive_clean          = 0;

  const auto clean_deadline = node_->get_clock()->now() + rclcpp::Duration(90s);

  while (rclcpp::ok() && node_->get_clock()->now() < clean_deadline && consecutive_clean < required_consecutive_clean) {

    sleep(0.2);

    std::scoped_lock lck(errors_mtx_);

    if (!last_diagnostics_manager_msg_.has_value()) {
      continue;
    }

    bool has_waiting_for_node = false;
    for (const auto &error : last_diagnostics_manager_msg_->errors) {
      if (error.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_NODE) {
        has_waiting_for_node = true;
        break;
      }
    }

    consecutive_clean = has_waiting_for_node ? 0 : consecutive_clean + 1;
  }

  if (consecutive_clean < required_consecutive_clean) {
    RCLCPP_ERROR(node_->get_logger(), "DiagnosticsManager still reported waiting_for_node errors on a healthy sim stack");
    return false;
  }

  {
    std::scoped_lock lck(errors_mtx_);
    if (!saw_any_waiting_error_) {
      RCLCPP_ERROR(node_->get_logger(), "never observed a waiting_for_node error, so clearing proves nothing");
      return false;
    }
  }

  return true;
}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Tester tester;

  test_result &= tester.test();

  tester.sleep(2.0);

  std::cout << "Test: reporting test results" << std::endl;

  tester.reportTestResult(test_result);

  tester.join();
}
