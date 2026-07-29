#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_msgs/msg/uav_info.hpp>

#include <mrs_uav_testing/test_generic.h>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {
  }

  bool test(void);
};

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

  auto sub = node_->create_subscription<mrs_msgs::msg::UavInfo>(
      "/" + uav_name + "/diagnostics_manager/uav_info", rclcpp::SystemDefaultsQoS(),
      [&received_msg](const mrs_msgs::msg::UavInfo::SharedPtr msg) { received_msg = msg; });

  const auto deadline = node_->get_clock()->now() + rclcpp::Duration(20s);

  while (!received_msg && node_->get_clock()->now() < deadline) {
    sleep(0.1);
  }

  if (!received_msg) {
    RCLCPP_ERROR(node_->get_logger(), "no message received on 'diagnostics_manager/uav_info' within 20s — DiagnosticsManager is not running as part of the core stack");
    return false;
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
