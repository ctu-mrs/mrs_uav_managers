#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

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

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh;

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
      return false;
    }

    uh = uhopt.value();
  }

  {
    auto [success, message] = uh->activateMidAir();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "midair activation failed with message: '%s'", message.c_str());
      return false;
    }
  }

  {
    mrs_msgs::msg::ReferenceStamped msg;

    msg.header.frame_id      = uav_name + "/world_origin";
    msg.reference.position.x = 0;
    msg.reference.position.y = 0;
    msg.reference.position.z = 2;
    msg.reference.heading    = 0;

    auto [success, message] = uh->validateReference(msg);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "reference #1 validation failed: '%s'", message.c_str());
      return false;
    }
  }

  {
    mrs_msgs::msg::ReferenceStamped msg;

    msg.header.frame_id      = uav_name + "/world_origin";
    msg.reference.position.x = 100;
    msg.reference.position.y = 0;
    msg.reference.position.z = 2;
    msg.reference.heading    = 0;

    auto [success, message] = uh->validateReference(msg);

    if (success) {
      RCLCPP_ERROR(node_->get_logger(), "reference #2 validation failed: '%s'", message.c_str());
      return false;
    }
  }

  {
    mrs_msgs::msg::ReferenceStamped msg;

    msg.header.frame_id      = uav_name + "/world_origin";
    msg.reference.position.x = 0;
    msg.reference.position.y = 0;
    msg.reference.position.z = -100;
    msg.reference.heading    = 0;

    auto [success, message] = uh->validateReference(msg);

    if (success) {
      RCLCPP_ERROR(node_->get_logger(), "reference #3 validation failed: '%s'", message.c_str());
      return false;
    }
  }

  {
    mrs_msgs::msg::ReferenceStamped msg;

    msg.header.frame_id      = "frame_that_does_not_exist";
    msg.reference.position.x = 0;
    msg.reference.position.y = 0;
    msg.reference.position.z = 2;
    msg.reference.heading    = 0;

    auto [success, message] = uh->validateReference(msg);

    if (success) {
      RCLCPP_ERROR(node_->get_logger(), "reference #4 validation failed: '%s'", message.c_str());
      return false;
    }
  }

  if (uh->isFlyingNormally()) {
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "not flying normally");
    return false;
  }
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
