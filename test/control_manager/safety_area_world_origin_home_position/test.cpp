#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {
  }

  bool test(void);

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh_;
};

bool Tester::test(void) {

  const std::string uav_name = "uav1";

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
      return false;
    }

    uh_ = uhopt.value();
  }

  // takeoff() is gated on UavManager's world_origin readiness check, so by the time it returns the
  // home position has been captured, adopted as the world_origin, and the safety area re-centered on
  // it - the precondition for the boundary checks below (activateMidAir does not gate on this)
  {
    auto [success, message] = uh_->takeoff();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "takeoff failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // the safety area is a +-50 m square in the world_origin frame; with use_home_position the
  // world_origin is pinned to home, so home (0,0) must be inside the fence and a point beyond
  // +-50 must be outside. A fence shifted away from home (the bug this guards) would flip these.

  // | ------------- home itself must be inside the fence ------------- |
  {
    mrs_msgs::msg::ReferenceStamped msg;

    msg.header.frame_id      = uav_name + "/world_origin";
    msg.reference.position.x = 0;
    msg.reference.position.y = 0;
    msg.reference.position.z = 3;
    msg.reference.heading    = 0;

    auto [success, message] = uh_->validateReference(msg);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "home (0,0) should be inside the fence but was rejected: '%s'", message.c_str());
      return false;
    }
  }

  // | -------------- a point well inside the +-50 fence -------------- |
  {
    mrs_msgs::msg::ReferenceStamped msg;

    msg.header.frame_id      = uav_name + "/world_origin";
    msg.reference.position.x = 40;
    msg.reference.position.y = 40;
    msg.reference.position.z = 3;
    msg.reference.heading    = 0;

    auto [success, message] = uh_->validateReference(msg);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "(40,40) should be inside the fence but was rejected: '%s'", message.c_str());
      return false;
    }
  }

  // | -------------- a point just outside the +-50 fence ------------- |
  {
    mrs_msgs::msg::ReferenceStamped msg;

    msg.header.frame_id      = uav_name + "/world_origin";
    msg.reference.position.x = 60;
    msg.reference.position.y = 60;
    msg.reference.position.z = 3;
    msg.reference.heading    = 0;

    auto [success, message] = uh_->validateReference(msg);

    if (success) {
      RCLCPP_ERROR(node_->get_logger(), "(60,60) is beyond the +-50 fence but was accepted");
      return false;
    }
  }

  // | --------------------- a point far outside ---------------------- |
  {
    mrs_msgs::msg::ReferenceStamped msg;

    msg.header.frame_id      = uav_name + "/world_origin";
    msg.reference.position.x = 100;
    msg.reference.position.y = 0;
    msg.reference.position.z = 3;
    msg.reference.heading    = 0;

    auto [success, message] = uh_->validateReference(msg);

    if (success) {
      RCLCPP_ERROR(node_->get_logger(), "(100,0) is far beyond the fence but was accepted");
      return false;
    }
  }

  if (uh_->isFlyingNormally()) {
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
