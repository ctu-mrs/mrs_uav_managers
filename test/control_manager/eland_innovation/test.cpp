#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

#define POS_JUMP_SIZE 4.0

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {

    sh_hw_api_odom_ =
        mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(*shopts_, "/" + _uav_name_ + "/hw_api/odometry_unchanged", &Tester::callbackOdometry, this);

    ph_odometry_ = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(node_, "/" + _uav_name_ + "/hw_api/odometry");
  }

  const std::string _uav_name_ = "uav1";

  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> sh_hw_api_odom_;

  mrs_lib::PublisherHandler<nav_msgs::msg::Odometry> ph_odometry_;

  bool test(void);

  void callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  std::atomic<bool> odom_jumped_ = false;
};

void Tester::callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  nav_msgs::msg::Odometry odom_out = *msg;

  if (odom_jumped_) {
    odom_out.pose.pose.position.x += POS_JUMP_SIZE;
  }

  ph_odometry_.publish(odom_out);
}

bool Tester::test(void) {

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh;

  {
    auto [uhopt, message] = getUAVHandler(_uav_name_);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", _uav_name_.c_str(), message.c_str());
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

  sleep(1.0);

  odom_jumped_ = true;

  // | -------------- wait for the eland to trigger ------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return false;
    }

    if (!uh->isFlyingNormally() && uh->getActiveController() == "EmergencyController" && uh->getActiveTracker() == "LandoffTracker") {
      break;
    }

    sleep(0.01);
  }

  // | -------------------- wait for landing -------------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return false;
    }

    if (!uh->isOutputEnabled()) {
      return true;
    }

    sleep(0.01);
  }

  return false;
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
