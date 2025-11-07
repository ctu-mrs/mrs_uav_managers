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

  sleep(1.0);

  // | -------------- switch to medium constraints -------------- |

  {
    const std::string constraints = "medium";

    auto [success, message] = uh->setConstraints(constraints);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "failed to switch to '%s'", constraints.c_str());
      return false;
    }
  }

  sleep(1.0);

  // | ---------------- override the constriants ---------------- |

  const double desired_horizontal_acc = 0.5;
  const double desired_vertical_acc   = 0.3;

  {
    auto [success, message] = uh->overrideConstraints(desired_horizontal_acc, desired_vertical_acc);

    if (!success) {

      RCLCPP_ERROR(node_->get_logger(), "constraints override service failed: '%s'", message.c_str());
      return false;
    }
  }

  sleep(3.0);

  // | ----------------------- check constraints ---------------------- |

  const double horizontal_acceleration    = uh->getCurrentConstraints()->horizontal_acceleration;
  const double vertical_asc_acceleration  = uh->getCurrentConstraints()->vertical_ascending_acceleration;
  const double vertical_desc_acceleration = uh->getCurrentConstraints()->vertical_descending_acceleration;
  ;

  if (std::abs(horizontal_acceleration - desired_horizontal_acc) > 0.1 || std::abs(vertical_asc_acceleration - desired_vertical_acc) > 0.1 ||
      std::abs(vertical_desc_acceleration - desired_vertical_acc) > 0.1) {
    RCLCPP_ERROR(node_->get_logger(), "constraints do not match");
    return false;
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
