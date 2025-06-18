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

  // | ---------------------- goto relative --------------------- |

  {
    auto [success, message] = uh->gotoRelativeService(200, 0, 0, 0);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "goto relative service failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // | -------------------------- wait -------------------------- |

  this->sleep(5.0);

  // | ---------------------- trigger hover --------------------- |

  {
    auto [success, message] = uh->hover();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "hover service failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // | -------------------------- wait -------------------------- |

  this->sleep(5.0);

  // | --------------- check if we are stationary --------------- |

  if (uh->isFlyingNormally() && uh->isStationary()) {
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "not flying normally || not stationary");
    return false;
  }
}

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Tester tester;

  test_result &= tester.test();

  tester.sleep(2.0);

  std::cout << "Test: reporting test results" << std::endl;

  tester.reportTestResult(test_result);

  tester.join();
}
