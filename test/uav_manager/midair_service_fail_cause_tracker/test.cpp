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

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh;

  const std::string uav_name = "uav1";

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
      return false;
    }

    uh = uhopt.value();
  }

  // | ---------- call midair activation in async task ---------- |

  {
    auto [success, message] = uh->activateMidAir();

    if (success) {
      RCLCPP_ERROR(node_->get_logger(), "midair activation succeeded, this should not happen: '%s'", message.c_str());
      return false;
    }
  }

  // | -------------------------- wait -------------------------- |

  sleep(1.0);

  // | -------------- check if we are in emergency -------------- |

  if (!uh->isFlyingNormally() && uh->getActiveController() == "EmergencyController" && uh->getActiveTracker() == "LandoffTracker") {
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "not in the ehover state");
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
