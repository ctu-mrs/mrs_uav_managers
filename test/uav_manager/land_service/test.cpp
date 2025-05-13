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

  {
    auto [uhopt, message] = getUAVHandler("uav1");

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", "uav1", message.c_str());
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

  // | -------------- wait for the land to trigger ------------- |

  {
    auto [success, message] = uh->land();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "calling for landing failed: '%s'", message.c_str());
      return false;
    }
  }

  sleep(0.5);

  // | -------------------- wait for landing -------------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return false;
    }

    if (!uh->isOutputEnabled()) {

      return true;

    } else {

      // TODO this is not the right way to check if the landing actually happens as planned
      if (uh->getActiveTracker() != "LandoffTracker" && uh->getActiveTracker() != "NullTracker") {
        RCLCPP_ERROR(node_->get_logger(), "not landing anymore");
        return false;
      }
    }
  }

  return false;
}

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Tester tester;

  test_result &= tester.test();

  tester.reportTestResult(test_result);

  tester.join();
}
