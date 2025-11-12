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

  // | ------------- wait for the system to be ready ------------ |

  while (true) {

    if (!rclcpp::ok()) {
      return false;
    }

    if (uh->mrsSystemReady()) {
      break;
    }
  }

  // | ------------------------ take off ------------------------ |

  {
    auto [success, message] = uh->takeoff();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "takeoff failed with message: '%s'", message.c_str());
      return false;
    }
  }

  sleep(1.0);

  // | --------------------- goto somewhere --------------------- |

  {
    auto [success, message] = uh->gotoAbs(10, 15, 5, 1.2);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "goto failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // | ----------------------- land there ----------------------- |

  double des_x   = -10;
  double des_y   = -15;
  double des_hdg = 3.14;

  {
    auto [success, message] = uh->landThere(des_x, des_y, des_hdg);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "land there failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // | ---------------- check the final position ---------------- |

  if (uh->isAtPosition(des_x, des_y, 0, des_hdg, 0.5)) {
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "land there did end in wrong place");
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
