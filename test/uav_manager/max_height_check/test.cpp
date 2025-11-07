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

  {
    auto [success, message] = uh->activateMidAir();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "activation failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // | --------------- goto to violate min height --------------- |

  {
    auto [success, message] = uh->gotoAbs(0, 0, 100, 0);

    if (success) {
      RCLCPP_ERROR(node_->get_logger(), "goto should fail");
      return false;
    }
  }

  // | --------- wait till we are flying normally again --------- |

  while (true) {

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e3, "waiting till it flies normally again");

    if (!rclcpp::ok()) {
      return false;
    }

    if (uh->isFlyingNormally()) {
      break;
    }
  }

  // | ------------------- check the altitude ------------------- |

  sleep(1.0);

  if (!uh->sh_max_height_.hasMsg()) {
    RCLCPP_ERROR(node_->get_logger(), "missing max height msgs");
    return false;
  }

  double max_height_agl = uh->sh_max_height_.getMsg()->value;

  auto height = uh->getHeightAgl();

  if (height) {
    if (height.value() < max_height_agl) {
      return true;
    }
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
