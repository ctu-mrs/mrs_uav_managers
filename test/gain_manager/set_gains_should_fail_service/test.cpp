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

  // | --------------- remember the current gains --------------- |

  const std::string old_gains = uh->sh_gain_manager_diag_.getMsg()->current_name;

  // | ------------------------ set gains ----------------------- |

  const std::string desired_gains = "do_not_exist";

  {

    auto [success, message] = uh->setGains(desired_gains);

    if (success) {
      RCLCPP_ERROR(node_->get_logger(), "switching gains to '%s' succeeded, which should not happen: '%s'", desired_gains.c_str(), message.c_str());
      return false;
    }
  }

  sleep(1.0);

  // | ----------------------- check gains ---------------------- |

  if (uh->sh_gain_manager_diag_.getMsg()->current_name != old_gains) {
    RCLCPP_ERROR(node_->get_logger(), "gains changed even though they should not");
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
