#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

#include <mrs_msgs/srv/float64_srv.hpp>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {

    sch_set_mass_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::Float64Srv>(node_, "/multirotor_simulator/uav1/set_mass");
  }

  mrs_lib::ServiceClientHandler<mrs_msgs::srv::Float64Srv> sch_set_mass_;

  bool setMass(const double mass);

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
      RCLCPP_ERROR(node_->get_logger(), "activation failed with message: '%s'", message.c_str());
      return false;
    }
  }

  sleep(1.0);

  // | ---------------- slowly increase the mass ---------------- |

  for (int i = 0; i < 10; i++) {

    bool res = setMass(2.0 + 0.5 * i);

    if (!res) {
      RCLCPP_ERROR(node_->get_logger(), "to set the mass");
      return false;
    }

    sleep(2.0);
  }

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

bool Tester::setMass(const double mass) {

  std::shared_ptr<mrs_msgs::srv::Float64Srv::Request> request = std::make_shared<mrs_msgs::srv::Float64Srv::Request>();
  request->value                                              = mass;

  auto response = sch_set_mass_.callSync(request);

  if (!response || !response.value()->success) {
    RCLCPP_ERROR(node_->get_logger(), "to call the service for setting mass");
    return false;
  } else {
    return true;
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
