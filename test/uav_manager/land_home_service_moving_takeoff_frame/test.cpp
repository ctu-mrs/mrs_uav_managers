#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

#include <mrs_msgs/srv/float64_srv.hpp>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {

    sch_set_ground_z_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::Float64Srv>(node_, "/multirotor_simulator/uav1/set_ground_z");
  }

  bool test(void);

  bool asyncSetGroundZ();

  mrs_lib::ServiceClientHandler<mrs_msgs::srv::Float64Srv> sch_set_ground_z_;
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

  // | ---------------- save the current position --------------- |

  auto takeoff_pos = uh->sh_uav_state_.getMsg()->pose.position;
  auto takeoff_hdg = mrs_lib::AttitudeConverter(uh->sh_uav_state_.getMsg()->pose.orientation).getHeading();

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
    auto [success, message] = uh->gotoRel(10, 0, 0, 0);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "goto failed with message: '%s'", message.c_str());
      return false;
    }
  }

  sleep(1.0);

  // | -------------------- switch estimator -------------------- |

  {
    auto [success, message] = uh->switchEstimator("gps_baro");

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "failed to switch the estimator gps_baro, message: '%s'", message.c_str());
      return false;
    }
  }

  sleep(1.0);

  // | -------- run async task for setting ground z later ------- |

  auto future_res = std::async(std::launch::async, &Tester::asyncSetGroundZ, this);

  // | ------------------------ land home ----------------------- |

  {
    auto [success, message] = uh->landHome();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "land home failed with message: '%s'", message.c_str());
      return false;
    }
  }

  if (!future_res.valid() || !future_res.get()) {
    RCLCPP_ERROR(node_->get_logger(), "was not able to set ground z");
    return false;
  }

  // | ---------------- check the final position ---------------- |

  RCLCPP_INFO(node_->get_logger(), "pees");

  if (uh->isAtPosition(takeoff_pos.x, takeoff_pos.y, takeoff_hdg, 0.5)) {
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "land home did end in wrong place");
    return false;
  }
}

bool Tester::asyncSetGroundZ() {

  this->sleep(1.5);

  std::shared_ptr<mrs_msgs::srv::Float64Srv::Request> request = std::make_shared<mrs_msgs::srv::Float64Srv::Request>();

  request->value = -1.0;

  auto response = sch_set_ground_z_.callSync(request);

  if (!response || !response.value()->success) {
    RCLCPP_ERROR(node_->get_logger(), "failed to call the service for setting ground z");
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
