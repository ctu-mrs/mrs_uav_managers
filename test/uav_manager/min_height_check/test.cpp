#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {

    _uav_name_ = "uav1";

    sch_min_height_check_ = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "/" + _uav_name_ + "/uav_manager/enable_min_height_check");
  }

  bool test(void);

  double _min_height_offset_;
  double _min_height_;

  std::string _uav_name_;

  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> sch_min_height_check_;

  bool toggleMinHeightCheck(const bool in);
};

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

  pl_->loadParam("mrs_uav_managers/uav_manager/min_height_checking/safety_height_offset", _min_height_offset_);
  pl_->loadParam("mrs_uav_managers/uav_manager/min_height_checking/min_height", _min_height_);

  if (!pl_->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load parameters");
    return false;
  }

  {
    auto [success, message] = uh->activateMidAir();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "midair activation failed with message: '%s'", message.c_str());
      return false;
    }
  }

  sleep(3.0);

  // | -------------- disable the min-height check -------------- |

  {
    RCLCPP_INFO(node_->get_logger(), "disabling min height check");

    bool success = toggleMinHeightCheck(false);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "failed to disable the min-height check");
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "min height check disabled");
  }

  sleep(1.0);

  // | --------------- goto to violate min height --------------- |

  {
    RCLCPP_INFO(node_->get_logger(), "going to [0, 0, 0.5, 0]");

    auto [success, message] = uh->gotoAbs(0, 0, 0.5, 0);

    RCLCPP_INFO(node_->get_logger(), "goto suceeded");

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "failed to descend");
      return false;
    }
  }

  // | --------------- enable the min-height check -------------- |

  {
    RCLCPP_INFO(node_->get_logger(), "enabling min height check");

    bool success = toggleMinHeightCheck(true);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "failed to enable the min-height check");
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "min height check enabled");
  }

  sleep(1.0);

  // | ------------- check if we are flying normally ------------ |

  if (uh->isFlyingNormally()) {
    RCLCPP_ERROR(node_->get_logger(), "we are still flying normally");
    return false;
  }

  // | --------- wait till we are flying normally again --------- |

  while (true) {

    RCLCPP_INFO(node_->get_logger(), "waiting till flying normally");

    if (!rclcpp::ok()) {
      return false;
    }

    if (uh->isFlyingNormally()) {
      break;
    }

    sleep(0.1);
  }

  RCLCPP_INFO(node_->get_logger(), "we are flying normally again");

  // | --------------- goto to violate min height --------------- |

  {
    RCLCPP_INFO(node_->get_logger(), "going to [0, 0, 0, 0]");

    auto [success, message] = uh->gotoAbs(0, 0, 0, 0);

    if (success) {
      RCLCPP_ERROR(node_->get_logger(), "goto should fail");
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "goto sucessfully failed");
  }

  // | --------- wait till we are flying normally again --------- |

  while (true) {

    RCLCPP_INFO(node_->get_logger(), "waiting till flying normally");

    if (!rclcpp::ok()) {
      return false;
    }

    if (uh->isFlyingNormally()) {
      break;
    }

    sleep(0.1);
  }

  RCLCPP_INFO(node_->get_logger(), "we are flying normally again");

  // | ------------------- check the altitude ------------------- |

  RCLCPP_INFO(node_->get_logger(), "checking our AGL");

  auto height = uh->getHeightAgl();

  if (height) {

    RCLCPP_INFO(node_->get_logger(), "AGL obtained");

    if (height.value() > _min_height_) {

      RCLCPP_INFO(node_->get_logger(), "AGL ok");

      return true;

    } else {

      RCLCPP_ERROR(node_->get_logger(), "AGL bad");

      return false;
    }

  } else {

    RCLCPP_ERROR(node_->get_logger(), "AGL cound not be obtained");

    return false;
  }

  return false;
}

bool Tester::toggleMinHeightCheck(const bool in) {

  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data                                            = in;

  auto response = sch_min_height_check_.callSync(request);

  if (!response || !response.value()->success) {
    RCLCPP_ERROR(node_->get_logger(), "failed to call the service for min height check");
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
