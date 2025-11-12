#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

class FailedTakeoffTest : public mrs_uav_testing::TestGeneric {

public:
  bool test();

  FailedTakeoffTest() : mrs_uav_testing::TestGeneric() {
  }
};

bool FailedTakeoffTest::test() {

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

  // | ---------------- wait for ready to takeoff --------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return false;
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "waiting for the MRS UAV System");

    if (uh->mrsSystemReady()) {
      RCLCPP_INFO(node_->get_logger(), "MRS UAV System is ready");
      break;
    }

    uh->sleep(0.01);
  }

  // | ---------------------- arm the drone --------------------- |

  RCLCPP_INFO(node_->get_logger(), "arming the drone");

  {
    std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data                                            = true;

    {
      auto response = uh->sch_arming_.callSync(request);

      if (!response || !response.value()->success) {
        return false;
      }
    }
  }

  // | ---------------------- wait a second --------------------- |

  sleep(2.0);

  // | --------------------- check if armed --------------------- |

  if (!uh->sh_hw_api_status_.getMsg()->armed) {
    return false;
  }

  // | ------------------- switch to offboard ------------------- |

  {
    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    {
      auto response = uh->sch_offboard_.callSync(request);

      if (!response || !response.value()->success) {
        return false;
      }
    }
  }

  // | -------------------------- wait -------------------------- |

  sleep(2.0);

  // | ------------------ check if in offboard ------------------ |

  if (!uh->sh_hw_api_status_.getMsg()->offboard) {
    return false;
  }

  sleep(2.0);

  // | ------------------------- takeoff ------------------------ |

  {
    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    {
      auto response = uh->sch_takeoff_.callSync(request);

      if (!response || response.value()->success) {
        RCLCPP_ERROR(node_->get_logger(), "takeoff call success, this should not happen");
        return false;
      }
    }
  }

  // | -------------------- check if disarmed ------------------- |

  sleep(2.0);

  if (uh->sh_hw_api_status_.getMsg()->armed) {
    RCLCPP_ERROR(node_->get_logger(), "the uav is still armed");
    return false;
  }

  return true;
}
