#include <mrs_uav_testing/test_generic.h>

class EscalatingFailsafeTest : public mrs_uav_testing::TestGeneric {

public:
  bool test();

  EscalatingFailsafeTest() : mrs_uav_testing::TestGeneric() {};

  virtual std::optional<std::tuple<bool, std::string>> escalatingFailsafe() = 0;

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh_;

  const std::string _uav_name_ = "uav1";
};

bool EscalatingFailsafeTest::test() {

  {
    auto [uhopt, message] = getUAVHandler(_uav_name_);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", _uav_name_.c_str(), message.c_str());
      return false;
    }

    uh_ = uhopt.value();
  }

  {
    auto [success, message] = uh_->activateMidAir();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "midair activation failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // | ----------------------- goto higher ---------------------- |

  {
    auto [success, message] = uh_->gotoAbs(0, 0, 15.0, 0);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "goto failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // | ------------------- trigger failsafe #1 ------------------ |

  {
    auto result = this->escalatingFailsafe();

    if (result) {

      auto [success, message] = result.value();

      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "escalating failsafe call #1 failed with message: '%s'", message.c_str());
        return false;
      }
    }
  }

  sleep(0.5);

  // | ----------- try again quickly, this should fail ---------- |

  {
    auto result = this->escalatingFailsafe();

    if (result) {

      auto [success, message] = result.value();

      if (success) {
        RCLCPP_ERROR(node_->get_logger(), "escalating failsafe call #1.5 suceeded with message: '%s'", message.c_str());
        return false;
      }
    }
  }

  // | -------------------- check for ehover -------------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return false;
    }

    if (!uh_->isFlyingNormally() && uh_->getActiveController() == "EmergencyController" && uh_->getActiveTracker() == "LandoffTracker") {
      break;
    }

    sleep(0.01);
  }

  sleep(3.0);

  // check if we have not moved

  if (!uh_->isAtPosition(0, 0, 15.0, 0, 0.5)) {
    RCLCPP_ERROR(node_->get_logger(), "we moved after ehover");
    return false;
  }

  // | ------------------- trigger failsafe #2 ------------------ |

  {
    auto result = this->escalatingFailsafe();

    if (result) {

      auto [success, message] = result.value();

      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "escalating failsafe call #2 failed with message: '%s'", message.c_str());
        return false;
      }
    }
  }

  sleep(0.5);

  // | ----------- try again quickly, this should fail ---------- |

  {
    auto result = this->escalatingFailsafe();

    if (result) {

      auto [success, message] = result.value();

      if (success) {
        RCLCPP_ERROR(node_->get_logger(), "escalating failsafe call #2.5 suceeded with message: '%s'", message.c_str());
        return false;
      }
    }
  }

  // | -------------------- wait few seconds -------------------- |

  sleep(2.0);

  // | ---------------- check if we are elanding ---------------- |

  auto uav_state = uh_->sh_uav_state_.getMsg();

  if (!(!uh_->isFlyingNormally() && uh_->getActiveController() == "EmergencyController" && uh_->getActiveTracker() == "LandoffTracker" &&
        uav_state->velocity.linear.z < -0.3)) {
    RCLCPP_ERROR(node_->get_logger(), "we are not elanding");
    return false;
  }

  // | ------------------- trigger failsafe #3 ------------------ |

  {
    auto result = this->escalatingFailsafe();

    if (result) {

      auto [success, message] = result.value();

      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "escalating failsafe call #3 failed with message: '%s'", message.c_str());
        return false;
      }
    }
  }

  sleep(0.5);

  // | ----------- try again quickly, this should fail ---------- |

  {
    auto result = this->escalatingFailsafe();

    if (result) {

      auto [success, message] = result.value();

      if (success) {
        RCLCPP_ERROR(node_->get_logger(), "escalating failsafe call #3.5 suceeded with message: '%s'", message.c_str());
        return false;
      }
    }
  }

  // | ------------ wait for the failsafe to trigger ------------ |

  sleep(2.0);

  if (!(!uh_->isFlyingNormally() && uh_->getActiveController() == "FailsafeController" && uav_state->velocity.linear.z < -0.3)) {
    RCLCPP_ERROR(node_->get_logger(), "we are not in failsafe");
    return false;
  }

  sleep(0.5);

  // | ----------- try again quickly, this should fail ---------- |

  {
    auto result = this->escalatingFailsafe();

    if (result) {

      auto [success, message] = result.value();

      if (success) {
        RCLCPP_ERROR(node_->get_logger(), "escalating failsafe call #4 suceeded with message: '%s'", message.c_str());
        return false;
      }
    }
  }

  // | -------------------- wait for landing -------------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return false;
    }

    if (!uh_->isOutputEnabled()) {
      return true;
    }

    sleep(0.01);
  }

  return false;
}
