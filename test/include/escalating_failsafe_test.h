#include <mrs_uav_testing/test_generic.h>

class EscalatingFailsafeTest : public mrs_uav_testing::TestGeneric {

public:
  bool test();

  EscalatingFailsafeTest() : mrs_uav_testing::TestGeneric(){};

  virtual std::optional<std::tuple<bool, std::string>> escalatingFailsafe() = 0;
};

bool EscalatingFailsafeTest::test() {

  {
    auto [success, message] = activateMidAir();

    if (!success) {
      ROS_ERROR("[%s]: midair activation failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ----------------------- goto higher ---------------------- |

  {
    auto [success, message] = this->gotoAbs(0, 0, 15.0, 0);

    if (!success) {
      ROS_ERROR("[%s]: goto failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ------------------- trigger failsafe #1 ------------------ |

  {
    auto result = escalatingFailsafe();

    if (result) {

      auto [success, message] = result.value();

      if (!success) {
        ROS_ERROR("[%s]: escalating failsafe call #1 failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
        return false;
      }
    }
  }

  sleep(0.5);

  // | ----------- try again quickly, this should fail ---------- |

  {
    auto result = escalatingFailsafe();

    if (result) {

      auto [success, message] = result.value();

      if (success) {
        ROS_ERROR("[%s]: escalating failsafe call #1.5 suceeded with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
        return false;
      }
    }
  }

  // | -------------------- check for ehover -------------------- |

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    if (!isFlyingNormally() && getActiveController() == "EmergencyController" && getActiveTracker() == "LandoffTracker") {
      break;
    }

    sleep(0.01);
  }

  sleep(3.0);

  // check if we have not moved

  if (!isAtPosition(0, 0, 15.0, 0, 0.5)) {
    ROS_ERROR("[%s]: we moved after ehover", ros::this_node::getName().c_str());
    return false;
  }

  // | ------------------- trigger failsafe #2 ------------------ |

  {
    auto result = escalatingFailsafe();

    if (result) {

      auto [success, message] = result.value();

      if (!success) {
        ROS_ERROR("[%s]: escalating failsafe call #2 failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
        return false;
      }
    }
  }

  sleep(0.5);

  // | ----------- try again quickly, this should fail ---------- |

  {
    auto result = escalatingFailsafe();

    if (result) {

      auto [success, message] = result.value();

      if (success) {
        ROS_ERROR("[%s]: escalating failsafe call #2.5 suceeded with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
        return false;
      }
    }
  }

  // | -------------------- wait few seconds -------------------- |

  sleep(1.5);

  // | ---------------- check if we are elanding ---------------- |

  auto uav_state = this->sh_uav_state_.getMsg();

  if (!(!isFlyingNormally() && getActiveController() == "EmergencyController" && getActiveTracker() == "LandoffTracker" &&
        uav_state->velocity.linear.z < -0.3)) {
    ROS_ERROR("[%s]: we are not elanding", ros::this_node::getName().c_str());
    return false;
  }

  // | ------------------- trigger failsafe #3 ------------------ |

  {
    auto result = escalatingFailsafe();

    if (result) {

      auto [success, message] = result.value();

      if (!success) {
        ROS_ERROR("[%s]: escalating failsafe call #3 failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
        return false;
      }
    }
  }

  sleep(0.5);

  // | ----------- try again quickly, this should fail ---------- |

  {
    auto result = escalatingFailsafe();

    if (result) {

      auto [success, message] = result.value();

      if (success) {
        ROS_ERROR("[%s]: escalating failsafe call #3.5 suceeded with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
        return false;
      }
    }
  }

  // | ------------ wait for the failsafe to trigger ------------ |

  sleep(1.5);

  if (!(!isFlyingNormally() && getActiveController() == "FailsafeController" && uav_state->velocity.linear.z < -0.3)) {
    ROS_ERROR("[%s]: we are not in failsafe", ros::this_node::getName().c_str());
    return false;
  }

  sleep(0.5);

  // | ----------- try again quickly, this should fail ---------- |

  {
    auto result = escalatingFailsafe();

    if (result) {

      auto [success, message] = result.value();

      if (success) {
        ROS_ERROR("[%s]: escalating failsafe call #4 suceeded with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
        return false;
      }
    }
  }

  // | -------------------- wait for landing -------------------- |

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    if (!this->isOutputEnabled()) {
      return true;
    }

    sleep(0.01);
  }

  return false;
}
