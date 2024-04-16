#include <mrs_uav_testing/test_generic.h>

class EscalatingFailsafeTest : public mrs_uav_testing::TestGeneric {

public:
  bool test();

  EscalatingFailsafeTest() : mrs_uav_testing::TestGeneric(){};

  virtual std::optional<std::tuple<bool, std::string>> escalatingFailsafe() = 0;
};

bool EscalatingFailsafeTest::test() {

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh;

  {
    auto [uhopt, message] = getUAVHandler(_uav_name_);

    if (!uhopt) {
      ROS_ERROR("[%s]: Failed obtain handler for '%s': '%s'", ros::this_node::getName().c_str(), _uav_name_.c_str(), message.c_str());
      return false;
    }

    uh = uhopt.value();
  }

  {
    auto [success, message] = uh->activateMidAir();

    if (!success) {
      ROS_ERROR("[%s]: midair activation failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ----------------------- goto higher ---------------------- |

  {
    auto [success, message] = uh->gotoAbs(0, 0, 15.0, 0);

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

    if (!uh->isFlyingNormally() && uh->getActiveController() == "EmergencyController" && uh->getActiveTracker() == "LandoffTracker") {
      break;
    }

    sleep(0.01);
  }

  sleep(3.0);

  // check if we have not moved

  if (!uh->isAtPosition(0, 0, 15.0, 0, 0.5)) {
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

  auto uav_state = uh->sh_uav_state_.getMsg();

  if (!(!uh->isFlyingNormally() && uh->getActiveController() == "EmergencyController" && uh->getActiveTracker() == "LandoffTracker" &&
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

  if (!(!uh->isFlyingNormally() && uh->getActiveController() == "FailsafeController" && uav_state->velocity.linear.z < -0.3)) {
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

    if (!uh->isOutputEnabled()) {
      return true;
    }

    sleep(0.01);
  }

  return false;
}
