#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

/* class Tester //{ */

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test();

  Tester();

  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_esc_failsafe_;

  std::tuple<bool, std::string> escalatingFailsafe();
};

Tester::Tester() {

  sch_esc_failsafe_ = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/control_manager/failsafe_escalating");
}

std::tuple<bool, std::string> Tester::escalatingFailsafe() {

  {
    std_srvs::Trigger srv;

    {
      bool service_call = sch_esc_failsafe_.call(srv);

      if (!service_call || !srv.response.success) {
        return {false, "escalating failsafe service call failed"};
      }
    }
  }

  return {true, "service called"};
}

//}

bool Tester::test() {

  {
    auto [success, message] = activateMidAir();

    if (!success) {
      ROS_ERROR("[%s]: midair activation failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ----------------------- goto higher ---------------------- |

  {
    auto [success, message] = this->gotoAbs(0, 0, 10.0, 0);

    if (!success) {
      ROS_ERROR("[%s]: goto failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ------------------- trigger failsage #1 ------------------ |

  {
    auto [success, message] = escalatingFailsafe();

    if (!success) {
      ROS_ERROR("[%s]: escalating failsafe call #1 failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  sleep(0.5);

  // | ----------- try again quickly, this should fail ---------- |

  {
    auto [success, message] = escalatingFailsafe();

    if (success) {
      ROS_ERROR("[%s]: escalating failsafe call #1.5 succeeded with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
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

  if (!isAtPosition(0, 0, 10.0, 0, 0.5)) {
    ROS_ERROR("[%s]: we moved after ehover", ros::this_node::getName().c_str());
    return false;
  }

  // | ------------------- trigger failsage #2 ------------------ |

  {
    auto [success, message] = escalatingFailsafe();

    if (!success) {
      ROS_ERROR("[%s]: escalating failsafe call #2 failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  sleep(0.5);

  // | ----------- try again quickly, this should fail ---------- |

  {
    auto [success, message] = escalatingFailsafe();

    if (success) {
      ROS_ERROR("[%s]: escalating failsafe call #2.5 succeeded with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
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

  // | ------------------- trigger failsage #3 ------------------ |

  {
    auto [success, message] = escalatingFailsafe();

    if (!success) {
      ROS_ERROR("[%s]: escalating failsafe call #3 failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  sleep(0.5);

  // | ----------- try again quickly, this should fail ---------- |

  {
    auto [success, message] = escalatingFailsafe();

    if (success) {
      ROS_ERROR("[%s]: escalating failsafe call #3.5 succeeded with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ------------ wait for the failsafe to trigger ------------ |

  sleep(1.5);

  if (!(!isFlyingNormally() && getActiveController() == "FailsafeController" && uav_state->velocity.linear.z < -0.3)) {
    ROS_ERROR("[%s]: we are not in failsafe", ros::this_node::getName().c_str());
    return false;
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


TEST(TESTSuite, test) {

  Tester tester;

  bool result = tester.test();

  if (result) {
    GTEST_SUCCEED();
  } else {
    GTEST_FAIL();
  }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "test");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
