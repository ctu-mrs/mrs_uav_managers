#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test();
};

bool Tester::test() {

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh;

  {
    auto [uhopt, message] = getUAVHandler(_uav_name_);

    if (!uhopt) {
      ROS_ERROR("[%s]: Failed obtain handler for '%s': '%s'", ros::this_node::getName().c_str(), _uav_name_.c_str(), message.c_str());
      return false;
    }

    uh = uhopt.value();
  }

  // | ---------- call midair activation in async task ---------- |

  {
    auto [success, message] = uh->activateMidAir();

    if (success) {
      ROS_ERROR("[%s]: midair activation succeeded, this should not happen: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | -------------------------- wait -------------------------- |

  sleep(1.0);

  // | -------------- check if we are in emergency -------------- |

  if (!uh->isFlyingNormally() && uh->getActiveController() == "EmergencyController" && uh->getActiveTracker() == "LandoffTracker") {
    return true;
  } else {
    ROS_ERROR("[%s]: not in the ehover state", ros::this_node::getName().c_str());
    return false;
  }
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
