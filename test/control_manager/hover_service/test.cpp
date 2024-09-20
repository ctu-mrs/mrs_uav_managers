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

  {
    auto [success, message] = uh->activateMidAir();

    if (!success) {
      ROS_ERROR("[%s]: midair activation failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ---------------------- goto relative --------------------- |

  {
    auto [success, message] = uh->gotoRelativeService(200, 0, 0, 0);

    if (!success) {
      ROS_ERROR("[%s]: goto relative service failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | -------------------------- wait -------------------------- |

  this->sleep(5.0);

  // | ---------------------- trigger hover --------------------- |

  {
    auto [success, message] = uh->hover();

    if (!success) {
      ROS_ERROR("[%s]: hover service failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | -------------------------- wait -------------------------- |

  this->sleep(5.0);

  // | --------------- check if we are stationary --------------- |

  if (uh->isFlyingNormally() && uh->isStationary()) {
    return true;
  } else {
    ROS_ERROR("[%s]: not flying normally || not stationary", ros::this_node::getName().c_str());
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
