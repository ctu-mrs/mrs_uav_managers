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

  sleep(1.0);

  // | ------------------------ set constraints ----------------------- |

  const std::string desired_constraints = "medium";

  {
    auto [success, message] = uh->setConstraints(desired_constraints);

    if (!success) {
      ROS_ERROR("[%s]: failed to switch constraints to '%s', message: '%s'", ros::this_node::getName().c_str(), desired_constraints.c_str(), message.c_str());
      return false;
    }
  }

  sleep(1.0);

  // | ----------------------- check constraints ---------------------- |

  if (uh->sh_constraint_manager_diag_.getMsg()->current_name != desired_constraints) {
    ROS_ERROR("[%s]: constraints not set", ros::this_node::getName().c_str());
    return false;
  }

  if (uh->isFlyingNormally()) {
    return true;
  } else {
    ROS_ERROR("[%s]: not flying normally", ros::this_node::getName().c_str());
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
