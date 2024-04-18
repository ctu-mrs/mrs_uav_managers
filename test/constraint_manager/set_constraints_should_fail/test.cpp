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

  // | --------------- remember the current constraints --------------- |

  const std::string old_constraints = uh->sh_constraint_manager_diag_.getMsg()->current_name;

  // | -------------------- try to set constraints -------------------- |

  {
    const std::string constraints = "do_not_exists";

    auto [success, message] = uh->setConstraints(constraints);

    if (success) {
      ROS_ERROR("[%s]: switching to constraints '%s' secceeded, which should not happen", ros::this_node::getName().c_str(), constraints.c_str());
      return false;
    }
  }

  sleep(1.0);

  // | ----------------------- check constraints ---------------------- |

  if (uh->sh_constraint_manager_diag_.getMsg()->current_name != old_constraints) {
    ROS_ERROR("[%s]: constraints changed even though they should not", ros::this_node::getName().c_str());
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
