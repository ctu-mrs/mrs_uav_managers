#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

#include <mrs_msgs/ConstraintsOverride.h>

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test();

  mrs_lib::ServiceClientHandler<mrs_msgs::ConstraintsOverride> sch_override_constraints_;

  Tester();
};

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  sch_override_constraints_ = mrs_lib::ServiceClientHandler<mrs_msgs::ConstraintsOverride>(nh_, "/" + _uav_name_ + "/constraint_manager/constraints_override");
}

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

  // | -------------- switch to medium constraints -------------- |

  {
    const std::string constraints = "medium";

    auto [success, message] = uh->setConstraints(constraints);

    if (!success) {
      ROS_ERROR("[%s]: failed to switch to '%s'", ros::this_node::getName().c_str(), constraints.c_str());
      return false;
    }
  }

  sleep(1.0);

  // | ---------------- override the constriants ---------------- |

  const double desired_horizontal_acc = 0.5;
  const double desired_vertical_acc   = 0.3;

  {
    mrs_msgs::ConstraintsOverride srv;

    srv.request.acceleration_horizontal = desired_horizontal_acc;
    srv.request.acceleration_vertical   = desired_vertical_acc;

    {
      bool service_call = sch_override_constraints_.call(srv);

      if (!service_call || !srv.response.success) {
        ROS_ERROR("[%s]: constraints override service call failed", ros::this_node::getName().c_str());
        return false;
      }
    }
  }

  sleep(3.0);

  // | ----------------------- check constraints ---------------------- |

  const double horizontal_acceleration    = uh->getCurrentConstraints()->horizontal_acceleration;
  const double vertical_asc_acceleration  = uh->getCurrentConstraints()->vertical_ascending_acceleration;
  const double vertical_desc_acceleration = uh->getCurrentConstraints()->vertical_descending_acceleration;
  ;

  if (std::abs(horizontal_acceleration - desired_horizontal_acc) > 0.1 || std::abs(vertical_asc_acceleration - desired_vertical_acc) > 0.1 ||
      std::abs(vertical_desc_acceleration - desired_vertical_acc) > 0.1) {
    ROS_ERROR("[%s]: constraints do not match", ros::this_node::getName().c_str());
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
