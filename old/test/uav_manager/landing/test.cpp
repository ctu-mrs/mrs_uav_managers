#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

#include <nav_msgs/Odometry.h>

#define POS_JUMP_SIZE 3.0

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test();

  Tester();

  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_land_;
};

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  sch_land_ = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/uav_manager/land");
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

  // | -------------- wait for the eland to trigger ------------- |

  {
    std_srvs::Trigger srv;

    {
      bool service_call = sch_land_.call(srv);

      if (!service_call || !srv.response.success) {
        ROS_ERROR("[%s]: land service call failed", ros::this_node::getName().c_str());
        return false;
      }
    }
  }

  sleep(0.5);

  // | -------------------- wait for landing -------------------- |

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    if (!uh->isOutputEnabled()) {

      return true;

    } else {

      // TODO this is not the right way to check if the landing actually happens as planned
      if (uh->getActiveTracker() != "LandoffTracker" && uh->getActiveTracker() != "NullTracker") {
        ROS_ERROR("[%s]: not landing anymore", ros::this_node::getName().c_str());
        return false;
      }
    }
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
