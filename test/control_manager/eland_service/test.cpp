#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

/* class Tester //{ */

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test();

  Tester();

  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_eland_;

  std::tuple<bool, std::string> eland();
};

Tester::Tester() {

  sch_eland_ = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/control_manager/eland");
}

std::tuple<bool, std::string> Tester::eland() {

  {
    std_srvs::Trigger srv;

    {
      bool service_call = sch_eland_.call(srv);

      if (!service_call || !srv.response.success) {
        return {false, "offboard service call failed"};
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

  // | ---------------------- trigger eland --------------------- |

  {
    auto [success, message] = eland();

    if (!success) {
      ROS_ERROR("[%s]: eland call failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | -------------- wait for the eland to trigger ------------- |

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    if (!isFlyingNormally() && getActiveController() == "EmergencyController" && getActiveTracker() == "LandoffTracker") {
      break;
    }

    sleep(0.01);
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
