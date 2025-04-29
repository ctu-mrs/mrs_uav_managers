#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

#include <mrs_msgs/Float64Srv.h>

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test();

  Tester();

  bool setMass(const double mass);

  mrs_lib::ServiceClientHandler<mrs_msgs::Float64Srv> sch_set_mass_;
};

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  sch_set_mass_ = mrs_lib::ServiceClientHandler<mrs_msgs::Float64Srv>(nh_, "/multirotor_simulator/uav1/set_mass");
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

  // | ---------------- slowly increase the mass ---------------- |

  for (int i = 0; i < 10; i++) {

    bool res = setMass(2.0 + 0.5 * i);

    if (!res) {
      ROS_ERROR("[%s]: failed to set the mass", ros::this_node::getName().c_str());
      return false;
    }

    sleep(2.0);
  }

  // | -------------- wait for the eland to trigger ------------- |

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    if (!uh->isFlyingNormally() && uh->getActiveController() == "EmergencyController" && uh->getActiveTracker() == "LandoffTracker") {
      break;
    }

    sleep(0.01);
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

bool Tester::setMass(const double mass) {

  mrs_msgs::Float64Srv srv;
  srv.request.value = mass;

  bool service_call = sch_set_mass_.call(srv);

  if (!service_call || !srv.response.success) {
    ROS_ERROR("[%s]: failed to call the service for setting mass", ros::this_node::getName().c_str());
    return false;
  } else {
    return true;
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
