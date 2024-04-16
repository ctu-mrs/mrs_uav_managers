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

  // TOOD should also work with midair activation
  {
    auto [success, message] = uh->takeoff();

    if (!success) {
      ROS_ERROR("[%s]: takeoff failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // TODO should not be needed
  sleep(1.0);

  std::list<std::string> estimators = {"gps_garmin", "rtk", "rtk_garmin", "gps_baro"};

  for (auto estimator : estimators) {
    {
      auto [success, message] = uh->switchEstimator(estimator);

      if (!success) {
        ROS_ERROR("[%s]: failed to switch the estimator '%s', message: '%s'", ros::this_node::getName().c_str(), estimator.c_str(), message.c_str());
        return false;
      }
    }

    sleep(3.0);

    if (uh->getActiveEstimator() != estimator) {
      ROS_ERROR("[%s]: '%s' estimator not active", ros::this_node::getName().c_str(), estimator.c_str());
      return false;
    }

    {
      auto [success, message] = uh->gotoRel(5.0, 0, 0, 0);

      if (!success) {
        ROS_ERROR("[%s]: goto failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
        return false;
      }
    }
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
