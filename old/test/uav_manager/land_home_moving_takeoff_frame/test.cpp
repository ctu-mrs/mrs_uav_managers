#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

#include <mrs_msgs/Float64Srv.h>

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test();

  Tester();

  bool asyncSetGroundZ();

  mrs_lib::ServiceClientHandler<mrs_msgs::Float64Srv> sch_set_ground_z_;
};

Tester::Tester() {

  sch_set_ground_z_ = mrs_lib::ServiceClientHandler<mrs_msgs::Float64Srv>(nh_, "/multirotor_simulator/uav1/set_ground_z");
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

  // | ------------- wait for the system to be ready ------------ |

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    if (uh->mrsSystemReady()) {
      break;
    }
  }

  // | ---------------- save the current position --------------- |

  auto takeoff_pos = uh->sh_uav_state_.getMsg()->pose.position;
  auto takeoff_hdg = mrs_lib::AttitudeConverter(uh->sh_uav_state_.getMsg()->pose.orientation).getHeading();

  // | ------------------------ take off ------------------------ |

  {
    auto [success, message] = uh->takeoff();

    if (!success) {
      ROS_ERROR("[%s]: takeoff failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  sleep(1.0);

  // | --------------------- goto somewhere --------------------- |

  {
    auto [success, message] = uh->gotoRel(10, 0, 0, 0);

    if (!success) {
      ROS_ERROR("[%s]: goto failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  sleep(1.0);

  // | -------------------- switch estimator -------------------- |

  {
    auto [success, message] = uh->switchEstimator("gps_baro");

    if (!success) {
      ROS_ERROR("[%s]: failed to switch the estimator gps_baro, message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  sleep(1.0);

  // | -------- run async task for setting ground z later ------- |

  auto future_res = std::async(std::launch::async, &Tester::asyncSetGroundZ, this);

  // | ------------------------ land home ----------------------- |

  {
    auto [success, message] = uh->landHome();

    if (!success) {
      ROS_ERROR("[%s]: land home failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  if (!future_res.valid() || !future_res.get()) {
    ROS_ERROR("[%s]: was not able to set ground z", ros::this_node::getName().c_str());
    return false;
  }

  // | ---------------- check the final position ---------------- |

  if (uh->isAtPosition(takeoff_pos.x, takeoff_pos.y, takeoff_hdg, 0.5)) {
    return true;
  } else {
    ROS_ERROR("[%s]: land home did end in wrong place", ros::this_node::getName().c_str());
    return false;
  }
}

bool Tester::asyncSetGroundZ() {

  this->sleep(1.5);

  mrs_msgs::Float64Srv srv;
  srv.request.value = -1.0;

  bool service_call = sch_set_ground_z_.call(srv);

  if (!service_call || !srv.response.success) {
    ROS_ERROR("[%s]: failed to call the service for setting ground z", ros::this_node::getName().c_str());
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
