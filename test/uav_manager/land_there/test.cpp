#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>
#include <mrs_msgs/ReferenceStampedSrv.h>

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test();

  Tester();

  mrs_lib::ServiceClientHandler<mrs_msgs::ReferenceStampedSrv> sch_land_there_;

  std::tuple<bool, std::string> landThere(const double x, const double y, const double z, const double hdg);

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh_;
};

Tester::Tester() {

  sch_land_there_ = mrs_lib::ServiceClientHandler<mrs_msgs::ReferenceStampedSrv>(nh_, "/" + _uav_name_ + "/uav_manager/land_there");
}

/* landThere() //{ */

std::tuple<bool, std::string> Tester::landThere(const double x, const double y, const double z, const double hdg) {

  if (!uh_->isFlyingNormally()) {
    return {false, "not flying normally in the beginning"};
  }

  // | ----------------- call land there service ----------------- |

  ROS_INFO("[%s]: calling for landing there", name_.c_str());

  {
    mrs_msgs::ReferenceStampedSrv srv;

    srv.request.reference.position.x = x;
    srv.request.reference.position.y = y;
    srv.request.reference.position.z = z;
    srv.request.reference.heading    = hdg;

    {
      bool service_call = sch_land_there_.call(srv);

      if (!service_call || !srv.response.success) {
        return {false, "land there service call failed"};
      }
    }
  }

  // | ---------------------- wait a second --------------------- |

  sleep(1.0);

  // | -------- wait till the right controller is active -------- |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    if (uh_->sh_control_manager_diag_.getMsg()->active_tracker == "LandoffTracker" &&
        uh_->sh_control_manager_diag_.getMsg()->active_controller == "MpcController") {
      break;
    }

    sleep(0.01);
  }

  // | ------------- wait for the landing to finish ------------- |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the landing to finish", name_.c_str());

    if (!uh_->isOutputEnabled()) {

      return {true, "landing finished"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

bool Tester::test() {

  {
    auto [uhopt, message] = getUAVHandler(_uav_name_);

    if (!uhopt) {
      ROS_ERROR("[%s]: Failed obtain handler for '%s': '%s'", ros::this_node::getName().c_str(), _uav_name_.c_str(), message.c_str());
      return false;
    }

    uh_ = uhopt.value();
  }

  // | ------------- wait for the system to be ready ------------ |

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    if (uh_->mrsSystemReady()) {
      break;
    }
  }

  // | ------------------------ take off ------------------------ |

  {
    auto [success, message] = uh_->takeoff();

    if (!success) {
      ROS_ERROR("[%s]: takeoff failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  sleep(1.0);

  // | --------------------- goto somewhere --------------------- |

  {
    auto [success, message] = uh_->gotoAbs(10, 15, 5, 1.2);

    if (!success) {
      ROS_ERROR("[%s]: goto failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ----------------------- land there ----------------------- |

  double des_x   = -10;
  double des_y   = -15;
  double des_z   = -10;
  double des_hdg = 3.14;

  {
    auto [success, message] = landThere(des_x, des_y, des_z, des_hdg);

    if (!success) {
      ROS_ERROR("[%s]: land there failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ---------------- check the final position ---------------- |

  if (uh_->isAtPosition(des_x, des_y, 0, des_hdg, 0.5)) {
    return true;
  } else {
    ROS_ERROR("[%s]: land there did end in wrong place", ros::this_node::getName().c_str());
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
