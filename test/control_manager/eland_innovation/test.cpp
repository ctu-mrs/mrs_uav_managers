#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

#include <nav_msgs/Odometry.h>

#define POS_JUMP_SIZE 4.0

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test();

  Tester();

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_hw_api_odom_;

  mrs_lib::PublisherHandler<nav_msgs::Odometry> ph_odometry_;

  void callbackOdometry(const nav_msgs::Odometry::ConstPtr msg);

  std::atomic<bool> odom_jumped_ = false;
};

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  sh_hw_api_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts_, "/" + _uav_name_ + "/hw_api/odometry_unchanged", &Tester::callbackOdometry, this);

  ph_odometry_ = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, "/" + _uav_name_ + "/hw_api/odometry");
}

void Tester::callbackOdometry(const nav_msgs::Odometry::ConstPtr msg) {

  nav_msgs::Odometry odom_out = *msg;

  if (odom_jumped_) {
    odom_out.pose.pose.position.x += POS_JUMP_SIZE;
  }

  ph_odometry_.publish(odom_out);
}

bool Tester::test() {

  {
    auto [success, message] = activateMidAir();

    if (!success) {
      ROS_ERROR("[%s]: midair activation failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  sleep(1.0);

  odom_jumped_ = true;

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
