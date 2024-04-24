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

  while (true) {

    if (!ros::ok()) {
      ROS_ERROR("[%s]: stopped from the outside", ros::this_node::getName().c_str());
      return false;
    }

    // | ------------------ publish the reference ----------------- |

    mrs_msgs::VelocityReferenceStamped msg;

    msg.reference.velocity.x = 1.2;
    msg.reference.velocity.y = 1.5;
    msg.reference.velocity.z = 0.5;

    uh->ph_velocity_reference_.publish(msg);

    // | ------------------- check for the state ------------------ |

    Eigen::Vector3d vel;

    {
      auto opt_vel = uh->getVelocity("");

      if (!opt_vel) {
        ROS_ERROR("[%s]: could not obtain UAV velocity", ros::this_node::getName().c_str());
        return false;
      }

      vel = opt_vel.value();
    }

    if (std::abs(msg.reference.velocity.x - vel(0)) < 0.1 && std::abs(msg.reference.velocity.y - vel(1)) < 0.1 &&
        std::abs(msg.reference.velocity.z - vel(2)) < 0.1) {
      ROS_INFO("[%s]: reached the desired velocity", ros::this_node::getName().c_str());
      break;
    }

    ros::Duration(0.01).sleep();
  }

  // | ----------- check if we become stationary again ---------- |

  while (true) {

    if (!ros::ok()) {
      ROS_ERROR("[%s]: stopped from the outside", ros::this_node::getName().c_str());
      return false;
    }

    std::optional<double> speed = uh->getSpeed();

    if (speed && speed.value() < 0.1) {
      ROS_INFO("[%s]: reached stopping speed", ros::this_node::getName().c_str());
      break;
    }
  }

  this->sleep(5.0);

  if (uh->isFlyingNormally()) {
    ROS_INFO("[%s]: still flying normally", ros::this_node::getName().c_str());
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
