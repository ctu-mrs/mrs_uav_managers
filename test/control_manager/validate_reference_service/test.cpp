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

  {
    mrs_msgs::ReferenceStamped msg;

    msg.header.frame_id      = _uav_name_ + "/world_origin";
    msg.reference.position.x = 0;
    msg.reference.position.y = 0;
    msg.reference.position.z = 2;
    msg.reference.heading    = 0;

    auto [success, message] = uh->validateReference(msg);

    if (!success) {
      ROS_ERROR("[%s]: reference #1 validation failed: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  {
    mrs_msgs::ReferenceStamped msg;

    msg.header.frame_id      = _uav_name_ + "/world_origin";
    msg.reference.position.x = 100;
    msg.reference.position.y = 0;
    msg.reference.position.z = 2;
    msg.reference.heading    = 0;

    auto [success, message] = uh->validateReference(msg);

    if (success) {
      ROS_ERROR("[%s]: reference #2 validation failed: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  {
    mrs_msgs::ReferenceStamped msg;

    msg.header.frame_id      = _uav_name_ + "/world_origin";
    msg.reference.position.x = 0;
    msg.reference.position.y = 0;
    msg.reference.position.z = -100;
    msg.reference.heading    = 0;

    auto [success, message] = uh->validateReference(msg);

    if (success) {
      ROS_ERROR("[%s]: reference #3 validation failed: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  {
    mrs_msgs::ReferenceStamped msg;

    msg.header.frame_id      = "frame_that_does_not_exist";
    msg.reference.position.x = 0;
    msg.reference.position.y = 0;
    msg.reference.position.z = 2;
    msg.reference.heading    = 0;

    auto [success, message] = uh->validateReference(msg);

    if (success) {
      ROS_ERROR("[%s]: reference #4 validation failed: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
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
