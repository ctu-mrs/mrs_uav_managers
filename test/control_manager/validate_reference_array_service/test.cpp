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

  mrs_msgs::ValidateReferenceArrayRequest req;

  req.array.header.frame_id = _uav_name_ + "/world_origin";

  {
    mrs_msgs::Reference msg;

    msg.position.x = 0;
    msg.position.y = 0;
    msg.position.z = 2;
    msg.heading    = 0;

    req.array.array.push_back(msg);
  }

  {
    mrs_msgs::Reference msg;

    msg.position.x = 100;
    msg.position.y = 0;
    msg.position.z = 2;
    msg.heading    = 0;

    req.array.array.push_back(msg);
  }

  {
    mrs_msgs::Reference msg;

    msg.position.x = 0;
    msg.position.y = 0;
    msg.position.z = -100;
    msg.heading    = 0;

    req.array.array.push_back(msg);
  }

  {
    mrs_msgs::Reference msg;

    msg.position.x = 10;
    msg.position.y = 10;
    msg.position.z = 3;
    msg.heading    = 0;

    req.array.array.push_back(msg);
  }

  {
    auto [success, response] = uh->validateReferenceArray(req);

    if (!success) {

      ROS_ERROR("[%s]: reference array validation service call failed: '%s'", ros::this_node::getName().c_str(), response->message.c_str());
      return false;

    } else {

      if (!response->success[0]) {
        ROS_ERROR("[%s]: reference #1 validation failed", ros::this_node::getName().c_str());
        return false;
      }

      if (response->success[1]) {
        ROS_ERROR("[%s]: reference #2 validation failed", ros::this_node::getName().c_str());
        return false;
      }

      if (response->success[2]) {
        ROS_ERROR("[%s]: reference #3 validation failed", ros::this_node::getName().c_str());
        return false;
      }

      if (!response->success[3]) {
        ROS_ERROR("[%s]: reference #4 validation failed", ros::this_node::getName().c_str());
        return false;
      }
    }
  }

  // | ---------------- now try with bad frame id --------------- |

  {
    req.array.header.frame_id = "frame_that_does_not_exist";

    auto [success, response] = uh->validateReferenceArray(req);

    if (success) {

      ROS_ERROR("[%s]: reference array validation service call succeeded but it should not have", ros::this_node::getName().c_str());
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
