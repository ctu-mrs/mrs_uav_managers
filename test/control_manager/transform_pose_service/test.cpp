#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

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
    std::string target_frame = _uav_name_ + "/world_origin";

    geometry_msgs::PoseStamped msg;

    msg.header.frame_id  = _uav_name_ + "/fcu";
    msg.pose.position.x  = 10;
    msg.pose.position.y  = 1;
    msg.pose.position.z  = 2;
    msg.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(2);

    auto gt_tfed_pose = uh->transformer_->transformSingle(msg, target_frame);

    if (!gt_tfed_pose) {
      ROS_ERROR("[%s]: failed to transform the pose", ros::this_node::getName().c_str());
      return false;
    }

    auto [success, message, pose_tfed] = uh->transformPose(msg, target_frame);

    if (!success) {
      ROS_ERROR("[%s]: pose #1 transformation failed: '%s'", ros::this_node::getName().c_str(), message->c_str());
      return false;
    }

    if (std::abs(gt_tfed_pose->pose.position.x - pose_tfed->pose.position.x) > 0.1 ||
        std::abs(gt_tfed_pose->pose.position.y - pose_tfed->pose.position.y) > 0.1 ||
        std::abs(gt_tfed_pose->pose.position.z - pose_tfed->pose.position.z) > 0.1 ||
        std::abs(sradians::diff(mrs_lib::AttitudeConverter(gt_tfed_pose->pose.orientation).getHeading(),
                                mrs_lib::AttitudeConverter(pose_tfed->pose.orientation).getHeading())) > 0.1) {

      ROS_ERROR("[%s]: pose #1 transformation failed, the poses don't match", ros::this_node::getName().c_str());
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

