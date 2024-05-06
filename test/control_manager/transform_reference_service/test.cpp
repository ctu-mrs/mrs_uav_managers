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

    mrs_msgs::ReferenceStamped msg;

    msg.header.frame_id      = _uav_name_ + "/fcu";
    msg.reference.position.x = 10;
    msg.reference.position.y = 1;
    msg.reference.position.z = 2;
    msg.reference.heading    = 2;

    auto gt_tfed_reference = uh->transformer_->transformSingle(msg, target_frame);

    if (!gt_tfed_reference) {
      ROS_ERROR("[%s]: failed to transform the reference", ros::this_node::getName().c_str());
      return false;
    }

    auto [success, message, ref_tfed] = uh->transformReference(msg, target_frame);

    if (!success) {
      ROS_ERROR("[%s]: reference #1 transformation failed: '%s'", ros::this_node::getName().c_str(), message->c_str());
      return false;
    }

    if (std::abs(gt_tfed_reference->reference.position.x - ref_tfed->reference.position.x) > 0.1 ||
        std::abs(gt_tfed_reference->reference.position.y - ref_tfed->reference.position.y) > 0.1 ||
        std::abs(gt_tfed_reference->reference.position.z - ref_tfed->reference.position.z) > 0.1 ||
        std::abs(sradians::diff(gt_tfed_reference->reference.heading, ref_tfed->reference.heading)) > 0.1) {

      ROS_ERROR("[%s]: reference #1 transformation failed, the references don't match", ros::this_node::getName().c_str());
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

