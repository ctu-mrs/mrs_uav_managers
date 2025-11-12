#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

#include <mrs_lib/geometry/cyclic.h>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {
  }

  bool test(void);
};

bool Tester::test(void) {

  const std::string uav_name = "uav1";

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh;

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
      return false;
    }

    uh = uhopt.value();
  }

  {
    auto [success, message] = uh->activateMidAir();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "midair activation failed with message: '%s'", message.c_str());
      return false;
    }
  }

  {
    std::string target_frame = uav_name + "/world_origin";

    geometry_msgs::msg::PoseStamped msg;

    msg.header.frame_id  = uav_name + "/fcu";
    msg.pose.position.x  = 10;
    msg.pose.position.y  = 1;
    msg.pose.position.z  = 2;
    msg.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(2);

    auto gt_tfed_pose = uh->transformer_->transformSingle(msg, target_frame);

    if (!gt_tfed_pose) {
      RCLCPP_ERROR(node_->get_logger(), "failed to transform the pose");
      return false;
    }

    auto [success, message, pose_tfed] = uh->transformPose(msg, target_frame);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "pose #1 transformation failed: '%s'", message->c_str());
      return false;
    }

    if (std::abs(gt_tfed_pose->pose.position.x - pose_tfed->pose.position.x) > 0.1 ||
        std::abs(gt_tfed_pose->pose.position.y - pose_tfed->pose.position.y) > 0.1 ||
        std::abs(gt_tfed_pose->pose.position.z - pose_tfed->pose.position.z) > 0.1 ||
        std::abs(mrs_lib::geometry::sradians::diff(mrs_lib::AttitudeConverter(gt_tfed_pose->pose.orientation).getHeading(),
                                                   mrs_lib::AttitudeConverter(pose_tfed->pose.orientation).getHeading())) > 0.1) {

      RCLCPP_ERROR(node_->get_logger(), "pose #1 transformation failed, the poses don't match");
      return false;
    }
  }

  if (uh->isFlyingNormally()) {
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "not flying normally");
    return false;
  }
}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Tester tester;

  test_result &= tester.test();

  tester.sleep(2.0);

  std::cout << "Test: reporting test results" << std::endl;

  tester.reportTestResult(test_result);

  tester.join();
}
