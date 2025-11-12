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

    mrs_msgs::msg::ReferenceStamped msg;

    msg.header.frame_id      = uav_name + "/fcu";
    msg.reference.position.x = 10;
    msg.reference.position.y = 1;
    msg.reference.position.z = 2;
    msg.reference.heading    = 2;

    auto gt_tfed_reference = uh->transformer_->transformSingle(msg, target_frame);

    if (!gt_tfed_reference) {
      RCLCPP_ERROR(node_->get_logger(), "failed to transform the reference");
      return false;
    }

    auto [success, message, ref_tfed] = uh->transformReference(msg, target_frame);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "reference #1 transformation failed: '%s'", message->c_str());
      return false;
    }

    if (std::abs(gt_tfed_reference->reference.position.x - ref_tfed->reference.position.x) > 0.1 ||
        std::abs(gt_tfed_reference->reference.position.y - ref_tfed->reference.position.y) > 0.1 ||
        std::abs(gt_tfed_reference->reference.position.z - ref_tfed->reference.position.z) > 0.1 ||
        std::abs(mrs_lib::geometry::sradians::diff(gt_tfed_reference->reference.heading, ref_tfed->reference.heading)) > 0.1) {

      RCLCPP_ERROR(node_->get_logger(), "reference #1 transformation failed, the references don't match");
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
