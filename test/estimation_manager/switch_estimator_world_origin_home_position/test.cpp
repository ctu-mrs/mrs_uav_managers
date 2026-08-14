#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {
  }

  bool test(void);

  // checks that the UAV is still near the origin of every frame that is anchored to the home position
  bool originFramesAnchoredAtHome(const std::string &uav_name, const std::string &when);

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh_;
};

bool Tester::originFramesAnchoredAtHome(const std::string &uav_name, const std::string &when) {

  if (!uh_->sh_uav_state_.hasMsg()) {
    RCLCPP_ERROR(node_->get_logger(), "no uav_state received %s", when.c_str());
    return false;
  }

  auto uav_state = uh_->sh_uav_state_.getMsg();

  mrs_msgs::msg::ReferenceStamped ref;
  ref.header.frame_id      = uav_state->header.frame_id;
  ref.reference.position.x = uav_state->pose.position.x;
  ref.reference.position.y = uav_state->pose.position.y;
  ref.reference.position.z = uav_state->pose.position.z;

  for (const auto &frame : {"world_origin", "local_origin"}) {

    auto tfed = uh_->transformer_->transformSingle(ref, uav_name + "/" + frame);

    if (!tfed) {
      RCLCPP_ERROR(node_->get_logger(), "failed to transform the UAV position into '%s' %s", frame, when.c_str());
      return false;
    }

    const double dist = std::hypot(tfed->reference.position.x, tfed->reference.position.y);

    if (dist > 5.0) {
      RCLCPP_ERROR(node_->get_logger(), "UAV is %.1f m from '%s' %s - the frame is not anchored at the home position", dist, frame, when.c_str());
      return false;
    }
  }

  return true;
}

bool Tester::test(void) {

  const std::string uav_name = "uav1";

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
      return false;
    }

    uh_ = uhopt.value();
  }

  // takeoff is gated on the world_origin readiness, so the home position is adopted by now
  {
    auto [success, message] = uh_->takeoff();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "takeoff failed with message: '%s'", message.c_str());
      return false;
    }
  }

  this->sleep(3.0);

  if (!originFramesAnchoredAtHome(uav_name, "after takeoff")) {
    return false;
  }

  // | ------------------- switch the utm source ------------------ |

  // both estimators are utm-based, so this hands the utm_origin/world_origin tfs over to the tf
  // source of gps_baro, which keeps its own reference message; if that source is not anchored to the
  // home position the frames jump by the whole world origin
  {
    auto [success, message] = uh_->switchEstimator("gps_baro");

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "failed to switch the estimator: '%s'", message.c_str());
      return false;
    }
  }

  this->sleep(5.0);

  if (uh_->getActiveEstimator() != "gps_baro") {
    RCLCPP_ERROR(node_->get_logger(), "'gps_baro' estimator not active after the switch");
    return false;
  }

  if (!originFramesAnchoredAtHome(uav_name, "after the estimator switch")) {
    return false;
  }

  if (uh_->isFlyingNormally()) {
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "not flying normally after the estimator switch");
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
