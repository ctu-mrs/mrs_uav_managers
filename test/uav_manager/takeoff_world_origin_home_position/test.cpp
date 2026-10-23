#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {
  }

  bool test(void);

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh_;
};

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

  // | -------------------------- takeoff ------------------------- |

  // takeoff() (not activateMidAir) goes through UavManager's world_origin readiness gate; if the
  // home position is never captured and confirmed, the gate never opens and this call fails
  {
    auto [success, message] = uh_->takeoff();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "takeoff failed with message: '%s'", message.c_str());
      return false;
    }
  }

  this->sleep(5.0);

  // | ---------- must still be flying normally in-fence --------- |

  // had the safety area been shifted away from home (the (0,0) UTM baseline bug), the UAV would be
  // outside the fence and get elanded
  if (!uh_->isFlyingNormally()) {
    RCLCPP_ERROR(node_->get_logger(), "not flying normally after takeoff");
    return false;
  }

  // | --------- the payoff: home became the world_origin -------- |

  // the UAV took off from home, so with use_home_position its world_origin-frame position must be
  // near zero - not the "huge" UTM numbers this feature exists to avoid
  {
    if (!uh_->sh_uav_state_.hasMsg()) {
      RCLCPP_ERROR(node_->get_logger(), "no uav_state received");
      return false;
    }

    auto uav_state = uh_->sh_uav_state_.getMsg();

    mrs_msgs::msg::ReferenceStamped ref;
    ref.header.frame_id      = uav_state->header.frame_id;
    ref.reference.position.x = uav_state->pose.position.x;
    ref.reference.position.y = uav_state->pose.position.y;
    ref.reference.position.z = uav_state->pose.position.z;

    auto ref_world = uh_->transformer_->transformSingle(ref, uav_name + "/world_origin");

    if (!ref_world) {
      RCLCPP_ERROR(node_->get_logger(), "failed to transform the UAV position into the world_origin frame");
      return false;
    }

    const double dist_from_origin = std::hypot(ref_world->reference.position.x, ref_world->reference.position.y);

    if (dist_from_origin > 5.0) {
      RCLCPP_ERROR(node_->get_logger(), "UAV is %.1f m from the world_origin - the home position was not applied as the world origin", dist_from_origin);
      return false;
    }
  }

  // | ------ the other origin frames must be anchored at home too ----- |

  // these are latched from the first uav_state; deferred estimator start means that now arrives
  // after the home position is adopted, so this check is defense-in-depth rather than the primary
  // regression guard - it still catches the frame staying offset by the whole home position if a
  // uav_state is ever published before adoption again (e.g. the deferred-start gate regressing)
  {
    auto uav_state = uh_->sh_uav_state_.getMsg();

    mrs_msgs::msg::ReferenceStamped ref;
    ref.header.frame_id      = uav_state->header.frame_id;
    ref.reference.position.x = uav_state->pose.position.x;
    ref.reference.position.y = uav_state->pose.position.y;
    ref.reference.position.z = uav_state->pose.position.z;

    for (const auto &frame : {"local_origin", "stable_origin", "fixed_origin"}) {

      auto tfed = uh_->transformer_->transformSingle(ref, uav_name + "/" + frame);

      if (!tfed) {
        RCLCPP_ERROR(node_->get_logger(), "failed to transform the UAV position into '%s'", frame);
        return false;
      }

      const double dist = std::hypot(tfed->reference.position.x, tfed->reference.position.y);

      if (dist > 5.0) {
        RCLCPP_ERROR(node_->get_logger(), "UAV is %.1f m from '%s' - the frame was not re-anchored after the home position was adopted", dist, frame);
        return false;
      }
    }
  }

  return true;
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
