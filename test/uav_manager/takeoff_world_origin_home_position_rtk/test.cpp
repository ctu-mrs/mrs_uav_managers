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

  // takeoff() (not activateMidAir) goes through UavManager's world_origin readiness gate, which in
  // home-position mode only opens once the home position has been captured - here from the RTK
  // callback, which gates on RTK_FIX and transforms the antenna position to the FCU
  {
    auto [success, message] = uh_->takeoff();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "takeoff failed with message: '%s'", message.c_str());
      return false;
    }
  }

  this->sleep(5.0);

  // | ------------- the home position came from RTK ------------- |

  if (uh_->getActiveEstimator() != "rtk_garmin") {
    RCLCPP_ERROR(node_->get_logger(), "rtk_garmin estimator is not active, the RTK path was not exercised");
    return false;
  }

  // | ---------- must still be flying normally in-fence --------- |

  // had the safety area been shifted away from home, the UAV would be outside the fence and elanded
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
