#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_lib/service_client_handler.h>

#include <mrs_msgs/srv/get_bool_srv.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>

#include <mrs_uav_testing/test_generic.h>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {
  }

  bool test(void);

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh_;

private:
  std::optional<geometry_msgs::msg::Point> positionInFrame(const std::string &frame);
  std::optional<geometry_msgs::msg::Point> waitForPositionInFrame(const std::string &frame, const double timeout);
};

/* positionInFrame() //{ */

// the current UAV position expressed in the given frame
std::optional<geometry_msgs::msg::Point> Tester::positionInFrame(const std::string &frame) {

  if (!uh_->sh_uav_state_.hasMsg()) {
    return std::nullopt;
  }

  auto uav_state = uh_->sh_uav_state_.getMsg();

  mrs_msgs::msg::ReferenceStamped ref;
  ref.header.frame_id      = uav_state->header.frame_id;
  ref.reference.position.x = uav_state->pose.position.x;
  ref.reference.position.y = uav_state->pose.position.y;
  ref.reference.position.z = uav_state->pose.position.z;

  auto tfed = uh_->transformer_->transformSingle(ref, frame);

  if (!tfed) {
    return std::nullopt;
  }

  return tfed->reference.position;
}

//}

/* waitForPositionInFrame() //{ */

// positionInFrame() can fail right after the system reports ready, before every static tf (e.g.
// local_origin) has actually been broadcast yet; poll instead of querying once
std::optional<geometry_msgs::msg::Point> Tester::waitForPositionInFrame(const std::string &frame, const double timeout) {

  const rclcpp::Time start = clock_->now();

  while (rclcpp::ok() && (clock_->now() - start).seconds() < timeout) {

    auto position = positionInFrame(frame);

    if (position) {
      return position;
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "waiting for the transform to '%s'", frame.c_str());

    this->sleep(0.1);
  }

  return std::nullopt;
}

//}

bool Tester::test(void) {

  const std::string uav_name = "uav1";

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
      return false;
    }

    uh_ = uhopt.value();
  }

  auto sch_set_world_origin = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "/" + uav_name + "/estimation_manager/set_world_origin");

  auto sch_safety_zone_enabled =
      mrs_lib::ServiceClientHandler<mrs_msgs::srv::GetBoolSrv>(node_, "/" + uav_name + "/safety_area_manager/is_safety_zone_enabled");

  // | ---- wait for the system, the origin can only be set on the ground ---- |

  while (rclcpp::ok() && !uh_->mrsSystemReady()) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "waiting for the MRS UAV System");
    this->sleep(0.01);
  }

  // the fence has to be up front, otherwise checking it after the rebuild proves nothing
  {
    auto response = sch_safety_zone_enabled.callSync(std::make_shared<mrs_msgs::srv::GetBoolSrv::Request>());

    if (!response || !response.value()->value) {
      RCLCPP_ERROR(node_->get_logger(), "the safety zone is not enabled before the world origin change");
      return false;
    }
  }

  // remember the border, the rebuild is detected by it moving with the origin
  std::vector<mrs_msgs::msg::Point2D> border_before;

  if (uh_->sh_safety_area_manager_diag_.hasMsg()) {
    border_before = uh_->sh_safety_area_manager_diag_.getMsg()->border.points;
  }

  if (border_before.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "no safety area border received before the world origin change");
    return false;
  }

  // the UAV stands still for the whole test, so utm_origin - a frame the origin change must not
  // move - fixes where it is; anything that shifts here is a tf that was left anchored to the
  // pre-reset estimator frame
  geometry_msgs::msg::Point utm_before;

  {
    auto position = waitForPositionInFrame(uav_name + "/utm_origin", 5.0);

    if (!position) {
      RCLCPP_ERROR(node_->get_logger(), "could not get the UAV position in utm_origin before the world origin change");
      return false;
    }

    utm_before = position.value();
  }

  // | ------------- move the world origin at runtime ------------ |

  // this resets the estimators onto the new origin, which re-anchors every frame derived from them,
  // and it rebuilds the safety area - the two things this test guards
  {
    auto request = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();

    request->header.frame_id = "latlon_origin";
    // ~25 m from the configured origin: far enough that a frame left un-anchored is unmistakable
    // (it lands megametres away), close enough to keep the UAV well inside the safety area
    request->reference.position.x = 47.397923;
    request->reference.position.y = 8.545794;

    auto response = sch_set_world_origin.callSync(request);

    if (!response || !response.value()->success) {
      RCLCPP_ERROR(node_->get_logger(), "set_world_origin failed: '%s'", response ? response.value()->message.c_str() : "no response");
      return false;
    }
  }

  // let the reset estimators produce a few states, so the tfs settle on the new origin
  this->sleep(3.0);

  // | ------- the origin frames have to keep describing the UAV ------ |

  // the utm/world tfs are derived from the tf source's first odom msg, captured before the reset;
  // if that reference is not invalidated, they stay offset by however far the origin moved
  {
    const double max_drift = 2.0; // [m]

    auto position = waitForPositionInFrame(uav_name + "/utm_origin", 5.0);

    if (!position) {
      RCLCPP_ERROR(node_->get_logger(), "could not get the UAV position in utm_origin after the world origin change");
      return false;
    }

    const double drift = std::hypot(position.value().x - utm_before.x, position.value().y - utm_before.y);

    RCLCPP_INFO(node_->get_logger(), "the UAV moved by %.2f m in utm_origin over the world origin change", drift);

    if (drift > max_drift) {
      RCLCPP_ERROR(node_->get_logger(), "the standing UAV moved by %.2f m in utm_origin, the tf reference was not re-anchored", drift);
      return false;
    }
  }

  // local_origin is anchored to the first uav_state, taken before the reset; once it is dropped the
  // next uav_state re-anchors it onto the UAV, so the UAV ends up back at its origin
  {
    const double max_distance = 3.0; // [m]

    auto position = waitForPositionInFrame(uav_name + "/local_origin", 5.0);

    if (!position) {
      RCLCPP_ERROR(node_->get_logger(), "could not get the UAV position in local_origin after the world origin change");
      return false;
    }

    const double distance = std::hypot(position.value().x, position.value().y);

    RCLCPP_INFO(node_->get_logger(), "the UAV is %.2f m from local_origin after the world origin change", distance);

    if (distance > max_distance) {
      RCLCPP_ERROR(node_->get_logger(), "the UAV is %.2f m from local_origin, the frame was not re-anchored", distance);
      return false;
    }
  }

  // | --------- the safety area has to survive being rebuilt -------- |

  // the manager rebuilds the zone from its own timer, a moment after it answers the service; wait
  // for the shifted border to show up, otherwise the checks below still describe the old zone
  {
    const double min_shift = 1.0;  // [m]
    const double timeout   = 20.0; // [s]

    bool rebuilt = false;

    const rclcpp::Time start = clock_->now();

    while (rclcpp::ok() && (clock_->now() - start).seconds() < timeout) {

      if (uh_->sh_safety_area_manager_diag_.hasMsg()) {

        const auto border_now = uh_->sh_safety_area_manager_diag_.getMsg()->border.points;

        if (border_now.size() == border_before.size() && !border_now.empty() &&
            std::hypot(border_now[0].x - border_before[0].x, border_now[0].y - border_before[0].y) > min_shift) {
          rebuilt = true;
          break;
        }
      }

      this->sleep(0.1);
    }

    if (!rebuilt) {
      RCLCPP_ERROR(node_->get_logger(), "the safety area border did not move after the world origin changed, the zone was not rebuilt");
      return false;
    }
  }

  // a freshly constructed zone defaults to disabled; without carrying the state over, the fence
  // silently disappears
  {
    auto response = sch_safety_zone_enabled.callSync(std::make_shared<mrs_msgs::srv::GetBoolSrv::Request>());

    if (!response || !response.value()->value) {
      RCLCPP_ERROR(node_->get_logger(), "the safety zone came back disabled after the world origin change");
      return false;
    }
  }

  // and it has to be enforced, not just reported as enabled
  {
    mrs_msgs::msg::ReferenceStamped msg;

    msg.header.frame_id      = uav_name + "/world_origin";
    msg.reference.position.x = 10000;
    msg.reference.position.y = 10000;
    msg.reference.position.z = 3;
    msg.reference.heading    = 0;

    auto [success, message] = uh_->validateReference(msg);

    if (success) {
      RCLCPP_ERROR(node_->get_logger(), "a reference 10 km away was accepted, the safety area is not enforced after the rebuild");
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
