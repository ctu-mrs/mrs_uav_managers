#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_lib/gps_conversions.h>
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

  // ~25 m from the configured origin: far enough that an un-anchored frame is unmistakable
  const double new_origin_lat = 47.397923;
  const double new_origin_lon = 8.545794;

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

  auto sch_point_in_safety_area_2d =
      mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "/" + uav_name + "/safety_area_manager/point_in_safety_area_2d");

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

  // remember the border and origin, so the expected shift below can be computed exactly
  std::vector<mrs_msgs::msg::Point2D> border_before;
  std::string                         border_vertical_frame_before;
  std::vector<mrs_msgs::msg::Prism>   obstacles_before;
  double                              origin_lat_before = 0.0;
  double                              origin_lon_before = 0.0;

  if (uh_->sh_safety_area_manager_diag_.hasMsg()) {
    const auto diag              = uh_->sh_safety_area_manager_diag_.getMsg();
    border_before                = diag->border.points;
    border_vertical_frame_before = diag->border.vertical_frame;
    obstacles_before             = diag->obstacles;
    origin_lat_before            = diag->world_origin.x;
    origin_lon_before            = diag->world_origin.y;
  }

  if (border_before.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "no safety area border received before the world origin change");
    return false;
  }

  // obstacle_0 is latlon_origin (untouched), obstacle_1 is world_origin (shifts like the border)
  if (obstacles_before.size() != 2 || obstacles_before[0].points.empty() || obstacles_before[1].points.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "expected exactly two obstacles with points before the world origin change");
    return false;
  }

  // the origin point (0, 0) - which this border is centered on - must be inside it
  {
    auto request             = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();
    request->header.frame_id = uav_name + "/world_origin";

    auto response = sch_point_in_safety_area_2d.callSync(request);

    if (!response || !response.value()->success) {
      RCLCPP_ERROR(node_->get_logger(), "the safety area center is not enforced as valid before the world origin change");
      return false;
    }
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

    request->header.frame_id      = "latlon_origin";
    request->reference.position.x = new_origin_lat;
    request->reference.position.y = new_origin_lon;

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

  // a physically-fixed border must shift by exactly -(new_utm - old_utm), catching a sign error
  double old_utm_x, old_utm_y, new_utm_x, new_utm_y;
  mrs_lib::UTM(origin_lat_before, origin_lon_before, &old_utm_x, &old_utm_y);
  mrs_lib::UTM(new_origin_lat, new_origin_lon, &new_utm_x, &new_utm_y);
  const double expected_delta_x = new_utm_x - old_utm_x;
  const double expected_delta_y = new_utm_y - old_utm_y;

  // the zone is rebuilt synchronously inside the service call; the diagnostics message only needs
  // one status tick to catch up, so a short poll is enough
  std::vector<mrs_msgs::msg::Point2D> border_now;
  {
    const double min_shift = 1.0; // [m]
    const double timeout   = 5.0; // [s]

    bool rebuilt = false;

    const rclcpp::Time start = clock_->now();

    while (rclcpp::ok() && (clock_->now() - start).seconds() < timeout) {

      if (uh_->sh_safety_area_manager_diag_.hasMsg()) {

        const auto candidate = uh_->sh_safety_area_manager_diag_.getMsg()->border.points;

        if (candidate.size() == border_before.size() && !candidate.empty() &&
            std::hypot(candidate[0].x - border_before[0].x, candidate[0].y - border_before[0].y) > min_shift) {
          border_now = candidate;
          rebuilt    = true;
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

  // the shift has to match -expected_delta exactly (within UTM/float rounding), not just be nonzero
  {
    const double tol = 1e-2; // [m]

    for (size_t i = 0; i < border_now.size(); i++) {
      const double expected_x = border_before[i].x - expected_delta_x;
      const double expected_y = border_before[i].y - expected_delta_y;
      const double error_x    = std::abs(border_now[i].x - expected_x);
      const double error_y    = std::abs(border_now[i].y - expected_y);

      if (error_x > tol || error_y > tol) {
        RCLCPP_ERROR(node_->get_logger(),
                     "border point %zu shifted to (%.3f, %.3f), expected (%.3f, %.3f) - wrong sign or magnitude in the world_origin compensation", i,
                     border_now[i].x, border_now[i].y, expected_x, expected_y);
        return false;
      }
    }
  }

  // vertical frame (local_origin, deliberately not world_origin) must survive the rebuild
  {
    const auto border_vertical_frame_now = uh_->sh_safety_area_manager_diag_.getMsg()->border.vertical_frame;

    if (border_vertical_frame_now != border_vertical_frame_before) {
      RCLCPP_ERROR(node_->get_logger(), "the border's vertical frame changed from '%s' to '%s' after the world origin change",
                   border_vertical_frame_before.c_str(), border_vertical_frame_now.c_str());
      return false;
    }
  }

  // exercises both branches of the per-obstacle rebuild
  {
    const auto obstacles_now = uh_->sh_safety_area_manager_diag_.getMsg()->obstacles;

    if (obstacles_now.size() != 2 || obstacles_now[0].points.size() != obstacles_before[0].points.size() ||
        obstacles_now[1].points.size() != obstacles_before[1].points.size()) {
      RCLCPP_ERROR(node_->get_logger(), "an obstacle changed shape after the world origin change");
      return false;
    }

    for (size_t i = 0; i < obstacles_now[0].points.size(); i++) {
      if (obstacles_now[0].points[i].x != obstacles_before[0].points[i].x || obstacles_now[0].points[i].y != obstacles_before[0].points[i].y) {
        RCLCPP_ERROR(node_->get_logger(), "the latlon_origin obstacle point %zu moved after the world origin change, but it is not in world_origin frame", i);
        return false;
      }
    }

    if (obstacles_now[1].vertical_frame != obstacles_before[1].vertical_frame) {
      RCLCPP_ERROR(node_->get_logger(), "the world_origin obstacle's vertical frame changed from '%s' to '%s' after the world origin change",
                   obstacles_before[1].vertical_frame.c_str(), obstacles_now[1].vertical_frame.c_str());
      return false;
    }

    const double tol = 1e-2; // [m]

    for (size_t i = 0; i < obstacles_now[1].points.size(); i++) {
      const double expected_x = obstacles_before[1].points[i].x - expected_delta_x;
      const double expected_y = obstacles_before[1].points[i].y - expected_delta_y;
      const double error_x    = std::abs(obstacles_now[1].points[i].x - expected_x);
      const double error_y    = std::abs(obstacles_now[1].points[i].y - expected_y);

      if (error_x > tol || error_y > tol) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "world_origin obstacle point %zu shifted to (%.3f, %.3f), expected (%.3f, %.3f) - wrong sign or magnitude in the world_origin compensation", i,
            obstacles_now[1].points[i].x, obstacles_now[1].points[i].y, expected_x, expected_y);
        return false;
      }
    }
  }

  // the same physical point, re-expressed in the new frame, must still be enforced as valid
  {
    auto request                  = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();
    request->header.frame_id      = uav_name + "/world_origin";
    request->reference.position.x = -expected_delta_x;
    request->reference.position.y = -expected_delta_y;

    auto response = sch_point_in_safety_area_2d.callSync(request);

    if (!response || !response.value()->success) {
      RCLCPP_ERROR(node_->get_logger(), "the safety area center is no longer enforced as valid after the world origin change");
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
