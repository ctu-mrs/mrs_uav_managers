#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_lib/gps_conversions.h>
#include <mrs_lib/service_client_handler.h>

#include <mrs_msgs/srv/reference_stamped_srv.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <mrs_uav_testing/test_generic.h>

#include <cmath>
#include <mutex>
#include <optional>
#include <vector>

using namespace std::chrono_literals;

/* class Tester //{ */

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {
  }

  bool test(void);

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh_;

private:
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_markers_;
  std::mutex                                                            markers_mtx_;
  std::optional<visualization_msgs::msg::MarkerArray>                   last_markers_;

  // the border edge marker, once it is being published with real geometry
  std::optional<visualization_msgs::msg::Marker> waitForBorderMarker(const double timeout);

  // the distinct polygon corners of a LINE_LIST border marker (the upper and lower ring collapse to one)
  std::vector<geometry_msgs::msg::Point> markerCorners(const visualization_msgs::msg::Marker &marker);
};

//}

/* waitForBorderMarker() //{ */

std::optional<visualization_msgs::msg::Marker> Tester::waitForBorderMarker(const double timeout) {

  const rclcpp::Time start = clock_->now();

  while (rclcpp::ok() && (clock_->now() - start).seconds() < timeout) {

    {
      std::scoped_lock lock(markers_mtx_);

      if (last_markers_) {
        for (const auto &marker : last_markers_->markers) {
          if (marker.type == visualization_msgs::msg::Marker::LINE_LIST && marker.action == visualization_msgs::msg::Marker::ADD && marker.points.size() >= 6) {
            return marker;
          }
        }
      }
    }

    this->sleep(0.2);
  }

  return std::nullopt;
}

//}

/* markerCorners() //{ */

std::vector<geometry_msgs::msg::Point> Tester::markerCorners(const visualization_msgs::msg::Marker &marker) {

  // a LINE_LIST border marker repeats every polygon corner several times (at min_z and max_z, and
  // shared between adjacent edges); collapse them back to the distinct corner positions by x and y
  std::vector<geometry_msgs::msg::Point> corners;

  for (const auto &point : marker.points) {

    bool duplicate = false;

    for (const auto &corner : corners) {
      if (std::abs(point.x - corner.x) < 0.05 && std::abs(point.y - corner.y) < 0.05) {
        duplicate = true;
        break;
      }
    }

    if (!duplicate) {
      corners.push_back(point);
    }
  }

  return corners;
}

//}

/* test() //{ */

bool Tester::test(void) {

  const std::string uav_name     = "uav1";
  const std::string world_frame  = uav_name + "/world_origin";
  const std::string latlon_frame = uav_name + "/latlon_origin";

  // the border corners as configured in config/world_config.yaml (x = latitude, y = longitude)
  const std::vector<std::pair<double, double>> config_corners = {
      {47.397243, 8.544894},
      {47.397243, 8.546294},
      {47.398243, 8.546294},
      {47.398243, 8.544894},
  };

  // ~25 m from the configured origin
  const double new_origin_lat = 47.397923;
  const double new_origin_lon = 8.545794;

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
      return false;
    }

    uh_ = uhopt.value();
  }

  // the test's transformer needs the uav prefix (so latlon_origin resolves) and a utm zone (to
  // invert world_origin -> latlon_origin)
  uh_->transformer_->setDefaultPrefix(uav_name);
  uh_->transformer_->setLatLon(47.397743, 8.545594);

  sub_markers_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>("/" + uav_name + "/safety_area_manager/static_markers", rclcpp::QoS(10),
                                                                                  [this](const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
                                                                                    std::scoped_lock lock(markers_mtx_);
                                                                                    last_markers_ = *msg;
                                                                                  });

  auto sch_set_world_origin = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "/" + uav_name + "/estimation_manager/set_world_origin");

  // | ---------------------- wait for the system --------------------- |

  while (rclcpp::ok() && !uh_->mrsSystemReady()) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "waiting for the MRS UAV System");
    this->sleep(0.01);
  }

  /* verify() //{ */

  // takes the published border marker, checks it is in an RViz-usable earth-fixed frame, and
  // transfers every vertex back to latlon_origin with mrs_lib to confirm the geometry RViz receives
  // still describes the configured fence
  const auto verify = [&](const std::string &when) -> bool {
    auto marker = waitForBorderMarker(30.0);

    if (!marker) {
      RCLCPP_ERROR(node_->get_logger(), "no safety area border marker received %s", when.c_str());
      return false;
    }

    if (marker->header.frame_id != world_frame) {
      RCLCPP_ERROR(node_->get_logger(), "border marker is published in '%s' %s, expected '%s' (latlon_origin is not RViz-transformable, local_origin drifts)",
                   marker->header.frame_id.c_str(), when.c_str(), world_frame.c_str());
      return false;
    }

    const auto corners = markerCorners(*marker);

    RCLCPP_INFO(node_->get_logger(), "border marker %s: frame '%s', %zu points, %zu distinct corners", when.c_str(), marker->header.frame_id.c_str(),
                marker->points.size(), corners.size());

    if (corners.size() != config_corners.size()) {
      RCLCPP_ERROR(node_->get_logger(), "border marker has %zu distinct corners %s, expected %zu", corners.size(), when.c_str(), config_corners.size());
      return false;
    }

    const double tolerance = 1.0; // [m]

    for (const auto &vertex : corners) {

      mrs_msgs::msg::ReferenceStamped ref;
      ref.header.frame_id      = world_frame;
      ref.reference.position.x = vertex.x;
      ref.reference.position.y = vertex.y;

      auto latlon = uh_->transformer_->transformSingle(ref, latlon_frame);

      if (!latlon) {
        RCLCPP_ERROR(node_->get_logger(), "could not transform a border marker vertex back to latlon_origin %s", when.c_str());
        return false;
      }

      double vertex_utm_x, vertex_utm_y;
      mrs_lib::UTM(latlon->reference.position.x, latlon->reference.position.y, &vertex_utm_x, &vertex_utm_y);

      bool matched = false;

      for (const auto &[corner_lat, corner_lon] : config_corners) {
        double corner_utm_x, corner_utm_y;
        mrs_lib::UTM(corner_lat, corner_lon, &corner_utm_x, &corner_utm_y);
        if (std::hypot(vertex_utm_x - corner_utm_x, vertex_utm_y - corner_utm_y) < tolerance) {
          matched = true;
          break;
        }
      }

      if (!matched) {
        RCLCPP_ERROR(node_->get_logger(), "border marker vertex (%.2f, %.2f) maps to lat %.6f lon %.6f %s - not a configured corner (stale or mis-projected)",
                     vertex.x, vertex.y, latlon->reference.position.x, latlon->reference.position.y, when.c_str());
        return false;
      }
    }

    return true;
  };

  //}

  if (!verify("on startup")) {
    return false;
  }

  // | ------------- move the world origin at runtime ------------ |

  // the latlon_origin border is physically unchanged, but world_origin re-anchors under it - a
  // correct visualization must drop its cached projection and rebuild against the new origin
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

  // let the estimators reset and the manager rebuild the visualization
  this->sleep(5.0);

  if (!verify("after the world origin change")) {
    return false;
  }

  return true;
}

//}

/* main() //{ */

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

//}
