#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/service_client_handler.h>

#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/safety_area_manager_diagnostics.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>

#include <mrs_uav_testing/test_generic.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <optional>

/* class Tester //{ */

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {
  }

  bool test(void);

private:
  std::shared_ptr<mrs_uav_testing::UAVHandler> uh_;

  std::mutex                                                 msgs_mtx_;
  std::optional<mrs_msgs::msg::SafetyAreaManagerDiagnostics> last_safety_area_diag_;
  std::optional<mrs_msgs::msg::GeneralRobotInfo>             last_general_robot_info_;

  rclcpp::Subscription<mrs_msgs::msg::SafetyAreaManagerDiagnostics>::SharedPtr sub_safety_area_diag_;
  rclcpp::Subscription<mrs_msgs::msg::GeneralRobotInfo>::SharedPtr             sub_general_robot_info_;
};

//}

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

  sub_safety_area_diag_ = node_->create_subscription<mrs_msgs::msg::SafetyAreaManagerDiagnostics>(
      "/" + uav_name + "/safety_area_manager/diagnostics", rclcpp::QoS(10), [this](const mrs_msgs::msg::SafetyAreaManagerDiagnostics::ConstSharedPtr msg) {
        std::scoped_lock lock(msgs_mtx_);
        last_safety_area_diag_ = *msg;
      });

  sub_general_robot_info_ = node_->create_subscription<mrs_msgs::msg::GeneralRobotInfo>(
      "/" + uav_name + "/diagnostics_manager/general_robot_info", rclcpp::QoS(10), [this](const mrs_msgs::msg::GeneralRobotInfo::ConstSharedPtr msg) {
        std::scoped_lock lock(msgs_mtx_);
        last_general_robot_info_ = *msg;
      });

  while (rclcpp::ok() && !uh_->mrsSystemReady()) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "waiting for the MRS UAV System");
    sleep(0.01);
  }

  // | ---------- the UAV really stands at the spawn point --------- |

  // SafetyAreaManager also reports an invalid position before it has one, so first make sure it has the real one
  const rclcpp::Time position_wait_start = clock_->now();

  while (true) {

    if (!rclcpp::ok() || (clock_->now() - position_wait_start).seconds() > 30.0) {
      RCLCPP_ERROR(node_->get_logger(), "the UAV state never showed the spawn point (500, 500)");
      return false;
    }

    const auto uav_state = uh_->sh_uav_state_.getMsg();

    if (uav_state && std::hypot(uav_state->pose.position.x - 500.0, uav_state->pose.position.y - 500.0) < 5.0) {
      break;
    }

    sleep(0.1);
  }

  // | ------- SafetyAreaManager is up and checking positions ------ |

  {
    auto sch_point_in_safety_area_2d =
        mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "/" + uav_name + "/safety_area_manager/point_in_safety_area_2d");

    auto request             = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();
    request->header.frame_id = uav_name + "/world_origin";

    auto response = sch_point_in_safety_area_2d.callSync(request);

    if (!response || !response.value()->success) {
      RCLCPP_ERROR(node_->get_logger(), "the safety area center (0, 0) is not reported as inside");
      return false;
    }
  }

  // | ------------- the UAV's own position is invalid ------------ |

  // a few seconds of fresh messages, all showing the UAV outside
  const rclcpp::Time check_start = clock_->now();

  {
    std::scoped_lock lock(msgs_mtx_);
    last_safety_area_diag_.reset();
    last_general_robot_info_.reset();
  }

  while ((clock_->now() - check_start).seconds() < 5.0) {

    if (!rclcpp::ok()) {
      return false;
    }

    {
      std::scoped_lock lock(msgs_mtx_);

      if (last_safety_area_diag_ && last_safety_area_diag_->position_valid_2d) {
        RCLCPP_ERROR(node_->get_logger(), "SafetyAreaManager reports the UAV inside the safety area, but it stands at (500, 500)");
        return false;
      }

      if (last_general_robot_info_ && last_general_robot_info_->preflight_status.position_known && last_general_robot_info_->preflight_status.position_valid) {
        RCLCPP_ERROR(node_->get_logger(), "DiagnosticsManager reports a valid position, but the UAV stands outside the safety area");
        return false;
      }
    }

    sleep(0.1);
  }

  std::scoped_lock lock(msgs_mtx_);

  if (!last_safety_area_diag_) {
    RCLCPP_ERROR(node_->get_logger(), "no SafetyAreaManager diagnostics received");
    return false;
  }

  if (!last_general_robot_info_ || !last_general_robot_info_->preflight_status.position_known) {
    RCLCPP_ERROR(node_->get_logger(), "DiagnosticsManager does not see SafetyAreaManager's verdict (position_known is false)");
    return false;
  }

  return true;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(2.0);
  tester.reportTestResult(result);
  tester.join();
}
