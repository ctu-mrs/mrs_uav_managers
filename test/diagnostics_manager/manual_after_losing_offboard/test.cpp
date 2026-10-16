#include <rclcpp/rclcpp.hpp>

#include <mrs_msgs/msg/hw_api_status.hpp>
#include <mrs_msgs/msg/state.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <mrs_uav_testing/test_generic.h>

#include <atomic>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test(void);
};

bool Tester::test(void) {

  auto [uhopt, message] = getUAVHandler("uav1");
  if (!uhopt) {
    RCLCPP_ERROR(node_->get_logger(), "no UAV handler: %s", message.c_str());
    return false;
  }
  auto uh = uhopt.value();

  std::atomic<uint8_t> state{255};
  std::atomic<uint8_t> airborne{255};
  std::atomic<bool>    saw_manual_in_air{false};

  auto sub_state = node_->create_subscription<mrs_msgs::msg::State>("/uav1/diagnostics_manager/uav_state", 10, [&](const mrs_msgs::msg::State::SharedPtr m) {
    state = m->state;
    if (m->state == mrs_msgs::msg::State::STATE_MANUAL && airborne == mrs_msgs::msg::HwApiStatus::AIRBORNE_YES) {
      saw_manual_in_air = true;
    }
  });
  auto sub_hw    = node_->create_subscription<mrs_msgs::msg::HwApiStatus>("/uav1/hw_api/status", 10,
                                                                          [&](const mrs_msgs::msg::HwApiStatus::SharedPtr m) { airborne = m->airborne; });

  auto wait_for = [&](auto pred, double timeout) {
    const auto start = node_->get_clock()->now();
    while (rclcpp::ok() && (node_->get_clock()->now() - start).seconds() < timeout) {
      if (pred()) {
        return true;
      }
      sleep(0.05);
    }
    return false;
  };

  if (!wait_for([&] { return airborne == mrs_msgs::msg::HwApiStatus::AIRBORNE_NO; }, 30.0)) {
    RCLCPP_ERROR(node_->get_logger(), "airborne not NO before takeoff (got %d)", int(airborne));
    return false;
  }

  {
    auto [success, msg] = uh->takeoff();
    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "takeoff failed: %s", msg.c_str());
      return false;
    }
  }

  if (!wait_for([&] { return airborne == mrs_msgs::msg::HwApiStatus::AIRBORNE_YES && state == mrs_msgs::msg::State::STATE_HOVER; }, 20.0)) {
    RCLCPP_ERROR(node_->get_logger(), "expected airborne YES + HOVER after takeoff (airborne %d, state %d)", int(airborne), int(state));
    return false;
  }

  auto client = node_->create_client<std_srvs::srv::SetBool>("/uav1/control_manager/toggle_output");
  auto req    = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data   = false;
  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node_->get_logger(), "toggle_output service not available");
    return false;
  }
  client->async_send_request(req);

  if (!wait_for([&] { return saw_manual_in_air.load(); }, 10.0)) {
    RCLCPP_ERROR(node_->get_logger(), "never saw MANUAL while in the air after losing offboard (state %d, airborne %d)", int(state), int(airborne));
    return false;
  }

  if (!wait_for(
          [&] {
            return airborne == mrs_msgs::msg::HwApiStatus::AIRBORNE_NO &&
                   (state == mrs_msgs::msg::State::STATE_ARMED || state == mrs_msgs::msg::State::STATE_DISARMED);
          },
          20.0)) {
    RCLCPP_ERROR(node_->get_logger(), "expected airborne NO + ARMED/DISARMED after falling (airborne %d, state %d)", int(airborne), int(state));
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
