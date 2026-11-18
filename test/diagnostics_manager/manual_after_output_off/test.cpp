#include <rclcpp/rclcpp.hpp>

#include <mrs_msgs/msg/hw_api_status.hpp>
#include <mrs_msgs/msg/state.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <mrs_uav_managers/diagnostics_manager/enums/uav_state.hpp>
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

  // created early, a request sent right after creating the client can get lost
  auto client = node_->create_client<std_srvs::srv::SetBool>("/uav1/control_manager/toggle_output");

  // shared, the callbacks run in the executor's thread and can outlive this function
  struct Observed
  {
    std::atomic<uint8_t> state{255};
    std::atomic<uint8_t> airborne{255};
    std::atomic<bool>    saw_manual_in_air{false};
  };
  auto obs = std::make_shared<Observed>();

  auto sub_state = node_->create_subscription<mrs_msgs::msg::State>("/uav1/diagnostics_manager/uav_state", 10, [obs](const mrs_msgs::msg::State::SharedPtr m) {
    obs->state = m->state;
    if (m->state == mrs_msgs::msg::State::STATE_MANUAL && obs->airborne == mrs_msgs::msg::HwApiStatus::AIRBORNE_YES) {
      obs->saw_manual_in_air = true;
    }
  });
  auto sub_hw    = node_->create_subscription<mrs_msgs::msg::HwApiStatus>("/uav1/hw_api/status", 10,
                                                                          [obs](const mrs_msgs::msg::HwApiStatus::SharedPtr m) { obs->airborne = m->airborne; });

  // 255: no message received yet
  auto state_name = [](const uint8_t state) -> std::string {
    namespace dm = mrs_uav_managers::diagnostics_manager;
    return state == 255 ? "none" : dm::to_string(dm::from_ros<dm::state_t>(state));
  };
  auto airborne_name = [](const uint8_t airborne) -> std::string {
    switch (airborne) {
    case mrs_msgs::msg::HwApiStatus::AIRBORNE_UNKNOWN:
      return "UNKNOWN";
    case mrs_msgs::msg::HwApiStatus::AIRBORNE_NO:
      return "NO";
    case mrs_msgs::msg::HwApiStatus::AIRBORNE_YES:
      return "YES";
    default:
      return "none";
    }
  };

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

  if (!wait_for([&] { return obs->airborne == mrs_msgs::msg::HwApiStatus::AIRBORNE_NO; }, 30.0)) {
    RCLCPP_ERROR(node_->get_logger(), "airborne not NO before takeoff (got %s)", airborne_name(obs->airborne).c_str());
    return false;
  }

  {
    auto [success, msg] = uh->takeoff();
    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "takeoff failed: %s", msg.c_str());
      return false;
    }
  }

  if (!wait_for([&] { return obs->airborne == mrs_msgs::msg::HwApiStatus::AIRBORNE_YES && obs->state == mrs_msgs::msg::State::STATE_HOVER; }, 20.0)) {
    RCLCPP_ERROR(node_->get_logger(), "expected airborne YES + HOVER after takeoff (airborne %s, state %s)", airborne_name(obs->airborne).c_str(),
                 state_name(obs->state).c_str());
    return false;
  }

  // the HW API leaves offboard only after its input timeout, climb so the UAV is still in the air by then
  {
    auto [success, msg] = uh->gotoAltitude(5.0);
    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "goto altitude failed: %s", msg.c_str());
      return false;
    }
  }

  auto req  = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = false;
  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node_->get_logger(), "toggle_output service not available");
    return false;
  }
  auto future = client->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "toggle_output service did not respond");
    return false;
  }
  const auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(node_->get_logger(), "toggle_output service failed: %s", response->message.c_str());
    return false;
  }

  if (!wait_for([&] { return obs->saw_manual_in_air.load(); }, 10.0)) {
    RCLCPP_ERROR(node_->get_logger(), "never saw MANUAL in the air after switching the output off (state %s, airborne %s)", state_name(obs->state).c_str(),
                 airborne_name(obs->airborne).c_str());
    return false;
  }

  if (!wait_for(
          [&] {
            return obs->airborne == mrs_msgs::msg::HwApiStatus::AIRBORNE_NO &&
                   (obs->state == mrs_msgs::msg::State::STATE_ARMED || obs->state == mrs_msgs::msg::State::STATE_DISARMED);
          },
          20.0)) {
    RCLCPP_ERROR(node_->get_logger(), "expected airborne NO + ARMED/DISARMED after falling (airborne %s, state %s)", airborne_name(obs->airborne).c_str(),
                 state_name(obs->state).c_str());
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
