#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>
#include <mrs_msgs/msg/errorgraph_element.hpp>
#include <mrs_msgs/msg/errorgraph_error.hpp>

#include <map>
#include <mutex>
#include <vector>
#include <string>
#include <optional>
#include <set>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester();

  bool test(void);

private:
  struct ManagerState
  {
    std::optional<mrs_msgs::msg::ErrorgraphElement> last_msg;
    bool                                            saw_waiting_error = false;
  };

  std::mutex                          mtx_;
  std::map<std::string, ManagerState> manager_states_;

  rclcpp::Subscription<mrs_msgs::msg::ErrorgraphElement>::SharedPtr sub_;

  const std::string topic_name_ = "/uav1/errors";

  const std::vector<std::string> expected_source_nodes_ = {
      "EstimationManager", "SafetyAreaManager", "TransformManager", "ConstraintManager", "GainManager", "ControlManager", "UavManager",
  };

  const std::set<std::string> known_dependencies_ = {
      "HwApiManager",
      "ControlManager",
      "EstimationManager",
  };

  void errorsCallback(const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg);
};

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  for (const auto &name : expected_source_nodes_) {
    manager_states_[name] = ManagerState{};
  }

  sub_ = node_->create_subscription<mrs_msgs::msg::ErrorgraphElement>(topic_name_, 100,
                                                                      [this](const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg) { errorsCallback(msg); });
}

void Tester::errorsCallback(const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg) {
  std::scoped_lock lck(mtx_);
  auto             it = manager_states_.find(msg->source_node.node);
  if (it != manager_states_.end()) {
    it->second.last_msg = *msg;
  }
}

bool Tester::test(void) {

  RCLCPP_INFO(node_->get_logger(), "Waiting for at least one manager to report waiting_for_node errors...");

  const double timeout_s   = 30.0;
  const double poll_rate_s = 0.2;

  double elapsed = 0.0;

  while (rclcpp::ok() && elapsed < timeout_s) {

    sleep(poll_rate_s);
    elapsed += poll_rate_s;

    std::scoped_lock lck(mtx_);

    for (auto &[name, state] : manager_states_) {

      if (!state.last_msg.has_value()) {
        continue;
      }

      const auto &element = state.last_msg.value();

      // Verify the source_node matches what we expect
      if (element.source_node.node != name || element.source_node.component != "main") {
        continue;
      }

      for (const auto &error : element.errors) {
        if (error.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_NODE) {

          // Verify waited_for_node is a known dependency
          if (known_dependencies_.count(error.waited_for_node.node) > 0) {
            state.saw_waiting_error = true;
            RCLCPP_INFO(node_->get_logger(), "%s is waiting for node: %s", name.c_str(), error.waited_for_node.node.c_str());
          }
        }
      }
    }

    // Check if at least one manager reported waiting errors
    for (const auto &[name, state] : manager_states_) {
      if (state.saw_waiting_error) {
        RCLCPP_INFO(node_->get_logger(), "SUCCESS: at least one manager reported waiting_for_node errors during startup.");
        return true;
      }
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "FAILED: no manager reported waiting_for_node errors within %.1f seconds.", timeout_s);
  return false;
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
