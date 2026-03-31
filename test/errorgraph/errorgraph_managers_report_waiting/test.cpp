#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>
#include <mrs_msgs/msg/errorgraph_element.hpp>
#include <mrs_msgs/msg/errorgraph_error.hpp>

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
  struct TopicState
  {
    rclcpp::Subscription<mrs_msgs::msg::ErrorgraphElement>::SharedPtr sub;
    std::optional<mrs_msgs::msg::ErrorgraphElement>                   last_msg;
    bool                                                              saw_waiting_error = false;
  };

  std::mutex              mtx_;
  std::vector<TopicState> topics_;

  // Expected source_node names and their known dependencies
  const std::vector<std::string> topic_names_ = {
      "/uav1/estimation_manager/errors",
      "/uav1/control_manager/errors",
      "/uav1/uav_manager/errors",
  };

  const std::vector<std::string> expected_source_nodes_ = {
      "EstimationManager",
      "ControlManager",
      "UavManager",
  };

  const std::set<std::string> known_dependencies_ = {
      "HwApiManager",
      "ControlManager",
      "EstimationManager",
  };

  void errorsCallback(size_t idx, const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg);
};

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  topics_.resize(topic_names_.size());

  for (size_t i = 0; i < topic_names_.size(); i++) {
    topics_[i].sub = node_->create_subscription<mrs_msgs::msg::ErrorgraphElement>(
        topic_names_[i], 100, [this, i](const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg) { errorsCallback(i, msg); });
  }
}

void Tester::errorsCallback(size_t idx, const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg) {
  std::scoped_lock lck(mtx_);
  topics_[idx].last_msg = *msg;
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

    for (size_t i = 0; i < topics_.size(); i++) {

      if (!topics_[i].last_msg.has_value()) {
        continue;
      }

      const auto &element = topics_[i].last_msg.value();

      // Verify the source_node matches what we expect
      if (element.source_node.node != expected_source_nodes_[i] || element.source_node.component != "main") {
        continue;
      }

      for (const auto &error : element.errors) {
        if (error.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_NODE) {

          // Verify waited_for_node is a known dependency
          if (known_dependencies_.count(error.waited_for_node.node) > 0) {
            topics_[i].saw_waiting_error = true;
            RCLCPP_INFO(node_->get_logger(), "%s is waiting for node: %s", expected_source_nodes_[i].c_str(), error.waited_for_node.node.c_str());
          }
        }
      }
    }

    // Check if at least one manager reported waiting errors
    for (const auto &t : topics_) {
      if (t.saw_waiting_error) {
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
