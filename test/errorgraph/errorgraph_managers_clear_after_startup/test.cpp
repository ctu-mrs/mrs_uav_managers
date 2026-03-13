#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>
#include <mrs_msgs/msg/errorgraph_element.hpp>
#include <mrs_msgs/msg/errorgraph_error.hpp>

#include <mutex>
#include <vector>
#include <string>
#include <optional>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester();

  bool test(void);

private:
  struct TopicState {
    rclcpp::Subscription<mrs_msgs::msg::ErrorgraphElement>::SharedPtr sub;
    std::optional<mrs_msgs::msg::ErrorgraphElement> last_msg;
    int consecutive_clean = 0;
  };

  std::mutex mtx_;
  std::vector<TopicState> topics_;

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

  void errorsCallback(size_t idx, const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg);
};

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  topics_.resize(topic_names_.size());

  for (size_t i = 0; i < topic_names_.size(); i++) {
    topics_[i].sub = node_->create_subscription<mrs_msgs::msg::ErrorgraphElement>(
        topic_names_[i], 100,
        [this, i](const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg) { errorsCallback(i, msg); });
  }
}

void Tester::errorsCallback(size_t idx, const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg) {
  std::scoped_lock lck(mtx_);
  topics_[idx].last_msg = *msg;
}

bool Tester::test(void) {

  RCLCPP_INFO(node_->get_logger(), "Waiting for all 3 managers' errorgraph errors to clear after startup...");

  const double timeout_s = 90.0;
  const double poll_rate_s = 0.2;
  const int required_consecutive_clean = 3;

  double elapsed = 0.0;

  while (rclcpp::ok() && elapsed < timeout_s) {

    sleep(poll_rate_s);
    elapsed += poll_rate_s;

    std::scoped_lock lck(mtx_);

    bool all_cleared = true;

    for (size_t i = 0; i < topics_.size(); i++) {

      if (topics_[i].consecutive_clean >= required_consecutive_clean) {
        continue;  // already cleared
      }

      if (!topics_[i].last_msg.has_value()) {
        all_cleared = false;
        continue;
      }

      const auto& element = topics_[i].last_msg.value();

      if (element.source_node.node != expected_source_nodes_[i] || element.source_node.component != "main") {
        all_cleared = false;
        continue;
      }

      bool has_waiting_for_node = false;
      for (const auto& error : element.errors) {
        if (error.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_NODE) {
          has_waiting_for_node = true;
          break;
        }
      }

      if (!has_waiting_for_node) {
        topics_[i].consecutive_clean++;
        RCLCPP_INFO(node_->get_logger(), "%s: clean message %d/%d",
                     expected_source_nodes_[i].c_str(), topics_[i].consecutive_clean, required_consecutive_clean);
      } else {
        topics_[i].consecutive_clean = 0;
      }

      if (topics_[i].consecutive_clean < required_consecutive_clean) {
        all_cleared = false;
      }
    }

    if (all_cleared) {
      RCLCPP_INFO(node_->get_logger(), "SUCCESS: all 3 managers' errorgraph errors cleared after startup.");
      return true;
    }
  }

  // Report which topics didn't clear
  for (size_t i = 0; i < topics_.size(); i++) {
    if (topics_[i].consecutive_clean < required_consecutive_clean) {
      RCLCPP_ERROR(node_->get_logger(), "FAILED: %s did not clear (consecutive_clean=%d/%d)",
                   expected_source_nodes_[i].c_str(), topics_[i].consecutive_clean, required_consecutive_clean);
    }
  }

  return false;
}

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Tester tester;

  test_result &= tester.test();

  tester.sleep(2.0);

  std::cout << "Test: reporting test results" << std::endl;

  tester.reportTestResult(test_result);

  tester.join();
}
