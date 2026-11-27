#include <gtest/gtest.h>

#include <mrs_uav_managers/diagnostics_manager/error_lines.hpp>

using mrs_lib::errorgraph::Errorgraph;
using mrs_lib::errorgraph::node_id_t;
using mrs_uav_managers::diagnostics_manager::error_lines;

namespace
{

Errorgraph::node_info_t node(const std::string &name, const std::vector<std::string> &error_types, bool not_reporting = false) {
  std::vector<Errorgraph::error_t> errors;
  for (const auto &type : error_types) {
    mrs_msgs::msg::ErrorgraphError msg;
    msg.type = type;
    errors.emplace_back(msg);
  }
  return Errorgraph::node_info_t{node_id_t{name, "main"}, errors, rclcpp::Time(), not_reporting};
}

Errorgraph::topic_info_t topic(const std::string &topic_name, const node_id_t &waiting_node, const node_id_t &expected_publisher, bool not_reporting = false) {
  return Errorgraph::topic_info_t{topic_name, waiting_node, expected_publisher, rclcpp::Time(), not_reporting};
}

} // namespace

TEST(ErrorLines, NodeErrors) {
  const auto lines = error_lines({node("EstimationManager", {"no state estimator", "no gnss"})});
  EXPECT_EQ(lines, (std::vector<std::string>{"EstimationManager.main: no state estimator", "EstimationManager.main: no gnss"}));
}

TEST(ErrorLines, NodeWaitsAreReadable) {
  // a node root with a genuine error also lists its waits: a wait for a node names the node, a wait for a topic is left
  // out (the topic has its own line)
  mrs_msgs::msg::ErrorgraphError genuine, waiting_for_node, waiting_for_topic, no_error;
  genuine.type                               = "no state estimator";
  waiting_for_node.type                      = mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_NODE;
  waiting_for_node.waited_for_node.node      = "HwApiManager";
  waiting_for_node.waited_for_node.component = "main";
  waiting_for_topic.type                     = mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_TOPIC;
  waiting_for_topic.waited_for_topic         = "/uav1/hw_api/gnss";
  no_error.type                              = mrs_msgs::msg::ErrorgraphError::TYPE_NO_ERROR;

  const Errorgraph::node_info_t estimation_manager{
      node_id_t{"EstimationManager", "main"},
      {Errorgraph::error_t(genuine), Errorgraph::error_t(waiting_for_node), Errorgraph::error_t(waiting_for_topic), Errorgraph::error_t(no_error)},
      rclcpp::Time(),
      false};

  const auto lines = error_lines({estimation_manager, topic("/uav1/hw_api/gnss", {"EstimationManager", "main"}, {"HwApiManager", "main"})});
  EXPECT_EQ(lines, (std::vector<std::string>{
                       "EstimationManager.main: no state estimator",
                       "EstimationManager.main: waiting for HwApiManager.main",
                       "EstimationManager.main: waiting for /uav1/hw_api/gnss (expected from HwApiManager.main)",
                   }));
}

TEST(ErrorLines, NodeNotResponding) {
  const auto lines = error_lines({node("HwApiManager", {}, true)});
  EXPECT_EQ(lines, (std::vector<std::string>{"HwApiManager.main: not responding"}));
}

TEST(ErrorLines, TopicWithExpectedPublisher) {
  const auto lines = error_lines({topic("/uav1/hw_api/imu", {"DiagnosticsManager", "imu"}, {"HwApiManager", "main"})});
  EXPECT_EQ(lines, (std::vector<std::string>{"DiagnosticsManager.imu: waiting for /uav1/hw_api/imu (expected from HwApiManager.main)"}));
}

TEST(ErrorLines, TopicWithoutExpectedPublisher) {
  const auto lines = error_lines({topic("/uav1/odometry/rtk", {"EstimationManager", "rtk"}, node_id_t{})});
  EXPECT_EQ(lines, (std::vector<std::string>{"EstimationManager.rtk: waiting for /uav1/odometry/rtk"}));
}

TEST(ErrorLines, NodesWaitingForTheSameTopicShareALine) {
  const node_id_t hw_api{"HwApiManager", "main"};
  const auto      lines =
      error_lines({topic("/uav1/hw_api/imu", {"DiagnosticsManager", "imu"}, hw_api), topic("/uav1/hw_api/gnss", {"DiagnosticsManager", "gnss"}, hw_api),
                   topic("/uav1/hw_api/imu", {"EstimationManager", "main"}, hw_api)});
  EXPECT_EQ(lines, (std::vector<std::string>{
                       "DiagnosticsManager.imu, EstimationManager.main: waiting for /uav1/hw_api/imu (expected from HwApiManager.main)",
                       "DiagnosticsManager.gnss: waiting for /uav1/hw_api/gnss (expected from HwApiManager.main)",
                   }));
}

TEST(ErrorLines, LongListOfWaitingNodesIsCut) {
  const node_id_t hw_api{"HwApiManager", "main"};
  const auto      lines =
      error_lines({topic("/uav1/hw_api/gnss", {"EstimationManager", "main"}, hw_api), topic("/uav1/hw_api/gnss", {"EstimationManager", "lat_gps"}, hw_api),
                   topic("/uav1/hw_api/gnss", {"EstimationManager", "alt_gps"}, hw_api), topic("/uav1/hw_api/gnss", {"EstimationManager", "hdg_gps"}, hw_api)});
  EXPECT_EQ(lines, (std::vector<std::string>{
                       "EstimationManager.main, EstimationManager.lat_gps +2 more: waiting for /uav1/hw_api/gnss (expected from HwApiManager.main)"}));
}

TEST(ErrorLines, SameTopicFromDifferentPublishersIsTwoLines) {
  const auto lines = error_lines({topic("/t", {"A", "main"}, {"P", "main"}), topic("/t", {"B", "main"}, node_id_t{})});
  EXPECT_EQ(lines, (std::vector<std::string>{"A.main: waiting for /t (expected from P.main)", "B.main: waiting for /t"}));
}

TEST(ErrorLines, NotRespondingWaitingNodeIsListedOnce) {
  // a waiting node that stopped reporting: its waits are stale, it is listed once as not responding
  const node_id_t waiting{"DiagnosticsManager", "imu"};
  const auto      lines = error_lines({topic("/a", waiting, node_id_t{}, true), topic("/b", waiting, node_id_t{}, true)});
  EXPECT_EQ(lines, (std::vector<std::string>{"DiagnosticsManager.imu: not responding"}));
}

TEST(ErrorLines, Mixed) {
  const auto lines = error_lines({node("ControlManager", {"no tracker"}), topic("/t", {"A", "main"}, node_id_t{}), node("UavManager", {}, true)});
  EXPECT_EQ(lines, (std::vector<std::string>{"ControlManager.main: no tracker", "UavManager.main: not responding", "A.main: waiting for /t"}));
}
