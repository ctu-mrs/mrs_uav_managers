#pragma once

#include <algorithm>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <mrs_lib/errorgraph/errorgraph.h>

namespace mrs_uav_managers::diagnostics_manager
{

/* error_lines() //{ */

// renders the errorgraph roots as the lines of GeneralRobotInfo.errors, each line once:
// - "<node>.<component>: not responding" for a root that stopped reporting
// - "<node>.<component>: <error>" for each error of a node root; "waiting for <node>.<component>" for a wait for a node, a
//   wait for a topic is left out (the topic's own line, or the root of its expected publisher, says more)
// - "<node>.<component>, ...: waiting for <topic> (expected from <node>.<component>)" for a missing topic, one line per
//   topic and expected publisher, listing the nodes waiting for it (at most max_listed_nodes, then "+N more"; a node that
//   stopped reporting is listed as not responding instead); without the part in brackets if the expected publisher is
//   unknown
inline std::vector<std::string> error_lines(const std::vector<mrs_lib::errorgraph::Errorgraph::element_info_t> &roots) {

  // the status TUI cuts long lines, so the topic must not be pushed out by the list of the nodes waiting for it
  constexpr size_t max_listed_nodes = 2;

  using mrs_lib::errorgraph::Errorgraph;
  using mrs_lib::errorgraph::node_id_t;

  const auto name = [](const node_id_t &id) { return id.node + "." + id.component; };

  std::vector<std::string> lines;

  const auto add_line = [&lines](const std::string &line) {
    if (std::find(lines.begin(), lines.end(), line) == lines.end()) {
      lines.push_back(line);
    }
  };

  // the nodes waiting for each missing topic, keyed by the line's text after the nodes, in the order of the roots
  std::vector<std::pair<std::string, std::vector<std::string>>> topic_waits;

  for (const auto &root : roots) {

    if (const auto *node = std::get_if<Errorgraph::node_info_t>(&root)) {

      if (node->not_reporting) {
        add_line(name(node->source_node) + ": not responding");
      }

      for (const auto &error : node->errors) {

        if (error.is_waiting_for_topic()) {
          continue;
        }

        if (error.is_waiting_for_node() && error.waited_for_node.has_value()) {
          add_line(name(node->source_node) + ": waiting for " + name(error.waited_for_node.value()));
        } else {
          add_line(name(node->source_node) + ": " + error.description);
        }
      }

    } else {

      const auto &topic = std::get<Errorgraph::topic_info_t>(root);

      // the wait of a node that stopped reporting is stale
      if (topic.not_reporting) {
        add_line(name(topic.source_node) + ": not responding");
        continue;
      }

      std::string what = "waiting for " + topic.topic_name;
      if (topic.expected_publisher != node_id_t{}) {
        what += " (expected from " + name(topic.expected_publisher) + ")";
      }

      auto it = std::find_if(topic_waits.begin(), topic_waits.end(), [&what](const auto &wait) { return wait.first == what; });
      if (it == topic_waits.end()) {
        it = topic_waits.insert(topic_waits.end(), {what, {}});
      }
      if (std::find(it->second.begin(), it->second.end(), name(topic.source_node)) == it->second.end()) {
        it->second.push_back(name(topic.source_node));
      }
    }
  }

  for (const auto &[what, waiting_nodes] : topic_waits) {

    std::string line;
    for (size_t i = 0; i < waiting_nodes.size() && i < max_listed_nodes; i++) {
      line += (line.empty() ? "" : ", ") + waiting_nodes[i];
    }
    if (waiting_nodes.size() > max_listed_nodes) {
      line += " +" + std::to_string(waiting_nodes.size() - max_listed_nodes) + " more";
    }

    add_line(line + ": " + what);
  }

  return lines;
}

//}

} // namespace mrs_uav_managers::diagnostics_manager
