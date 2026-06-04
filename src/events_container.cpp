#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager_isolated.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // 1. Instantiate the Isolated Component Manager, templated with the EventsExecutor
  // This tells the manager to create a new EventsExecutor for each loaded node.
  auto manager = std::make_shared<rclcpp_components::ComponentManagerIsolated<rclcpp::experimental::executors::EventsExecutor>>();

  // 2. The manager itself still needs an executor to handle the /ComponentManager service calls (load/unload)
  rclcpp::experimental::executors::EventsExecutor container_executor;
  container_executor.add_node(manager);

  // 3. Spin the container's service thread. 
  // As your launch file loads components, the manager will spin up separate threads for them automatically.
  container_executor.spin();

  rclcpp::shutdown();
  return 0;
}
