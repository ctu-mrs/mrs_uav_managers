/**
 * @file diagnostics_manager.hpp
 * @brief UAV state monitoring and diagnostics aggregation node.
 *
 * The DiagnosticsManager is a ROS2 composable node that aggregates data from multiple
 * subsystems (HW API, control manager, estimation, sensors) into unified
 * diagnostics messages. It maintains a UAV state machine, publishes system
 * health information (CPU, RAM, GNSS, magnetometer, WiFi signal), and monitors
 * sensor status via pluginlib-loaded sensor handler plugins.
 *
 * Published topics include: GeneralRobotInfo, StateEstimationInfo, ControlInfo,
 * CollisionAvoidanceInfo, UavInfo, SystemHealthInfo, UavState, and error graphs.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include <mrs_msgs/msg/collision_avoidance_info.hpp>
#include <mrs_msgs/msg/control_info.hpp>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/cpu_load.hpp>
#include <mrs_msgs/msg/errorgraph_element.hpp>
#include <mrs_msgs/msg/errorgraph_element_array.hpp>
#include <mrs_msgs/msg/estimation_diagnostics.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>
#include <mrs_msgs/msg/constraint_manager_diagnostics.hpp>
#include <mrs_msgs/msg/gain_manager_diagnostics.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/gps_info.hpp>
#include <mrs_msgs/msg/hw_api_capabilities.hpp>
#include <mrs_msgs/msg/hw_api_rc_rssi.hpp>
#include <mrs_msgs/msg/hw_api_status.hpp>
#include <mrs_msgs/msg/mpc_tracker_diagnostics.hpp>
#include <mrs_msgs/msg/safety_area_manager_diagnostics.hpp>
#include <mrs_msgs/msg/sensor_status.hpp>
#include <mrs_msgs/msg/state_estimation_info.hpp>
#include <mrs_msgs/msg/system_health_info.hpp>
#include <mrs_msgs/msg/tracker_command.hpp>
#include <mrs_msgs/msg/uav_info.hpp>
#include <mrs_msgs/msg/uav_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <mrs_lib/errorgraph/errorgraph.h>
#include <mrs_lib/errorgraph/error_publisher.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/subscriber_handler.h>

#include <mrs_uav_managers/diagnostics_manager/enums/helpers/enum_helpers.hpp>
#include <mrs_uav_managers/diagnostics_manager/enums/robot_type.hpp>
#include <mrs_uav_managers/diagnostics_manager/enums/tracker_state.hpp>
#include <mrs_uav_managers/diagnostics_manager/enums/uav_state.hpp>

#include <mrs_uav_managers/diagnostics_manager/diagnostics_sensor_handler.hpp>
#include <mrs_uav_managers/diagnostics_manager/preflight_checker.hpp>

#include <mrs_uav_managers/diagnostics_manager/utils/flight_timer.hpp>
#include <mrs_uav_managers/diagnostics_manager/utils/host_stats.hpp>
#include <mrs_uav_managers/diagnostics_manager/utils/rate_tracker.hpp>
#include <mrs_uav_managers/diagnostics_manager/utils/wh_drained_integrator.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <sstream>
#include <cstring>

#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>

namespace mrs_uav_managers
{
namespace diagnostics_manager
{

/**
 * @brief ROS2 composable node that aggregates UAV diagnostics from multiple subsystems.
 *
 * Subscribes to HW API, control manager, estimation, battery, GNSS, magnetometer,
 * and RC channels. Publishes unified diagnostics messages at configurable rates.
 * Maintains a UAV state machine (DISARMED, ARMED, OFFBOARD, TAKEOFF, HOVER, etc.)
 * and monitors system health including CPU, RAM, WiFi signal strength, and sensor status.
 */
class DiagnosticsManager : public mrs_lib::Node {

public:
  /**
   * @brief Construct the DiagnosticsManager node.
   * @param options ROS2 node options (used for composable node loading).
   */
  DiagnosticsManager(rclcpp::NodeOptions options);

  /**
   * @brief Result of checking a subscriber for new messages.
   * @tparam T The ROS message type.
   */
  template <typename T>
  struct subscriptionResult_t
  {
    bool                       hasNewMessage; ///< true if a new message arrived since last check
    typename T::ConstSharedPtr message;       ///< latest message, or nullptr if timed out
  };

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;   ///< callback group for subscribers
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_; ///< callback group for timers

  std::unique_ptr<PreflightChecker> preflight_checker_; ///< helper object for performing preflight checks
  /** @brief Load parameters, create subscribers/publishers/timers, initialize plugins. */
  void initialize(void);

  /** @brief Graceful shutdown. */
  void shutdown();

  std::atomic<bool> is_initialized_ = false;
  std::string       _uav_name_;
  std::string       _body_frame_;

  std::shared_ptr<mrs_lib::ParamLoader> param_loader_;

  std::mutex                          uav_state_mutex_; ///< guards uav_state_ across timer callbacks
  enum_helpers::enum_updater<state_t> uav_state_;

  std::mutex errorgraph_mtx_; ///< guards errorgraph_ across timer and subscriber callbacks

  mrs_lib::errorgraph::Errorgraph      errorgraph_;            ///< dependency/error graph for readiness tracking
  rclcpp::Duration                     not_reporting_timeout_; ///< timeout before marking a topic as not reporting
  const mrs_lib::errorgraph::node_id_t autostart_node_id_ = {"AutomaticStart", "main"};

  std::unique_ptr<mrs_lib::errorgraph::ErrorPublisher> error_publisher_; ///< reports this manager's own fatal errors to the errorgraph

  std::string  _robot_name_;
  std::string  robot_ip_address_;
  robot_type_t robot_type_;

  std::vector<mrs_msgs::msg::SensorStatus> available_sensors_;

  // | ---------------------- ROS subscribers --------------------- |
  std::shared_ptr<mrs_lib::TimeoutManager> tim_mgr_;

  // | -------------------- GeneralRobotInfo -------------------- |
  mrs_lib::PublisherHandler<mrs_msgs::msg::GeneralRobotInfo> ph_general_robot_info_;
  mrs_msgs::msg::GeneralRobotInfo                            last_general_robot_info_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::BatteryState> sh_battery_state_;

  // | ------------------- StateEstimationInfo ------------------ |
  mrs_lib::PublisherHandler<mrs_msgs::msg::StateEstimationInfo>    ph_state_estimation_info_;
  mrs_msgs::msg::StateEstimationInfo                               last_state_estimation_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics> sh_estimation_diagnostics_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix>          sh_hw_api_gnss_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::GpsInfo>               sh_hw_api_gnss_status_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>        sh_control_manager_heading_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>        sh_hw_api_mag_heading_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiRcRssi>           sh_hw_api_rc_rssi_;

  // | ----------------------- ControlInfo ---------------------- |
  mrs_lib::PublisherHandler<mrs_msgs::msg::ControlInfo>                   ph_control_info_;
  mrs_msgs::msg::ControlInfo                                              last_control_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ConstraintManagerDiagnostics> sh_constraint_manager_diagnostics_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>    sh_control_manager_diagnostics_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::GainManagerDiagnostics>       sh_gain_manager_diagnostics_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Float64>                      sh_control_manager_thrust_;

  // | ----------------- CollisionAvoidanceInfo ----------------- |
  mrs_lib::PublisherHandler<mrs_msgs::msg::CollisionAvoidanceInfo> ph_collision_avoidance_info_;
  mrs_msgs::msg::CollisionAvoidanceInfo                            last_collision_avoidance_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::MpcTrackerDiagnostics> sh_mpc_tracker_diagnostics_;

  // | ------------------------- UavInfo ------------------------ |
  mrs_lib::PublisherHandler<mrs_msgs::msg::UavInfo>         ph_uav_info_;
  mrs_msgs::msg::UavInfo                                    last_uav_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus>    sh_hw_api_status_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand> sh_tracker_cmd_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Float64>        sh_mass_nominal_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Float64>        sh_mass_estimate_;

  // | -------------------- SystemHealthInfo -------------------- |
  mrs_lib::PublisherHandler<mrs_msgs::msg::SystemHealthInfo>  ph_system_health_info_;
  mrs_msgs::msg::SystemHealthInfo                             last_system_health_info_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField> sh_hw_api_magnetic_field_;
  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>         sh_hw_api_odometry_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::UavState>         sh_estimator_uav_state_;

  void cbk_hw_api_odometry_rate(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void cbk_estimator_uav_state_rate(const mrs_msgs::msg::UavState::ConstSharedPtr msg);
  void cbk_control_manager_diag_rate(const mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr msg);

  // | -------------------- Acquisition utils ------------------- |
  // Host stats / flight timer / wh-drained integrator
  std::unique_ptr<utils::HostStats>           host_stats_;
  std::unique_ptr<utils::FlightTimer>         flight_timer_;
  std::unique_ptr<utils::WhDrainedIntegrator> wh_drained_integrator_;

  utils::RateTracker rate_hw_api_odometry_{50};
  utils::RateTracker rate_control_manager_diag_{50};
  utils::RateTracker rate_estimator_uav_state_{50};

  /** @brief 1 Hz refresh of /proc-backed stats. */
  std::shared_ptr<TimerType> timer_host_info_;
  void                       timerHostInfo();

  // | ------------------------ UAV state ----------------------- |
  mrs_lib::PublisherHandler<mrs_msgs::msg::State> ph_uav_state_;

  // | ----------------------- Root errors ----------------------- |
  mrs_lib::PublisherHandler<mrs_msgs::msg::ErrorgraphElementArray> ph_root_errors_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ErrorgraphElement>     sh_errorgraph_error_msg_;

  // | -------------------- Sensor handlers --------------------- |
  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_managers::DiagnosticsSensorHandler>> sensor_handler_loader_; ///< pluginlib loader for sensor handler plugins
  std::vector<std::string>                                                            _sensor_handler_names_;
  std::vector<std::shared_ptr<mrs_uav_managers::DiagnosticsSensorHandler>>            sensor_handlers_;
  std::mutex                                                                          mutex_sensor_handler_list_;

  // | ----------------------- Timers --------------------------- |

  std::shared_ptr<TimerType> timer_main_;
  /**
   * @brief Main diagnostics loop.
   * Polls all subscribers and publishes diagnostics. Each *_info
   * topic is (re)published only when fresh input arrived since the last tick —
   * polling the latest message.
   */
  void timerMain();

  std::shared_ptr<TimerType> timer_uav_state_;
  /**
   * @brief Fast UAV-state path: recomputes the state machine from hw_api/status
   * (100 Hz) + control_manager diagnostics and publishes uav_state the moment the
   * state changes.
   */
  void timerUavState();

  std::shared_ptr<TimerType> timer_error_publishing_;
  /** @brief Publishes root error graph elements for upstream monitoring. */
  void timerErrorPublishing();

  std::shared_ptr<TimerType> timer_update_sensor_status_;
  /** @brief Polls all sensor handler plugins and updates available_sensors_. */
  void timerUpdateSensorStatus();

  // | ------------------------ Callbacks ----------------------- |

  /** @brief Callback for incoming error graph elements from other nodes. */
  void cbk_errorgraph_element(const mrs_msgs::msg::ErrorgraphElement::ConstSharedPtr element_msg);

  /**
   * @brief Check a subscriber for new messages, applying the not-reporting timeout.
   * @tparam sh_T The ROS message type of the subscriber.
   * @param sh The subscriber handler to check.
   * @return subscriptionResult_t with the latest message and whether it is new.
   *
   * If the subscriber has not received a message within not_reporting_timeout_,
   * the message is set to nullptr and hasNewMessage is set to true (to trigger
   * downstream handling of the "not reporting" case).
   */
  template <typename sh_T>
  subscriptionResult_t<sh_T> processIncomingMessage(mrs_lib::SubscriberHandler<sh_T> &sh);

  // | -------------------- Parsing methods --------------------- |

  /** @brief Map ControlManagerDiagnostics tracker status to internal tracker_state_t. */
  tracker_state_t parse_tracker_state(mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr control_manager_diagnostics);

  /** @brief Map a robot type string (e.g. "multirotor") to robot_type_t enum. */
  robot_type_t parse_robot_type(const std::string &robot_type_str);

  /** @brief Determine overall UAV state from HW API status and control manager diagnostics. */
  state_t parse_uav_state(mrs_msgs::msg::HwApiStatus::ConstSharedPtr               hw_api_status,
                          mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr control_manager_diagnostics);

  /** @brief Build GeneralRobotInfo from battery state, preflight result used to populate ready_to_start and problems_preventing_start fields. */
  mrs_msgs::msg::GeneralRobotInfo parse_general_robot_info(sensor_msgs::msg::BatteryState::ConstSharedPtr battery_state);

  /** @brief Build StateEstimationInfo from estimation diagnostics, headings, and GNSS. */
  mrs_msgs::msg::StateEstimationInfo parse_state_estimation_info(mrs_msgs::msg::EstimationDiagnostics::ConstSharedPtr estimation_diagnostics,
                                                                 mrs_msgs::msg::Float64Stamped::ConstSharedPtr        local_heading,
                                                                 sensor_msgs::msg::NavSatFix::ConstSharedPtr          global_position,
                                                                 mrs_msgs::msg::Float64Stamped::ConstSharedPtr        global_heading);

  /** @brief Build ControlInfo from control manager diagnostics, thrust, and the current tracker command setpoint. */
  mrs_msgs::msg::ControlInfo parse_control_info(mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr    control_manager_diagnostics,
                                                mrs_msgs::msg::ConstraintManagerDiagnostics::ConstSharedPtr constraint_manager_diagnostics,
                                                mrs_msgs::msg::GainManagerDiagnostics::ConstSharedPtr       gain_manager_diagnostics,
                                                std_msgs::msg::Float64::ConstSharedPtr thrust, mrs_msgs::msg::TrackerCommand::ConstSharedPtr tracker_cmd);

  /** @brief Build CollisionAvoidanceInfo from MPC tracker diagnostics. */
  mrs_msgs::msg::CollisionAvoidanceInfo parse_collision_avoidance_info(mrs_msgs::msg::MpcTrackerDiagnostics::ConstSharedPtr     mpc_tracker_diagnostics,
                                                                       mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr control_manager_diagnostics);

  /** @brief Build UavInfo from HW API status, locally-tracked flight timer, and mass estimates. */
  mrs_msgs::msg::UavInfo parse_uav_info(mrs_msgs::msg::HwApiStatus::ConstSharedPtr hw_api_status, std_msgs::msg::Float64::ConstSharedPtr mass_nominal,
                                        std_msgs::msg::Float64::ConstSharedPtr mass_estimate);

  /** @brief Build SystemHealthInfo from locally-collected host stats, rate trackers, sensor handlers, and WiFi. */
  mrs_msgs::msg::SystemHealthInfo parse_system_health_info();

  // | ------------------- Init methods ------------------------- |

  /** @brief Create a default-initialized StateEstimationInfo message (NaN-filled for nested types). */
  mrs_msgs::msg::StateEstimationInfo init_state_estimation_info();
};

/**
 * @brief Template definition for processIncomingMessage.
 *
 * Must be in the header because it is a template method instantiated
 * with multiple message types in the .cpp file.
 */
template <typename sh_T>
DiagnosticsManager::subscriptionResult_t<sh_T> DiagnosticsManager::processIncomingMessage(mrs_lib::SubscriberHandler<sh_T> &sh) {
  DiagnosticsManager::subscriptionResult_t<sh_T> msg;
  msg.hasNewMessage = sh.newMsg();
  msg.message       = msg.hasNewMessage ? sh.getMsg() : sh.peekMsg();
  if (msg.message != nullptr) {
    if (clock_->now() - sh.lastMsgTime() > not_reporting_timeout_) {
      msg.message       = nullptr;
      msg.hasNewMessage = true;
    }
  }
  return msg;
}

} // namespace diagnostics_manager
} // namespace mrs_uav_managers
