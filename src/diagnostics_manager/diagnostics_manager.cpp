#include <mrs_uav_managers/diagnostics_manager/diagnostics_manager.hpp>

namespace mrs_uav_managers::diagnostics_manager
{

/* DiagnosticsManager() //{ */

DiagnosticsManager::DiagnosticsManager(rclcpp::NodeOptions options)
    : mrs_lib::Node("diagnostics_manager", options), uav_state_(this_node_ptr()->get_logger(), "UAV STATE", state_t::UNKNOWN),
      errorgraph_(this_node_ptr()->get_clock()), not_reporting_timeout_(rclcpp::Duration::from_seconds(0.0)) {


  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  error_publisher_ = std::make_unique<mrs_lib::errorgraph::ErrorPublisher>(node_, clock_, "DiagnosticsManager", "main");
  cbkgrp_subs_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  initialize();
}

//}

/* initialize() //{ */

void DiagnosticsManager::initialize() {

  rclcpp::on_shutdown([this]() { this->shutdown(); });

  RCLCPP_INFO(node_->get_logger(), "Initializing...");

  auto use_intra = node_->get_node_options().use_intra_process_comms();
  RCLCPP_INFO(node_->get_logger(), "Intra-process comms is: %s", use_intra ? "ON" : "OFF");

  /*//{ load parameters */
  mrs_lib::ParamLoader param_loader(node_, "DiagnosticsManager");

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("private_config");
  param_loader.addYamlFileFromParam("public_config");
  param_loader.addYamlFileFromParam("private_sensor_handlers");
  param_loader.addYamlFileFromParam("public_sensor_handlers");

  std::string robot_type;
  param_loader.loadParam("robot_name", _robot_name_);
  param_loader.loadParam("robot_type", robot_type);
  param_loader.loadParam("simulation", _simulation_);

  robot_type_ = parse_robot_type(robot_type);
  /*//}*/

  resolveRobotIpAddress();

  std::string wifi_interface;
  param_loader.loadParam("mrs_uav_managers/diagnostics_manager/wifi_interface", wifi_interface, std::string(""));

  auto       main_timer_rate             = param_loader.loadParam2<double>("mrs_uav_managers/diagnostics_manager/rate/main");
  const auto state_timer_rate            = param_loader.loadParam2<double>("mrs_uav_managers/diagnostics_manager/rate/state");
  auto       error_publisher_timer_rate  = param_loader.loadParam2<double>("mrs_uav_managers/diagnostics_manager/rate/error_publisher");
  const auto host_info_timer_rate        = param_loader.loadParam2<double>("mrs_uav_managers/diagnostics_manager/rate/host_info");
  const auto node_cpu_discovery_period_s = param_loader.loadParam2<double>("mrs_uav_managers/diagnostics_manager/node_cpu_discovery_period");

  not_reporting_timeout_ = param_loader.loadParam2<rclcpp::Duration>("mrs_uav_managers/diagnostics_manager/timeout/not_reporting");

  param_loader.setPrefix("mrs_uav_managers/diagnostics_manager/sensor_handlers/");
  const auto update_status_rate = param_loader.loadParam2<double>("update_timer_rate");

  param_loader.loadParam("active_sensor_handlers", _sensor_handler_names_);

  loadSensorHandlers(param_loader);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    return;
  }

  // | ----------------------- subscribers ---------------------- |

  tim_mgr_ = std::make_shared<mrs_lib::TimeoutManager>(node_, rclcpp::Rate(1.0));
  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.node_name                           = "DiagnosticsManager";
  shopts.no_message_timeout                  = rclcpp::Duration(not_reporting_timeout_);
  shopts.timeout_manager                     = tim_mgr_;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  ph_root_errors_ = mrs_lib::PublisherHandler<mrs_msgs::msg::ErrorgraphElementArray>(node_, "~/errors_out");
  sh_errorgraph_error_msg_ =
      mrs_lib::SubscriberHandler<mrs_msgs::msg::ErrorgraphElement>(shopts, "~/errors_in", &DiagnosticsManager::cbk_errorgraph_element, this);

  // | -------------------- GeneralRobotInfo -------------------- |
  ph_general_robot_info_ = mrs_lib::PublisherHandler<mrs_msgs::msg::GeneralRobotInfo>(node_, "~/general_robot_info_out");
  sh_battery_state_      = mrs_lib::SubscriberHandler<sensor_msgs::msg::BatteryState>(shopts, "~/battery_state_in");

  // | ------------------- StateEstimationInfo ------------------ |
  ph_state_estimation_info_   = mrs_lib::PublisherHandler<mrs_msgs::msg::StateEstimationInfo>(node_, "~/state_estimation_info_out");
  last_state_estimation_info_ = init_state_estimation_info();
  sh_estimation_diagnostics_  = mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>(shopts, "~/estimation_diagnostics_in");
  sh_hw_api_gnss_             = mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix>(shopts, "~/hw_api_gnss_in");
  sh_control_manager_heading_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>(shopts, "~/control_manager_heading_in");
  sh_hw_api_mag_heading_      = mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>(shopts, "~/hw_api_mag_heading_in");

  // | ----------------------- ControlInfo ---------------------- |
  ph_control_info_                   = mrs_lib::PublisherHandler<mrs_msgs::msg::ControlInfo>(node_, "~/control_info_out");
  sh_constraint_manager_diagnostics_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::ConstraintManagerDiagnostics>(shopts, "~/constraint_manager_diagnostics_in");
  sh_control_manager_diagnostics_    = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(
      shopts, "~/control_manager_diagnostics_in", &DiagnosticsManager::cbk_control_manager_diag_rate, this);
  sh_control_manager_thrust_   = mrs_lib::SubscriberHandler<std_msgs::msg::Float64>(shopts, "~/control_manager_thrust_in");
  sh_gain_manager_diagnostics_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::GainManagerDiagnostics>(shopts, "~/gain_manager_diagnostics_in");

  // | ----------------- CollisionAvoidanceInfo ----------------- |
  ph_collision_avoidance_info_ = mrs_lib::PublisherHandler<mrs_msgs::msg::CollisionAvoidanceInfo>(node_, "~/collision_avoidance_info_out");
  sh_mpc_tracker_diagnostics_  = mrs_lib::SubscriberHandler<mrs_msgs::msg::MpcTrackerDiagnostics>(shopts, "~/mpc_tracker_diagnostics_in");

  // | ------------------------- UavInfo ------------------------ |
  ph_uav_info_      = mrs_lib::PublisherHandler<mrs_msgs::msg::UavInfo>(node_, "~/uav_info_out");
  sh_hw_api_status_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus>(shopts, "~/hw_api_status_in");
  sh_tracker_cmd_   = mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>(shopts, "~/tracker_cmd_in");
  {
    // control_manager publishes mass_nominal once, latched (transient_local); match its QoS here
    mrs_lib::SubscriberHandlerOptions shopts_mass_nominal = shopts;
    shopts_mass_nominal.qos                               = rclcpp::QoS(1).transient_local();
    sh_mass_nominal_                                      = mrs_lib::SubscriberHandler<std_msgs::msg::Float64>(shopts_mass_nominal, "~/mass_nominal_in");
  }
  sh_mass_estimate_ = mrs_lib::SubscriberHandler<std_msgs::msg::Float64>(shopts, "~/mass_estimate_in");

  // | -------- Acquisition utils ------ |
  host_stats_ = std::make_unique<utils::HostStats>();
  host_stats_->setWifiInterface(wifi_interface);
  {
    // Derive sample period from host_info_timer_rate; discovery period from the loaded param.
    const double safe_host_info_timer_rate = (host_info_timer_rate > 0.0) ? host_info_timer_rate : 1.0;
    const auto   sample_ms                 = std::chrono::milliseconds(static_cast<long>(1000.0 / safe_host_info_timer_rate));
    const auto   discovery_ms              = std::chrono::milliseconds(static_cast<long>(node_cpu_discovery_period_s * 1000.0));
    host_stats_->setNodeCpuPeriods(sample_ms, discovery_ms);
  }
  // Default paths match mrs_uav_status's legacy data_acquisition node (real robots: one host per UAV).
  // In simulation multiple UAVs share a host, so namespace by robot name there to avoid cross-UAV corruption.
  if (_simulation_) {
    flight_timer_          = std::make_unique<utils::FlightTimer>(clock_, "/tmp/mrs_status_flight_time_" + _robot_name_ + ".txt");
    wh_drained_integrator_ = std::make_unique<utils::WhDrainedIntegrator>(clock_, "/tmp/mrs_status_wh_drained_" + _robot_name_ + ".txt");
  } else {
    flight_timer_          = std::make_unique<utils::FlightTimer>(clock_);
    wh_drained_integrator_ = std::make_unique<utils::WhDrainedIntegrator>(clock_);
  }

  preflight_checker_ = std::make_unique<PreflightChecker>(node_, _robot_name_, not_reporting_timeout_);

  // | -------------------- SystemHealthInfo -------------------- |
  ph_system_health_info_ = mrs_lib::PublisherHandler<mrs_msgs::msg::SystemHealthInfo>(node_, "~/system_health_info_out");

  sh_hw_api_odometry_ =
      mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/hw_api_odometry_in", &DiagnosticsManager::cbk_hw_api_odometry_rate, this);
  sh_estimator_uav_state_ =
      mrs_lib::SubscriberHandler<mrs_msgs::msg::UavState>(shopts, "~/estimator_uav_state_in", &DiagnosticsManager::cbk_estimator_uav_state_rate, this);

  // | ------------------------ UAV state ----------------------- |
  ph_uav_state_ = mrs_lib::PublisherHandler<mrs_msgs::msg::State>(node_, "~/uav_state_out");

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fcn = std::bind(&DiagnosticsManager::timerMain, this);

    timer_main_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(main_timer_rate, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&DiagnosticsManager::timerUavState, this);

    timer_uav_state_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(state_timer_rate, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&DiagnosticsManager::timerErrorPublishing, this);

    timer_error_publishing_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(error_publisher_timer_rate, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&DiagnosticsManager::timerUpdateSensorStatus, this);

    timer_update_sensor_status_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(update_status_rate, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&DiagnosticsManager::timerHostInfo, this);

    timer_host_info_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(host_info_timer_rate, clock_), callback_fcn);
  }

  // | --------------------- finish the init -------------------- |

  RCLCPP_INFO(node_->get_logger(), " initialized ");
  RCLCPP_INFO(node_->get_logger(), "--------------------");
  is_initialized_ = true;
}

//}

/* resolveRobotIpAddress() //{ */

void DiagnosticsManager::resolveRobotIpAddress() {

  std::vector<char> hostname(1024);

  if (gethostname(hostname.data(), hostname.size()) == 0) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Hostname: " << hostname.data());
  } else {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to get hostname");
  }

  if (hostname.data() != _robot_name_) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Hostname '"
                                                << hostname.data() << "' does not match the robot name '" << _robot_name_
                                                << "'. This might lead to issues in IP resolution, if you are using the hostname to connect to the robot, "
                                                   "please check your network configuration and make sure the hostname is correct");
  }

  addrinfo hints{};
  hints.ai_family   = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  addrinfo *res = nullptr;

  if (getaddrinfo(hostname.data(), nullptr, &hints, &res) != 0) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to resolve "
                                                << hostname.data()
                                                << ", skipping IP resolution for this robot, if you are using the hostname to connect to the robot, "
                                                   "please check your network configuration and make sure the hostname is correct");
  } else {
    char  ip[INET_ADDRSTRLEN];
    void *addr = &((sockaddr_in *)res->ai_addr)->sin_addr;
    inet_ntop(AF_INET, addr, ip, sizeof(ip));

    robot_ip_address_ = std::string(ip);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Resolved IP address: " << robot_ip_address_);

    freeaddrinfo(res);
  }
}

//}

/* loadSensorHandlers() //{ */

void DiagnosticsManager::loadSensorHandlers(mrs_lib::ParamLoader &param_loader) {

  sensor_handler_loader_ =
      std::make_unique<pluginlib::ClassLoader<DiagnosticsSensorHandler>>("mrs_uav_managers", "mrs_uav_managers::diagnostics_manager::DiagnosticsSensorHandler");

  // For each plugin: load pluginlib address, create instance, and initialize.
  // A load or initialization failure is fatal (matches EstimationManager/ControlManager).
  for (const auto &config_key : _sensor_handler_names_) {

    std::string address;
    param_loader.loadParam(config_key + "/address", address);

    std::shared_ptr<DiagnosticsSensorHandler> handler;
    try {
      RCLCPP_INFO(node_->get_logger(), "[%s]: loading sensor handler (%s)", config_key.c_str(), address.c_str());
      handler = sensor_handler_loader_->createSharedInstance(address);
    }
    catch (pluginlib::CreateClassException &ex1) {
      RCLCPP_ERROR(node_->get_logger(), "[%s]: CreateClassException: %s", config_key.c_str(), ex1.what());
      error_publisher_->addOneshotError("Failed to load the sensor handler " + config_key + ": " + ex1.what());
      error_publisher_->flushAndShutdown();
      continue;
    }
    catch (pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(node_->get_logger(), "[%s]: PluginlibException: %s", config_key.c_str(), ex.what());
      error_publisher_->addOneshotError("Failed to load the sensor handler " + config_key + ": " + ex.what());
      error_publisher_->flushAndShutdown();
      continue;
    }

    try {
      if (!handler->initialize(node_, config_key, _robot_name_, cbkgrp_subs_)) {
        RCLCPP_ERROR(node_->get_logger(), "[%s]: failed to initialize", config_key.c_str());
        error_publisher_->addOneshotError("Sensor handler " + config_key + " failed to initialize");
        error_publisher_->flushAndShutdown();
        continue;
      }
      sensor_handlers_.push_back(handler);
    }
    catch (std::runtime_error &ex) {
      RCLCPP_ERROR(node_->get_logger(), "[%s]: exception during initialization: %s", config_key.c_str(), ex.what());
      error_publisher_->addOneshotError("Exception during sensor handler " + config_key + " initialization: " + ex.what());
      error_publisher_->flushAndShutdown();
    }
  }

  RCLCPP_INFO(node_->get_logger(), "%zu sensor handlers initialized successfully", sensor_handlers_.size());
}

//}

/* shutdown() //{ */

void DiagnosticsManager::shutdown() {

  RCLCPP_INFO(node_->get_logger(), "shutdown(): called");

  if (timer_main_) {
    timer_main_->stop();
  }

  if (timer_uav_state_) {
    timer_uav_state_->stop();
  }

  if (timer_error_publishing_) {
    timer_error_publishing_->stop();
  }

  if (timer_update_sensor_status_) {
    timer_update_sensor_status_->stop();
  }

  if (timer_host_info_) {
    timer_host_info_->stop();
  }

  RCLCPP_INFO(node_->get_logger(), "shutdown(): unloading %zu sensor handlers", sensor_handlers_.size());

  sensor_handlers_.clear();

  RCLCPP_INFO(node_->get_logger(), "shutdown(): done");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void DiagnosticsManager::timerMain() {
  if (!is_initialized_) {
    return;
  }
  std::scoped_lock lck(uav_state_mutex_);

  const auto now                            = clock_->now();
  const auto battery_state                  = processIncomingMessage(sh_battery_state_);
  const auto control_manager_diagnostics    = processIncomingMessage(sh_control_manager_diagnostics_);
  const auto control_manager_heading        = processIncomingMessage(sh_control_manager_heading_);
  const auto control_manager_thrust         = processIncomingMessage(sh_control_manager_thrust_);
  const auto constraint_manager_diagnostics = processIncomingMessage(sh_constraint_manager_diagnostics_);
  const auto gain_manager_diagnostics       = processIncomingMessage(sh_gain_manager_diagnostics_);
  const auto estimation_diagnostics         = processIncomingMessage(sh_estimation_diagnostics_);
  const auto hw_api_gnss                    = processIncomingMessage(sh_hw_api_gnss_);
  const auto hw_api_mag_heading             = processIncomingMessage(sh_hw_api_mag_heading_);
  const auto hw_api_status                  = processIncomingMessage(sh_hw_api_status_);
  const auto mass_estimate                  = processIncomingMessage(sh_mass_estimate_);
  const auto mpc_tracker_diagnostics        = processIncomingMessage(sh_mpc_tracker_diagnostics_);
  const auto tracker_cmd                    = processIncomingMessage(sh_tracker_cmd_);

  // mass_nominal is published once, latched
  // Once received, treat it as valid indefinitely instead.
  subscriptionResult_t<std_msgs::msg::Float64> mass_nominal;
  mass_nominal.hasNewMessage = sh_mass_nominal_.newMsg();
  mass_nominal.message       = mass_nominal.hasNewMessage ? sh_mass_nominal_.getMsg() : sh_mass_nominal_.peekMsg();

  // Watt-hour integration on each new battery sample.
  if (battery_state.hasNewMessage && battery_state.message != nullptr)
    wh_drained_integrator_->integrate(battery_state.message->voltage, battery_state.message->current);

  // Flight timer advances while a real tracker is active.
  if (control_manager_diagnostics.message != nullptr) {
    const bool null_tracker = (control_manager_diagnostics.message->active_tracker == "NullTracker");
    flight_timer_->tick(null_tracker);
  }

  // Process fresh input arrived since the last tick.
  if (estimation_diagnostics.hasNewMessage || control_manager_heading.hasNewMessage || hw_api_gnss.hasNewMessage || hw_api_mag_heading.hasNewMessage) {
    last_state_estimation_info_ =
        parse_state_estimation_info(estimation_diagnostics.message, control_manager_heading.message, hw_api_gnss.message, hw_api_mag_heading.message);
  }

  if (control_manager_diagnostics.hasNewMessage || control_manager_thrust.hasNewMessage || constraint_manager_diagnostics.hasNewMessage ||
      gain_manager_diagnostics.hasNewMessage || tracker_cmd.hasNewMessage) {
    last_control_info_ = parse_control_info(control_manager_diagnostics.message, constraint_manager_diagnostics.message, gain_manager_diagnostics.message,
                                            control_manager_thrust.message, tracker_cmd.message);
  }

  if (mpc_tracker_diagnostics.hasNewMessage || control_manager_diagnostics.hasNewMessage) {
    last_collision_avoidance_info_ = parse_collision_avoidance_info(mpc_tracker_diagnostics.message, control_manager_diagnostics.message);
  }

  if (hw_api_status.hasNewMessage || mass_nominal.hasNewMessage || mass_estimate.hasNewMessage) {
    last_uav_info_ = parse_uav_info(hw_api_status.message, mass_nominal.message, mass_estimate.message);
  }

  // Publish data
  last_state_estimation_info_.header.stamp = now;
  ph_state_estimation_info_.publish(last_state_estimation_info_);
  ph_control_info_.publish(last_control_info_);
  ph_collision_avoidance_info_.publish(last_collision_avoidance_info_);
  ph_uav_info_.publish(last_uav_info_);

  last_general_robot_info_ = parse_general_robot_info(battery_state.message);
  ph_general_robot_info_.publish(last_general_robot_info_);

  last_system_health_info_ = parse_system_health_info();
  ph_system_health_info_.publish(last_system_health_info_);

  // transitions are published immediately by timerUavState
  mrs_msgs::msg::State uav_state_msg;
  uav_state_msg.stamp = now;
  uav_state_msg.state = to_ros(uav_state_.value());
  ph_uav_state_.publish(uav_state_msg);

  // to avoid getting timeout warnings on this latched message
  if (mass_nominal.hasNewMessage)
    sh_mass_nominal_.setNoMessageTimeout(mrs_lib::no_timeout);
}

//}

/* timerUavState() //{ */

void DiagnosticsManager::timerUavState() {
  if (!is_initialized_) {
    return;
  }
  std::scoped_lock lck(uav_state_mutex_);

  // Non-consuming peeks: this fast path must not steal the newMsg() flags that
  // timerMain relies on to (re)publish uav_info from the same hw_api/status.
  // peekFreshMsg() applies the same not_reporting_timeout_ staleness gate as
  // processIncomingMessage(), so this path and timerMain() agree on stale data.
  const auto new_state = parse_uav_state(peekFreshMsg(sh_hw_api_status_), peekFreshMsg(sh_control_manager_diagnostics_));

  if (new_state == uav_state_.value())
    return;

  uav_state_.set(new_state);

  mrs_msgs::msg::State uav_state_msg;
  uav_state_msg.stamp = clock_->now();
  uav_state_msg.state = to_ros(uav_state_.value());
  ph_uav_state_.publish(uav_state_msg);
}

//}

/* timerErrorPublishing() //{ */

void DiagnosticsManager::timerErrorPublishing() {
  if (!is_initialized_) {
    return;
  }

  std::scoped_lock lck(errorgraph_mtx_);

  mrs_msgs::msg::ErrorgraphElementArray root_errors_msg;
  root_errors_msg.stamp = clock_->now();

  const auto root_errors = errorgraph_.find_error_roots();

  for (const auto &error : root_errors) {
    root_errors_msg.elements.push_back(std::visit([](const auto &info) { return info.to_msg(); }, error));
  }

  ph_root_errors_.publish(root_errors_msg);
}

//}

/* timerUpdateSensorStatus() //{ */

void DiagnosticsManager::timerUpdateSensorStatus() {

  if (!is_initialized_) {
    return;
  }

  std::scoped_lock lck(mutex_sensor_handler_list_);
  available_sensors_.clear();
  for (auto &handler : sensor_handlers_) {
    auto sensor_status_msg = handler->updateStatus();
    available_sensors_.push_back(sensor_status_msg);
  }
}

//}

/* timerHostInfo() //{ */

void DiagnosticsManager::timerHostInfo() {
  if (!is_initialized_) {
    return;
  }
  host_stats_->update();
}

//}

// | ------------------------ callbacks ----------------------- |

/* cbk_errorgraph_element() //{ */

void DiagnosticsManager::cbk_errorgraph_element(const mrs_msgs::msg::ErrorgraphElement::ConstSharedPtr element_msg) {
  std::scoped_lock lck(errorgraph_mtx_);
  errorgraph_.add_element_from_msg(*element_msg);
}

//}

/* rate-counting callbacks //{ */

// Rate-counting callbacks — record the arrival timestamp exactly once per message.
void DiagnosticsManager::cbk_hw_api_odometry_rate(const nav_msgs::msg::Odometry::ConstSharedPtr /*msg*/) {
  rate_hw_api_odometry_.record(clock_->now());
}

void DiagnosticsManager::cbk_estimator_uav_state_rate(const mrs_msgs::msg::UavState::ConstSharedPtr /*msg*/) {
  rate_estimator_uav_state_.record(clock_->now());
}

void DiagnosticsManager::cbk_control_manager_diag_rate(const mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr /*msg*/) {
  rate_control_manager_diag_.record(clock_->now());
}

//}

// | -------------------- support functions ------------------- |

/* parse_robot_type() //{ */

robot_type_t DiagnosticsManager::parse_robot_type(const std::string &robot_type_str) {

  // Convert to lowercase for case-insensitive comparison
  std::string lower_str = robot_type_str;
  std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), [](unsigned char c) { return std::tolower(c); });

  if (lower_str == "multirotor") {
    return robot_type_t::MULTIROTOR;
  } else if (lower_str == "boat") {
    return robot_type_t::BOAT;
  } else {
    return robot_type_t::UNKNOWN;
  }
}

//}

// | --------------------- Parsing methods -------------------- |

/* parse_tracker_state() //{ */

tracker_state_t DiagnosticsManager::parse_tracker_state(mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr control_manager_diagnostics) {

  if (control_manager_diagnostics == nullptr)
    return tracker_state_t::UNKNOWN;

  if (control_manager_diagnostics->active_tracker == "NullTracker")
    return tracker_state_t::INVALID;

  switch (control_manager_diagnostics->tracker_status.state) {

  case mrs_msgs::msg::TrackerStatus::STATE_INVALID:
    return tracker_state_t::INVALID;
  case mrs_msgs::msg::TrackerStatus::STATE_IDLE:
    return tracker_state_t::IDLE;
  case mrs_msgs::msg::TrackerStatus::STATE_TAKEOFF:
    return tracker_state_t::TAKEOFF;
  case mrs_msgs::msg::TrackerStatus::STATE_HOVER:
    return tracker_state_t::HOVER;
  case mrs_msgs::msg::TrackerStatus::STATE_REFERENCE:
    return tracker_state_t::REFERENCE;
  case mrs_msgs::msg::TrackerStatus::STATE_TRAJECTORY:
    return tracker_state_t::TRAJECTORY;
  case mrs_msgs::msg::TrackerStatus::STATE_LAND:
    return tracker_state_t::LAND;
  default:
    return tracker_state_t::UNKNOWN;
  }
}

//}

/* parse_uav_state() //{ */

state_t DiagnosticsManager::parse_uav_state(mrs_msgs::msg::HwApiStatus::ConstSharedPtr               hw_api_status,
                                            mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr control_manager_diagnostics) {
  if (hw_api_status == nullptr || control_manager_diagnostics == nullptr)
    return state_t::UNKNOWN;

  const bool hw_armed = hw_api_status->armed;
  // not armed
  if (!hw_armed)
    return state_t::DISARMED;

  // armed, flying in manual mode
  const bool manual_mode = hw_api_status->mode == "MANUAL";
  if (control_manager_diagnostics->joystick_active && manual_mode)
    return state_t::MANUAL;

  // armed, not flying
  const auto tracker_state = parse_tracker_state(control_manager_diagnostics);
  const bool null_tracker  = tracker_state == tracker_state_t::INVALID;
  if (hw_armed && null_tracker) {
    const bool offboard = hw_api_status->offboard;
    if (offboard)
      return state_t::OFFBOARD;
    return state_t::ARMED;
  }
  // flying using the MRS system in RC joystick mode
  if (control_manager_diagnostics->joystick_active)
    return state_t::RC_MODE;

  // LandoffTracker goes into idle state when deactivating
  if (control_manager_diagnostics->active_tracker == "LandoffTracker" && tracker_state == tracker_state_t::IDLE)
    return state_t::TAKEOFF;

  // unless the RC mode is active, just parse the tracker state
  switch (tracker_state) {
  case tracker_state_t::TAKEOFF:
    return state_t::TAKEOFF;
  case tracker_state_t::HOVER:
    return state_t::HOVER;
  case tracker_state_t::REFERENCE:
    return state_t::GOTO;
  case tracker_state_t::TRAJECTORY:
    return state_t::TRAJECTORY;
  case tracker_state_t::LAND:
    return state_t::LAND;
  default:
    return state_t::UNKNOWN;
  }
}

//}

/* parse_general_robot_info() //{ */

mrs_msgs::msg::GeneralRobotInfo DiagnosticsManager::parse_general_robot_info(sensor_msgs::msg::BatteryState::ConstSharedPtr battery_state) {
  mrs_msgs::msg::GeneralRobotInfo msg;
  msg.stamp            = clock_->now();
  msg.robot_name       = _robot_name_;
  msg.robot_type       = static_cast<int>(robot_type_);
  msg.robot_ip_address = robot_ip_address_;

  if (battery_state != nullptr) {
    msg.battery_state.percentage = battery_state->percentage;
    msg.battery_state.voltage    = battery_state->voltage;
    msg.battery_state.current    = battery_state->current;
  }
  msg.battery_state.wh_drained = static_cast<float>(wh_drained_integrator_->whDrained());

  const auto uav_state      = uav_state_.value();
  const bool state_offboard = uav_state == state_t::OFFBOARD;

  msg.problems_preventing_start.clear();

  /*//{ diagnose problems preventing start */

  // If not flying, explain why we're not ready. When flying autonomously, we
  // assume everything was fine at takeoff and skip the diagnosis.
  if (!is_flying_autonomously(uav_state)) {
    const auto preflight_result = preflight_checker_->runPreflightChecks();

    msg.ready_to_start = preflight_result.can_takeoff && state_offboard;

    for (const auto &v : preflight_result.violations)
      msg.problems_preventing_start.push_back(v);

    switch (uav_state) {
    case state_t::UNKNOWN:
      msg.problems_preventing_start.emplace_back("UAV state is UNKNOWN");
      break;
    case state_t::MANUAL:
      msg.problems_preventing_start.emplace_back("UAV state is in MANUAL mode");
      break;
    case state_t::DISARMED:
      msg.problems_preventing_start.emplace_back("UAV is DISARMED");
      break;
    default:
      if (!state_offboard)
        msg.problems_preventing_start.emplace_back("UAV is not in OFFBOARD mode");
      break;
    }
  }
  /*//}*/

  { // find all errors
    std::scoped_lock lck(errorgraph_mtx_);

    const auto error_roots = errorgraph_.find_error_roots();
    for (const auto &root : error_roots) {
      std::visit(
          [&msg](const auto &info) {
            using T = std::decay_t<decltype(info)>;
            if (info.not_reporting) {
              std::stringstream ss;
              ss << info.source_node.node << "." << info.source_node.component << ": not responding";
              msg.errors.push_back(ss.str());
            }
            if constexpr (std::is_same_v<T, mrs_lib::errorgraph::Errorgraph::node_info_t>) {
              for (const auto &error : info.errors)
                msg.errors.push_back(error.type);
            }
          },
          root);
    }
  }
  return msg;
}

//}

/* parse_state_estimation_info() //{ */

mrs_msgs::msg::StateEstimationInfo DiagnosticsManager::parse_state_estimation_info(mrs_msgs::msg::EstimationDiagnostics::ConstSharedPtr estimation_diagnostics,
                                                                                   mrs_msgs::msg::Float64Stamped::ConstSharedPtr        local_heading,
                                                                                   sensor_msgs::msg::NavSatFix::ConstSharedPtr          global_position,
                                                                                   mrs_msgs::msg::Float64Stamped::ConstSharedPtr        global_heading) {
  auto init_msg         = init_state_estimation_info();
  init_msg.header.stamp = clock_->now();

  mrs_msgs::msg::StateEstimationInfo msg = init_msg;

  const bool is_estimation_diagnostics_valid = estimation_diagnostics != nullptr;
  const bool is_local_heading_valid          = local_heading != nullptr;
  const bool is_global_position_valid        = global_position != nullptr;
  const bool is_global_heading_valid         = global_heading != nullptr;

  if (is_estimation_diagnostics_valid) {
    msg.header = estimation_diagnostics->header;

    msg.local_pose.position       = estimation_diagnostics->pose.position;
    msg.above_ground_level_height = estimation_diagnostics->agl_height;

    msg.velocity     = estimation_diagnostics->velocity;
    msg.acceleration = estimation_diagnostics->acceleration;

    if (!estimation_diagnostics->running_state_estimators.empty())
      msg.current_estimator = estimation_diagnostics->running_state_estimators.at(0);

    msg.running_estimators    = estimation_diagnostics->running_state_estimators;
    msg.switchable_estimators = estimation_diagnostics->switchable_state_estimators;

    msg.horizontal_estimator = estimation_diagnostics->estimator_horizontal;
    msg.vertical_estimator   = estimation_diagnostics->estimator_vertical;
    msg.heading_estimator    = estimation_diagnostics->estimator_heading;
    msg.agl_estimator        = estimation_diagnostics->estimator_agl_height;
    msg.max_flight_z         = static_cast<float>(estimation_diagnostics->max_flight_z);
  }

  if (is_local_heading_valid)
    msg.local_pose.heading = local_heading->value;

  if (is_global_position_valid) {
    msg.global_pose.position.x = global_position->latitude;
    msg.global_pose.position.y = global_position->longitude;
    msg.global_pose.position.z = global_position->altitude;
  }

  if (is_global_heading_valid)
    msg.global_pose.heading = global_heading->value;

  return msg;
}

//}

/* parse_control_info() //{ */

mrs_msgs::msg::ControlInfo DiagnosticsManager::parse_control_info(mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr    control_manager_diagnostics,
                                                                  mrs_msgs::msg::ConstraintManagerDiagnostics::ConstSharedPtr constraint_manager_diagnostics,
                                                                  mrs_msgs::msg::GainManagerDiagnostics::ConstSharedPtr       gain_manager_diagnostics,
                                                                  std_msgs::msg::Float64::ConstSharedPtr                      thrust,
                                                                  mrs_msgs::msg::TrackerCommand::ConstSharedPtr               tracker_cmd) {

  mrs_msgs::msg::ControlInfo msg;

  const bool is_control_manager_diagnostics_valid    = control_manager_diagnostics != nullptr;
  const bool is_constraint_manager_diagnostics_valid = constraint_manager_diagnostics != nullptr;
  const bool is_gain_manager_diagnostics_valid       = gain_manager_diagnostics != nullptr;
  const bool is_thrust_valid                         = thrust != nullptr;
  const bool is_tracker_cmd_valid                    = tracker_cmd != nullptr;

  if (is_control_manager_diagnostics_valid) {
    msg.active_controller     = control_manager_diagnostics->active_controller;
    msg.available_controllers = control_manager_diagnostics->available_controllers;
    msg.active_tracker        = control_manager_diagnostics->active_tracker;
    msg.available_trackers    = control_manager_diagnostics->available_trackers;

    msg.flying_normally     = control_manager_diagnostics->flying_normally;
    msg.have_goal           = control_manager_diagnostics->tracker_status.have_goal;
    msg.tracking_trajectory = control_manager_diagnostics->tracker_status.tracking_trajectory;
    msg.callbacks_enabled   = control_manager_diagnostics->tracker_status.callbacks_enabled;
  }

  if (is_thrust_valid)
    msg.thrust = thrust->data;

  if (is_constraint_manager_diagnostics_valid) {
    msg.active_constraints    = constraint_manager_diagnostics->current_name;
    msg.available_constraints = constraint_manager_diagnostics->available;
  }

  if (is_gain_manager_diagnostics_valid) {
    msg.active_gains    = gain_manager_diagnostics->current_name;
    msg.available_gains = gain_manager_diagnostics->available;
  }

  if (is_tracker_cmd_valid) {
    msg.cmd_pose.position = tracker_cmd->position;
    msg.cmd_pose.heading  = tracker_cmd->heading;
  }

  return msg;
}

//}

/* parse_collision_avoidance_info() //{ */

mrs_msgs::msg::CollisionAvoidanceInfo
DiagnosticsManager::parse_collision_avoidance_info(mrs_msgs::msg::MpcTrackerDiagnostics::ConstSharedPtr     mpc_tracker_diagnostics,
                                                   mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr control_manager_diagnostics) {
  mrs_msgs::msg::CollisionAvoidanceInfo msg;

  const bool is_mpc_tracker_diagnostics_valid     = mpc_tracker_diagnostics != nullptr;
  const bool is_control_manager_diagnostics_valid = control_manager_diagnostics != nullptr;

  if (is_mpc_tracker_diagnostics_valid) {
    msg.collision_avoidance_enabled = mpc_tracker_diagnostics->collision_avoidance_active;
    msg.avoiding_collision          = mpc_tracker_diagnostics->avoiding_collision;
    msg.other_robots_visible        = mpc_tracker_diagnostics->avoidance_active_uavs;
  }

  if (is_control_manager_diagnostics_valid) {
    msg.bumper_active = control_manager_diagnostics->bumper_active;
  }

  return msg;
}

//}

/* parse_uav_info() //{ */

mrs_msgs::msg::UavInfo DiagnosticsManager::parse_uav_info(mrs_msgs::msg::HwApiStatus::ConstSharedPtr hw_api_status,
                                                          std_msgs::msg::Float64::ConstSharedPtr     mass_nominal,
                                                          std_msgs::msg::Float64::ConstSharedPtr     mass_estimate) {
  mrs_msgs::msg::UavInfo msg;

  if (hw_api_status != nullptr) {
    msg.armed    = hw_api_status->armed;
    msg.offboard = hw_api_status->offboard;
  }

  msg.flight_duration = static_cast<float>(flight_timer_->secsFlown());
  msg.flight_state    = to_string(uav_state_.value());

  if (mass_nominal != nullptr)
    msg.mass_nominal = mass_nominal->data;

  if (mass_estimate != nullptr)
    msg.mass_estimate = mass_estimate->data;

  return msg;
}

//}

/* parse_system_health_info() //{ */

mrs_msgs::msg::SystemHealthInfo DiagnosticsManager::parse_system_health_info() {
  mrs_msgs::msg::SystemHealthInfo msg;

  // Host CPU / RAM / disk / per-node CPU loads / WiFi
  const auto snap                             = host_stats_->snapshot();
  msg.onboard_computer_info.cpu_load          = snap.cpu_load;
  msg.onboard_computer_info.cpu_ghz           = snap.cpu_ghz;
  msg.onboard_computer_info.cpu_temperature   = snap.cpu_temperature;
  msg.onboard_computer_info.free_ram          = snap.free_ram;
  msg.onboard_computer_info.total_ram         = snap.total_ram;
  msg.onboard_computer_info.free_hdd          = snap.free_hdd;
  msg.onboard_computer_info.node_cpu_loads    = snap.node_cpu_loads;
  msg.onboard_computer_info.wifi_interface    = snap.wifi_interface;
  msg.onboard_computer_info.wifi_signal_dbm   = snap.wifi_signal_dbm;
  msg.onboard_computer_info.wifi_link_quality = snap.wifi_link_quality;

  // RateTracker doesn't decay on its own, so gate each rate on topic freshness. hasMsg() must be
  // checked first -- lastMsgTime() defaults to a different clock type before any message arrives,
  // and subtracting it from clock_->now() throws.
  msg.hw_api_rate = (sh_hw_api_odometry_.hasMsg() && clock_->now() - sh_hw_api_odometry_.lastMsgTime() <= not_reporting_timeout_)
                        ? static_cast<float>(rate_hw_api_odometry_.rate())
                        : 0.0f;
  msg.control_manager_rate =
      (sh_control_manager_diagnostics_.hasMsg() && clock_->now() - sh_control_manager_diagnostics_.lastMsgTime() <= not_reporting_timeout_)
          ? static_cast<float>(rate_control_manager_diag_.rate())
          : 0.0f;
  msg.state_estimation_rate =
      (sh_estimator_uav_state_.hasMsg() && clock_->now() - sh_estimator_uav_state_.lastMsgTime() <= not_reporting_timeout_)
          ? static_cast<float>(rate_estimator_uav_state_.rate())
          : 0.0f;

  {
    std::scoped_lock lck(mutex_sensor_handler_list_);
    msg.available_sensors = available_sensors_;
  }

  return msg;
}

//}

// | -------------------- Msg init methods -------------------- |

/* init_state_estimation_info() //{ */

mrs_msgs::msg::StateEstimationInfo DiagnosticsManager::init_state_estimation_info() {
  mrs_msgs::msg::StateEstimationInfo msg;

  msg.header.stamp    = clock_->now();
  msg.header.frame_id = "";

  msg.local_pose.position.x = std::numeric_limits<double>::quiet_NaN();
  msg.local_pose.position.y = std::numeric_limits<double>::quiet_NaN();
  msg.local_pose.position.z = std::numeric_limits<double>::quiet_NaN();
  msg.local_pose.heading    = std::numeric_limits<double>::quiet_NaN();

  msg.velocity.linear.x  = std::numeric_limits<double>::quiet_NaN();
  msg.velocity.linear.y  = std::numeric_limits<double>::quiet_NaN();
  msg.velocity.linear.z  = std::numeric_limits<double>::quiet_NaN();
  msg.velocity.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.velocity.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.velocity.angular.z = std::numeric_limits<double>::quiet_NaN();

  msg.acceleration.linear.x  = std::numeric_limits<double>::quiet_NaN();
  msg.acceleration.linear.y  = std::numeric_limits<double>::quiet_NaN();
  msg.acceleration.linear.z  = std::numeric_limits<double>::quiet_NaN();
  msg.acceleration.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.acceleration.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.acceleration.angular.z = std::numeric_limits<double>::quiet_NaN();

  msg.above_ground_level_height = std::numeric_limits<double>::quiet_NaN();

  msg.global_pose.position.x = std::numeric_limits<double>::quiet_NaN();
  msg.global_pose.position.y = std::numeric_limits<double>::quiet_NaN();
  msg.global_pose.position.z = std::numeric_limits<double>::quiet_NaN();
  msg.global_pose.heading    = std::numeric_limits<double>::quiet_NaN();

  msg.current_estimator = "unknown";

  return msg;
}

//}

} // namespace mrs_uav_managers::diagnostics_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_managers::diagnostics_manager::DiagnosticsManager)
