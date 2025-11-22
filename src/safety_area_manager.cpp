/* includes //{ */

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <mrs_lib/mutex.h>
#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/safety_zone.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/utils.h>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/estimation_diagnostics.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>
#include <mrs_msgs/msg/hw_api_capabilities.hpp>
#include <mrs_msgs/msg/reference_stamped.hpp>
#include <mrs_msgs/msg/uav_state.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/srv/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <boost/geometry.hpp>
#include <limits>
#include <memory>

#include <mrs_msgs/msg/point2_d.hpp>
#include <mrs_msgs/msg/safety_area_manager_diagnostics.hpp>
#include <mrs_msgs/srv/get_bool_srv.hpp>
#include <mrs_msgs/srv/get_reference_stamped_srv.hpp>
#include <mrs_msgs/srv/set_obstacle_srv.hpp>
#include <mrs_msgs/srv/set_safety_border_srv.hpp>
#include <mrs_msgs/srv/validate_path_to_point_srv.hpp>

#include <mrs_lib/safety_zone/static_edges_visualization.h>

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

//}

namespace mrs_uav_managers
{

namespace safety_area_manager
{

/* class SafetyAreaManager //{ */

class SafetyAreaManager : public mrs_lib::Node {

public:
  SafetyAreaManager(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  std::shared_ptr<mrs_lib::Transformer> transformer_;
  std::atomic<bool>                     is_initialized_       = false;
  std::atomic<bool>                     world_origin_changed_ = false;

  // | ------------------- scope timer logger ------------------- |

  bool                                       scope_timer_enabled_ = false;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;
  std::string                                _uav_name_;
  std::string                                _world_config_;

  struct VisualizationComponents
  {
    std::vector<std::unique_ptr<mrs_lib::StaticEdgesVisualization>> static_edges;

    void safeCleanup() {
      static_edges.clear();
    }
  };

  struct SafetyZoneParams
  {
    bool                       use_safety_area;
    mrs_msgs::msg::WorldOrigin world_origin;
  };

  struct SafetyZoneHandler
  {
    std::shared_ptr<mrs_lib::safety_zone::SafetyZone> safety_zone;
    // Safety zone with global coordinates, used for world origin updates
    std::shared_ptr<mrs_lib::safety_zone::SafetyZone> global_safety_zone;
    VisualizationComponents                           visualization_components;
    SafetyZoneParams                                  parameters;

    SafetyZoneHandler()                                = default;
    SafetyZoneHandler(SafetyZoneHandler &&)            = default;
    SafetyZoneHandler &operator=(SafetyZoneHandler &&) = default;
  } safety_zone_handler_;

  std::mutex mutex_safety_area_;

  // profiling
  mrs_lib::Profiler profiler_;
  bool              profiler_enabled_  = false;
  int               status_timer_rate_ = 0;

  // diagnostics publishing
  void publishDiagnostics(void);

  std::tuple<bool, bool> isPositionValid(mrs_msgs::msg::UavState);

  void initialize();
  void shutdown();

  // | -------------- uav_state/odometry subscriber ------------- |

  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>       sh_odometry_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped> sh_max_z_;
  mrs_msgs::msg::UavState                                   uav_state_;
  std::mutex                                                mutex_uav_state_;

  mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix> sh_gnss_;

  // | --------------------- service servers -------------------- |

  mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv>    ss_point_in_safety_area_3d_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv>    ss_point_in_safety_area_2d_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::ValidatePathToPointSrv> ss_path_in_safety_area_3d_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::ValidatePathToPointSrv> ss_path_in_safety_area_2d_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::SetSafetyBorderSrv>     ss_set_safety_border_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>                ss_toggle_safety_area_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv>    ss_add_obstacle_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::SetObstacleSrv>         ss_set_obstacle_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::GetReferenceStampedSrv> ss_get_max_z_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::GetReferenceStampedSrv> ss_get_min_z_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::GetBoolSrv>             ss_is_safety_zone_enabled_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv>    ss_update_world_origin_;

  // | --------------------- service clients --------------------- |
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv> sc_set_world_origin_;

  // | ----------------------- subscribers ----------------------- |

  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiCapabilities>         sh_hw_api_capabilities_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>     sh_estimation_diag_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mrs_msgs::msg::SafetyAreaManagerDiagnostics> ph_diagnostics_;

  // | ----------------------- timers ----------------------- |

  // this timer will check till we already got the hardware api diagnostics
  // the initialization of the SafetyAreaManager
  rclcpp::TimerBase::SharedPtr timer_prerequisites_;
  void                         timerPrerequisites();

  // timer for regular status publishing
  std::shared_ptr<TimerType> timer_status_;
  void                       timerStatus();

  // | ----------------------- callbacks ----------------------- |
  // topic callbacks
  void callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void callbackGNSS(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);

  // services
  bool callbackValidatePoint3d(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request>  request,
                               const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response);
  bool callbackValidatePoint2d(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request>  request,
                               const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response);
  bool callbackValidatePathToPoint3d(const std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Request>  request,
                                     const std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Response> response);
  bool callbackValidatePathToPoint2d(const std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Request>  request,
                                     const std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Response> response);

  bool callbackSetSafetyBorder(const std::shared_ptr<mrs_msgs::srv::SetSafetyBorderSrv::Request>  request,
                               const std::shared_ptr<mrs_msgs::srv::SetSafetyBorderSrv::Response> response);
  bool callbackToggleSafetyArea(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                                const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  bool callbackAddObstacle(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request>  request,
                           const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response);
  bool callbackSetObstacle(const std::shared_ptr<mrs_msgs::srv::SetObstacleSrv::Request>  request,
                           const std::shared_ptr<mrs_msgs::srv::SetObstacleSrv::Response> response);
  bool callbackGetMaxZ(const std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Request>  request,
                       const std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Response> response);
  bool callbackGetMinZ(const std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Request>  request,
                       const std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Response> response);
  bool callbackIsSafetyZoneEnabled([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::GetBoolSrv::Request> request,
                                   std::shared_ptr<mrs_msgs::srv::GetBoolSrv::Response>                       response);
  bool callbackUpdateWorldOrigin(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request>  request,
                                 const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response);


  // | ----------------------- routines ----------------------- |

  // Safety area building
  std::unique_ptr<mrs_lib::safety_zone::Prism> makePrism(const Eigen::MatrixXd &matrix, const double max_z, const double min_z,
                                                         const std::string &horizontal_frame, const std::string &vertical_frame);
  std::unique_ptr<mrs_lib::safety_zone::Prism> makePrism(const std::vector<mrs_msgs::msg::Point2D> &points, const double max_z, const double min_z,
                                                         const std::string &horizontal_frame, const std::string &vertical_frame);
  std::tuple<bool, std::vector<mrs_lib::safety_zone::Point2d>> transformPoints(const std::vector<mrs_lib::safety_zone::Point2d> &points,
                                                                               const std::string &from_frame, const std::string &to_frame);

  std::tuple<bool, double>                                  transformZ(const std::string &current_frame, const std::string &target_frame, const double z);
  bool                                                      initializationFromFile(mrs_lib::ParamLoader &param_loader, const std::string &filename);
  std::vector<std::unique_ptr<mrs_lib::safety_zone::Prism>> copyExistingObstacles();
  bool                                                      initializationFromMsg(const mrs_msgs::msg::Prism &prism_msg, bool keep_obstacles);
  std::optional<SafetyZoneHandler>                          createSafetyZone(std::unique_ptr<mrs_lib::safety_zone::Prism>            &&border,
                                                                             std::vector<std::unique_ptr<mrs_lib::safety_zone::Prism>> obstacle_prisms);

  std::tuple<bool, std::string> validateMsg(const mrs_msgs::msg::Prism &prism_msg);

  // Reference validation
  bool   isPointInSafetyArea2d(const mrs_msgs::msg::ReferenceStamped &point);
  bool   isPointInSafetyArea3d(const mrs_msgs::msg::ReferenceStamped &point);
  bool   isPathToPointInSafetyArea2d(const mrs_msgs::msg::ReferenceStamped &from, const mrs_msgs::msg::ReferenceStamped &to);
  bool   isPathToPointInSafetyArea3d(const mrs_msgs::msg::ReferenceStamped &from, const mrs_msgs::msg::ReferenceStamped &to);
  double getMaxZ(const std::string &frame_id);
  double getMinZ(const std::string &frame_id);

}; // class SafetyAreaManager

//}

/* SafetyAreaManager::SafetyAreaManager() //{ */

SafetyAreaManager::SafetyAreaManager(rclcpp::NodeOptions options) : mrs_lib::Node("safety_area_manager", options) {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  mrs_lib::ParamLoader param_loader(node_, "SafetyAreaManager");
  param_loader.loadParam("uav_name", _uav_name_);

  // | ---------------------- tf-transformer ----------------------- |
  transformer_ = std::make_shared<mrs_lib::Transformer>(node_);
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_hw_api_capabilities_  = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiCapabilities>(shopts, "~/hw_api_capabilities_in");
  sh_gnss_                 = mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix>(shopts, "~/gnss_in", &SafetyAreaManager::callbackGNSS, this);
  sh_control_manager_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(shopts, "~/control_manager_diagnostics_in");
  sh_estimation_diag_      = mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>(shopts, "~/estimation_diagnostics_in");

  timer_prerequisites_ = node_->create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&SafetyAreaManager::timerPrerequisites, this));
}

//}

/* initialize() //{ */

void SafetyAreaManager::initialize() {

  RCLCPP_INFO(node_->get_logger(), "initializing");

  // | --------------------- parameters ---------------------- |
  mrs_lib::ParamLoader param_loader(node_, "SafetyAreaManager");
  param_loader.loadParam("world_config", _world_config_);
  param_loader.addYamlFile(_world_config_);

  param_loader.addYamlFileFromParam("private_config");
  // param_loader.addYamlFileFromParam("public_config");

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("enable_profiler", profiler_enabled_);

  param_loader.setPrefix("mrs_uav_managers/safety_area_manager/");
  param_loader.loadParam("status_timer_rate", status_timer_rate_);

  // | ------------------------ profiler ------------------------ |
  profiler_ = mrs_lib::Profiler(node_, "SafetyAreaManager", profiler_enabled_);


  // | ------------------- scope timer logger ------------------- |

  param_loader.loadParam("scope_timer/enabled", scope_timer_enabled_);
  const std::string scope_timer_log_filename = param_loader.loadParam2("scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<mrs_lib::ScopeTimerLogger>(node_, scope_timer_log_filename, scope_timer_enabled_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  param_loader.setPrefix("");

  // | ---------------------- safety zone ----------------------- |
  bool success = initializationFromFile(param_loader, _world_config_);

  if (!success) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize safety area from file.");
    rclcpp::shutdown();
    exit(1);
  }

  // | ----------------------- publishers ----------------------- |
  ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_msgs::msg::SafetyAreaManagerDiagnostics>(node_, "~/diagnostics_out");

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  // | ----------------------- Subscribers ----------------------- |

  sh_odometry_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/odometry_in", &SafetyAreaManager::callbackOdometry, this);
  sh_max_z_    = mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>(shopts, "~/max_z_in");

  // | ------------------------ services ------------------------ |
  ss_point_in_safety_area_3d_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv>(
      node_, "~/point_in_safety_area_3d_in",
      [this](std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request, std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response) {
        callbackValidatePoint3d(request, response);
      },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_point_in_safety_area_2d_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv>(
      node_, "~/point_in_safety_area_2d_in",
      [this](std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request, std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response) {
        callbackValidatePoint2d(request, response);
      },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_path_in_safety_area_3d_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::ValidatePathToPointSrv>(
      node_, "~/path_in_safety_area_3d_in",
      [this](std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Request>  request,
             std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Response> response) { callbackValidatePathToPoint3d(request, response); },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_path_in_safety_area_2d_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::ValidatePathToPointSrv>(
      node_, "~/path_in_safety_area_2d_in",
      [this](std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Request>  request,
             std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Response> response) { callbackValidatePathToPoint2d(request, response); },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_set_safety_border_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::SetSafetyBorderSrv>(
      node_, "~/set_safety_border_in",
      [this](std::shared_ptr<mrs_msgs::srv::SetSafetyBorderSrv::Request> request, std::shared_ptr<mrs_msgs::srv::SetSafetyBorderSrv::Response> response) {
        callbackSetSafetyBorder(request, response);
      },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_toggle_safety_area_ = mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>(
      node_, "~/toggle_safety_area_in",
      [this](std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        callbackToggleSafetyArea(request, response);
      },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_add_obstacle_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv>(
      node_, "~/add_obstacle_in",
      [this](std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request, std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response) {
        callbackAddObstacle(request, response);
      },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_set_obstacle_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::SetObstacleSrv>(
      node_, "~/set_obstacle_in",
      [this](std::shared_ptr<mrs_msgs::srv::SetObstacleSrv::Request> request, std::shared_ptr<mrs_msgs::srv::SetObstacleSrv::Response> response) {
        callbackSetObstacle(request, response);
      },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_get_max_z_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::GetReferenceStampedSrv>(
      node_, "~/get_max_z_in",
      [this](std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Request>  request,
             std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Response> response) { callbackGetMaxZ(request, response); },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_get_min_z_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::GetReferenceStampedSrv>(
      node_, "~/get_min_z_in",
      [this](std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Request>  request,
             std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Response> response) { callbackGetMinZ(request, response); },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_is_safety_zone_enabled_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::GetBoolSrv>(
      node_, "~/is_safety_zone_enabled_in",
      [this](std::shared_ptr<mrs_msgs::srv::GetBoolSrv::Request> request, std::shared_ptr<mrs_msgs::srv::GetBoolSrv::Response> response) {
        callbackIsSafetyZoneEnabled(request, response);
      },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_update_world_origin_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv>(
      node_, "~/update_world_origin_in",
      [this](std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request, std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response) {
        callbackUpdateWorldOrigin(request, response);
      },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  // | ----------------------- service clients ---------------------- |
  sc_set_world_origin_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "~/set_world_origin_out", cbkgrp_sc_);

  // | ------------------------- timers ------------------------- |
  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fcn = std::bind(&SafetyAreaManager::timerStatus, this);

    timer_status_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(status_timer_rate_, clock_), callback_fcn);
  }

  // | ----------------------- finish init ---------------------- |
  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "Safety area initialized");
}

//}

/* shutdown() //{ */

void SafetyAreaManager::shutdown() {

  std::cout << "SafetyAreaManager: shutdown(): called" << std::endl;

  timer_status_->stop();

  std::cout << "SafetyAreaManager: finished shutdown()" << std::endl;
}

//}

// --------------------------------------------------------------
// |                          timers                            |
// --------------------------------------------------------------

/* timerPrerequisites() //{ */

void SafetyAreaManager::timerPrerequisites() {
  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerPrerequisites");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "SafetyAreaManager::timerPrerequisites", scope_timer_logger_, scope_timer_enabled_);

  bool got_hw_api_capabilities = sh_hw_api_capabilities_.hasMsg();
  bool got_estimation_diag     = sh_estimation_diag_.hasMsg();
  bool got_gnss                = sh_gnss_.hasMsg();

  if (!got_hw_api_capabilities || !got_estimation_diag || !got_gnss) {
    RCLCPP_WARN(node_->get_logger(), "waiting for data: HW Api=%s EstimationManager=%s, GNSS=%s", got_hw_api_capabilities ? " TRUE " : " FALSE ",
                got_estimation_diag ? "true" : "FALSE", got_gnss ? "true" : "FALSE");
    return;
  }

  initialize();
  timer_prerequisites_->cancel();
}
//}

/* timerStatus() //{ */

void SafetyAreaManager::timerStatus() {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerStatus");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "SafetyAreaManager::timerStatus", scope_timer_logger_, scope_timer_enabled_);

  bool got_odom = sh_odometry_.hasMsg();

  if (!got_odom) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 5000, "waiting for data: Odometry=%s", got_odom ? "true" : "FALSE");
    return;
  }

  if (world_origin_changed_) {

    // Rebuild safety area according to new world origin, keeping the global coordinates of the previous world origin (if it was set given to the previous world
    // origin)
    std::scoped_lock lock(mutex_safety_area_);

    auto       current_border                 = safety_zone_handler_.safety_zone->getBorder();
    auto       global_border                  = safety_zone_handler_.global_safety_zone->getBorder();
    const auto global_points                  = global_border.getPoints();
    auto [success, transformed_border_points] = transformPoints(global_points, global_border.getHorizontalFrame(), current_border.getHorizontalFrame());

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to re-initialize safety area after world origin change - could not transform border points.");
      return;
    }
    auto new_border_prism = std::make_unique<mrs_lib::safety_zone::Prism>(transformed_border_points, global_border.getMaxZ(), global_border.getMinZ(),
                                                                          current_border.getHorizontalFrame(), current_border.getVerticalFrame());
    // Recompute Obstacles
    const auto                                               &global_obstacles_ptrs = safety_zone_handler_.global_safety_zone->getObstacles();
    std::vector<std::unique_ptr<mrs_lib::safety_zone::Prism>> new_obstacle_prisms;

    for (const auto &[key, global_obstacle_ptr] : global_obstacles_ptrs) {

      const auto &global_obstacle_points = global_obstacle_ptr->getPoints();
      auto [success, transformed_obstacle_points] =
          transformPoints(global_obstacle_points, global_obstacle_ptr->getHorizontalFrame(), current_border.getHorizontalFrame());

      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to re-initialize safety area after world origin change - could not transform obstacle points.");
        return;
      }

      new_obstacle_prisms.push_back(std::make_unique<mrs_lib::safety_zone::Prism>(transformed_obstacle_points, global_obstacle_ptr->getMaxZ(),
                                                                                  global_obstacle_ptr->getMinZ(), current_border.getHorizontalFrame(),
                                                                                  global_obstacle_ptr->getVerticalFrame()));
    }

    auto new_safety_zone = createSafetyZone(std::move(new_border_prism), std::move(new_obstacle_prisms));

    if (!new_safety_zone) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to re-initialize safety area after world origin change.");
    }

    // Update values of new safety zone
    safety_zone_handler_.visualization_components.safeCleanup();
    safety_zone_handler_  = std::move(*new_safety_zone);
    world_origin_changed_ = false;
  }

  // Publishing
  publishDiagnostics();
}

//}

// --------------------------------------------------------------
// |                          callbacks                          |
// --------------------------------------------------------------

// | --------------------- topic callbacks -------------------- |

/* //{ callbackOdometry() */

void SafetyAreaManager::callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackOdometry");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "SafetyAreaManager::callbackOdometry", scope_timer_logger_, scope_timer_enabled_);

  // | ------------------ check for time stamp ------------------ |

  {
    std::scoped_lock lock(mutex_uav_state_);

    if (uav_state_.header.stamp == msg->header.stamp) {
      return;
    }
  }

  // | ----------- copy the odometry to the uav_state ----------- |

  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_                  = mrs_msgs::msg::UavState();
    uav_state_.header           = msg->header;
    uav_state_.pose             = msg->pose.pose;
    uav_state_.velocity.angular = msg->twist.twist.angular;
  }
}

//}

/* //{  point.callbackGNSS() */

void SafetyAreaManager::callbackGNSS(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackGNSS");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "SafetyAreaManager::callbackGNSS", scope_timer_logger_, scope_timer_enabled_);

  transformer_->setLatLon(msg->latitude, msg->longitude);
}

//}

// | -------------------- service callbacks ------------------- |

/* callbackAddObstacle() //{ */

bool SafetyAreaManager::callbackAddObstacle(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request>  request,
                                            const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response) {

  if (!is_initialized_) {
    response->message = "not initialized";
    response->success = false;
    RCLCPP_WARN(node_->get_logger(), "Service request not possible, not initialized");
    return true;
  }

  std::scoped_lock lock(mutex_safety_area_);

  mrs_msgs::msg::ReferenceStamped point;
  point.header    = request->header;
  point.reference = request->reference;
  // Get the safety area horizontal frame
  std::string horizontal_frame = safety_zone_handler_.safety_zone->getBorder().getHorizontalFrame();
  auto        tfed_horizontal  = transformer_->transformSingle(point, horizontal_frame);

  if (!tfed_horizontal) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform the point to the safety area horizontal frame");
    response->message = "Could not transform the point to the safety area horizontal frame";
    response->success = false;
    return true;
  }

  double offset_x = tfed_horizontal->reference.position.x;
  double offset_y = tfed_horizontal->reference.position.y;

  std::vector<mrs_lib::safety_zone::Point2d> points = {
      mrs_lib::safety_zone::Point2d{2.5 + offset_x, 2.5 + offset_y},
      mrs_lib::safety_zone::Point2d{2.5 + offset_x, -2.5 + offset_y},
      mrs_lib::safety_zone::Point2d{-2.5 + offset_x, -2.5 + offset_y},
      mrs_lib::safety_zone::Point2d{-2.5 + offset_x, 2.5 + offset_y},
  };

  int id = safety_zone_handler_.safety_zone->addObstacle(std::make_unique<mrs_lib::safety_zone::Prism>(points, 5, 0));

  safety_zone_handler_.visualization_components.static_edges.push_back(
      std::make_unique<mrs_lib::StaticEdgesVisualization>(safety_zone_handler_.safety_zone.get(), id, _uav_name_, horizontal_frame, node_, 2));

  RCLCPP_INFO(node_->get_logger(), "Obstacle loaded successfully");

  return true;
}

//}

/* callbackSetObstacle() //{ */

bool SafetyAreaManager::callbackSetObstacle(const std::shared_ptr<mrs_msgs::srv::SetObstacleSrv::Request>  request,
                                            const std::shared_ptr<mrs_msgs::srv::SetObstacleSrv::Response> response) {

  if (!is_initialized_) {
    response->message = "not initialized";
    response->success = false;
    RCLCPP_WARN(node_->get_logger(), "Service request not possible, not initialized");
    return true;
  }

  std::scoped_lock lock(mutex_safety_area_);

  auto [valid, message] = validateMsg(request->prism);

  if (!valid) {
    RCLCPP_WARN(node_->get_logger(), "Service request invalid: %s", message.c_str());
    response->message = "Service request invalid: " + message;
    response->success = false;
    return true;
  }

  auto prism_ptr = makePrism(request->prism.points, request->prism.max_z, request->prism.min_z, request->prism.horizontal_frame, request->prism.vertical_frame);

  if (!prism_ptr) {
    RCLCPP_WARN(node_->get_logger(), "Could not create prism from the provided message");
    response->message = "Could not create prism from the provided message";
    response->success = false;
    return true;
  }

  int id = safety_zone_handler_.safety_zone->addObstacle(std::move(prism_ptr));

  safety_zone_handler_.visualization_components.static_edges.push_back(
      std::make_unique<mrs_lib::StaticEdgesVisualization>(safety_zone_handler_.safety_zone.get(), id, _uav_name_, request->prism.horizontal_frame, node_, 2));

  RCLCPP_INFO(node_->get_logger(), "Obstacle loaded successfully");
  response->message = "Successfully added the obstacle";
  response->success = true;
  return true;
}

//}

/* callbackToggleSafetyArea() //{ */

bool SafetyAreaManager::callbackToggleSafetyArea(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                                                 const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  if (!is_initialized_) {
    return false;
  }
  safety_zone_handler_.safety_zone->enableSafetyZone(request->data);
  response->message = "safety area usage has been turned " + std::string((request->data ? "on" : "off"));
  response->success = true;
  RCLCPP_INFO(node_->get_logger(), "safety area usage has been turned %s", (request->data ? "on" : "off"));
  return true;
}

//}

// /* callbackSetSafetyBorder() //{ */

bool SafetyAreaManager::callbackSetSafetyBorder(const std::shared_ptr<mrs_msgs::srv::SetSafetyBorderSrv::Request>  request,
                                                const std::shared_ptr<mrs_msgs::srv::SetSafetyBorderSrv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  if (!sh_control_manager_diag_.hasMsg()) {
    RCLCPP_WARN(node_->get_logger(), "No control manager diagnostics received yet.");
    response->message = "No control manager diagnostics received yet.";
    response->success = false;
    return true;
  }

  auto control_manager_diagnostics = sh_control_manager_diag_.getMsg();

  if (control_manager_diagnostics->tracker_status.have_goal) {
    RCLCPP_WARN(node_->get_logger(), "Can only modify safety area in IDLE state.");
    response->message = "Can only modify safety area in IDLE state.";
    response->success = false;
    return true;
  }

  std::scoped_lock lock(mutex_safety_area_);

  bool success = initializationFromMsg(request->prism, request->keep_obstacles);

  if (!success) {
    RCLCPP_WARN(node_->get_logger(), "Failed to set safety border.");
    response->message = "Failed to set border";
    response->success = false;
    return true;
  }

  if (request->update_world_origin) {
    auto ref_request             = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();
    ref_request->header.frame_id = "latlon_origin";

    // Get centroid of the safety border
    if (!safety_zone_handler_.global_safety_zone) {
      RCLCPP_WARN(node_->get_logger(), "Global safety zone not set, cannot update world origin.");
      response->success = false;
      response->message = "Global safety zone not set, cannot update world origin.";
      return true;
    }

    auto center                       = safety_zone_handler_.global_safety_zone->getBorder().getCenter();
    ref_request->reference.position.x = center.get<0>();
    ref_request->reference.position.y = center.get<1>();

    RCLCPP_INFO(node_->get_logger(), "Updating world origin to lat: %.6f lon: %.6f", center.get<0>(), center.get<1>());

    auto res = sc_set_world_origin_.callAsync(ref_request);

    if (!res.has_value()) {
      RCLCPP_WARN(node_->get_logger(), "Could not call EstimationManager set_world_origin service.");
      response->success = false;
      response->message = "Could not call EstimationManager set_world_origin service.";
      return true;
    }
  }

  RCLCPP_INFO(node_->get_logger(), "New safety border, with %lu vertices.", request->prism.points.size());
  response->message = "Successfully loaded safety border msg.";
  response->success = true;
  return true;
}

//}

// /* callbackValidatePoint3d() //{ */

bool SafetyAreaManager::callbackValidatePoint3d(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request>  request,
                                                const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  std::scoped_lock lock(mutex_safety_area_);

  mrs_msgs::msg::ReferenceStamped point;
  point.header    = request->header;
  point.reference = request->reference;

  std::string border_horizontal_frame = safety_zone_handler_.safety_zone->getBorder().getHorizontalFrame();
  std::string border_vertical_frame   = safety_zone_handler_.safety_zone->getBorder().getVerticalFrame();
  auto        tfed_horizontal         = transformer_->transformSingle(point, border_horizontal_frame);

  if (!tfed_horizontal) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform the point to the safety area horizontal frame");
    response->message = "Could not transform the point to the safety area horizontal frame";
    response->success = false;
    return true;
  }

  // As the vertical frame can be different from horizontal frame
  auto [success, transformed_z] = transformZ(point.header.frame_id, border_vertical_frame, tfed_horizontal->reference.position.z);

  if (!safety_zone_handler_.safety_zone->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y, transformed_z)) {
    response->message = "The point is not in the safety area";
    return true;
  }

  response->message = "The point is in the safety area";
  return true;
}

//}

// /* callbackValidatePoint2d() //{ */

bool SafetyAreaManager::callbackValidatePoint2d(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request>  request,
                                                const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response) {
  if (!is_initialized_) {
    return false;
  }

  std::scoped_lock lock(mutex_safety_area_);

  mrs_msgs::msg::ReferenceStamped point;
  point.reference = request->reference;
  point.header    = request->header;

  std::string border_horizontal_frame = safety_zone_handler_.safety_zone->getBorder().getHorizontalFrame();
  auto        tfed_horizontal         = transformer_->transformSingle(point, border_horizontal_frame);

  if (!tfed_horizontal) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform the point to the safety area horizontal frame");
    response->message = "Could not transform the point to the safety area horizontal frame";
    response->success = false;
    return true;
  }

  if (!safety_zone_handler_.safety_zone->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y)) {
    response->message = "The point is not in the safety area";
    response->success = false;
    return true;
  }

  response->message = "The point is in the safety area";
  response->success = true;
  return true;
}

//}

// /* callbackValidatePathtoPoint3d() //{ */

bool SafetyAreaManager::callbackValidatePathToPoint3d(const std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Request>  request,
                                                      const std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  std::scoped_lock lock(mutex_safety_area_);

  // transform points
  geometry_msgs::msg::PointStamped start_transformed, end_transformed;

  std::string border_horizontal_frame = safety_zone_handler_.safety_zone->getBorder().getHorizontalFrame();

  {
    auto resp = transformer_->transformSingle(request->start, border_horizontal_frame);

    if (!resp) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1.0, "Could not transform the point to the safety area horizontal frame");
      response->message = "Could not transform the first point in the path";
      response->success = false;
      return true;
    }

    start_transformed = resp.value();
  }

  {
    auto resp = transformer_->transformSingle(request->end, border_horizontal_frame);

    if (!resp) {
      RCLCPP_WARN(node_->get_logger(), "Could not transform the point to the safety area horizontal frame");
      response->success = false;
      response->message = "Could not transform the point in the path";
      return true;
    }

    end_transformed = resp.value();
  }

  // verify the whole path
  mrs_lib::safety_zone::Point3d start_point, end_point;
  start_point.set<0>(start_transformed.point.x);
  start_point.set<1>(start_transformed.point.y);
  start_point.set<2>(start_transformed.point.z);
  end_point.set<0>(end_transformed.point.x);
  end_point.set<1>(end_transformed.point.y);
  end_point.set<2>(end_transformed.point.z);

  if (!safety_zone_handler_.safety_zone->isPathValid(start_point, end_point)) {
    response->success = false;
    response->message = "The path is not valid";
    return true;
  }

  response->message = "The path is valid";
  response->success = true;
  return true;
}

//}

/* callbackValidatePathtoPoint2d() //{ */

bool SafetyAreaManager::callbackValidatePathToPoint2d(const std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Request>  request,
                                                      const std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  std::scoped_lock lock(mutex_safety_area_);

  // transform points
  geometry_msgs::msg::PointStamped start_transformed, end_transformed;

  std::string border_horizontal_frame = safety_zone_handler_.safety_zone->getBorder().getHorizontalFrame();

  {
    auto resp = transformer_->transformSingle(request->start, border_horizontal_frame);

    if (!resp) {
      RCLCPP_WARN(node_->get_logger(), "Could not transform the point to the safety area horizontal frame");
      response->message = "Could not transform the first point in the path";
      response->success = false;
      return true;
    }

    start_transformed = resp.value();
  }

  {
    auto resp = transformer_->transformSingle(request->end, border_horizontal_frame);

    if (!resp) {
      RCLCPP_WARN(node_->get_logger(), "Could not transform the point to the safety area horizontal frame");
      response->message = "Could not transform the point in the path";
      response->success = false;
      return true;
    }

    end_transformed = resp.value();
  }

  // verify the whole path
  mrs_lib::safety_zone::Point2d start_point, end_point;
  start_point.set<0>(start_transformed.point.x);
  start_point.set<1>(start_transformed.point.y);
  end_point.set<0>(end_transformed.point.x);
  end_point.set<1>(end_transformed.point.y);

  if (!safety_zone_handler_.safety_zone->isPathValid(start_point, end_point)) {
    response->success = false;
    response->message = "The path is not valid";
    return true;
  }

  response->message = "The path is valid";
  response->success = true;
  return true;
}

// /* callbackGetMaxZ() //{ */

bool SafetyAreaManager::callbackGetMaxZ([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Request> request,
                                        const std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }
  std::scoped_lock lock(mutex_safety_area_);

  if (!safety_zone_handler_.safety_zone) {
    RCLCPP_WARN(node_->get_logger(), "No safety border defined.");
    return false;
  }

  auto border = safety_zone_handler_.safety_zone->getBorder();

  double      max_z          = border.getMaxZ();
  std::string vertical_frame = border.getVerticalFrame();

  response->reference.header.frame_id      = vertical_frame;
  response->reference.reference.position.x = 0;
  if (safety_zone_handler_.safety_zone->safetyZoneEnabled()) {
    response->reference.reference.position.z = max_z;
  } else {
    response->reference.reference.position.z = std::numeric_limits<double>::max();
  }

  response->success = true;
  return true;
}

//}

/* callbackGetMinZ() //{ */

bool SafetyAreaManager::callbackGetMinZ([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Request> request,
                                        const std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  std::scoped_lock lock(mutex_safety_area_);

  if (!safety_zone_handler_.safety_zone) {
    RCLCPP_WARN(node_->get_logger(), "No safety border defined.");
    return false;
  }

  auto border = safety_zone_handler_.safety_zone->getBorder();

  double      min_z          = border.getMinZ();
  std::string vertical_frame = border.getVerticalFrame();

  response->reference.header.frame_id      = vertical_frame;
  response->reference.reference.position.x = 0;
  if (safety_zone_handler_.safety_zone->safetyZoneEnabled()) {
    response->reference.reference.position.z = min_z;
  } else {
    response->reference.reference.position.z = std::numeric_limits<double>::lowest();
  }

  response->success = true;
  return true;
}

//}

/* callbackIsSafetyZoneEnabled() //{ */

bool SafetyAreaManager::callbackIsSafetyZoneEnabled([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::GetBoolSrv::Request> request,
                                                    const std::shared_ptr<mrs_msgs::srv::GetBoolSrv::Response>                 response) {
  if (!is_initialized_) {
    return false;
  }

  std::scoped_lock lock(mutex_safety_area_);

  if (!safety_zone_handler_.safety_zone) {
    RCLCPP_WARN(node_->get_logger(), "No safety border defined.");
    return false;
  }

  response->value = safety_zone_handler_.safety_zone->safetyZoneEnabled();
  return true;
}

//}
//
bool SafetyAreaManager::callbackUpdateWorldOrigin(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request>  request,
                                                  const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response) {

  if (!is_initialized_) {
    response->success = false;
    response->message = "Could not set world origin";
    return true;
  }

  std::scoped_lock lock(mutex_safety_area_);
  if (request->header.frame_id.find("latlon_origin") != std::string::npos) {
    RCLCPP_INFO(node_->get_logger(), "Setting world origin to lat: %.6f lon: %.6f", request->reference.position.x, request->reference.position.y);
  } else if (request->header.frame_id.find("utm_origin") != std::string::npos) {
    RCLCPP_INFO(node_->get_logger(), "Setting world origin to x: %.2f y: %.2f UTM", request->reference.position.x, request->reference.position.y);
  } else {
    RCLCPP_WARN(node_->get_logger(), "Requested unsupported frame_id: \"%s\" in set_world_origin service. Supported are: latlon_origin, utm_origin",
                request->header.frame_id.c_str());
    response->success = false;
    response->message = "Requested unsupported frame_id. Supported are: latlon_origin, utm_origin";
    return true;
  }

  safety_zone_handler_.parameters.world_origin.x     = request->reference.position.x;
  safety_zone_handler_.parameters.world_origin.y     = request->reference.position.y;
  safety_zone_handler_.parameters.world_origin.units = (request->header.frame_id.find("latlon_origin") != std::string::npos) ? "LATLON" : "UTM";

  response->success     = true;
  response->message     = "World origin set successfully";
  world_origin_changed_ = true;

  return true;
}
// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* initializationFromFile() //{ */

bool SafetyAreaManager::initializationFromFile(mrs_lib::ParamLoader &param_loader, const std::string &filename) {

  if (!param_loader.addYamlFile(filename)) {
    return false;
  }

  std::string world_origin_units;
  double      origin_x;
  double      origin_y;
  std::string horizontal_frame;
  std::string vertical_frame;
  bool        safety_area_enabled;

  param_loader.loadParam("world_origin/units", world_origin_units);
  param_loader.loadParam("world_origin/origin_x", origin_x);
  param_loader.loadParam("world_origin/origin_y", origin_y);
  param_loader.loadParam("safety_area/enabled", safety_area_enabled);
  param_loader.loadParam("safety_area/horizontal/frame_name", horizontal_frame);
  param_loader.loadParam("safety_area/vertical/frame_name", vertical_frame);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load world config parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  // Make border prism
  const Eigen::MatrixXd border_points = param_loader.loadMatrixDynamic2("safety_area/horizontal/points", -1, 2);
  const auto            max_z         = param_loader.loadParam2<double>("safety_area/vertical/max_z");
  const auto            min_z         = param_loader.loadParam2<double>("safety_area/vertical/min_z");

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load safety area border parameters!");
    return false;
  }

  auto border = makePrism(border_points, max_z, min_z, horizontal_frame, vertical_frame);

  if (!border) {
    RCLCPP_WARN(node_->get_logger(), "Failed to create border prism!");
    return false;
  }

  // Making obstacle prisms
  std::vector<std::unique_ptr<mrs_lib::safety_zone::Prism>> obstacles;

  bool obstacles_present = false;

  param_loader.loadParam("obstacles/present", obstacles_present);

  // If any is present, fill obstacles
  if (obstacles_present) {

    // Read parameters for obstacles
    int obstacles_count = 0;

    param_loader.loadParam("obstacles/count", obstacles_count);

    // std::vector<Eigen::MatrixXd> obstacles;
    obstacles.reserve(obstacles_count);

    for (int i = 0; i < obstacles_count; i++) {
      double      max_z, min_z;
      std::string horizontal_frame, vertical_frame;

      std::string obstacle_path = "obstacles/obstacle_" + std::to_string(i);

      std::string     points_path = obstacle_path + "/horizontal/points";
      Eigen::MatrixXd obstacle    = param_loader.loadMatrixDynamic2(points_path, -1, 2);

      std::string horizontal_frame_path = obstacle_path + "/horizontal/frame_name";
      param_loader.loadParam(horizontal_frame_path, horizontal_frame);

      std::string vertical_frame_path = obstacle_path + "/vertical/frame_name";
      param_loader.loadParam(vertical_frame_path, vertical_frame);

      std::string max_z_path = obstacle_path + "/vertical/max_z";
      param_loader.loadParam(max_z_path, max_z);

      std::string min_z_path = obstacle_path + "/vertical/min_z";
      param_loader.loadParam(min_z_path, min_z);

      if (!param_loader.loadedSuccessfully()) {
        RCLCPP_ERROR(node_->get_logger(), "could not load safety area obstacle %d parameters!", i);
        return false;
      }

      // Make obstacle prism
      auto prism = makePrism(obstacle, max_z, min_z, horizontal_frame, vertical_frame);

      if (!prism) {
        RCLCPP_WARN(node_->get_logger(), "Failed to create obstacle prism %d!", i);
        return false;
      }
      obstacles.push_back(std::move(prism));
    }
  }

  auto new_safety_zone = createSafetyZone(std::move(border), std::move(obstacles));

  if (!new_safety_zone) {
    RCLCPP_WARN(node_->get_logger(), "Failed to create new safety zone.");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "New safety zone created");

  // Update values of new safety zone
  safety_zone_handler_.visualization_components.safeCleanup();
  safety_zone_handler_                               = std::move(*new_safety_zone);
  safety_zone_handler_.parameters.world_origin.units = world_origin_units;
  safety_zone_handler_.parameters.world_origin.x     = origin_x;
  safety_zone_handler_.parameters.world_origin.y     = origin_y;

  return true;
}

//}

/* initializionFromMsg(Border) //{ */

std::vector<std::unique_ptr<mrs_lib::safety_zone::Prism>> SafetyAreaManager::copyExistingObstacles() {
  if (!safety_zone_handler_.safety_zone) {
    return {};
  }

  const auto                                               &obstacles_map = safety_zone_handler_.safety_zone->getObstacles();
  std::vector<std::unique_ptr<mrs_lib::safety_zone::Prism>> obstacles_copy;
  obstacles_copy.reserve(obstacles_map.size());

  for (const auto &[id, obstacle_ptr] : obstacles_map) {
    obstacles_copy.push_back(std::make_unique<mrs_lib::safety_zone::Prism>(*obstacle_ptr));
  }

  return obstacles_copy;
}

bool SafetyAreaManager::initializationFromMsg(const mrs_msgs::msg::Prism &prism_msg, bool keep_obstacles) {

  auto [res, message] = validateMsg(prism_msg);

  if (!res) {
    RCLCPP_WARN(node_->get_logger(), "Message not valid, error message: %s", message.c_str());
    return false;
  }

  // Make border prism
  auto border = makePrism(prism_msg.points, prism_msg.max_z, prism_msg.min_z, prism_msg.horizontal_frame, prism_msg.vertical_frame);

  if (!border) {
    RCLCPP_WARN(node_->get_logger(), "Failed to create border prism!");
    return false;
  }

  auto new_safety_zone = keep_obstacles ? createSafetyZone(std::move(border), copyExistingObstacles()) : createSafetyZone(std::move(border), {});

  RCLCPP_INFO(node_->get_logger(), "New safety zone created");

  if (!new_safety_zone) {
    RCLCPP_WARN(node_->get_logger(), "Failed to create new safety zone.");
    return false;
  }

  // Update values of new safety zone
  safety_zone_handler_.visualization_components.safeCleanup();
  safety_zone_handler_ = std::move(*new_safety_zone);

  return true;
}

//}

/* createSafetyZone () //{ */

std::optional<SafetyAreaManager::SafetyZoneHandler>
SafetyAreaManager::createSafetyZone(std::unique_ptr<mrs_lib::safety_zone::Prism>            &&border,
                                    std::vector<std::unique_ptr<mrs_lib::safety_zone::Prism>> obstacle_prisms = {}) {
  SafetyZoneHandler safety_zone_handler;
  safety_zone_handler.safety_zone = std::make_shared<mrs_lib::safety_zone::SafetyZone>(std::move(border), std::move(obstacle_prisms));

  if (!safety_zone_handler.safety_zone) {
    return std::nullopt;
  }

  safety_zone_handler.safety_zone->enableSafetyZone(true);

  // RViz Visualizations
  std::string horizontal_frame = safety_zone_handler.safety_zone->getBorder().getHorizontalFrame();
  safety_zone_handler.visualization_components.static_edges.push_back(
      std::make_unique<mrs_lib::StaticEdgesVisualization>(safety_zone_handler.safety_zone.get(), _uav_name_, horizontal_frame, node_, 2));

  /* // Obstacles, overloading for obstacles */
  const auto &obstacles = safety_zone_handler.safety_zone->getObstacles();
  for (const auto &[id, obstacle_ptr] : obstacles) {
    safety_zone_handler.visualization_components.static_edges.push_back(
        std::make_unique<mrs_lib::StaticEdgesVisualization>(safety_zone_handler.safety_zone.get(), id, _uav_name_, horizontal_frame, node_, 2));
  }

  // Save current safety zone in a global reference frame
  auto              safety_border           = safety_zone_handler.safety_zone->getBorder();
  const auto        points                  = safety_border.getPoints();
  const std::string border_horizontal_frame = safety_border.getHorizontalFrame();
  const std::string border_vertical_frame   = safety_border.getVerticalFrame();
  const double      border_max_z            = safety_border.getMaxZ();
  const double      border_min_z            = safety_border.getMinZ();

  auto [success, transformed_border_points] = transformPoints(points, border_horizontal_frame, "latlon_origin");

  if (!success) {
    RCLCPP_WARN(node_->get_logger(), "Failed to transform safety border points to latlon_origin frame.");
    return std::nullopt;
  }

  // Log the border points in latlon
  RCLCPP_INFO(node_->get_logger(), "Safety area border points in latlon_origin frame:");
  for (const auto &point : transformed_border_points) {
    RCLCPP_INFO(node_->get_logger(), "  lat: %.6f lon: %.6f", point.get<0>(), point.get<1>());
  }

  auto global_border_prism =
      std::make_unique<mrs_lib::safety_zone::Prism>(transformed_border_points, border_max_z, border_min_z, "latlon_origin", border_vertical_frame);

  std::vector<std::unique_ptr<mrs_lib::safety_zone::Prism>> global_obstacle_prisms;
  if (!obstacle_prisms.empty()) {
    // getObstacles return a vector with the obstacle ptr's
    const auto &obstacles_ptrs = safety_zone_handler.safety_zone->getObstacles();

    for (const auto &[key, obstacle_ptr] : obstacles_ptrs) {
      mrs_msgs::msg::Prism tmp_obstacle;
      const auto          &obstacle_points = obstacle_ptr->getPoints();
      tmp_obstacle.horizontal_frame        = obstacle_ptr->getHorizontalFrame();
      tmp_obstacle.vertical_frame          = obstacle_ptr->getVerticalFrame();
      tmp_obstacle.max_z                   = obstacle_ptr->getMaxZ();
      tmp_obstacle.min_z                   = obstacle_ptr->getMinZ();

      auto [success, transformed_obstacle_points] = transformPoints(obstacle_points, tmp_obstacle.horizontal_frame, "latlon_origin");

      if (!success) {
        RCLCPP_WARN(node_->get_logger(), "Failed to transform obstacle points to latlon_origin frame.");
        return std::nullopt;
      }

      RCLCPP_INFO(node_->get_logger(), "Saved obstacle in latlon_origin frame:");
      for (const auto &point : transformed_obstacle_points) {
        RCLCPP_INFO(node_->get_logger(), "  lat: %.6f lon: %.6f", point.get<0>(), point.get<1>());
      }

      global_obstacle_prisms.push_back(std::make_unique<mrs_lib::safety_zone::Prism>(transformed_obstacle_points, tmp_obstacle.max_z, tmp_obstacle.min_z,
                                                                                     "latlon_origin", tmp_obstacle.vertical_frame));
    }
  }

  safety_zone_handler.global_safety_zone =
      std::make_shared<mrs_lib::safety_zone::SafetyZone>(std::move(global_border_prism), std::move(global_obstacle_prisms));

  if (!safety_zone_handler.global_safety_zone) {
    RCLCPP_WARN(node_->get_logger(), "Failed to create global safety zone.");
    return std::nullopt;
  }

  // Copy parameters from previous safety zone if exists
  if (safety_zone_handler_.safety_zone) {
    safety_zone_handler.parameters = safety_zone_handler_.parameters;
  }

  return safety_zone_handler;
}

//}

/* validateMsg() //{ */

std::tuple<bool, std::string> SafetyAreaManager::validateMsg(const mrs_msgs::msg::Prism &prism_msg) {

  if (prism_msg.points.empty()) {
    return std::make_tuple(false, "Prism points are empty");
  }

  if (prism_msg.horizontal_frame.empty() || prism_msg.vertical_frame.empty()) {
    return std::make_tuple(false, "Frame units (horizontal_frame and vertical_frame) are empty");
  }

  return std::make_tuple(true, "Valid message");
}

//}

/* makePrism(matrix) //{ */

std::unique_ptr<mrs_lib::safety_zone::Prism> SafetyAreaManager::makePrism(const Eigen::MatrixXd &matrix, const double max_z, const double min_z,
                                                                          const std::string &horizontal_frame, const std::string &vertical_frame) {

  if (matrix.rows() < 3) {
    RCLCPP_WARN(node_->get_logger(), "Invalid polygon, must have at least 3 points. Provided:  %zu", matrix.rows());
  }

  std::vector<mrs_lib::safety_zone::Point2d> points;
  points.reserve(matrix.rows());

  for (int i = 0; i < matrix.rows(); i++) {
    points.emplace_back(mrs_lib::safety_zone::Point2d{matrix(i, 0), matrix(i, 1)});
  }

  auto [success, transformed_points] = transformPoints(points, horizontal_frame, "world_origin");

  if (!success) {
    RCLCPP_WARN(node_->get_logger(), "Failed to transform prism points to world_origin frame.");
    return nullptr;
  }

  auto [z_success_max, transformed_max_z] = transformZ(vertical_frame, "world_origin", max_z);
  auto [z_success_min, transformed_min_z] = transformZ(vertical_frame, "world_origin", min_z);

  if (!z_success_max || !z_success_min) {
    RCLCPP_WARN(node_->get_logger(), "Failed to transform prism Z coordinates to world_origin frame.");
    return nullptr;
  }

  return std::make_unique<mrs_lib::safety_zone::Prism>(transformed_points, transformed_max_z, transformed_min_z, "world_origin", "world_origin");
}

//}

/* makePrism(points) //{ */
std::unique_ptr<mrs_lib::safety_zone::Prism> SafetyAreaManager::makePrism(const std::vector<mrs_msgs::msg::Point2D> &points, const double max_z,
                                                                          const double min_z, const std::string &horizontal_frame,
                                                                          const std::string &vertical_frame) {

  if (points.size() < 3) {
    RCLCPP_WARN(node_->get_logger(), "Invalid polygon, must have at least 3 points. Provided:  %zu", points.size());
  }

  std::vector<mrs_lib::safety_zone::Point2d> tmp_points;

  tmp_points.reserve(points.size());

  for (const auto &point : points) {
    tmp_points.emplace_back(mrs_lib::safety_zone::Point2d{point.x, point.y});
  }

  auto [success, transformed_points] = transformPoints(tmp_points, horizontal_frame, "world_origin");
  if (!success) {
    RCLCPP_WARN(node_->get_logger(), "Failed to transform prism points to world_origin frame.");
    return nullptr;
  }

  auto [z_success_max, transformed_max_z] = transformZ(vertical_frame, "world_origin", max_z);
  auto [z_success_min, transformed_min_z] = transformZ(vertical_frame, "world_origin", min_z);

  if (!z_success_max || !z_success_min) {
    RCLCPP_WARN(node_->get_logger(), "Failed to transform prism Z coordinates to world_origin frame.");
    return nullptr;
  }

  return std::make_unique<mrs_lib::safety_zone::Prism>(transformed_points, transformed_max_z, transformed_min_z, "world_origin", "world_origin");
}

//}

/* transformPoints() //{ */

std::tuple<bool, std::vector<mrs_lib::safety_zone::Point2d>> SafetyAreaManager::transformPoints(const std::vector<mrs_lib::safety_zone::Point2d> &points,
                                                                                                const std::string &from_frame, const std::string &to_frame) {

  std::vector<mrs_lib::safety_zone::Point2d> transformed_points;
  transformed_points.reserve(points.size());
  mrs_msgs::msg::ReferenceStamped reference_tmp;

  for (const auto &point : points) {
    reference_tmp.header.frame_id      = from_frame;
    reference_tmp.header.stamp         = rclcpp::Time(0);
    reference_tmp.reference.position.x = boost::geometry::get<0>(point);
    reference_tmp.reference.position.y = boost::geometry::get<1>(point);
    reference_tmp.reference.position.z = 0;

    if (auto transformed_reference = transformer_->transformSingle(reference_tmp, to_frame)) {
      reference_tmp = transformed_reference.value();
      transformed_points.emplace_back(mrs_lib::safety_zone::Point2d{reference_tmp.reference.position.x, reference_tmp.reference.position.y});
    } else {
      RCLCPP_WARN(node_->get_logger(), "Could not transform point from %s to %s.", from_frame.c_str(), to_frame.c_str());
      return std::make_tuple(false, std::vector<mrs_lib::safety_zone::Point2d>{});
    }
  }

  return std::make_tuple(true, transformed_points);
}

//}

/* transformZ() //{ */

std::tuple<bool, double> SafetyAreaManager::transformZ(const std::string &current_frame, const std::string &target_frame, const double z) {
  geometry_msgs::msg::Point point;
  point.x = 0;
  point.y = 0;
  point.z = z;

  const auto res = transformer_->transformSingle(current_frame, point, target_frame);
  if (!res) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform point from %s to %s.", current_frame.c_str(), target_frame.c_str());
    return std::make_tuple(false, 0);
  }

  return std::make_tuple(true, res.value().z);
}

//}

/* //{ isPointInSafetyArea2d() */
bool SafetyAreaManager::isPointInSafetyArea2d(const mrs_msgs::msg::ReferenceStamped &point) {

  std::scoped_lock lock(mutex_safety_area_);
  std::string      horizontal_frame = safety_zone_handler_.safety_zone->getBorder().getHorizontalFrame();
  auto             tfed_horizontal  = transformer_->transformSingle(point, horizontal_frame);

  if (!tfed_horizontal) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform the point to the safety area horizontal frame");
    return false;
  }

  if (!safety_zone_handler_.safety_zone->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y)) {
    return false;
  }

  return true;
}

//}

/* //{ isPointInSafetyArea3d() */

bool SafetyAreaManager::isPointInSafetyArea3d(const mrs_msgs::msg::ReferenceStamped &point) {

  std::scoped_lock lock(mutex_safety_area_);
  std::string      horizontal_frame = safety_zone_handler_.safety_zone->getBorder().getHorizontalFrame();
  auto             tfed_horizontal  = transformer_->transformSingle(point, horizontal_frame);

  if (!tfed_horizontal) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform the point to the safety area horizontal frame");
    return false;
  }

  if (!safety_zone_handler_.safety_zone->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y,
                                                      tfed_horizontal->reference.position.z)) {
    return false;
  }

  return true;
}

//}

/* //{ isPathToPointInSafetyArea2d() */

bool SafetyAreaManager::isPathToPointInSafetyArea2d(const mrs_msgs::msg::ReferenceStamped &start, const mrs_msgs::msg::ReferenceStamped &end) {

  mrs_msgs::msg::ReferenceStamped start_transformed, end_transformed;

  if (!isPointInSafetyArea2d(start) || !isPointInSafetyArea2d(end)) {
    return false;
  }

  std::scoped_lock lock(mutex_safety_area_);

  if (!safety_zone_handler_.safety_zone) {
    RCLCPP_WARN(node_->get_logger(), "No safety border defined.");
    return false;
  }

  auto border = safety_zone_handler_.safety_zone->getBorder();

  std::string border_frame = border.getHorizontalFrame();
  {
    auto ret = transformer_->transformSingle(start, border_frame);

    if (!ret) {
      RCLCPP_WARN(node_->get_logger(), "Could not transform the first point in the path");
      return false;
    }

    start_transformed = ret.value();
  }

  {
    auto ret = transformer_->transformSingle(end, border_frame);

    if (!ret) {
      RCLCPP_WARN(node_->get_logger(), "Could not transform the first point in the path");
      return false;
    }

    end_transformed = ret.value();
  }

  // verify the whole path
  mrs_lib::safety_zone::Point2d start_point, end_point;
  start_point.set<0>(start_transformed.reference.position.x);
  start_point.set<1>(start_transformed.reference.position.y);
  end_point.set<0>(end_transformed.reference.position.x);
  end_point.set<1>(end_transformed.reference.position.y);

  return safety_zone_handler_.safety_zone->isPathValid(start_point, end_point);
}

//}

/* //{ isPathToPointInSafetyArea3d() */

bool SafetyAreaManager::isPathToPointInSafetyArea3d(const mrs_msgs::msg::ReferenceStamped &start, const mrs_msgs::msg::ReferenceStamped &end) {

  if (!isPointInSafetyArea3d(start) || !isPointInSafetyArea3d(end)) {
    return false;
  }

  mrs_msgs::msg::ReferenceStamped start_transformed, end_transformed;

  std::scoped_lock lock(mutex_safety_area_);
  if (!safety_zone_handler_.safety_zone) {
    RCLCPP_WARN(node_->get_logger(), "No safety border defined.");
    return false;
  }

  auto border = safety_zone_handler_.safety_zone->getBorder();

  std::string border_frame = border.getHorizontalFrame();

  {
    auto ret = transformer_->transformSingle(start, border_frame);

    if (!ret) {
      RCLCPP_WARN(node_->get_logger(), "Could not transform the first point in the path");
      return false;
    }

    start_transformed = ret.value();
  }

  {
    auto ret = transformer_->transformSingle(end, border_frame);

    if (!ret) {
      RCLCPP_WARN(node_->get_logger(), "Could not transform the first point in the path");
      return false;
    }

    end_transformed = ret.value();
  }

  // verify the whole path
  mrs_lib::safety_zone::Point3d start_point, end_point;
  start_point.set<0>(start_transformed.reference.position.x);
  start_point.set<1>(start_transformed.reference.position.y);
  start_point.set<2>(start_transformed.reference.position.z);
  end_point.set<0>(end_transformed.reference.position.x);
  end_point.set<1>(end_transformed.reference.position.y);
  end_point.set<2>(end_transformed.reference.position.z);

  return safety_zone_handler_.safety_zone->isPathValid(start_point, end_point);
}

//}

/* //{ getMaxZ() */

double SafetyAreaManager::getMaxZ(const std::string &frame_id) {

  // | ---------- first, get max_z from the safety area --------- |

  std::scoped_lock lock(mutex_safety_area_);
  if (!safety_zone_handler_.safety_zone) {
    RCLCPP_WARN(node_->get_logger(), "No safety border defined.");
    return std::numeric_limits<float>::max();
  }

  auto border = safety_zone_handler_.safety_zone->getBorder();

  double safety_area_max_z = std::numeric_limits<float>::max();

  std::string horizontal_frame = border.getHorizontalFrame();

  geometry_msgs::msg::PointStamped point;
  point.header.frame_id = horizontal_frame;
  point.point.x         = 0;
  point.point.y         = 0;
  point.point.z         = safety_zone_handler_.safety_zone->getBorder().getMaxZ();

  auto ret = transformer_->transformSingle(point, frame_id);

  if (!ret) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform safety area's max_z to '%s'", frame_id.c_str());
  }
  // | ------------ overwrite from estimation manager ----------- |

  double estimation_manager_max_z = std::numeric_limits<float>::max();

  {
    // if possible, override it with max z from the estimation manager
    if (sh_max_z_.hasMsg()) {

      auto msg = sh_max_z_.getMsg();

      // transform it into the safety area frame
      geometry_msgs::msg::PointStamped point;
      point.header  = msg->header;
      point.point.x = 0;
      point.point.y = 0;
      point.point.z = msg->value;

      auto ret = transformer_->transformSingle(point, frame_id);

      if (!ret) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform estimation manager's max_z to the "
                                         "current control frame");
      }

      estimation_manager_max_z = ret->point.z;
    }
  }

  if (estimation_manager_max_z < safety_area_max_z) {
    return estimation_manager_max_z;
  } else {
    return safety_area_max_z;
  }
}

//}

/* //{ getMinZ() */

double SafetyAreaManager::getMinZ(const std::string &frame_id) {

  // | ---------- first, get min_z from the safety area --------- |
  std::scoped_lock lock(mutex_safety_area_);

  if (!safety_zone_handler_.safety_zone) {
    RCLCPP_WARN(node_->get_logger(), "No safety border defined.");
    return std::numeric_limits<float>::lowest();
  }

  if (!safety_zone_handler_.safety_zone->safetyZoneEnabled()) {
    return std::numeric_limits<float>::lowest();
  }

  std::string horizontal_frame = safety_zone_handler_.safety_zone->getBorder().getHorizontalFrame();

  geometry_msgs::msg::PointStamped point;
  point.header.frame_id = horizontal_frame;
  point.point.x         = 0;
  point.point.y         = 0;
  point.point.z         = safety_zone_handler_.safety_zone->getBorder().getMinZ();

  auto ret = transformer_->transformSingle(point, frame_id);

  if (!ret) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform safety area's min_z to '%s'", frame_id.c_str());
    return std::numeric_limits<float>::lowest();
  }

  return ret->point.z;
}

//}

/* publishDiagnostics() //{ */
void SafetyAreaManager::publishDiagnostics(void) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("publishDiagnostics");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "SafetyAreaManager::publishDiagnostics", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::msg::SafetyAreaManagerDiagnostics diagnostics_msg;

  // copy member variables
  auto uav_state          = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto world_origin_units = mrs_lib::get_mutexed(mutex_safety_area_, safety_zone_handler_.parameters.world_origin.units);
  auto origin_x           = mrs_lib::get_mutexed(mutex_safety_area_, safety_zone_handler_.parameters.world_origin.x);
  auto origin_y           = mrs_lib::get_mutexed(mutex_safety_area_, safety_zone_handler_.parameters.world_origin.y);

  diagnostics_msg.stamp                             = clock_->now();
  diagnostics_msg.uav_name                          = _uav_name_;
  auto [is_position_valid_2d, is_position_valid_3d] = isPositionValid(uav_state);
  diagnostics_msg.position_valid_2d                 = is_position_valid_2d;
  diagnostics_msg.position_valid_3d                 = is_position_valid_3d;

  // Fill world origin
  diagnostics_msg.world_origin.units = world_origin_units;
  diagnostics_msg.world_origin.x     = origin_x;
  diagnostics_msg.world_origin.y     = origin_y;

  // | ------------------------- Fill in the safety zone data
  // ------------------------ |
  {
    std::scoped_lock lock(mutex_safety_area_);

    // Enable flag
    diagnostics_msg.safety_area_enabled = safety_zone_handler_.safety_zone->safetyZoneEnabled();

    if (!safety_zone_handler_.safety_zone) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "No safety border defined.");
      ph_diagnostics_.publish(diagnostics_msg);
      return;
    }

    // Get border points
    auto safety_border = safety_zone_handler_.safety_zone->getBorder();

    diagnostics_msg.border.horizontal_frame = safety_border.getHorizontalFrame();
    diagnostics_msg.border.vertical_frame   = safety_border.getVerticalFrame();

    const auto border_points = safety_border.getPoints();

    mrs_msgs::msg::Point2D tmp_point;
    for (const auto &point : border_points) {
      tmp_point.x = boost::geometry::get<0>(point);
      tmp_point.y = boost::geometry::get<1>(point);
      diagnostics_msg.border.points.push_back(tmp_point);
    }

    // Get safety_border max and min z
    diagnostics_msg.border.max_z = safety_border.getMaxZ();
    diagnostics_msg.border.min_z = safety_border.getMinZ();

    // getObstacles return a vector with the obstacle ptr's
    const auto &obstacles_ptrs = safety_zone_handler_.safety_zone->getObstacles();

    diagnostics_msg.obstacles_present = obstacles_ptrs.size() == 0 ? false : true;

    // Iterate over vector of ObstaclePtr
    for (const auto &[key, obstaclePtr] : obstacles_ptrs) {

      mrs_msgs::msg::Prism tmp_obstacle;

      const auto          &obstacle_points = obstaclePtr->getPoints();
      tmp_obstacle.horizontal_frame        = obstaclePtr->getHorizontalFrame();
      tmp_obstacle.vertical_frame          = obstaclePtr->getVerticalFrame();
      tmp_obstacle.max_z                   = obstaclePtr->getMaxZ();
      tmp_obstacle.min_z                   = obstaclePtr->getMinZ();

      // Extract the points of the obstacle
      for (const auto &point : obstacle_points) {
        tmp_point.x = boost::geometry::get<0>(point);
        tmp_point.y = boost::geometry::get<1>(point);
        tmp_obstacle.points.push_back(tmp_point);
      }

      diagnostics_msg.obstacles.push_back(tmp_obstacle);
    }
  }

  // | ------------------------- publish ------------------------ |

  ph_diagnostics_.publish(diagnostics_msg);
}

//}

/* isPositionValid() //{ */

std::tuple<bool, bool> SafetyAreaManager::isPositionValid(mrs_msgs::msg::UavState uav_state) {

  if (!is_initialized_) {
    return std::make_tuple(false, false);
  }

  mrs_msgs::msg::ReferenceStamped current_position;

  current_position.header.frame_id    = uav_state.header.frame_id;
  current_position.reference.position = uav_state.pose.position;

  RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Initial current position x:  " << current_position.reference.position.x
                                                                               << " y: " << current_position.reference.position.y
                                                                               << " z: " << current_position.reference.position.z);

  auto is_position_valid_2d = isPointInSafetyArea2d(current_position);
  auto is_position_valid_3d = isPointInSafetyArea3d(current_position);

  if (!is_position_valid_3d) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Outside of safety area (3D validation) current "
                         "position x:  %.3f y: %.3f z: %.3f",
                         current_position.reference.position.x, current_position.reference.position.y, current_position.reference.position.z);
  }

  if (!is_position_valid_2d) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Outside of safety area (2D validation) current "
                         "position x:  %.3f y: %.3f",
                         current_position.reference.position.x, current_position.reference.position.y);
  }

  return std::make_tuple(is_position_valid_2d, is_position_valid_3d);
}

//}

} // namespace safety_area_manager

} // namespace mrs_uav_managers

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_managers::safety_area_manager::SafetyAreaManager)
