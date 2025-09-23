/* includes //{ */

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

// #include <mrs_uav_managers/safety_area_manager/common_handlers.h>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/safety_zone.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/utils.h>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/estimation_diagnostics.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>
// #include <mrs_msgs/GetBoolSrv.h>
// #include <mrs_msgs/msg/GetPointStamped.h>
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


#include <mrs_msgs/msg/safety_area_manager_diagnostics.hpp>
#include <mrs_msgs/msg/point2_d.hpp>
#include <mrs_msgs/srv/prism_srv.hpp>
#include <mrs_msgs/srv/validate_path_to_point_srv.hpp>
#include <mrs_msgs/srv/get_reference_stamped_srv.hpp>
#include <mrs_msgs/srv/get_bool_srv.hpp>

#include <mrs_lib/safety_zone/static_edges_visualization.h>

// TODO add into mrs_lib
// #include <mrs_lib/safety_zone/int_edges_visualization.h>
// #include <mrs_lib/safety_zone/vertex_control.h>
// #include <mrs_lib/safety_zone/center_control.h>
// #include <mrs_lib/safety_zone/bounds_control.h>
// #include <mrs_lib/safety_zone/yaml_export_visitor.h>

//}

namespace bg = boost::geometry;
namespace mrs_uav_managers
{

namespace safety_area_manager
{

/* class SafetyAreaManager //{ */

class SafetyAreaManager : public rclcpp::Node {

public:
  SafetyAreaManager(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;

  rclcpp::TimerBase::SharedPtr timer_preinitialization_;
  void                         timerPreInitialization();

  std::shared_ptr<mrs_lib::Transformer> transformer_;
  std::atomic<bool> is_initialized_ = false;
  std::atomic<bool> set_latlon_set_ = false;

  // | ------------------- scope timer logger ------------------- |

  bool scope_timer_enabled_ = false;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;
  std::string _uav_name_;
  std::string _world_config_;

  struct Obstacle
  {
    std::vector<mrs_msgs::msg::Point2D> data;
    double max_z;
    double min_z;

    // Obstacle(const std::vector<mrs_msgs::Point2D> &pts, double maxZ, double minZ) : data(pts), max_z(maxZ), min_z(minZ) {}
  };

  struct VisualizationComponents
  {
    std::vector<std::unique_ptr<mrs_lib::StaticEdgesVisualization>> static_edges;

    // TO REMOVE
    // std::vector<std::unique_ptr<mrs_lib::IntEdgesVisualization>> int_edges;
    // std::vector<std::unique_ptr<mrs_lib::VertexControl>> vertices;
    // std::vector<std::unique_ptr<mrs_lib::CenterControl>> centers;
    // std::vector<std::unique_ptr<mrs_lib::BoundsControl>> bounds;

    void safeCleanup() {
      // bounds.clear();
      // centers.clear();
      // vertices.clear();
      // int_edges.clear();
      static_edges.clear();
    }
  };

  struct SafetyZoneParams
  {
    bool use_safety_area;
    // TODO: Evaluate if make sense to take care of world_origin or delegate from estimation manager
    mrs_msgs::msg::WorldOrigin world_origin;
    std::string horizontal_frame;
    std::string vertical_frame;
  };

  struct SafetyZoneHandler
  {
    std::shared_ptr<mrs_lib::safety_zone::SafetyZone> safety_zone;
    VisualizationComponents visualization_components;
    SafetyZoneParams parameters;

    SafetyZoneHandler()                                = default;
    SafetyZoneHandler(SafetyZoneHandler &&)            = default;
    SafetyZoneHandler &operator=(SafetyZoneHandler &&) = default;
  } safety_zone_handler_;

  std::mutex mutex_safety_area_;

  // Useful to check if world origin changed
  geometry_msgs::msg::TransformStamped tf_fcu_to_world_origin_;
  geometry_msgs::msg::TransformStamped tf_viz_;

  // profiling
  mrs_lib::Profiler profiler_;
  bool profiler_enabled_ = false;
  int status_timer_rate_ = 0;

  // diagnostics publishing
  void publishDiagnostics(void);
  void getSafetyZoneData(void);

  std::tuple<bool, bool> isPositionValid(mrs_msgs::msg::UavState);

  void initialize();

  // | -------------- uav_state/odometry subscriber ------------- |

  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> sh_odometry_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped> sh_max_z_;
  mrs_msgs::msg::UavState uav_state_;
  std::mutex mutex_uav_state_;

  mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix> sh_gnss_;


  // | --------------------- service servers -------------------- |

  // safety area services
  rclcpp::Service<mrs_msgs::srv::ReferenceStampedSrv>::SharedPtr service_server_point_in_safety_area_3d_;
  rclcpp::Service<mrs_msgs::srv::ReferenceStampedSrv>::SharedPtr service_server_point_in_safety_area_2d_;
  rclcpp::Service<mrs_msgs::srv::ValidatePathToPointSrv>::SharedPtr service_server_path_in_safety_area_3d_;
  rclcpp::Service<mrs_msgs::srv::ValidatePathToPointSrv>::SharedPtr service_server_path_in_safety_area_2d_;
  rclcpp::Service<mrs_msgs::srv::PrismSrv>::SharedPtr service_server_set_safety_border_;
  rclcpp::Service<mrs_msgs::srv::String>::SharedPtr service_server_set_world_config_;
  rclcpp::Service<mrs_msgs::srv::String>::SharedPtr service_server_get_world_config_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_toggle_safety_area_;
  rclcpp::Service<mrs_msgs::srv::ReferenceStampedSrv>::SharedPtr service_server_add_obstacle_;
  rclcpp::Service<mrs_msgs::srv::PrismSrv>::SharedPtr service_server_set_obstacle_;
  rclcpp::Service<mrs_msgs::srv::GetReferenceStampedSrv>::SharedPtr service_server_get_max_z_;
  rclcpp::Service<mrs_msgs::srv::GetReferenceStampedSrv>::SharedPtr service_server_get_min_z_;
  rclcpp::Service<mrs_msgs::srv::GetBoolSrv>::SharedPtr service_server_is_safety_zone_enabled_;

  // | --------------------- service clients -------------------- |
  // TODO add service into estimation manager
  // mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>
  // sch_set_world_origin_;

  // | ----------------------- subscribers ----------------------- |

  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiCapabilities> sh_hw_api_capabilities_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics> sh_estimation_diag_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mrs_msgs::msg::SafetyAreaManagerDiagnostics> ph_diagnostics_;

  // contains handlers that are used with the safety area visualization/interactive tools
  // std::shared_ptr<mrs_uav_managers::safety_area_manager::CommonHandlers_t> common_handlers_;

  // | ----------------------- timers ----------------------- |

  // this timer will check till we already got the hardware api diagnostics
  // then it will trigger the initialization of the controllers and finish
  // the initialization of the SafetyAreaManager
  rclcpp::TimerBase::SharedPtr timer_prerequisites_;
  void timerPrerequisites();

  // timer for regular status publishing
  std::shared_ptr<TimerType> timer_status_;
  void timerStatus();

  // | ----------------------- callbacks ----------------------- |
  // topic callbacks
  void callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void callbackGNSS(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);

  // services
  bool callbackValidatePoint3d(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response);
  bool callbackValidatePoint2d(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response);
  bool callbackValidatePathToPoint3d(const std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Response> response);
  bool callbackValidatePathToPoint2d(const std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Response> response); 

  bool callbackSetSafetyBorder(const std::shared_ptr<mrs_msgs::srv::PrismSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::PrismSrv::Response> response);
  bool callbackToggleSafetyArea(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  bool callbackAddObstacle(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response);
  bool callbackSetObstacle(const std::shared_ptr<mrs_msgs::srv::PrismSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::PrismSrv::Response> response);
  bool callbackGetMaxZ(const std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Response> response);
  bool callbackGetMinZ(const std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Response> response);
  bool callbackIsSafetyZoneEnabled([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::GetBoolSrv::Request> request, std::shared_ptr<mrs_msgs::srv::GetBoolSrv::Response> response); 

  // | ----------------------- routines ----------------------- |

  // Safety area building
  std::unique_ptr<mrs_lib::safety_zone::Prism> makePrism(const Eigen::MatrixXd matrix, const double max_z, const double min_z, const std::string &horizontal_frame);
  // TODO change it to Prism msg
  std::unique_ptr<mrs_lib::safety_zone::Prism> makePrism(const std::vector<mrs_msgs::msg::Point2D> &points, const double max_z, const double min_z,
                                            const std::string &horizontal_frame);
  std::vector<mrs_lib::safety_zone::Point2d> transformPoints(const std::vector<mrs_lib::safety_zone::Point2d> &points, const std::string &from_frame, const std::string &to_frame);

  double transformZ(const std::string &current_frame, const std::string &target_frame, const double z);
  bool initializationFromFile(mrs_lib::ParamLoader &param_loader, const std::string &filename);
  std::vector<std::unique_ptr<mrs_lib::safety_zone::Prism>> copyExistingObstacles();
  bool initializationFromMsg(const mrs_msgs::msg::Prism &prism_msg, bool keep_obstacles);
  // bool initializationFromMsg(const mrs_msgs::SafetyArea &safety_area_msg);
  std::optional<SafetyZoneHandler> createSafetyZone(const std::unique_ptr<mrs_lib::safety_zone::Prism> &border);
  std::optional<SafetyZoneHandler> createSafetyZone(const std::unique_ptr<mrs_lib::safety_zone::Prism> &border,
                                                    std::vector<std::unique_ptr<mrs_lib::safety_zone::Prism>> &&obstacle_prisms);

  std::tuple<bool, std::string> validateMsg(const mrs_msgs::msg::Prism &prism_msg);
  // std::tuple<bool, std::string> validateMsg(const mrs_msgs::SafetyArea &safety_area_msg);

  // Reference validation
  // those are passed to trackers using the common_handlers object  TODO
  bool isPointInSafetyArea2d(const mrs_msgs::msg::ReferenceStamped &point);
  bool isPointInSafetyArea3d(const mrs_msgs::msg::ReferenceStamped &point);
  bool isPathToPointInSafetyArea2d(const mrs_msgs::msg::ReferenceStamped &from, const mrs_msgs::msg::ReferenceStamped &to);
  bool isPathToPointInSafetyArea3d(const mrs_msgs::msg::ReferenceStamped &from, const mrs_msgs::msg::ReferenceStamped &to);
  double getMaxZ(const std::string &frame_id);
  double getMinZ(const std::string &frame_id);

}; // class SafetyAreaManager

//}
/* SafetyAreaManager::SafetyAreaManager() //{ */

SafetyAreaManager::SafetyAreaManager(rclcpp::NodeOptions options) : Node("safety_area_manager", options) {

  timer_preinitialization_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&SafetyAreaManager::timerPreInitialization, this));
}

//}

/* timerPreInitialization() //{ */

void SafetyAreaManager::timerPreInitialization() {

  node_  = this->shared_from_this();
  clock_ = node_->get_clock();

  cbkgrp_subs_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_   = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_   = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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

  timer_prerequisites_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&SafetyAreaManager::timerPrerequisites, this));

  timer_preinitialization_->cancel();
}

//}

/* initialize() //{ */

void SafetyAreaManager::initialize() {

  RCLCPP_INFO(node_->get_logger(), "initializing");

  // | --------------------- parameters ---------------------- |
  mrs_lib::ParamLoader param_loader(node_, "SafetyAreaManager");
  param_loader.loadParam("world_config", _world_config_);
  param_loader.addYamlFile(_world_config_);

  // param_loader.loadParam("world_origin/units", safety_zone_handler_.parameters.world_origin_units);
  // param_loader.loadParam("world_origin/origin_x", safety_zone_handler_.parameters.origin_x);
  // param_loader.loadParam("world_origin/origin_y", safety_zone_handler_.parameters.origin_x);
  // param_loader.loadParam("safety_area/enabled", safety_zone_handler_.parameters.safety_area_enabled);
  // // TODO this might be extended for obstacles as well
  // param_loader.loadParam("safety_area/border/horizontal_frame", safety_zone_handler_.parameters.horizontal_frame);
  // param_loader.loadParam("safety_area/border/vertical_frame", safety_zone_handler_.parameters.vertical_frame);

  param_loader.addYamlFileFromParam("private_config");
  // param_loader.addYamlFileFromParam("public_config");

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("enable_profiler", profiler_enabled_);

  param_loader.setPrefix("mrs_uav_managers/safety_area_manager/");
  param_loader.loadParam("status_timer_rate", status_timer_rate_);

  // | ------------------------ profiler ------------------------ |
  profiler_ = mrs_lib::Profiler(node_, "SafetyAreaManager", profiler_enabled_);

  // | ---------------------- tf-transformer ----------------------- |
  transformer_ = std::make_shared<mrs_lib::Transformer>(node_);
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | ------------------- scope timer logger ------------------- |

  param_loader.loadParam("scope_timer/enabled", scope_timer_enabled_);
  const std::string scope_timer_log_filename = param_loader.loadParam2("scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<mrs_lib::ScopeTimerLogger>(node_,scope_timer_log_filename, scope_timer_enabled_);

  // // binding of common handlers
  // common_handlers_->transformer                       = transformer_; common_handlers_->scope_timer.enabled               = scope_timer_enabled_; common_handlers_->scope_timer.logger                = scope_timer_logger_;
  // common_handlers_->safety_area.use_safety_area       = safety_zone_handler_.parameters.safety_area_enabled;
  // common_handlers_->safety_area.isPointInSafetyArea2d = boost::bind(&SafetyAreaManager::isPointInSafetyArea2d, this, _1);
  // common_handlers_->safety_area.isPointInSafetyArea3d = boost::bind(&SafetyAreaManager::isPointInSafetyArea3d, this, _1);
  // common_handlers_->safety_area.getMinZ               = boost::bind(&SafetyAreaManager::getMinZ, this, _1);
  // common_handlers_->safety_area.getMaxZ               = boost::bind(&SafetyAreaManager::getMaxZ, this, _1);
  // common_handlers_->uav_name                          = _uav_name_;
  // common_handlers_->parent_nh                         = nh_;

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  param_loader.setPrefix("");

  // | ---------------------- safety zone ----------------------- |
  // Note: safety_zone is initialized even if the use_safety_area_ is false
  // The manager will just always return true untill it's turned on
  bool safety_zone_inited = false;
  safety_zone_inited = initializationFromFile(param_loader, _world_config_);
  
  if (!safety_zone_inited) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize safety area from file.");
    rclcpp::shutdown();
    exit(1);
  }

  // | ----------------------- publishers ----------------------- |
  ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_msgs::msg::SafetyAreaManagerDiagnostics>(node_, "~/safety_area_diagnostics_out"); 

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

  service_server_point_in_safety_area_3d_   = node_->create_service<mrs_msgs::srv::ReferenceStampedSrv>("~/point_in_safety_area_3d_in",
      [this](std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request, std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response){
    callbackValidatePoint3d(request, response);
    },
    rclcpp::SystemDefaultsQoS(),
    cbkgrp_ss_
  );

  service_server_point_in_safety_area_2d_   = node_->create_service<mrs_msgs::srv::ReferenceStampedSrv>("~/point_in_safety_area_2d_in",
      [this](std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request, std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response){
    callbackValidatePoint2d(request, response);
    },
    rclcpp::SystemDefaultsQoS(),
    cbkgrp_ss_
  );

  service_server_path_in_safety_area_3d_    = node_->create_service<mrs_msgs::srv::ValidatePathToPointSrv>("~/path_in_safety_area_3d_in",
      [this](std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Request> request, std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Response> response){
    callbackValidatePathToPoint3d(request, response);
    },
    rclcpp::SystemDefaultsQoS(),
    cbkgrp_ss_
  );

  service_server_path_in_safety_area_2d_    = node_->create_service<mrs_msgs::srv::ValidatePathToPointSrv>("~/path_in_safety_area_2d_in",
      [this](std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Request> request, std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Response> response){
    callbackValidatePathToPoint2d(request, response);
    },
    rclcpp::SystemDefaultsQoS(),
    cbkgrp_ss_
  );
  
  service_server_set_safety_border_         = node_->create_service<mrs_msgs::srv::PrismSrv>("~/set_safety_border_in",
      [this](std::shared_ptr<mrs_msgs::srv::PrismSrv::Request> request, std::shared_ptr<mrs_msgs::srv::PrismSrv::Response> response){
    callbackSetSafetyBorder(request, response);
    },
    rclcpp::SystemDefaultsQoS(),
    cbkgrp_ss_
  );

  service_server_toggle_safety_area_           = node_->create_service<std_srvs::srv::SetBool>("~/set_use_safety_area_in",
      [this](std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    callbackToggleSafetyArea(request, response);
    },
    rclcpp::SystemDefaultsQoS(),
    cbkgrp_ss_
  );

  service_server_add_obstacle_              = node_->create_service<mrs_msgs::srv::ReferenceStampedSrv>("~/add_obstacle_in",
      [this](std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request, std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response){
    callbackAddObstacle(request, response);
    },
    rclcpp::SystemDefaultsQoS(),
    cbkgrp_ss_
  );

  service_server_set_obstacle_              = node_->create_service<mrs_msgs::srv::PrismSrv>("~/set_obstacle_in",
      [this](std::shared_ptr<mrs_msgs::srv::PrismSrv::Request> request, std::shared_ptr<mrs_msgs::srv::PrismSrv::Response> response){
    callbackSetObstacle(request, response);
    },
    rclcpp::SystemDefaultsQoS(),
    cbkgrp_ss_
  );

  service_server_get_max_z_                 = node_->create_service<mrs_msgs::srv::GetReferenceStampedSrv>("~/get_max_z_in",
      [this](std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Request> request, std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Response> response){
    callbackGetMaxZ(request, response);
    },
    rclcpp::SystemDefaultsQoS(),
    cbkgrp_ss_
  );
  
  service_server_get_min_z_                 = node_->create_service<mrs_msgs::srv::GetReferenceStampedSrv>("~/get_min_z_in",
      [this](std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Request> request, std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Response> response){
    callbackGetMinZ(request, response);
    },
    rclcpp::SystemDefaultsQoS(),
    cbkgrp_ss_
  );

  service_server_is_safety_zone_enabled_    = node_->create_service<mrs_msgs::srv::GetBoolSrv>("~/is_safety_zone_enabled_in",
      [this](std::shared_ptr<mrs_msgs::srv::GetBoolSrv::Request> request, std::shared_ptr<mrs_msgs::srv::GetBoolSrv::Response> response){
    callbackIsSafetyZoneEnabled(request, response);
    },
    rclcpp::SystemDefaultsQoS(),
    cbkgrp_ss_
  );
  

  // | --------------------- service clients -------------------- |

  // clients to communicate with SafetyAreaManager
  // service_client_set_world_origin_ = nh_.serviceClient<mrs_msgs::msg::ReferenceStampedSrv>("set_world_origin");

  // | ------------------------- timers ------------------------- |
  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node      = node_;
  timer_opts_start.autostart = true;

  {
    std::function<void()> callback_fcn = std::bind(&SafetyAreaManager::timerStatus, this);

    timer_status_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(status_timer_rate_, clock_), callback_fcn);
  }

  // | ----------------------- finish init ---------------------- |

  if (!safety_zone_inited) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize safety area.");
    rclcpp::shutdown();
    exit(1);
  }

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "Safety area initialized");

}

//}

// --------------------------------------------------------------
// |                          timers                            |
// --------------------------------------------------------------

/* timerPrerequisites() //{ */

void SafetyAreaManager::timerPrerequisites() {
  mrs_lib::Routine profiler_routine = profiler_.createRoutine("timerPrerequisites");
  // mrs_lib::ScopeTimer timer         = mrs_lib::ScopeTimer(node_,"SafetyAreaManager::timerPrerequisites", scope_timer_logger_, scope_timer_enabled_);

  bool got_hw_api_capabilities  = sh_hw_api_capabilities_.hasMsg();
  bool got_control_manager_diag = sh_control_manager_diag_.hasMsg();
  bool got_estimation_diag      = sh_estimation_diag_.hasMsg();

  if (!got_hw_api_capabilities || !got_control_manager_diag || !got_estimation_diag) {
    RCLCPP_WARN(node_->get_logger(),"waiting for data: ControlManager=%s, HW Api=%s EstimationManager=%s",
                         got_control_manager_diag ? "true" : "FALSE", got_hw_api_capabilities ? "true" : "FALSE", got_estimation_diag ? "true" : "FALSE");
    return;
  }

  // auto ret = transformer_->getTransform(safety_zone_handler_.parameters.horizontal_frame, "local_origin", rclcpp::Time(0));
  // if (ret) {
  //   RCLCPP_INFO_ONCE(node_->get_logger(), "got TF %s -> local_origin", safety_zone_handler_.parameters.horizontal_frame.c_str());
  //   tf_viz_ = ret.value();
  // } else {
  //   RCLCPP_INFO_ONCE(node_->get_logger(), "waiting for TF %s -> local_origin", safety_zone_handler_.parameters.horizontal_frame.c_str());
  //   return;
  // }
  //
  // // We need to have the UTM zone established to be able to transform 'latlon_origin' input points
  // if (!set_latlon_set_) {
  //   RCLCPP_INFO_ONCE(node_->get_logger(), "waiting for UTM zone to be set");
  //   return;
  // }

  initialize();
  timer_prerequisites_->cancel();
}
//}

/* timerStatus //{ */

void SafetyAreaManager::timerStatus() {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("timerStatus");
  mrs_lib::ScopeTimer timer         = mrs_lib::ScopeTimer(node_,"SafetyAreaManager::timerStatus", scope_timer_logger_, scope_timer_enabled_);

  bool got_odom = sh_odometry_.hasMsg();

  if (!got_odom) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 5000, "waiting for data: Odometry=%s", got_odom ? "true" : "FALSE");
    return;
  }

  // Get the current SafetyZone data
  getSafetyZoneData();

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

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackOdometry");
  mrs_lib::ScopeTimer timer         = mrs_lib::ScopeTimer(node_,"SafetyAreaManager::callbackOdometry", scope_timer_logger_, scope_timer_enabled_);

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

/* //{ callbackGNSS() */

void SafetyAreaManager::callbackGNSS(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackGNSS");
  mrs_lib::ScopeTimer timer         = mrs_lib::ScopeTimer(node_,"SafetyAreaManager::callbackGNSS", scope_timer_logger_, scope_timer_enabled_);

  transformer_->setLatLon(msg->latitude, msg->longitude);
  set_latlon_set_ = true;
}

//}

// | -------------------- service callbacks ------------------- |

/* callbackAddObstacle() //{ */

bool SafetyAreaManager::callbackAddObstacle(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response) {

  if (!is_initialized_) {
    response->message = "not initialized";
    response->success = false;
    RCLCPP_WARN(node_->get_logger(), "Service request not possible, not initialized");
    return true;
  }

  std::scoped_lock lock(mutex_safety_area_);

  mrs_msgs::msg::ReferenceStamped point;
  point.header         = request->header;
  point.reference      = request->reference;
  auto tfed_horizontal = transformer_->transformSingle(point, safety_zone_handler_.parameters.horizontal_frame);

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

  safety_zone_handler_.visualization_components.static_edges.push_back(std::make_unique<mrs_lib::StaticEdgesVisualization>(
      safety_zone_handler_.safety_zone.get(), id, _uav_name_, safety_zone_handler_.parameters.horizontal_frame, node_, 2));
  // safety_zone_handler_.visualization_components.int_edges.push_back(std::make_unique<mrs_lib::IntEdgesVisualization>(
  //     safety_zone_handler_.safety_zone.get(), id, _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  // safety_zone_handler_.visualization_components.vertices.push_back(
  //     std::make_unique<mrs_lib::VertexControl>(safety_zone_handler_.safety_zone.get(), id, _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  // safety_zone_handler_.visualization_components.centers.push_back(
  //     std::make_unique<mrs_lib::CenterControl>(safety_zone_handler_.safety_zone.get(), id, _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  // safety_zone_handler_.visualization_components.bounds.push_back(
  //     std::make_unique<mrs_lib::BoundsControl>(safety_zone_handler_.safety_zone.get(), id, _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));

  RCLCPP_INFO(node_->get_logger(), "Obstacle loaded successfully");

  return true;
}

//}

/* callbackSetObstacle() //{ */

bool SafetyAreaManager::callbackSetObstacle(const std::shared_ptr<mrs_msgs::srv::PrismSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::PrismSrv::Response> response) {

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


  const auto transformed_obs_max_z = transformZ(request->prism.vertical_frame, "world_origin", request->prism.max_z);
  const auto transformed_obs_min_z = transformZ(request->prism.vertical_frame, "world_origin", request->prism.min_z);

  int id = safety_zone_handler_.safety_zone->addObstacle(makePrism(request->prism.points, transformed_obs_max_z, transformed_obs_min_z, request->prism.horizontal_frame));

  safety_zone_handler_.visualization_components.static_edges.push_back(std::make_unique<mrs_lib::StaticEdgesVisualization>(
      safety_zone_handler_.safety_zone.get(), id, _uav_name_, safety_zone_handler_.parameters.horizontal_frame, node_, 2));
  // safety_zone_handler_.visualization_components.int_edges.push_back(std::make_unique<mrs_lib::IntEdgesVisualization>(
  //     safety_zone_handler_.safety_zone.get(), id, _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  // safety_zone_handler_.visualization_components.vertices.push_back(
  //     std::make_unique<mrs_lib::VertexControl>(safety_zone_handler_.safety_zone.get(), id, _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  // safety_zone_handler_.visualization_components.centers.push_back(
  //     std::make_unique<mrs_lib::CenterControl>(safety_zone_handler_.safety_zone.get(), id, _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  // safety_zone_handler_.visualization_components.bounds.push_back(
  //     std::make_unique<mrs_lib::BoundsControl>(safety_zone_handler_.safety_zone.get(), id, _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));

  RCLCPP_INFO(node_->get_logger(), "Obstacle loaded successfully");
  response->message = "Succesfully added the obstacle";
  response->success = true;
  return true;
}

//}

/* callbackToggleSafetyArea() //{ */

bool SafetyAreaManager::callbackToggleSafetyArea(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  if (!is_initialized_) {
    return false;
  }
  safety_zone_handler_.safety_zone->enableSafetyZone(request->data);
  response->message = "safety area usage has been turned " + std::string((request->data ? "on" : "off"));
  response->success = true;
  RCLCPP_INFO(node_->get_logger(), "safety area usage has been turned %s", (request->data ? "on" : "off"));
  return true;
}

// //}
//
// /* callbackLoadWorldConfig() //{ */
//
// bool SafetyAreaManager::callbackLoadWorldConfig(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {
//
//   if (!is_initialized_) {
//     res.message = "not initialized";
//     res.success = false;
//     return true;
//   }
//
//
//   auto control_manager_diagnostics = sh_control_manager_diag_.getMsg();
//
//   if (control_manager_diagnostics->tracker_status.have_goal) {
//
//     ROS_WARN("[SafetyAreaManager]: Can only modify safety area in IDLE state");
//     res.message = "Can only modify safety area in IDLE state.";
//     res.success = false;
//     return true;
//   }
//
//   std::scoped_lock lock(mutex_safety_area_);
//   mrs_lib::ParamLoader param_loader(nh_, "SafetyAreaManager");
//   bool success = initializationFromFile(param_loader, req.value);
//
//   if (!success) {
//     ROS_WARN("[SafetyAreaManager]: Could not read the file. Probably data format is not correct.");
//     res.message = "Could not read the file. Probably data format is not correct.";
//     res.success = false;
//   }
//
//   res.message = "Successfully loaded world config.";
//   res.success = true;
//   return true;
// }
//
// //}

// /* callbackSetSafetyBorder() //{ */

bool SafetyAreaManager::callbackSetSafetyBorder(const std::shared_ptr<mrs_msgs::srv::PrismSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::PrismSrv::Response> response) {

  if (!is_initialized_) {
    return false;
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

  RCLCPP_INFO(node_->get_logger(), "New safety border, with %lu vertices.", request->prism.points.size());
  response->message = "Succesfully loaded safety border msg.";
  response->success = true;
  return true;
}

// //}
//
// /* callbackSetSafetyArea() //{ */
//
// bool SafetyAreaManager::callbackSetSafetyArea(mrs_msgs::SetSafetyAreaSrv::Request &req, mrs_msgs::SetSafetyAreaSrv::Response &res) {
//
//   if (!is_initialized_) {
//     res.message = "not initialized";
//     res.success = false;
//     return true;
//   }
//
//   auto control_manager_diagnostics = sh_control_manager_diag_.getMsg();
//
//   if (control_manager_diagnostics->tracker_status.have_goal) {
//
//     ROS_WARN("[SafetyAreaManager]: Can only modify safety area in IDLE state");
//     res.message = "Can only modify safety area in IDLE state.";
//     res.success = false;
//     return true;
//   }
//
//   std::scoped_lock lock(mutex_safety_area_);
//   bool success = initializationFromMsg(req.safety_area);
//
//   if (!success) {
//     ROS_WARN("[SafetyAreaManager]: Could not load world config. Please, check the config file.");
//     res.message = "Could not load world config. Please, check the config file.";
//     res.success = false;
//     return false;
//   }
//
//   ROS_INFO("[SafetyAreaManager]: Succesfull service call, world config loaded.");
//   res.message = "Succesfully loaded safety area msg.";
//   res.success = true;
//   return true;
// }
//
// //}
//
// /* callbackSetWorldConfig() //{ */
// bool SafetyAreaManager::callbackSetWorldConfig(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {
//
//   if (!is_initialized_) {
//     res.message = "not initialized";
//     res.success = false;
//     return true;
//   }
//
//   auto control_manager_diagnostics = sh_control_manager_diag_.getMsg();
//
//   if (control_manager_diagnostics->tracker_status.have_goal) {
//     ROS_WARN("[SafetyAreaManager]: Can only modify safety area in IDLE state");
//     res.message = "Can only modify safety area in IDLE state.";
//     res.success = false;
//     return true;
//   }
//
//   std::string filename = "/tmp/cur_world_config.yaml";
//   std::ofstream ofs(filename, std::ofstream::out | std::ofstream::trunc);
//   if (!ofs.is_open()) {
//     ROS_WARN("[SafetyAreaManager]: Could not open file %s", filename.c_str());
//     res.success = false;
//     res.message = "Could not open file " + filename;
//     return true;
//   }
//
//   ofs << req.value;
//   ofs.close();
//
//   mrs_msgs::String load_config_srv;
//   load_config_srv.request.value = filename;
//   callbackLoadWorldConfig(load_config_srv.request, load_config_srv.response);
//   res = load_config_srv.response;
//   return true;
// }
//
// //}
//
// /* callbackSaveWorldConfig() //{ */
//
// bool SafetyAreaManager::callbackSaveWorldConfig(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {
//   if (!is_initialized_) {
//     res.message = "not initialized";
//     res.success = false;
//     return true;
//   }
//
//   /* std::scoped_lock lock(mutex_safety_area_); */
//
//   mrs_lib::YamlExportVisitor visitor(_uav_name_, safety_zone_handler_.parameters.horizontal_frame, safety_zone_handler_.parameters.horizontal_frame,
//                                      safety_zone_handler_.parameters.vertical_frame, safety_zone_handler_.parameters.world_origin_units,
//                                      safety_zone_handler_.parameters.origin_x, safety_zone_handler_.parameters.origin_x, transformer_);
//
//   safety_zone_handler_.safety_zone->accept(visitor);
//
//   if (!visitor.isSuccessful()) {
//     res.message = "Something went wrong during exporting parameters";
//     res.success = false;
//     return true;
//   }
//
//   std::ofstream ofs(req.value, std::ofstream::out | std::ofstream::trunc);
//   if (!ofs.is_open()) {
//     ROS_WARN("[SafetyAreaManager]: Could not open file %s", req.value.c_str());
//     res.success = false;
//     res.message = "Could not open file " + req.value;
//     return true;
//   }
//
//   ofs << visitor.getResult();
//   ofs.close();
//   res.success = true;
//   ROS_INFO("[SafetyAreaManager]: world config has been saved to %s", req.value.c_str());
//   return true;
// }
//
// //}
//
// /* callbackGetWorldConfig() //{ */
//
// bool SafetyAreaManager::callbackGetWorldConfig([[maybe_unused]] mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {
//
//   if (!is_initialized_) {
//     res.message = "not initialized";
//     res.success = false;
//     return true;
//   }
//
//
//   std::scoped_lock lock(mutex_safety_area_);
//
//   mrs_lib::YamlExportVisitor visitor(_uav_name_, safety_zone_handler_.parameters.horizontal_frame, safety_zone_handler_.parameters.horizontal_frame,
//                                      safety_zone_handler_.parameters.vertical_frame, safety_zone_handler_.parameters.world_origin_units,
//                                      safety_zone_handler_.parameters.origin_x, safety_zone_handler_.parameters.origin_x, transformer_);
//
//   safety_zone_handler_.safety_zone->accept(visitor);
//
//   if (!visitor.isSuccessful()) {
//     res.message = "Something went wrong during exporting parameters";
//     res.success = false;
//     return true;
//   }
//
//   res.success = true;
//   res.message = visitor.getResult();
//   ROS_INFO("[SafetyAreaManager]: world config has been extracted");
//   return true;
// }
//
// //}

// TODO check if need of mutex when validating
// /* callbackValidatePoint3d() //{ */
bool SafetyAreaManager::callbackValidatePoint3d(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  mrs_msgs::msg::ReferenceStamped point;
  point.header    = request->header;
  point.reference = request->reference;

  // Transform to "world_origin" as is the default frame we use for easier validation and interaction with safety area border points.
  auto tfed_horizontal = transformer_->transformSingle(point, "world_origin");


  if (!tfed_horizontal) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform the point to the safety area horizontal frame");
    response->message = "Could not transform the point to the safety area horizontal frame";
    response->success = false;
    return true;
  }

  // TODO consider if need to transform the z value
  // As the vertical frame can be different from horizontal frame
  /* auto transformed_pos_z = transformZ(safety_zone_handler_.parameters.horizontal_frame, safety_zone_handler_.parameters.vertical_frame,
   * tfed_horizontal->reference.position.z); */

  /* ROS_INFO_STREAM("[SafetyAreaManager/isPointInSafetyArea3d]: Transformed z value : " << transformed_pos_z); */


  if (!safety_zone_handler_.safety_zone->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y,
                                                      tfed_horizontal->reference.position.z)) {
    response->message = "The point is not in the safety area";
    return true;
  }

  response->message = "The point is in the safety area";
  return true;
}

//}

// /* callbackValidatePoint2d() //{ */
bool SafetyAreaManager::callbackValidatePoint2d(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response) {
  if (!is_initialized_) {
    return false;
  }

  mrs_msgs::msg::ReferenceStamped point;
  point.reference = request->reference;
  point.header    = request->header;

  // Transform to "world_origin" as is the default frame we use for easier validation and interaction with safety area border points.
  auto tfed_horizontal = transformer_->transformSingle(point, "world_origin");

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

bool SafetyAreaManager::callbackValidatePathToPoint3d(const std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  geometry_msgs::msg::PointStamped start = request->start;
  geometry_msgs::msg::PointStamped end   = request->end;

  // transform points
  geometry_msgs::msg::PointStamped start_transformed, end_transformed;

  {
    auto resp = transformer_->transformSingle(start, "world_origin");

    if (!resp) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
      response->message = "Could not transform the first point in the path";
      response->success = false;
      return true;
    }

    start_transformed = resp.value();
  }

  {
    auto resp = transformer_->transformSingle(end, "world_origin");

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

bool SafetyAreaManager::callbackValidatePathToPoint2d(const std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  geometry_msgs::msg::PointStamped start = request->start;
  geometry_msgs::msg::PointStamped end   = request->end;

  // transform points
  geometry_msgs::msg::PointStamped start_transformed, end_transformed;

  {
    auto resp = transformer_->transformSingle(start, "world_origin");

    if (!resp) {
      RCLCPP_WARN(node_->get_logger(), "Could not transform the point to the safety area horizontal frame");
      response->message = "Could not transform the first point in the path";
      response->success = false;
      return true;
    }

    start_transformed = resp.value();
  }

  {
    auto resp = transformer_->transformSingle(end, "world_origin");

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

bool SafetyAreaManager::callbackGetMaxZ([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Response> response) {

  if (!is_initialized_) {
    return false;
  }
  std::scoped_lock lock(mutex_safety_area_);

  // TODO this will be within the safety zone prism
  response->reference.header.frame_id = safety_zone_handler_.parameters.horizontal_frame;
  response->reference.reference.position.x         = 0;
  if (safety_zone_handler_.safety_zone->safetyZoneEnabled()) {
    response->reference.reference.position.z = safety_zone_handler_.safety_zone->getBorder()->getMaxZ();
  } else {
    response->reference.reference.position.z = std::numeric_limits<double>::max();
  }

  response->success = true;
  return true;
}

//}

/* callbackGetMinZ() //{ */

bool SafetyAreaManager::callbackGetMinZ([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Response> response) { 

  if (!is_initialized_) {
    return false;
  }

  response->reference.header.frame_id = safety_zone_handler_.parameters.horizontal_frame;
  response->reference.reference.position.x         = 0;
  response->reference.reference.position.y         = 0;

  if (safety_zone_handler_.safety_zone->safetyZoneEnabled()) {
    response->reference.reference.position.z = safety_zone_handler_.safety_zone->getBorder()->getMinZ();
  } else {
    response->reference.reference.position.z = std::numeric_limits<double>::lowest();
  }

  response->success = true;
  return true;
}

//}

/* callbackIsSafetyZoneEnabled() //{ */

bool SafetyAreaManager::callbackIsSafetyZoneEnabled([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::GetBoolSrv::Request> request, const std::shared_ptr<mrs_msgs::srv::GetBoolSrv::Response> response) {
  if (!is_initialized_) {
    return false;
  }

  std::scoped_lock lock(mutex_safety_area_);

  response->value = safety_zone_handler_.safety_zone->safetyZoneEnabled();
  return true;
}

//}


// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* initializationFromFile() //{ */

bool SafetyAreaManager::initializationFromFile(mrs_lib::ParamLoader &param_loader, const std::string &filename) {
  if (!param_loader.addYamlFile(filename)) {
    return false;
  }

  // Reload parameters for every call, it can be called multiple times if using the Rviz plugin and loading different world configurations
  // safety_zone_handler_.parameters.
  std::string world_origin_units;
  double origin_x;
  double origin_y;
  std::string horizontal_frame;
  std::string vertical_frame;
  bool safety_area_enabled;
  param_loader.loadParam("world_origin/units", world_origin_units);
  param_loader.loadParam("world_origin/origin_x", origin_x);
  param_loader.loadParam("world_origin/origin_y", origin_y);
  param_loader.loadParam("safety_area/enabled", safety_area_enabled);
  param_loader.loadParam("safety_area/border/horizontal_frame", horizontal_frame);
  param_loader.loadParam("safety_area/border/vertical_frame", vertical_frame);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load world config parameters!");
    rclcpp::shutdown();
    exit(1);
  }
  //TODO set of world origin service not implemented yet
  // // As this routine will be called pre-initialization to load safety area directly from world config
  // if (is_initialized_) {
  //   mrs_msgs::msg::ReferenceStampedSrv SetOriginSrv;
  //   /* TODO: Support for UTM */
  //   SetOriginSrv.request.header.frame_id      = "latlon_origin";
  //   SetOriginSrv.request.header.stamp         = ros::Time::now();
  //   SetOriginSrv.request.reference.position.x = origin_x;
  //   SetOriginSrv.request.reference.position.y = origin_x;
  //
  //   if (!service_client_set_world_origin_.call(SetOriginSrv)) {
  //     ROS_WARN("[SafetyAreaManager]: Failed to set world_origin.");
  //     return false;
  //   }
  // }

  // Make border prism
  const Eigen::MatrixXd border_points = param_loader.loadMatrixDynamic2("safety_area/border/points", -1, 2);
  const auto max_z                    = param_loader.loadParam2<double>("safety_area/border/max_z");
  const auto min_z                    = param_loader.loadParam2<double>("safety_area/border/min_z");

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load safety area border parameters!");
    return false;
  }

  const auto transformed_max_z        = transformZ(vertical_frame, "world_origin", max_z);
  const auto transformed_min_z        = transformZ(vertical_frame, "world_origin", min_z);

  auto border = makePrism(border_points, transformed_max_z, transformed_min_z, horizontal_frame);

  // Making obstacle prisms
  std::vector<std::unique_ptr<mrs_lib::safety_zone::Prism>> obstacles;

  bool obstacles_present = false;
  param_loader.loadParam("safety_area/obstacles/present", obstacles_present);

  // If any is present, fill obstacles
  if (obstacles_present) {
    // Read parameters for obstacles
    std::vector<Eigen::MatrixXd> obstacles_mat;
    Eigen::MatrixXd max_z_mat;
    Eigen::MatrixXd min_z_mat;
    param_loader.loadMatrixDynamic("safety_area/obstacles/max_z", max_z_mat, -1, 1);
    param_loader.loadMatrixDynamic("safety_area/obstacles/min_z", min_z_mat, -1, 1);

    Eigen::MatrixXd current_mat = param_loader.loadMatrixDynamic2("safety_area/obstacles/data", -1, 2);
    Eigen::MatrixXd rows        = param_loader.loadMatrixDynamic2("safety_area/obstacles/rows", -1, 1);

    obstacles.reserve(current_mat.size());

    int start_row = 0;
    obstacles_mat.reserve(rows.rows());

    // Iterate over obstacles matrix and extract points based on rows matrix
    //"rows" matrix define the points for every obstacle
    for (int i = 0; i < rows.rows(); i++) {
      int row_num = static_cast<int>(rows(i, 0));

      if (row_num < 0 || start_row + row_num > current_mat.rows()) {
        RCLCPP_WARN(node_->get_logger(), "Invalid obstacle rows!, check your config file");
        return false;
      }

      Eigen::MatrixXd obstacle_mat = current_mat.block(start_row, 0, row_num, current_mat.cols());
      obstacles_mat.push_back(obstacle_mat);
      start_row += row_num;
    }

    if (start_row != current_mat.rows()) {
      RCLCPP_WARN(node_->get_logger(), "Invalid obstacle rows!, check your config file");
      return false;
    }

    if (!(max_z_mat.rows() == min_z_mat.rows() && min_z_mat.rows() == static_cast<long int>(obstacles_mat.size()))) {
      RCLCPP_WARN(node_->get_logger(), "Inconsistent obstacles data: max_z rows: %ld, min_z rows: %ld, obstacles number: %ld", max_z_mat.rows(), min_z_mat.rows(),
                  obstacles_mat.size());
      return false;
    }

    // Make obstacle prisms
    for (size_t i = 0; i < obstacles_mat.size(); i++) {
      const auto obs_max_z             = max_z_mat(i, 0);
      const auto obs_min_z             = min_z_mat(i, 0);
      const auto transformed_obs_max_z = transformZ(vertical_frame, "world_origin", obs_max_z);
      const auto transformed_obs_min_z = transformZ(vertical_frame, "world_origin", obs_min_z);
      auto prism                       = makePrism(obstacles_mat[i], transformed_obs_max_z, transformed_obs_min_z, horizontal_frame);

      if (prism) {
        obstacles.push_back(std::move(prism));
      } else {
        RCLCPP_WARN(node_->get_logger(), "Failed to create obstacle prism!");
      }
    }
  }

  auto new_safety_zone = createSafetyZone(std::move(border), std::move(obstacles));

  if (!new_safety_zone) {
    RCLCPP_WARN(node_->get_logger(), "Failed to create new safety zone.");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "New safety zone created");

  safety_zone_handler_.visualization_components.safeCleanup();
  safety_zone_handler_                                = std::move(*new_safety_zone);
  safety_zone_handler_.parameters.horizontal_frame    = horizontal_frame;
  safety_zone_handler_.parameters.vertical_frame      = vertical_frame;
  safety_zone_handler_.parameters.world_origin.units  = world_origin_units;
  safety_zone_handler_.parameters.world_origin.x      = origin_x;
  safety_zone_handler_.parameters.world_origin.y      = origin_y;

  return true;
}

//}

/* initializionFromMsg(Border) //{ */

std::vector<std::unique_ptr<mrs_lib::safety_zone::Prism>> SafetyAreaManager::copyExistingObstacles() {
  if (!safety_zone_handler_.safety_zone) {
    return {};
  }

  const auto &obstacles_map = safety_zone_handler_.safety_zone->getObstacles();
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
  std::vector<mrs_msgs::msg::Point2D> border_points = prism_msg.points;

  const auto transformed_max_z = transformZ(prism_msg.vertical_frame, "world_origin", prism_msg.max_z);
  const auto transformed_min_z = transformZ(prism_msg.vertical_frame, "world_origin", prism_msg.min_z);

  auto border = makePrism(border_points, transformed_max_z, transformed_min_z, prism_msg.horizontal_frame);

  auto new_safety_zone = keep_obstacles ? createSafetyZone(std::move(border), copyExistingObstacles()) : createSafetyZone(std::move(border));

  RCLCPP_INFO(node_->get_logger(), "New safety zone created");

  if (!new_safety_zone) {
    RCLCPP_WARN(node_->get_logger(), "Failed to create new safety zone.");
    return false;
  }

  // Update values of new safety zone
  safety_zone_handler_.visualization_components.safeCleanup();
  safety_zone_handler_                                = std::move(*new_safety_zone);
  safety_zone_handler_.parameters.horizontal_frame    = prism_msg.horizontal_frame;
  safety_zone_handler_.parameters.vertical_frame      = prism_msg.vertical_frame;

  return true;
}

//}

/* createSafetyZone () //{ */

std::optional<SafetyAreaManager::SafetyZoneHandler> SafetyAreaManager::createSafetyZone(const std::unique_ptr<mrs_lib::safety_zone::Prism> &border,
                                                                                        std::vector<std::unique_ptr<mrs_lib::safety_zone::Prism>> &&obstacle_prisms) {
  SafetyZoneHandler safety_zone_handler;
  std::cout << "safety zone disabled" << std::endl;
  safety_zone_handler.safety_zone = std::make_shared<mrs_lib::safety_zone::SafetyZone>(*border, std::move(obstacle_prisms));

  if (!safety_zone_handler.safety_zone) {
    return std::nullopt;
  }

  safety_zone_handler.safety_zone->enableSafetyZone(true);

  // RViz Visualizations
  // TODO visualization not implemented yet
  safety_zone_handler.visualization_components.static_edges.push_back(std::make_unique<mrs_lib::StaticEdgesVisualization>(
      safety_zone_handler.safety_zone.get(), _uav_name_, safety_zone_handler_.parameters.horizontal_frame, node_, 2));
  // safety_zone_handler.visualization_components.int_edges.push_back(std::make_unique<mrs_lib::IntEdgesVisualization>(
  //     safety_zone_handler.safety_zone.get(), _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  // safety_zone_handler.visualization_components.vertices.push_back(
  //     std::make_unique<mrs_lib::VertexControl>(safety_zone_handler.safety_zone.get(), _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  // safety_zone_handler.visualization_components.centers.push_back(
  //     std::make_unique<mrs_lib::CenterControl>(safety_zone_handler.safety_zone.get(), _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  // safety_zone_handler.visualization_components.bounds.push_back(
  //     std::make_unique<mrs_lib::BoundsControl>(safety_zone_handler.safety_zone.get(), _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  //
  
  /* // Obstacles, overloading for obstacles */
  for (auto it = safety_zone_handler.safety_zone->getObstaclesBegin(); it != safety_zone_handler.safety_zone->getObstaclesEnd(); it++) {
    safety_zone_handler.visualization_components.static_edges.push_back(std::make_unique<mrs_lib::StaticEdgesVisualization>(
        safety_zone_handler.safety_zone.get(), it->first, _uav_name_, safety_zone_handler_.parameters.horizontal_frame, node_, 2));
  //   safety_zone_handler.visualization_components.int_edges.push_back(std::make_unique<mrs_lib::IntEdgesVisualization>(
  //       safety_zone_handler.safety_zone.get(), it->first, _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  //   safety_zone_handler.visualization_components.vertices.push_back(std::make_unique<mrs_lib::VertexControl>(
  //       safety_zone_handler.safety_zone.get(), it->first, _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  //   safety_zone_handler.visualization_components.centers.push_back(std::make_unique<mrs_lib::CenterControl>(
  //       safety_zone_handler.safety_zone.get(), it->first, _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  //   safety_zone_handler.visualization_components.bounds.push_back(std::make_unique<mrs_lib::BoundsControl>(
  //       safety_zone_handler.safety_zone.get(), it->first, _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  }

  return safety_zone_handler;
}

std::optional<SafetyAreaManager::SafetyZoneHandler> SafetyAreaManager::createSafetyZone(const std::unique_ptr<mrs_lib::safety_zone::Prism> &border) {

  SafetyZoneHandler safety_zone_handler;
  safety_zone_handler.safety_zone = std::make_shared<mrs_lib::safety_zone::SafetyZone>(*border);

  if (!safety_zone_handler.safety_zone) {
    return std::nullopt;
  }

  safety_zone_handler.safety_zone->enableSafetyZone(true);

  // RViz Visualizations
  safety_zone_handler.visualization_components.static_edges.push_back(std::make_unique<mrs_lib::StaticEdgesVisualization>(
      safety_zone_handler.safety_zone.get(), _uav_name_, safety_zone_handler_.parameters.horizontal_frame, node_, 2));
  // safety_zone_handler.visualization_components.int_edges.push_back(std::make_unique<mrs_lib::IntEdgesVisualization>(
  //     safety_zone_handler.safety_zone.get(), _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  // safety_zone_handler.visualization_components.vertices.push_back(
  //     std::make_unique<mrs_lib::VertexControl>(safety_zone_handler.safety_zone.get(), _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  // safety_zone_handler.visualization_components.centers.push_back(
  //     std::make_unique<mrs_lib::CenterControl>(safety_zone_handler.safety_zone.get(), _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));
  // safety_zone_handler.visualization_components.bounds.push_back(
  //     std::make_unique<mrs_lib::BoundsControl>(safety_zone_handler.safety_zone.get(), _uav_name_, safety_zone_handler_.parameters.horizontal_frame, nh_));

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

std::unique_ptr<mrs_lib::safety_zone::Prism> SafetyAreaManager::makePrism(const Eigen::MatrixXd matrix, const double max_z, const double min_z,
                                                             const std::string &horizontal_frame) {

  if (matrix.rows() < 3) {
    RCLCPP_WARN(node_->get_logger(), "Invalid polygon, must have at least 3 points. Provided:  %zu", matrix.rows());
  }

  std::vector<mrs_lib::safety_zone::Point2d> points;
  points.reserve(matrix.rows());

  for (int i = 0; i < matrix.rows(); i++) {
    points.emplace_back(mrs_lib::safety_zone::Point2d{matrix(i, 0), matrix(i, 1)});
  }

  auto transformed_points = transformPoints(points, horizontal_frame, "world_origin");

  return std::make_unique<mrs_lib::safety_zone::Prism>(transformed_points, max_z, min_z);
}

//}

/* makePrism(points) //{ */
std::unique_ptr<mrs_lib::safety_zone::Prism> SafetyAreaManager::makePrism(const std::vector<mrs_msgs::msg::Point2D> &points, const double max_z, const double min_z,
                                                             const std::string &horizontal_frame) {

  if (points.size() < 3) {
    RCLCPP_WARN(node_->get_logger(), "Invalid polygon, must have at least 3 points. Provided:  %zu", points.size());
  }

  std::vector<mrs_lib::safety_zone::Point2d> tmp_points;

  tmp_points.reserve(points.size());

  for (const auto &point : points) {
    tmp_points.emplace_back(mrs_lib::safety_zone::Point2d{point.x, point.y});
  }

  auto transformed_points = transformPoints(tmp_points, horizontal_frame, "world_origin");
  return std::make_unique<mrs_lib::safety_zone::Prism>(transformed_points, max_z, min_z);
}

//}

/* transformPoints() //{ */

std::vector<mrs_lib::safety_zone::Point2d> SafetyAreaManager::transformPoints(const std::vector<mrs_lib::safety_zone::Point2d> &points, const std::string &from_frame,
                                                                 const std::string &to_frame) {

  // Transforming into local origin for visualization
  std::vector<mrs_lib::safety_zone::Point2d> transformed_points;
  mrs_msgs::msg::ReferenceStamped reference_tmp;

  for (const auto &point : points) {
    reference_tmp.header.frame_id      = from_frame;
    reference_tmp.header.stamp         = rclcpp::Time(0); 
    reference_tmp.reference.position.x = boost::geometry::get<0>(point);
    reference_tmp.reference.position.y = boost::geometry::get<1>(point);
    reference_tmp.reference.position.z = 0;

    auto ret = transformer_->getTransform(from_frame, to_frame, rclcpp::Time(0));
    if (!ret) {
      RCLCPP_WARN(node_->get_logger(), "Could not get transform from %s to %s", from_frame.c_str(), to_frame.c_str());
      return transformed_points;
    }

    if (auto transformed_reference = transformer_->transform(reference_tmp, ret.value())) {
      reference_tmp = transformed_reference.value();
      transformed_points.emplace_back(mrs_lib::safety_zone::Point2d{reference_tmp.reference.position.x, reference_tmp.reference.position.y});
    }
  }

  return transformed_points;
}

//}

/* transformZ() //{ */

double SafetyAreaManager::transformZ(const std::string &current_frame, const std::string &target_frame, const double z) {
  geometry_msgs::msg::Point point;
  point.x = 0;
  point.y = 0;
  point.z = z;

  const auto res = transformer_->transformSingle(current_frame, point, target_frame);
  if (!res) {
    // TODO improve the return with a better error handling
    RCLCPP_WARN(node_->get_logger(), "Could not transform point from %s to %s.", current_frame.c_str(), target_frame.c_str());
    return 0;
  }

  return res.value().z;
}

//}

/* //{ isPointInSafetyArea2d() */
bool SafetyAreaManager::isPointInSafetyArea2d(const mrs_msgs::msg::ReferenceStamped &point) {

  // Transform to "world_origin" as is the default frame we use for easier validation and interaction with safety area border points.
  auto tfed_horizontal = transformer_->transformSingle(point, "world_origin");

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

  // Transform to "world_origin" as is the default frame we use for easier validation and interaction with safety area border points.
  auto tfed_horizontal = transformer_->transformSingle(point, "world_origin");


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

  {
    auto ret = transformer_->transformSingle(start, safety_zone_handler_.parameters.horizontal_frame);

    if (!ret) {
      RCLCPP_WARN(node_->get_logger(), "Could not transform the first point in the path");
      return false;
    }

    start_transformed = ret.value();
  }

  {
    auto ret = transformer_->transformSingle(end, safety_zone_handler_.parameters.horizontal_frame);

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

  {
    auto ret = transformer_->transformSingle(start, safety_zone_handler_.parameters.horizontal_frame);

    if (!ret) {
      RCLCPP_WARN(node_->get_logger(), "Could not transform the first point in the path");
      return false;
    }

    start_transformed = ret.value();
  }

  {
    auto ret = transformer_->transformSingle(end, safety_zone_handler_.parameters.horizontal_frame);

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

  double safety_area_max_z = std::numeric_limits<float>::max();

  geometry_msgs::msg::PointStamped point;
  point.header.frame_id = safety_zone_handler_.parameters.horizontal_frame;
  point.point.x         = 0;
  point.point.y         = 0;
  point.point.z         = safety_zone_handler_.safety_zone->getBorder()->getMaxZ();

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
        RCLCPP_WARN(node_->get_logger(),"Could not transform estimation manager's max_z to the current control frame");
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

  if (!safety_zone_handler_.safety_zone->safetyZoneEnabled()) {
    return std::numeric_limits<float>::lowest();
  }

  geometry_msgs::msg::PointStamped point;
  point.header.frame_id = safety_zone_handler_.parameters.horizontal_frame;
  point.point.x         = 0;
  point.point.y         = 0;
  point.point.z         = safety_zone_handler_.safety_zone->getBorder()->getMinZ();

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

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("publishDiagnostics");
  mrs_lib::ScopeTimer timer         = mrs_lib::ScopeTimer(node_,"SafetyAreaManager::publishDiagnostics", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::msg::SafetyAreaManagerDiagnostics diagnostics_msg;

  // copy member variables
  auto uav_state                    = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  // auto use_safety_area              = mrs_lib::get_mutexed(mutex_safety_area_, safety_zone_handler_.parameters.safety_area_enabled);
  auto world_origin_units           = mrs_lib::get_mutexed(mutex_safety_area_, safety_zone_handler_.parameters.world_origin.units);
  auto origin_x                     = mrs_lib::get_mutexed(mutex_safety_area_, safety_zone_handler_.parameters.world_origin.x);
  auto origin_y                     = mrs_lib::get_mutexed(mutex_safety_area_, safety_zone_handler_.parameters.world_origin.y);
  auto safety_area_horizontal_frame = mrs_lib::get_mutexed(mutex_safety_area_, safety_zone_handler_.parameters.horizontal_frame);
  auto safety_area_vertical_frame   = mrs_lib::get_mutexed(mutex_safety_area_, safety_zone_handler_.parameters.vertical_frame);

  diagnostics_msg.stamp                             = clock_->now();
  diagnostics_msg.uav_name                          = _uav_name_;
  // diagnostics_msg.safety_area_enabled               = use_safety_area;
  auto [is_position_valid_2d, is_position_valid_3d] = isPositionValid(uav_state);
  diagnostics_msg.position_valid_2d                 = is_position_valid_2d;
  diagnostics_msg.position_valid_3d                 = is_position_valid_3d;

  // Fill world origin
  diagnostics_msg.world_origin.units   = world_origin_units;
  diagnostics_msg.world_origin.x       = origin_x;
  diagnostics_msg.world_origin.y       = origin_y;

  // TODO to remove, to add into prism to include the horizontal and vertical frame
  diagnostics_msg.border.horizontal_frame = safety_area_horizontal_frame;
  diagnostics_msg.border.vertical_frame   = safety_area_vertical_frame;

  // | ------------------------- Fill in the safety zone data ------------------------ |
  {
    std::scoped_lock lock(mutex_safety_area_);

    // Enable flag
    diagnostics_msg.safety_area_enabled = safety_zone_handler_.safety_zone->safetyZoneEnabled();

    // Get border points
    const auto safety_border = safety_zone_handler_.safety_zone->getBorder();
    const auto border_points = safety_border->getPoints();

    // Transform border points to "world_origin" frame for diagnostics
    auto transformed_border_points = transformPoints(border_points, "world_origin", safety_zone_handler_.parameters.horizontal_frame);

    // Fill transformed border points
    mrs_msgs::msg::Point2D tmp_point;
    for (const auto &point : transformed_border_points) {
      tmp_point.x = boost::geometry::get<0>(point);
      tmp_point.y = boost::geometry::get<1>(point);
      diagnostics_msg.border.points.push_back(tmp_point);
    }

    // Get safety_border max and min z
    auto safety_border_max_z              = safety_border->getMaxZ();
    diagnostics_msg.border.max_z         = transformZ("world_origin", safety_zone_handler_.parameters.vertical_frame, safety_border_max_z);
    auto safety_border_min_z              = safety_border->getMinZ();
    diagnostics_msg.border.min_z         = transformZ("world_origin", safety_zone_handler_.parameters.vertical_frame, safety_border_min_z);

    // getObstacles return a vector with the obstacle ptr's
    const auto &obstacles_ptrs = safety_zone_handler_.safety_zone->getObstacles();

    diagnostics_msg.obstacles_present = obstacles_ptrs.size() == 0 ? false : true;

    // Iterate over vector of ObstaclePtr
    for (const auto &[key, obstaclePtr] : obstacles_ptrs) {
      const auto &obstacle = obstaclePtr->getPoints();
      mrs_msgs::msg::Prism tmp_obstacle;

      // TODO properly fill the horizontal and vertical frame of the obstacle
      auto transformed_obstacle = transformPoints(obstacle, "world_origin", safety_zone_handler_.parameters.horizontal_frame);
      tmp_obstacle.max_z   = transformZ("world_origin", safety_zone_handler_.parameters.vertical_frame, obstaclePtr->getMaxZ());
      tmp_obstacle.min_z   = transformZ("world_origin", safety_zone_handler_.parameters.vertical_frame, obstaclePtr->getMinZ());

      // Extract the points of the osbstacle
      for (const auto &point : transformed_obstacle) {
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

/* getSafetyZoneData() //{ */
void SafetyAreaManager::getSafetyZoneData(void) {
  if (!is_initialized_) {
    return;
  }

  // TODO: to remove when estimation manager service is implemented, and updates the origin directly to safety area manager
  // auto estimation_sm_state = sh_estimation_diag_.getMsg()->sm_state;
  // if (estimation_sm_state == "INITIALIZED_STATE" || estimation_sm_state == "READY_FOR_FLIGHT_STATE") {
  //   auto fcu_tf = transformer_->getTransform("fcu", "world_origin", rclcpp::Time(0));
  //
  //   // Check if the world origin changed, through the estimation Mgr service
  //   //  TODO change this, add estimation manager to call a service to update the origin, as its doing with transform manager
  //   bool translation_x_change = std::abs(fcu_tf.value().transform.translation.x - tf_fcu_to_world_origin_.transform.translation.x) > 1e-2; // more than mm
  //   bool translation_y_change = std::abs(fcu_tf.value().transform.translation.y - tf_fcu_to_world_origin_.transform.translation.y) > 1e-2; // more than mm
  //
  //   if (translation_x_change || translation_y_change) {
  //     // TODO this to be replaced with estimation manager service to update origin
  //     RCLCPP_INFO(node_->get_logger(), "World origin changed, updating the safety area origin");
  //     RCLCPP_INFO(node_->get_logger(), "Old origin x: %.3f y: %.3f", tf_fcu_to_world_origin_.transform.translation.x,
  //                 tf_fcu_to_world_origin_.transform.translation.y);
  //     RCLCPP_INFO(node_->get_logger(), "New origin x: %.3f y: %.3f", fcu_tf.value().transform.translation.x, fcu_tf.value().transform.translation.y);
  //     auto world_tf = transformer_->getTransform("world_origin", "latlon_origin", rclcpp::Time(0));
  //     mrs_msgs::msg::ReferenceStamped temp_ref;
  //
  //     temp_ref.header.frame_id      = "latlon_origin";
  //     temp_ref.reference.position.x = 0;
  //     temp_ref.reference.position.y = 0;
  //
  //     if (auto ret = transformer_->transform(temp_ref, world_tf.value())) {
  //       temp_ref = ret.value();
  //       RCLCPP_INFO(node_->get_logger(), "Setting new origin x: %.3f y: %.3f", temp_ref.reference.position.x, temp_ref.reference.position.y);
  //       safety_zone_handler_.parameters.origin_x = temp_ref.reference.position.x;
  //       safety_zone_handler_.parameters.origin_x = temp_ref.reference.position.y;
  //     }
  //     tf_fcu_to_world_origin_ = fcu_tf.value();
  //   }
  // }

  std::scoped_lock lock(mutex_safety_area_);
  {
    const auto safety_border = safety_zone_handler_.safety_zone->getBorder();
    const auto border_points = safety_border->getPoints();

    auto transformed_border_points = transformPoints(border_points, "world_origin", safety_zone_handler_.parameters.horizontal_frame);

    mrs_msgs::msg::SafetyAreaManagerDiagnostics diagnostics_data;

    mrs_msgs::msg::Point2D tmp_point;

    // Get safety border points
    for (const auto &point : transformed_border_points) {
      tmp_point.x = boost::geometry::get<0>(point);
      tmp_point.y = boost::geometry::get<1>(point);
      diagnostics_data.border.points.push_back(tmp_point);
    }

    // Get safety_border max and min z
    auto safety_border_max_z              = safety_border->getMaxZ();
    diagnostics_data.border.max_z         = transformZ("world_origin", safety_zone_handler_.parameters.vertical_frame, safety_border_max_z);
    auto safety_border_min_z              = safety_border->getMinZ();
    diagnostics_data.border.min_z         = transformZ("world_origin", safety_zone_handler_.parameters.vertical_frame, safety_border_min_z);

    // getObstacles return a vector with the obstacle ptr's
    const auto &obstacles_ptrs = safety_zone_handler_.safety_zone->getObstacles();

    diagnostics_data.obstacles_present = obstacles_ptrs.size() == 0 ? false : true;

    // Iterate over vector of ObstaclePtr
    for (const auto &[key, obstaclePtr] : obstacles_ptrs) {
      const auto &obstacle = obstaclePtr->getPoints();
      mrs_msgs::msg::Prism tmp_obstacle;

      // TODO properly fill the horizontal and vertical frame of the obstacle
      auto transformed_obstacle = transformPoints(obstacle, "world_origin", safety_zone_handler_.parameters.horizontal_frame);
      tmp_obstacle.max_z   = transformZ("world_origin", safety_zone_handler_.parameters.vertical_frame, obstaclePtr->getMaxZ());
      tmp_obstacle.min_z   = transformZ("world_origin", safety_zone_handler_.parameters.vertical_frame, obstaclePtr->getMinZ());

      // Extract the points of the osbstacle
      for (const auto &point : transformed_obstacle) {
        tmp_point.x = boost::geometry::get<0>(point);
        tmp_point.y = boost::geometry::get<1>(point);
        tmp_obstacle.points.push_back(tmp_point);
      }
      diagnostics_data.obstacles.push_back(tmp_obstacle);
    }
  }
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
                         current_position.reference.position.x,
                         current_position.reference.position.y,
                         current_position.reference.position.z);
  }

  if (!is_position_valid_2d) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Outside of safety area (2D validation) current "
                         "position x:  %.3f y: %.3f",
                         current_position.reference.position.x,
                         current_position.reference.position.y);
  }

  return std::make_tuple(is_position_valid_2d, is_position_valid_3d);
}

//}

} // namespace safety_area_manager

} // namespace mrs_uav_managers

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_managers::safety_area_manager::SafetyAreaManager)
