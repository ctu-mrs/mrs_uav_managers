#include "mrs_uav_managers/safety_area_manager/safety_area_manager.h"

// TODO: 
// Provide sevices and implement them
//
// Do i need use_safety_area? It seems to be ControlManagers's responsibility
// Like, I can ask for point verification even if its using is disabled.

namespace mrs_uav_managers
{

namespace safety_area_manager 
{

void SafetyAreaManager::onInit(){
  ROS_INFO("SafetyAreaManager: onInit called");
  publisher_ = nh_.advertise<std_msgs::String>("chatter", 1000);
  ROS_INFO("SafetyAreaManager: topic inited");
  std_msgs::String msg;
  msg.data = "hello world";
  publisher_.publish(msg);
  ROS_INFO("SafetyAreaManager: msg published");
  preinitialize();
}

void SafetyAreaManager::preinitialize(){
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "SafetyAreaManager";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_hw_api_capabilities_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiCapabilities>(shopts, "hw_api_capabilities_in");

  timer_hw_api_capabilities_ = nh_.createTimer(ros::Rate(1.0), &SafetyAreaManager::timerHwApiCapabilities, this);
}

void SafetyAreaManager::timerHwApiCapabilities(const ros::TimerEvent& event) {
mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerHwApiCapabilities", status_timer_rate_, 1.0, event);

  if (!sh_hw_api_capabilities_.hasMsg()) {
    ROS_INFO_THROTTLE(1.0, "[SafetyAreaManager]: waiting for HW API capabilities");
    ROS_INFO("[SafetyAreaManager]: waiting for HW API capabilities");
    return;
  }

  // Note: all the hw_ap_capabilities seemed to be useless

  initialize();
  timer_hw_api_capabilities_.stop();
}

void SafetyAreaManager::initialize() {

  ROS_INFO("[SafetyAreaManager]: initializing");
  
  // --------------------------------------------------------------
  // |                           params                           |
  // --------------------------------------------------------------

  std::string world_config;
  mrs_lib::ParamLoader param_loader(nh_, "SafetyAreaManager");
  param_loader.loadParam("world_config", world_config);

  ROS_INFO("[SafetyAreaManager] Debug: world_config = %s", world_config.c_str());
  if (world_config != "") {
    param_loader.addYamlFile(world_config);
  }

  param_loader.addYamlFileFromParam("private_config");
  param_loader.addYamlFileFromParam("public_config");

  const std::string yaml_prefix = "mrs_uav_managers/safety_area_manager/";

  param_loader.loadParam("uav_name", uav_name_);
  param_loader.loadParam("body_frame", body_frame_);
  param_loader.loadParam("enable_profiler", profiler_enabled_);

  param_loader.loadParam(yaml_prefix + "status_timer_rate", status_timer_rate_); // todo: add parameter

  param_loader.loadParam("safety_area/use_safety_area", use_safety_area_);
  param_loader.loadParam("safety_area/min_z", safety_area_min_z_);
  param_loader.loadParam("safety_area/frame_name", safety_area_frame_);
  param_loader.loadParam("safety_area/max_z", safety_area_max_z_);

  if (use_safety_area_) {
    Eigen::MatrixXd border_points = param_loader.loadMatrixDynamic2("safety_area/safety_area", -1, 2);

    param_loader.loadParam("safety_area/polygon_obstacles/enabled", obstacle_polygons_enabled_);
    std::vector<Eigen::MatrixXd> polygon_obstacle_points;

    if (obstacle_polygons_enabled_) {
      polygon_obstacle_points = param_loader.loadMatrixArray2("safety_area/polygon_obstacles", std::vector<Eigen::MatrixXd>{});
    } else {
      polygon_obstacle_points = std::vector<Eigen::MatrixXd>();
    }

    param_loader.loadParam("safety_area/point_obstacles/enabled", obstacle_points_enabled_);
    std::vector<Eigen::MatrixXd> point_obstacle_points;

    if (obstacle_points_enabled_) {

      point_obstacle_points = param_loader.loadMatrixArray2("safety_area/point_obstacles", std::vector<Eigen::MatrixXd>{});

      if (safety_area_frame_ == "latlon_origin") {

        for (int i = 0; i < int(point_obstacle_points.size()); i++) {

          Eigen::MatrixXd temp = point_obstacle_points[i];
          temp(0, 2) *= 8.9832e-06;
          point_obstacle_points[i] = temp;
        }
      }

    } else {
      point_obstacle_points = std::vector<Eigen::MatrixXd>();
    }

    // TODO: remove this when param loader supports proper loading
    for (auto& matrix : polygon_obstacle_points) {
      matrix.transposeInPlace();
    }

    try {
      safety_zone_ = std::make_unique<mrs_lib::SafetyZone>(border_points, polygon_obstacle_points, point_obstacle_points);
    }

    catch (mrs_lib::SafetyZone::BorderError& e) {
      ROS_ERROR("[SafetyAreaManager]: SafetyArea: wrong configruation for the safety zone border polygon");
      ros::shutdown();
    }
    catch (mrs_lib::SafetyZone::PolygonObstacleError& e) {
      ROS_ERROR("[SafetyAreaManager]: SafetyArea: wrong configuration for one of the safety zone polygon obstacles");
      ros::shutdown();
    }
    catch (mrs_lib::SafetyZone::PointObstacleError& e) {
      ROS_ERROR("[SafetyAreaManager]: SafetyArea: wrong configuration for one of the safety zone point obstacles");
      ros::shutdown();
    }
    catch (...) {
      ROS_ERROR("[SafetyAreaManager]: SafetyArea: unhandler exception!");
      ros::shutdown();
    }

    transformer_ = std::make_shared<mrs_lib::Transformer>(nh_, "SafetyAreaManager");
    transformer_->setDefaultPrefix(uav_name_);
    transformer_->retryLookupNewest(true);

  }

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "SafetyAreaManager", profiler_enabled_);

  // ROS_INFO("[SafetyAreaManager]: uav_name:          %s", uav_name_.c_str());
  // ROS_INFO("[SafetyAreaManager]: safety_area_frame: %s", safety_area_frame_.c_str());
  // ROS_INFO("[SafetyAreaManager]: body_frame:        %s", body_frame_.c_str());
  // ROS_INFO("[SafetyAreaManager]: safety_area_min_z: %f", safety_area_min_z_);
  // ROS_INFO("[SafetyAreaManager]: safety_area_max_z: %f", safety_area_max_z_);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[SafetyAreaManager]: could not load all parameters!");
    ros::shutdown();
  }

  is_initialized_ = true;

  ROS_INFO("[SafetyAreaManager]: initialized");
}

bool SafetyAreaManager::isPointInSafetyArea3d(const mrs_msgs::ReferenceStamped point) {
  // copy member variables
  auto min_z = mrs_lib::get_mutexed(mutex_safety_area_min_z_, safety_area_min_z_);

  auto ret = transformer_->transformSingle(point, safety_area_frame_);

  if (!ret) {
      ROS_ERROR_THROTTLE(1.0, "[SafetyAreaManager]: SafetyArea: Could not transform the point to the safety area frame");
      return false;
  }

  mrs_msgs::ReferenceStamped point_transformed = ret.value();

  if (safety_zone_->isPointValid3d(point_transformed.reference.position.x, point_transformed.reference.position.y, point_transformed.reference.position.z) &&
      point_transformed.reference.position.z >= min_z && point_transformed.reference.position.z <= getMaxZ(safety_area_frame_)) {
      return true;
  }

  return false;
}

bool SafetyAreaManager::isPointInSafetyArea2d(const mrs_msgs::ReferenceStamped point) {

  auto ret = transformer_->transformSingle(point, safety_area_frame_);

  if (!ret) {

    ROS_ERROR_THROTTLE(1.0, "[SafetyAreaManager]: SafetyArea: Could not transform reference to the safety area frame");

    return false;
  }

  mrs_msgs::ReferenceStamped point_transformed = ret.value();

  return safety_zone_->isPointValid2d(point_transformed.reference.position.x, point_transformed.reference.position.y);
}

bool SafetyAreaManager::isPathToPointInSafetyArea3d(const mrs_msgs::ReferenceStamped start, const mrs_msgs::ReferenceStamped end) {
  mrs_msgs::ReferenceStamped start_transformed, end_transformed;

  {
    auto ret = transformer_->transformSingle(start, safety_area_frame_);

    if (!ret) {

      ROS_ERROR("[SafetyAreaManager]: SafetyArea: Could not transform the first point in the path");

      return false;
    }

    start_transformed = ret.value();
  }

  {
    auto ret = transformer_->transformSingle(end, safety_area_frame_);

    if (!ret) {

      ROS_ERROR("[SafetyAreaManager]: SafetyArea: Could not transform the first point in the path");

      return false;
    }

    end_transformed = ret.value();
  }

  return safety_zone_->isPathValid3d(start_transformed.reference.position.x, start_transformed.reference.position.y, start_transformed.reference.position.z,
                                    end_transformed.reference.position.x, end_transformed.reference.position.y, end_transformed.reference.position.z);
}


bool SafetyAreaManager::isPathToPointInSafetyArea2d(const mrs_msgs::ReferenceStamped start, const mrs_msgs::ReferenceStamped end) {
  mrs_msgs::ReferenceStamped start_transformed, end_transformed;

  {
    auto ret = transformer_->transformSingle(start, safety_area_frame_);

    if (!ret) {

      ROS_ERROR("[SafetyAreaManager]: SafetyArea: Could not transform the first point in the path");

      return false;
    }

    start_transformed = ret.value();
  }

  {
    auto ret = transformer_->transformSingle(end, safety_area_frame_);

    if (!ret) {

      ROS_ERROR("[SafetyAreaManager]: SafetyArea: Could not transform the first point in the path");

      return false;
    }

    end_transformed = ret.value();
  }

  return safety_zone_->isPathValid2d(start_transformed.reference.position.x, start_transformed.reference.position.y, end_transformed.reference.position.x,
                                    end_transformed.reference.position.y);
}

} // namespace safety_area_manager

} // namespace mrs_uav_managers


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::safety_area_manager::SafetyAreaManager, nodelet::Nodelet)
