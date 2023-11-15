#include "mrs_uav_managers/safety_area_manager/safety_area_manager.h"

// TODO: 
// Provide services and implement them
//
// Do i need to use_safety_area? It seems to be ControlManagers's responsibility
// Like, I can ask for point verification even if its using is disabled.
// On the other hand, why would a user need it?

// Add "set use_safety_area" service callback

// One more: to implement the bumper or not to implement? - I think general
// safetyness is ControlManagers's responsibility

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

  param_loader.loadParam("uav_name", uav_name_);
  param_loader.loadParam("body_frame", body_frame_);
  param_loader.loadParam("enable_profiler", profiler_enabled_);

  const std::string yaml_prefix = "mrs_uav_managers/safety_area_manager/";
  param_loader.loadParam(yaml_prefix + "status_timer_rate", status_timer_rate_);

  param_loader.loadParam("safety_area/horizontal/frame_name", safety_area_horizontal_frame_);

  param_loader.loadParam("safety_area/vertical/frame_name", safety_area_vertical_frame_);
  param_loader.loadParam("safety_area/vertical/min_z", safety_area_min_z_);
  param_loader.loadParam("safety_area/vertical/max_z", safety_area_max_z_);

  param_loader.loadParam("safety_area/enabled", use_safety_area_);
  // param_loader.loadParam("safety_area/frame_name", safety_area_frame_);

  if (use_safety_area_) {
    
    Eigen::MatrixXd border_points = param_loader.loadMatrixDynamic2("safety_area/horizontal/points", -1, 2);

    try {

      std::vector<Eigen::MatrixXd> polygon_obstacle_points;
      std::vector<Eigen::MatrixXd> point_obstacle_points;

      safety_zone_ = std::make_unique<mrs_lib::SafetyZone>(border_points);
    }

    catch (mrs_lib::SafetyZone::BorderError& e) {
      ROS_ERROR("[SafetyAreaManager]: wrong configruation for the safety zone border polygon");
      ros::shutdown();
    }
    catch (...) {
      ROS_ERROR("[SafetyAreaManager]: unhandled exception!");
      ros::shutdown();
    }

    ROS_INFO("[SafetyAreaManager]: safety area initialized");
  }

  // | ------------------------ services ------------------------ |

  service_server_point_in_safety_area_3d_ = nh_.advertiseService("point_in_safety_area_3d_in", &SafetyAreaManager::isPointInSafetyArea3d, this);
  service_server_point_in_safety_area_2d_ = nh_.advertiseService("point_in_safety_area_2d_in", &SafetyAreaManager::isPointInSafetyArea2d, this);
  service_server_path_in_safety_area_3d_  = nh_.advertiseService("path_in_safety_area_3d_in", &SafetyAreaManager::isPathToPointInSafetyArea3d, this);
  service_server_path_in_safety_area_2d_  = nh_.advertiseService("path_in_safety_area_2d_in", &SafetyAreaManager::isPathToPointInSafetyArea2d, this);

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "SafetyAreaManager", profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[SafetyAreaManager]: could not load all parameters!");
    ros::shutdown();
  }

  is_initialized_ = true;

  ROS_INFO("[SafetyAreaManager]: initialized");
}

bool SafetyAreaManager::isPointInSafetyArea3d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res) {
  ROS_INFO("point 3d");
  if (!use_safety_area_) {
    res.success = true;
    res.message = "Safety area is disabled";
    return true;
  }

  mrs_msgs::ReferenceStamped point;
  point.header = req.header;
  point.reference = req.reference;

  auto tfed_horizontal = transformer_->transformSingle(point, safety_area_horizontal_frame_);

  if (!tfed_horizontal) {
    res.message = "Could not transform the point to the safety area horizontal frame";
    ROS_ERROR_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
    res.success = false;
    return true;
  }

  if (!safety_zone_->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y)) {
    res.success = false;
    return true;
  }

  if (point.reference.position.z < getMinZ(point.header.frame_id) || point.reference.position.z > getMaxZ(point.header.frame_id)) {
    res.success = false;
    return true;
  }

  res.success = true;
  return true;
}

bool SafetyAreaManager::isPointInSafetyArea2d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res) {
  ROS_INFO("point 2d");
  if (!use_safety_area_) {
    res.success = true;
    res.message = "Safety area is disabled";
    return true;
  }
  
  ROS_INFO("point 2d about to transform");
  mrs_msgs::ReferenceStamped point;
  point.reference = req.reference;
  point.header    = req.header;
  ROS_INFO("safety_area_hor_frame = %s", safety_area_horizontal_frame_.c_str());
  ROS_INFO("point frame = %s", point.header.frame_id.c_str());
  auto tfed_horizontal = transformer_->transformSingle(point, safety_area_horizontal_frame_);

  ROS_INFO("point 2d transformed");
  if (!tfed_horizontal) {
    ROS_INFO("point 2d transform was not successfull");
    ROS_ERROR_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
    res.message = "Could not transform the point to the safety area horizontal frame";
    res.success = false;
    return true;
  }

  ROS_INFO("point 2d transform was successfull");

  if (!safety_zone_->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y)) {
    ROS_INFO("Point 2d is not valid");
    res.success = false;
    return true;
  }
  ROS_INFO("Point 2d is valid");

  res.success = true;
  return true;
}

// Note: it was decided not to use isPointInSafetyAreaNd() because
// 1) It requires making new service message instances 
// 2) If primary verification fails, we have no info of what exactly went wrong
// 3) The same transformations have to be done twice
bool SafetyAreaManager::isPathToPointInSafetyArea3d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res) {
  ROS_INFO("path 3d");
  if (!use_safety_area_) {
    res.success = true;
    res.message = "Safety area is disabled";
    return true;
  }
  
  mrs_msgs::ReferenceStamped start = req.start;
  mrs_msgs::ReferenceStamped end   = req.end;

  // transform points
  mrs_msgs::ReferenceStamped start_transformed, end_transformed;

  {
    auto ret = transformer_->transformSingle(start, safety_area_horizontal_frame_);

    if (!ret) {
      ROS_ERROR_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
      res.message = "Could not transform the first point in the path";
      res.success = false;
      return true;
    }

    start_transformed = ret.value();
  }

  {
    auto ret = transformer_->transformSingle(end, safety_area_horizontal_frame_);

    if (!ret) {
      ROS_ERROR_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
      res.message = "Could not transform the first point in the path";
      return true;
    }

    end_transformed = ret.value();
  }

  // verify if the points are in safety area
  if(!safety_zone_->isPointValid(start_transformed.reference.position.x, start_transformed.reference.position.y)){
    res.message = "The first point is not in safety area";
    res.success = false;
    return true;
  }

  if(start.reference.position.z < getMinZ(start.header.frame_id) || start.reference.position.z > getMaxZ(start.header.frame_id)){
    res.message = "The first point is not in safety area";
    res.success = false;
    return true;
  }

  if(!safety_zone_->isPointValid(end_transformed.reference.position.x, end_transformed.reference.position.y)){
    res.message = "The second point is not in safety area";
    res.success = false;
    return true;
  }

  if(end.reference.position.z < getMinZ(end.header.frame_id) || end.reference.position.z > getMaxZ(end.header.frame_id)){
    res.message = "The second point is not in safety area";
    res.success = false;
    return true;
  }

  // verify the whole path
  if(safety_zone_->isPathValid(start_transformed.reference.position.x, start_transformed.reference.position.y, end_transformed.reference.position.x,
                                   end_transformed.reference.position.y)){
    res.message = "The path is not valid";
    res.success = false;
    return true;
  }

  res.message = "";
  res.success = true;
  return true;
}


bool SafetyAreaManager::isPathToPointInSafetyArea2d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res) {
  ROS_INFO("path 2d");
  if (!use_safety_area_) {
    res.success = true;
    res.message = "Safety area is disabled";
    return true;
  }
  
  mrs_msgs::ReferenceStamped start = req.start;
  mrs_msgs::ReferenceStamped end   = req.end;

  // transform points
  mrs_msgs::ReferenceStamped start_transformed, end_transformed;
  {
    auto ret = transformer_->transformSingle(start, safety_area_frame_);

    if (!ret) {
      ROS_ERROR("[SafetyAreaManager]: Could not transform the first point in the path");
      res.message = "Could not transform the first point in the path";
      res.success = false;
      return true;
    }

    start_transformed = ret.value();
  }

  {
    auto ret = transformer_->transformSingle(end, safety_area_frame_);

    if (!ret) {
      ROS_ERROR("[SafetyAreaManager]: Could not transform the second point in the path");
      res.message = "Could not transform the second point in the path";
      res.success = false;
      return true;
    }

    end_transformed = ret.value();
  }

  // verify if the points are in safety area
  if(!safety_zone_->isPointValid(start_transformed.reference.position.x, start_transformed.reference.position.y)){
    res.message = "The first point is not in safety area";
    res.success = false;
    return true;
  }

  if(!safety_zone_->isPointValid(end_transformed.reference.position.x, end_transformed.reference.position.y)){
    res.message = "The second point is not in safety area";
    res.success = false;
    return true;
  }

  // verify the whole path
  res.message = "";
  if(safety_zone_->isPathValid(start_transformed.reference.position.x, start_transformed.reference.position.y, end_transformed.reference.position.x,
                                   end_transformed.reference.position.y)){
    res.success = false;
    res.message = "The path is not valid";
    return true;
  }

  res.success = true;
  return true;
}

double SafetyAreaManager::getMaxZ(const std::string& frame_id){
  return 1000;
}

double SafetyAreaManager::getMinZ(const std::string& frame_id){
  return -1000;
}

} // namespace safety_area_manager

} // namespace mrs_uav_managers


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::safety_area_manager::SafetyAreaManager, nodelet::Nodelet)
