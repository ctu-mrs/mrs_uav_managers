#include "mrs_uav_managers/safety_area_manager/safety_area_manager.h"

#include <mrs_lib/safety_zone/yaml_export_visitor.h>

#include <XmlRpcException.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

#include <iostream>
#include <fstream>
#include <limits>

#include <boost/geometry.hpp>

namespace bg = boost::geometry;

namespace mrs_uav_managers
{

namespace safety_area_manager 
{

void SafetyAreaManager::onInit(){
  preinitialize();
}

SafetyAreaManager::~SafetyAreaManager(){
  delete safety_zone_;
  for(size_t i=0; i<static_edges_.size(); i++){
    delete static_edges_[i];
  }
  for(size_t i=0; i<int_edges_.size(); i++){
    delete int_edges_[i];
  }
  for(size_t i=0; i<vertices_.size(); i++){
    delete vertices_[i];
  }
  for(size_t i=0; i<centers_.size(); i++){
    delete centers_[i];
  }
  for(size_t i=0; i<bounds_.size(); i++){
    delete bounds_[i];
  }
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

void SafetyAreaManager::timerHwApiCapabilities( [[maybe_unused]] const ros::TimerEvent& event) {
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

  // | --------------------- parameters ---------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "SafetyAreaManager");
  
  std::string world_config;
  param_loader.loadParam("world_config", world_config);

  param_loader.addYamlFileFromParam("private_config");
  param_loader.addYamlFileFromParam("public_config");

  param_loader.loadParam("uav_name", uav_name_);
  param_loader.loadParam("enable_profiler", profiler_enabled_);

  const std::string yaml_prefix = "mrs_uav_managers/safety_area_manager/";
  param_loader.loadParam(yaml_prefix + "status_timer_rate", status_timer_rate_);

  // | ---------------------- transformer ----------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(nh_, "SafetyAreaManager");
  transformer_->setDefaultPrefix(uav_name_);

  // | ---------------------- safety zone ----------------------- |
  // Note: safety_zone is initialized even if the use_safety_area_ is false
  // The manager will just always return true untill it's turned on
  bool safety_zone_inited = false;
  try {
    safety_zone_inited = initializeSafetyZone(param_loader, world_config);
  }
  catch (std::invalid_argument& e){
    ROS_ERROR("[SafetyAreaManager]: wrong configruation for the safety zone polygons. %s", e.what());
    ros::shutdown();
  }
  catch (XmlRpc::XmlRpcException& e){
    ROS_ERROR("[SafetyAreaManager]: error during parsing parameters. Please make sure parameters are written correctly");
    ros::shutdown();
  }
  catch(...){
    ROS_ERROR("[SafetyAreaManager]: unhandled exception!");
    ros::shutdown();
  }

  ROS_INFO("[SafetyAreaManager]: safety area initialized");
  
  // | ------------------------ services ------------------------ |

  service_server_get_safety_zone_at_height_ = nh_.advertiseService("get_safety_zone_at_height_in", &SafetyAreaManager::getSafeZoneAtHeight, this);
  service_server_point_in_safety_area_3d_   = nh_.advertiseService("point_in_safety_area_3d_in",   &SafetyAreaManager::isPointInSafetyArea3d, this);
  service_server_point_in_safety_area_2d_   = nh_.advertiseService("point_in_safety_area_2d_in",   &SafetyAreaManager::isPointInSafetyArea2d, this);
  service_server_path_in_safety_area_3d_    = nh_.advertiseService("path_in_safety_area_3d_in",    &SafetyAreaManager::isPathToPointInSafetyArea3d, this);
  service_server_path_in_safety_area_2d_    = nh_.advertiseService("path_in_safety_area_2d_in",    &SafetyAreaManager::isPathToPointInSafetyArea2d, this);
  service_server_save_world_config_         = nh_.advertiseService("save_world_config_in",         &SafetyAreaManager::saveWorldConfig, this);
  service_server_load_world_config_         = nh_.advertiseService("load_world_config_in",         &SafetyAreaManager::loadWorldConfig, this);
  service_server_use_safety_area_           = nh_.advertiseService("set_use_safety_area_in",       &SafetyAreaManager::setUseSafetyArea, this);
  service_server_add_obstacle_              = nh_.advertiseService("add_obstacle_in",              &SafetyAreaManager::addObstacle, this);
  service_server_get_max_z_                 = nh_.advertiseService("get_max_z_in",                 &SafetyAreaManager::getMaxZ, this);
  service_server_get_min_z_                 = nh_.advertiseService("get_min_z_in",                 &SafetyAreaManager::getMinZ, this);
  service_server_get_use_                   = nh_.advertiseService("get_use_in",                   &SafetyAreaManager::getUse, this);

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "SafetyAreaManager", profiler_enabled_);
  
  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loadedSuccessfully() || !safety_zone_inited) {
    ROS_ERROR("[SafetyAreaManager]: could not load all parameters!");
    ros::shutdown();
  }

  ROS_INFO("[SafetyAreaManager]: initialized");
}

mrs_lib::Prism* SafetyAreaManager::makePrism(Eigen::MatrixXd matrix, double max_z, double min_z) {
  std::vector<mrs_lib::Point2d> points = std::vector<mrs_lib::Point2d>();
  for(int i=0; i<matrix.cols(); i++){
    points.push_back(mrs_lib::Point2d{matrix(0, i), matrix(1, i)});
  }

  return new mrs_lib::Prism(points, max_z, min_z);
}

bool SafetyAreaManager::initializeSafetyZone(mrs_lib::ParamLoader& param_loader, std::string filename) {
  if(!param_loader.addYamlFile(filename)){
    return false;
  }

  param_loader.loadParam("world_origin/units", world_origin_units_);
  param_loader.loadParam("world_origin/origin_x", origin_x_);
  param_loader.loadParam("world_origin/origin_y", origin_y_);
  param_loader.loadParam("safety_area/enabled", use_safety_area_);
  param_loader.loadParam("safety_area/horizontal_frame", safety_area_horizontal_frame_);
  param_loader.loadParam("safety_area/vertical_frame", safety_area_vertical_frame_);

  // Make border prism
  Eigen::MatrixXd border_points = param_loader.loadMatrixDynamic2("safety_area/border/points", 2, -1);
  double max_z = param_loader.loadParam2("safety_area/border/max_z", max_z);
  double min_z = param_loader.loadParam2("safety_area/border/min_z", min_z);
  max_z = transformZ(safety_area_vertical_frame_, safety_area_horizontal_frame_, max_z);
  min_z = transformZ(safety_area_vertical_frame_, safety_area_horizontal_frame_, min_z);

  mrs_lib::Prism* tmp = makePrism(border_points, max_z, min_z);
  mrs_lib::Prism border = *tmp;
  delete tmp;

  // Making obstacle prisms
  std::vector<mrs_lib::Prism*> obstacles;

  bool is_obstacle_present = false;
  param_loader.loadParam("safety_area/obstacles/present", is_obstacle_present, is_obstacle_present);

  // If any is present, fill obstacles
  if(is_obstacle_present){
    // Read parameters for obstacles
    std::vector<Eigen::MatrixXd> obstacles_mat;
    param_loader.loadMatrixArray("safety_area/obstacles", obstacles_mat);

    Eigen::MatrixXd max_z_mat;
    Eigen::MatrixXd min_z_mat;
    param_loader.loadMatrixDynamic("safety_area/obstacles/max_z", max_z_mat, -1, 1);
    param_loader.loadMatrixDynamic("safety_area/obstacles/min_z", min_z_mat, -1, 1);

    if(!(max_z_mat.rows() == min_z_mat.rows() && min_z_mat.rows() == (long int)obstacles_mat.size())){
      ROS_WARN("[SafetyAreaManager]: The number of obstacles is not consistent! No obstacle has been added");
      return false;
    }

    // Make obstacle prisms
    for(size_t i=0; i<obstacles_mat.size(); i++){
      max_z = max_z_mat(i, 0);
      min_z = min_z_mat(i, 0);
      max_z = transformZ(safety_area_vertical_frame_, safety_area_horizontal_frame_, max_z);
      min_z = transformZ(safety_area_vertical_frame_, safety_area_horizontal_frame_, min_z);
      obstacles.push_back(makePrism(obstacles_mat[i], max_z, min_z));
    }
  }

  // Initialize safety_zone_
  safety_zone_ = new mrs_lib::SafetyZone(border, obstacles);
  
  // Add visualizations
  static_edges_.push_back(new mrs_lib::StaticEdgesVisualization(safety_zone_, uav_name_ + "/" + safety_area_horizontal_frame_, nh_, 2));
  int_edges_.push_back(new mrs_lib::IntEdgesVisualization(safety_zone_, uav_name_ + "/" + safety_area_horizontal_frame_, nh_));
  vertices_.push_back(new mrs_lib::VertexControl(safety_zone_, uav_name_ + "/" + safety_area_horizontal_frame_, nh_));
  centers_.push_back(new mrs_lib::CenterControl(safety_zone_, uav_name_ + "/" + safety_area_horizontal_frame_, nh_)); 
  bounds_.push_back(new mrs_lib::BoundsControl(safety_zone_, uav_name_ + "/" + safety_area_horizontal_frame_, nh_));

  for(auto it= safety_zone_->getObstaclesBegin(); it != safety_zone_->getObstaclesEnd(); it++){
    static_edges_.push_back(new mrs_lib::StaticEdgesVisualization(safety_zone_, it->first, uav_name_ + "/" + safety_area_horizontal_frame_, nh_, 2));
    int_edges_.push_back(new mrs_lib::IntEdgesVisualization(safety_zone_, it->first, uav_name_ + "/" + safety_area_horizontal_frame_, nh_));
    vertices_.push_back(new mrs_lib::VertexControl(safety_zone_, it->first, uav_name_ + "/" + safety_area_horizontal_frame_, nh_));
    centers_.push_back(new mrs_lib::CenterControl(safety_zone_, it->first, uav_name_ + "/" + safety_area_horizontal_frame_, nh_)); 
    bounds_.push_back(new mrs_lib::BoundsControl(safety_zone_, it->first, uav_name_ + "/" + safety_area_horizontal_frame_, nh_));
  }

  return param_loader.loadedSuccessfully();
}

double SafetyAreaManager::transformZ(std::string from, std::string to, double z) {
  geometry_msgs::Point point;
  point.x = 0;
  point.y = 0;
  point.z = z;
  
  auto res = transformer_->transformSingle(from, point, to);
  if(!res){
    ROS_ERROR("[SafetyAreaManager]: Could not transform point from %s to %s.", from.c_str(), to.c_str());
    return 0;
  }

  return res.value().z;
}

// --------------------------------------------------------------
// |                          services                          |
// --------------------------------------------------------------

bool SafetyAreaManager::addObstacle(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res) {
  mrs_msgs::ReferenceStamped point;
  point.header = req.header;
  point.reference = req.reference;
  auto tfed_horizontal = transformer_->transformSingle(point, safety_area_horizontal_frame_);

  if (!tfed_horizontal) {
    ROS_ERROR_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
    res.message = "Could not transform the point to the safety area horizontal frame";
    res.success = false;
    return true;
  }

  double offset_x = tfed_horizontal->reference.position.x;
  double offset_y = tfed_horizontal->reference.position.y;
  
  std::vector<mrs_lib::Point2d> points = {
    mrs_lib::Point2d{2.5 + offset_x, 2.5 + offset_y},
    mrs_lib::Point2d{2.5 + offset_x, -2.5 + offset_y},
    mrs_lib::Point2d{-2.5 + offset_x, -2.5 + offset_y},
    mrs_lib::Point2d{-2.5 + offset_x, 2.5 + offset_y},
    };

  int id = safety_zone_->addObstacle(new mrs_lib::Prism(points, 5, 0));

  static_edges_.push_back(new mrs_lib::StaticEdgesVisualization(safety_zone_, id, uav_name_ + "/" +safety_area_horizontal_frame_, nh_, 2));
  int_edges_.push_back(new mrs_lib::IntEdgesVisualization(safety_zone_, id, uav_name_ + "/" +safety_area_horizontal_frame_, nh_));
  vertices_.push_back(new mrs_lib::VertexControl(safety_zone_, id, uav_name_ + "/" +safety_area_horizontal_frame_, nh_));
  centers_.push_back(new mrs_lib::CenterControl(safety_zone_, id, uav_name_ + "/" +safety_area_horizontal_frame_, nh_)); 
  bounds_.push_back(new mrs_lib::BoundsControl(safety_zone_, id, uav_name_ + "/" +safety_area_horizontal_frame_, nh_));
  
  return true;
}

bool SafetyAreaManager::setUseSafetyArea(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  use_safety_area_ = req.data;
  res.success = true;
  ROS_INFO("[SafetyAreaManager]: safety area usage has been turned %s", (use_safety_area_ ? "on" : "off"));
  return true;
}

bool SafetyAreaManager::loadWorldConfig(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res) {
  // Backup
  mrs_lib::SafetyZone*                            old_safety_zone = safety_zone_;
  std::vector<mrs_lib::StaticEdgesVisualization*> old_static_edges = static_edges_;
  std::vector<mrs_lib::IntEdgesVisualization*>    old_int_edges = int_edges_;
  std::vector<mrs_lib::VertexControl*>            old_vertices = vertices_;
  std::vector<mrs_lib::CenterControl*>            old_centers = centers_;
  std::vector<mrs_lib::BoundsControl*>            old_bounds = bounds_;
  std::string                                     old_world_origin_units = world_origin_units_;
  std::string                                     old_safety_area_horizontal_frame = safety_area_horizontal_frame_;
  std::string                                     old_safety_area_vertical_frame = safety_area_vertical_frame_;
  double                                          old_origin_y = origin_y_;
  double                                          old_origin_x = origin_x_;
  bool                                            old_use_safety_area = use_safety_area_;
  safety_zone_ = nullptr;
  static_edges_.clear();
  int_edges_.clear();
  vertices_.clear();
  centers_.clear();
  bounds_.clear();

  mrs_lib::ParamLoader param_loader(nh_, "SafetyAreaManager");
  bool success = initializeSafetyZone(param_loader, req.value);

  res.success = true;
  if(!param_loader.loadedSuccessfully()){
    ROS_WARN("[SafetyAreaManager]: Could not read the file. Probably data format is not correct.");
    res.message = "Could not read the file. Probably data format is not correct.";
    res.success = false;
  } 
  if(!success){
    ROS_WARN("[SafetyAreaManager]: Could not load world config. Please, check the config file.");
    res.message = "Could not load world config. Please, check the config file.";
    res.success = false;
  }

  // In case of success, clean the backup
  if(res.success){
    delete old_safety_zone;
    for(size_t i=0; i<old_static_edges.size(); i++){
      delete old_static_edges[i];
    }
    for(size_t i=0; i<old_int_edges.size(); i++){
      delete old_int_edges[i];
    }
    for(size_t i=0; i<old_vertices.size(); i++){
      delete old_vertices[i];
    }
    for(size_t i=0; i<old_centers.size(); i++){
      delete old_centers[i];
    }
    for(size_t i=0; i<old_bounds.size(); i++){
      delete old_bounds[i];
    }
  }
  // If smth went wrong, restore from backup
  else{
    if(safety_zone_){
      delete safety_zone_;
    }
    for(size_t i=0; i<static_edges_.size(); i++){
      delete static_edges_[i];
    }
    for(size_t i=0; i<int_edges_.size(); i++){
      delete int_edges_[i];
    }
    for(size_t i=0; i<vertices_.size(); i++){
      delete vertices_[i];
    }
    for(size_t i=0; i<centers_.size(); i++){
      delete centers_[i];
    }
    for(size_t i=0; i<bounds_.size(); i++){
      delete bounds_[i];
    }
    safety_zone_ = old_safety_zone;
    static_edges_ = old_static_edges;
    int_edges_ = old_int_edges;
    vertices_ = old_vertices;
    centers_ = old_centers;
    bounds_ = old_bounds;
    world_origin_units_ = old_world_origin_units;
    safety_area_horizontal_frame_ = old_safety_area_horizontal_frame;
    safety_area_vertical_frame_ = old_safety_area_vertical_frame;
    origin_y_ = old_origin_y;
    origin_x_ = old_origin_x;
    use_safety_area_ = old_use_safety_area;
  }
  return true;
}

bool SafetyAreaManager::saveWorldConfig(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res) {
  mrs_lib::YamlExportVisitor visitor(uav_name_, safety_area_horizontal_frame_, 
                            safety_area_horizontal_frame_, safety_area_vertical_frame_, 
                            world_origin_units_, origin_x_, origin_y_);

  safety_zone_->accept(visitor);

  if(!visitor.isSuccessful()){
    res.message = "Something went wrong during exporting parameters";
    res.success = false;
    return true;
  }

  std::ofstream ofs(req.value, std::ofstream::out | std::ofstream::trunc);
  ofs << visitor.getResult();
  ofs.close();
  res.success = true;
  ROS_INFO("[SafetyAreaManager]: world config has been saved to %s", req.value.c_str());
  return true;
}

bool SafetyAreaManager::isPointInSafetyArea3d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res) {
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
    ROS_ERROR_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
    res.message = "Could not transform the point to the safety area horizontal frame";
    res.success = false;
    return true;
  }

  if (!safety_zone_->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y, tfed_horizontal->reference.position.z)) {
    res.success = false;
    return true;
  }

  res.success = true;
  return true;
}

bool SafetyAreaManager::isPointInSafetyArea2d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res) {
  if (!use_safety_area_) {
    res.success = true;
    res.message = "Safety area is disabled";
    return true;
  }
  
  mrs_msgs::ReferenceStamped point;
  point.reference = req.reference;
  point.header    = req.header;

  auto tfed_horizontal = transformer_->transformSingle(point, safety_area_horizontal_frame_);

  if (!tfed_horizontal) {
    ROS_ERROR_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
    res.message = "Could not transform the point to the safety area horizontal frame";
    res.success = false;
    return true;
  }

  if (!safety_zone_->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y)) {
    res.success = false;
    
    return true;
  }
  res.success = true;
  return true;
}

bool SafetyAreaManager::isPathToPointInSafetyArea3d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res) {
  if (!use_safety_area_) {
    res.success = true;
    res.message = "Safety area is disabled";
    return true;
  }
  
  geometry_msgs::PointStamped start = req.start;
  geometry_msgs::PointStamped end   = req.end;

  // transform points
  geometry_msgs::PointStamped start_transformed, end_transformed;

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

  // verify the whole path
  mrs_lib::Point3d start_point, end_point;
  start_point.set<0>(start_transformed.point.x);
  start_point.set<1>(start_transformed.point.y);
  start_point.set<2>(start_transformed.point.z);
  end_point.set<0>(end_transformed.point.x);
  end_point.set<1>(end_transformed.point.y);
  end_point.set<2>(end_transformed.point.z);

  if(!safety_zone_->isPathValid(start_point, end_point)){
    res.message = "The path is not valid";
    res.success = false;
    return true;
  }

  res.success = true;
  return true;
}

bool SafetyAreaManager::isPathToPointInSafetyArea2d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res) {
  if (!use_safety_area_) {
    res.success = true;
    res.message = "Safety area is disabled";
    return true;
  }
  
  geometry_msgs::PointStamped start = req.start;
  geometry_msgs::PointStamped end   = req.end;

  // transform points
  geometry_msgs::PointStamped start_transformed, end_transformed;

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

  // verify the whole path
  mrs_lib::Point2d start_point, end_point;
  start_point.set<0>(start_transformed.point.x);
  start_point.set<1>(start_transformed.point.y);
  end_point.set<0>(end_transformed.point.x);
  end_point.set<1>(end_transformed.point.y);

  if(!safety_zone_->isPathValid(start_point, end_point)){
    res.message = "The path is not valid";
    res.success = false;
    return true;
  }

  res.success = true;
  return true;
}

bool SafetyAreaManager::getSafeZoneAtHeight(mrs_msgs::GetSafeZoneAtHeight::Request& req, mrs_msgs::GetSafeZoneAtHeight::Response& res){
  // Transform height to the current frame
  double height = transformZ(req.header.frame_id, safety_area_horizontal_frame_, req.height);
  
  // If main prism is on different height, safety zone is empty
  if(safety_zone_->getBorder()->getMaxZ() < height || height < safety_zone_->getBorder()->getMinZ()){
    return true;
  }

  // Set response header
  res.header.frame_id = uav_name_ + "/" + safety_area_horizontal_frame_;
  
  // Add polygon of main prism
  auto border = safety_zone_->getBorder()->getPolygon().outer();
  geometry_msgs::Polygon border_polygon;
  for(size_t i=0; i<border.size(); i++){
    geometry_msgs::Point32 p;
    p.x = bg::get<0>(border[i]);
    p.y = bg::get<1>(border[i]);
    border_polygon.points.push_back(p);
  }
  res.safety_zone.push_back(border_polygon);

  // Add polygons of required obstacles
  for(auto it= safety_zone_->getObstaclesBegin(); it != safety_zone_->getObstaclesEnd(); it++){
    if(it->second->getMaxZ() < height || height < it->second->getMinZ()){
      continue;
    } 
    
    geometry_msgs::Polygon polygon;
    auto outer_ring = it->second->getPolygon().outer();
    for(size_t i=0; i<outer_ring.size(); i++){
      geometry_msgs::Point32 p;
      p.x = bg::get<0>(outer_ring[i]);
      p.y = bg::get<1>(outer_ring[i]);
      polygon.points.push_back(p);
    }
    res.safety_zone.push_back(polygon);
  }

  return true;
}

bool SafetyAreaManager::getMaxZ( [[maybe_unused]] mrs_msgs::GetPointStamped::Request& req, mrs_msgs::GetPointStamped::Response& res){
  res.result.header.frame_id = safety_area_horizontal_frame_;
  res.result.point.x = 0;
  res.result.point.y = 0;
  if(use_safety_area_){
    res.result.point.z = safety_zone_->getBorder()->getMaxZ();
  }else{
    res.result.point.z = std::numeric_limits<double>::max();
  }

  res.success = true;
  return true;
}

bool SafetyAreaManager::getMinZ( [[maybe_unused]] mrs_msgs::GetPointStamped::Request& req, mrs_msgs::GetPointStamped::Response& res){
  res.result.header.frame_id = safety_area_horizontal_frame_;
  res.result.point.x = 0;
  res.result.point.y = 0;
  if(use_safety_area_){
    res.result.point.z = safety_zone_->getBorder()->getMinZ();
  }else{
    res.result.point.z = std::numeric_limits<double>::lowest();
  }

  res.success = true;
  return true;
}

bool SafetyAreaManager::getUse( [[maybe_unused]] mrs_msgs::GetBool::Request& req, mrs_msgs::GetBool::Response& res){
  res.result = use_safety_area_;
  return true;
}

} // namespace safety_area_manager

} // namespace mrs_uav_managers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::safety_area_manager::SafetyAreaManager, nodelet::Nodelet)
