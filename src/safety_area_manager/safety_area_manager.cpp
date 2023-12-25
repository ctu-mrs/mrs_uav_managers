#include "mrs_uav_managers/safety_area_manager/safety_area_manager.h"

#include <mrs_lib/safety_zone/yaml_export_visitor.h>

#include <XmlRpcException.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <fstream>

namespace mrs_uav_managers
{

namespace safety_area_manager 
{

void SafetyAreaManager::onInit(){
  preinitialize();
}

SafetyAreaManager::~SafetyAreaManager(){
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

void SafetyAreaManager::timerHwApiCapabilities(const ros::TimerEvent& event) {
mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerHwApiCapabilities", status_timer_rate_, 1.0, event);

  // TODO: uncomment this
  // if (!sh_hw_api_capabilities_.hasMsg()) {
  //   ROS_INFO_THROTTLE(1.0, "[SafetyAreaManager]: waiting for HW API capabilities");
  //   ROS_INFO("[SafetyAreaManager]: waiting for HW API capabilities");
  //   return;
  // }

  // Note: all the hw_ap_capabilities seemed to be useless

  initialize();
  timer_hw_api_capabilities_.stop();
}

void SafetyAreaManager::initialize() {

  ROS_INFO("[SafetyAreaManager]: initializing");

  // --------------------------------------------------------------
  // |                          services                          |
  // --------------------------------------------------------------

  // service_server_point_in_safety_area_3d_ = nh_.advertiseService()
  // service_server_point_in_safety_area_2d_ = nh_.advertiseService()
  // service_server_path_in_safety_area_3d_ = nh_.advertiseService()
  // service_server_path_in_safety_area_2d_ = nh_.advertiseService()
  service_server_save_world_config_ = nh_.advertiseService("save_world_config_in", &SafetyAreaManager::saveWorldConfig, this);
  service_server_use_safety_area_ = nh_.advertiseService("set_use_safety_area_in", &SafetyAreaManager::setUseSafetyArea, this);

  // --------------------------------------------------------------
  // |                           params                           |
  // --------------------------------------------------------------

  std::string world_config;
  mrs_lib::ParamLoader param_loader(nh_, "SafetyAreaManager");
  
  param_loader.loadParam("world_config", world_config);
  ROS_INFO("[SafetyAreaManager] Debug: world_config file = %s", world_config.c_str());
  if (world_config != "") {
    param_loader.addYamlFile(world_config);
  }

  param_loader.addYamlFileFromParam("private_config");
  param_loader.addYamlFileFromParam("public_config");

  param_loader.loadParam("uav_name", uav_name_);
  param_loader.loadParam("enable_profiler", profiler_enabled_);

  const std::string yaml_prefix = "mrs_uav_managers/safety_area_manager/";
  param_loader.loadParam(yaml_prefix + "status_timer_rate", status_timer_rate_);

  param_loader.loadParam("world_origin/units", world_origin_units_);
  param_loader.loadParam("world_origin/origin_x", origin_x_);
  param_loader.loadParam("world_origin/origin_y", origin_y_);

  param_loader.loadParam("safety_area/enabled", use_safety_area_);

  if (use_safety_area_) {
    try {
      initializeSafetyZone(param_loader);
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

mrs_lib::Prism SafetyAreaManager::makePrism(Eigen::MatrixXd matrix, double max_z, double min_z) {
  std::vector<mrs_lib::Point2d> points = std::vector<mrs_lib::Point2d>();
  for(int i=0; i<matrix.cols(); i++){
    points.push_back(mrs_lib::Point2d{matrix(0, i), matrix(1, i)});
  }

  mrs_lib::Prism result = mrs_lib::Prism(points, max_z, min_z);
  return result;
}

void SafetyAreaManager::initializeSafetyZone(mrs_lib::ParamLoader& param_loader) {
  param_loader.loadParam("safety_area/horizontal_frame", safety_area_horizontal_frame_);
  param_loader.loadParam("safety_area/vertical_frame", safety_area_vertical_frame_);

  // Make border prism
  Eigen::MatrixXd border_points = param_loader.loadMatrixDynamic2("safety_area/border/points", 2, -1);
  double max_z = param_loader.loadParam2("safety_area/border/max_z", max_z);
  double min_z = param_loader.loadParam2("safety_area/border/min_z", min_z);
  max_z = transformZ(safety_area_vertical_frame_, safety_area_horizontal_frame_, max_z);
  min_z = transformZ(safety_area_vertical_frame_, safety_area_horizontal_frame_, min_z);

  mrs_lib::Prism border = makePrism(border_points, max_z, min_z);

  // Read parameters for obstacles
  std::vector<Eigen::MatrixXd> obstacles_mat;
  param_loader.loadMatrixArray("safety_area/obstacles", obstacles_mat);

  Eigen::MatrixXd max_z_mat;
  Eigen::MatrixXd min_z_mat;
  param_loader.loadMatrixDynamic("safety_area/obstacles/max_z", max_z_mat, -1, 1);
  param_loader.loadMatrixDynamic("safety_area/obstacles/min_z", min_z_mat, -1, 1);

  if(!(max_z_mat.rows() == min_z_mat.rows() && min_z_mat.rows() == obstacles_mat.size())){
    ROS_WARN("[SafetyAreaManager]: The number of obstacles is not consistent! No obstacle has been added");
    return;
  }

  // Make obstacle prisms
  std::vector<mrs_lib::Prism> obstacles;
  for(size_t i=0; i<obstacles_mat.size(); i++){
    max_z = max_z_mat(i, 0);
    min_z = min_z_mat(i, 0);
    max_z = transformZ(safety_area_vertical_frame_, safety_area_horizontal_frame_, max_z);
    min_z = transformZ(safety_area_vertical_frame_, safety_area_horizontal_frame_, min_z);
    obstacles.push_back(makePrism(obstacles_mat[i], max_z, min_z));
  }

  // Initialize safety_zone_
  safety_zone_ = new mrs_lib::SafetyZone(border, obstacles);
  
  // Add visualizations
  static_edges_.push_back(new mrs_lib::StaticEdgesVisualization(safety_zone_, safety_area_horizontal_frame_, nh_, 2));
  int_edges_.push_back(new mrs_lib::IntEdgesVisualization(safety_zone_, safety_area_horizontal_frame_, nh_));
  vertices_.push_back(new mrs_lib::VertexControl(safety_zone_, safety_area_horizontal_frame_, nh_));
  centers_.push_back(new mrs_lib::CenterControl(safety_zone_, safety_area_horizontal_frame_, nh_)); 
  bounds_.push_back(new mrs_lib::BoundsControl(safety_zone_, safety_area_horizontal_frame_, nh_));

  for(auto it= safety_zone_->getObstaclesBegin(); it != safety_zone_->getObstaclesEnd(); it++){
    static_edges_.push_back(new mrs_lib::StaticEdgesVisualization(safety_zone_, it->first, safety_area_horizontal_frame_, nh_, 2));
    int_edges_.push_back(new mrs_lib::IntEdgesVisualization(safety_zone_, it->first, safety_area_horizontal_frame_, nh_));
    vertices_.push_back(new mrs_lib::VertexControl(safety_zone_, it->first, safety_area_horizontal_frame_, nh_));
    centers_.push_back(new mrs_lib::CenterControl(safety_zone_, it->first, safety_area_horizontal_frame_, nh_)); 
    bounds_.push_back(new mrs_lib::BoundsControl(safety_zone_, it->first, safety_area_horizontal_frame_, nh_));
  }
}

double SafetyAreaManager::transformZ(std::string from, std::string to, double z) {
  // TODO: delete this if
  if(from == to){
    return z;
  }
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

bool SafetyAreaManager::setUseSafetyArea(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  use_safety_area_ = req.data;
  res.success = true;
  ROS_INFO("[SafetyAreaManager]: safety area usage has been turned %s", (use_safety_area_ ? "on" : "off"));
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
  // ROS_INFO("point 3d");
  // if (!use_safety_area_) {
  //   res.success = true;
  //   res.message = "Safety area is disabled";
  //   return true;
  // }

  // mrs_msgs::ReferenceStamped point;
  // point.header = req.header;
  // point.reference = req.reference;

  // auto tfed_horizontal = transformer_->transformSingle(point, safety_area_horizontal_frame_);

  // if (!tfed_horizontal) {
  //   res.message = "Could not transform the point to the safety area horizontal frame";
  //   ROS_ERROR_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
  //   res.success = false;
  //   return true;
  // }

  // if (!safety_zone_->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y)) {
  //   res.success = false;
  //   return true;
  // }

  // if (point.reference.position.z < getMinZ(point.header.frame_id) || point.reference.position.z > getMaxZ(point.header.frame_id)) {
  //   res.success = false;
  //   return true;
  // }

  // res.success = true;
  // return true;
}

bool SafetyAreaManager::isPointInSafetyArea2d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res) {
  // ROS_INFO("point 2d");
  // if (!use_safety_area_) {
  //   res.success = true;
  //   res.message = "Safety area is disabled";
  //   return true;
  // }
  
  // ROS_INFO("point 2d about to transform");
  // mrs_msgs::ReferenceStamped point;
  // point.reference = req.reference;
  // point.header    = req.header;
  // ROS_INFO("safety_area_hor_frame = %s", safety_area_horizontal_frame_.c_str());
  // ROS_INFO("point frame = %s", point.header.frame_id.c_str());
  // auto tfed_horizontal = transformer_->transformSingle(point, safety_area_horizontal_frame_);

  // ROS_INFO("point 2d transformed");
  // if (!tfed_horizontal) {
  //   ROS_INFO("point 2d transform was not successfull");
  //   ROS_ERROR_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
  //   res.message = "Could not transform the point to the safety area horizontal frame";
  //   res.success = false;
  //   return true;
  // }

  // ROS_INFO("point 2d transform was successfull");

  // if (!safety_zone_->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y)) {
  //   ROS_INFO("Point 2d is not valid");
  //   res.success = false;
  //   return true;
  // }
  // ROS_INFO("Point 2d is valid");

  // res.success = true;
  // return true;
}

// Note: it was decided not to use isPointInSafetyAreaNd() because
// 1) It requires making new service message instances 
// 2) If primary verification fails, we have no info of what exactly went wrong
// 3) The same transformations have to be done twice
bool SafetyAreaManager::isPathToPointInSafetyArea3d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res) {
  // ROS_INFO("path 3d");
  // if (!use_safety_area_) {
  //   res.success = true;
  //   res.message = "Safety area is disabled";
  //   return true;
  // }
  
  // mrs_msgs::ReferenceStamped start = req.start;
  // mrs_msgs::ReferenceStamped end   = req.end;

  // // transform points
  // mrs_msgs::ReferenceStamped start_transformed, end_transformed;

  // {
  //   auto ret = transformer_->transformSingle(start, safety_area_horizontal_frame_);

  //   if (!ret) {
  //     ROS_ERROR_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
  //     res.message = "Could not transform the first point in the path";
  //     res.success = false;
  //     return true;
  //   }

  //   start_transformed = ret.value();
  // }

  // {
  //   auto ret = transformer_->transformSingle(end, safety_area_horizontal_frame_);

  //   if (!ret) {
  //     ROS_ERROR_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
  //     res.message = "Could not transform the first point in the path";
  //     return true;
  //   }

  //   end_transformed = ret.value();
  // }

  // // verify if the points are in safety area
  // if(!safety_zone_->isPointValid(start_transformed.reference.position.x, start_transformed.reference.position.y)){
  //   res.message = "The first point is not in safety area";
  //   res.success = false;
  //   return true;
  // }

  // if(start.reference.position.z < getMinZ(start.header.frame_id) || start.reference.position.z > getMaxZ(start.header.frame_id)){
  //   res.message = "The first point is not in safety area";
  //   res.success = false;
  //   return true;
  // }

  // if(!safety_zone_->isPointValid(end_transformed.reference.position.x, end_transformed.reference.position.y)){
  //   res.message = "The second point is not in safety area";
  //   res.success = false;
  //   return true;
  // }

  // if(end.reference.position.z < getMinZ(end.header.frame_id) || end.reference.position.z > getMaxZ(end.header.frame_id)){
  //   res.message = "The second point is not in safety area";
  //   res.success = false;
  //   return true;
  // }

  // // verify the whole path
  // if(safety_zone_->isPathValid(start_transformed.reference.position.x, start_transformed.reference.position.y, end_transformed.reference.position.x,
  //                                  end_transformed.reference.position.y)){
  //   res.message = "The path is not valid";
  //   res.success = false;
  //   return true;
  // }

  // res.message = "";
  // res.success = true;
  // return true;
}


bool SafetyAreaManager::isPathToPointInSafetyArea2d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res) {
  // ROS_INFO("path 2d");
  // if (!use_safety_area_) {
  //   res.success = true;
  //   res.message = "Safety area is disabled";
  //   return true;
  // }
  
  // mrs_msgs::ReferenceStamped start = req.start;
  // mrs_msgs::ReferenceStamped end   = req.end;

  // // transform points
  // mrs_msgs::ReferenceStamped start_transformed, end_transformed;
  // {
  //   auto ret = transformer_->transformSingle(start, safety_area_frame_);

  //   if (!ret) {
  //     ROS_ERROR("[SafetyAreaManager]: Could not transform the first point in the path");
  //     res.message = "Could not transform the first point in the path";
  //     res.success = false;
  //     return true;
  //   }

  //   start_transformed = ret.value();
  // }

  // {
  //   auto ret = transformer_->transformSingle(end, safety_area_frame_);

  //   if (!ret) {
  //     ROS_ERROR("[SafetyAreaManager]: Could not transform the second point in the path");
  //     res.message = "Could not transform the second point in the path";
  //     res.success = false;
  //     return true;
  //   }

  //   end_transformed = ret.value();
  // }

  // // verify if the points are in safety area
  // if(!safety_zone_->isPointValid(start_transformed.reference.position.x, start_transformed.reference.position.y)){
  //   res.message = "The first point is not in safety area";
  //   res.success = false;
  //   return true;
  // }

  // if(!safety_zone_->isPointValid(end_transformed.reference.position.x, end_transformed.reference.position.y)){
  //   res.message = "The second point is not in safety area";
  //   res.success = false;
  //   return true;
  // }

  // // verify the whole path
  // res.message = "";
  // if(safety_zone_->isPathValid(start_transformed.reference.position.x, start_transformed.reference.position.y, end_transformed.reference.position.x,
  //                                  end_transformed.reference.position.y)){
  //   res.success = false;
  //   res.message = "The path is not valid";
  //   return true;
  // }

  // res.success = true;
  // return true;
}

// double SafetyAreaManager::getMaxZ(const std::string& frame_id){
//   return 1000;
// }

// double SafetyAreaManager::getMinZ(const std::string& frame_id){
//   return -1000;
// }

} // namespace safety_area_manager

} // namespace mrs_uav_managers


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::safety_area_manager::SafetyAreaManager, nodelet::Nodelet)
