#include <ros/ros.h>

#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/PathToPointInSafetyArea.h>
#include <mrs_msgs/HwApiCapabilities.h>
#include <mrs_msgs/String.h>

#include <std_srvs/SetBool.h>

#include <mrs_lib/mutex.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/safety_zone/safety_zone.h>
#include <mrs_lib/safety_zone/static_edges_visualization.h>
#include <mrs_lib/safety_zone/int_edges_visualization.h>
#include <mrs_lib/safety_zone/vertex_control.h>
#include <mrs_lib/safety_zone/center_control.h>
#include <mrs_lib/safety_zone/bounds_control.h>

#include <nodelet/nodelet.h>

// TODO: viz .cpp file

namespace mrs_uav_managers
{

namespace safety_area_manager 
{

class SafetyAreaManager : public nodelet::Nodelet
{
private:
  std::shared_ptr<mrs_lib::Transformer> transformer_;
  bool is_initialized_ = false;

  mrs_lib::SafetyZone* safety_zone_;
  std::string          uav_name_;
  std::string          safety_area_horizontal_frame_;
  std::string          safety_area_vertical_frame_;
  std::string          world_origin_units_;
  double               origin_y_;
  double               origin_x_;
  bool                 use_safety_area_;

  // Visualization objects
  std::vector<mrs_lib::StaticEdgesVisualization*> static_edges_;
  std::vector<mrs_lib::IntEdgesVisualization*>    int_edges_;
  std::vector<mrs_lib::VertexControl*>            vertices_;
  std::vector<mrs_lib::CenterControl*>            centers_;
  std::vector<mrs_lib::BoundsControl*>            bounds_;

  ros::NodeHandle nh_;

  // profiling
  mrs_lib::Profiler profiler_;
  bool              profiler_enabled_ = false;
  int status_timer_rate_   = 0;

  // safety area services
  ros::ServiceServer service_server_point_in_safety_area_3d_;
  ros::ServiceServer service_server_point_in_safety_area_2d_;
  ros::ServiceServer service_server_path_in_safety_area_3d_;
  ros::ServiceServer service_server_path_in_safety_area_2d_;
  ros::ServiceServer service_server_save_world_config_;
  ros::ServiceServer service_server_use_safety_area_;
  ros::ServiceServer service_server_add_obstacle_;
  
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiCapabilities> sh_hw_api_capabilities_;

  void preinitialize();

  void initialize();

  mrs_lib::Prism makePrism(Eigen::MatrixXd matrix, double max_z, double min_z);

  void initializeSafetyZone(mrs_lib::ParamLoader& param_loader);

  double transformZ(std::string from, std::string to, double z);

  // this timer will check till we already got the hardware api diagnostics
  // then it will trigger the initialization of the controllers and finish
  // the initialization of the ControlManager
  ros::Timer timer_hw_api_capabilities_;
  void timerHwApiCapabilities(const ros::TimerEvent& event);

  // Services

  bool isPointInSafetyArea3d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);

  bool isPointInSafetyArea2d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);

  bool isPathToPointInSafetyArea3d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res);

  bool isPathToPointInSafetyArea2d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res);

  bool saveWorldConfig(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);

  bool setUseSafetyArea(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  bool addObstacle(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);

  // double getMaxZ(const std::string& frame_id);

  // double getMinZ(const std::string& frame_id);

public:
  virtual void onInit();

  ~SafetyAreaManager();

}; // class SafetyAreaManager

} // namespace safety_area_manager 

} // namespace mrs_uav_managers