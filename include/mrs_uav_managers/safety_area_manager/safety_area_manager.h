#include <ros/ros.h>

#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/PathToPointInSafetyArea.h>
#include <mrs_msgs/HwApiCapabilities.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/GetPointStamped.h>
#include <mrs_msgs/GetBool.h>
#include <mrs_msgs/GetSafeZoneAtHeight.h>

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

namespace mrs_uav_managers
{

namespace safety_area_manager 
{

class SafetyAreaManager : public nodelet::Nodelet
{
private:
  std::shared_ptr<mrs_lib::Transformer> transformer_;
  mrs_lib::SafetyZone*                  safety_zone_;
  ros::NodeHandle                       nh_;

  // Wolrd config parameters
  std::string uav_name_;
  std::string safety_area_horizontal_frame_;
  std::string safety_area_vertical_frame_;
  std::string world_origin_units_;
  double      origin_y_;
  double      origin_x_;
  bool        use_safety_area_;

  // Visualization objects
  std::vector<mrs_lib::StaticEdgesVisualization*> static_edges_;
  std::vector<mrs_lib::IntEdgesVisualization*>    int_edges_;
  std::vector<mrs_lib::VertexControl*>            vertices_;
  std::vector<mrs_lib::CenterControl*>            centers_;
  std::vector<mrs_lib::BoundsControl*>            bounds_;

  // profiling
  mrs_lib::Profiler profiler_;
  bool              profiler_enabled_  = false;
  int               status_timer_rate_ = 0;

  // safety area services
  ros::ServiceServer service_server_get_safety_zone_at_height_;
  ros::ServiceServer service_server_point_in_safety_area_3d_;
  ros::ServiceServer service_server_point_in_safety_area_2d_;
  ros::ServiceServer service_server_path_in_safety_area_3d_;
  ros::ServiceServer service_server_path_in_safety_area_2d_;
  ros::ServiceServer service_server_save_world_config_;
  ros::ServiceServer service_server_load_world_config_;
  ros::ServiceServer service_server_set_world_config_;
  ros::ServiceServer service_server_get_world_config_;
  ros::ServiceServer service_server_use_safety_area_;
  ros::ServiceServer service_server_add_obstacle_;
  ros::ServiceServer service_server_get_max_z_;
  ros::ServiceServer service_server_get_min_z_;
  ros::ServiceServer service_server_get_use_;
  
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiCapabilities> sh_hw_api_capabilities_;

  // Tools for convenience
  std::unique_ptr<mrs_lib::Prism> makePrism(const Eigen::MatrixXd matrix, double max_z, double min_z) const;
  double transformZ(std::string from, std::string to, double z);
  bool initializeSafetyZone(mrs_lib::ParamLoader& param_loader, std::string filename);

  // this timer will check till we already got the hardware api diagnostics
  // then it will trigger the initialization of the controllers and finish
  // the initialization of the SafetyAreaManager
  ros::Timer timer_hw_api_capabilities_;
  void timerHwApiCapabilities( [[maybe_unused]] const ros::TimerEvent& event);
  void preinitialize();
  void initialize();

  // Services
  bool isPointInSafetyArea3d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);
  bool isPointInSafetyArea2d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);
  bool isPathToPointInSafetyArea3d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res);
  bool isPathToPointInSafetyArea2d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res);
  bool getSafeZoneAtHeight(mrs_msgs::GetSafeZoneAtHeight::Request& req, mrs_msgs::GetSafeZoneAtHeight::Response& res);
  bool saveWorldConfig(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);
  bool getWorldConfig( [[maybe_unused]] mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);
  bool loadWorldConfig(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);
  bool setWorldConfig(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);
  bool setUseSafetyArea(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool addObstacle(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);
  bool getMaxZ( [[maybe_unused]] mrs_msgs::GetPointStamped::Request& req, mrs_msgs::GetPointStamped::Response& res);
  bool getMinZ( [[maybe_unused]] mrs_msgs::GetPointStamped::Request& req, mrs_msgs::GetPointStamped::Response& res);
  bool getUse( [[maybe_unused]] mrs_msgs::GetBool::Request& req, mrs_msgs::GetBool::Response& res);

public:
  virtual void onInit();

  ~SafetyAreaManager();

}; // class SafetyAreaManager

} // namespace safety_area_manager 

} // namespace mrs_uav_managers
