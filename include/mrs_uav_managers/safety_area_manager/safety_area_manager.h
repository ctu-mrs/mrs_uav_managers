#include <ros/ros.h>

#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/PathToPointInSafetyArea.h>
#include <mrs_msgs/HwApiCapabilities.h>

#include <std_msgs/String.h>

#include <mrs_lib/mutex.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/safety_zone/safety_zone.h>
#include <mrs_lib/safety_zone/int_edges_visualization.h>
#include <mrs_lib/safety_zone/static_edges_visualization.h>
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

  // std::mutex                           mutex_safety_area_min_z_;
  bool                                 use_safety_area_;
  // bool                                 obstacle_polygons_enabled_ = false;
  // bool                                 obstacle_points_enabled_   = false;
  // double                               safety_area_min_z_ = 0;
  // double                               safety_area_max_z_ = 0;
  std::string                          safety_area_frame_;
  std::string                          safety_area_horizontal_frame_;
  std::string                          safety_area_vertical_frame_;
  std::string                          uav_name_;
  // std::string                          body_frame_;
  mrs_lib::SafetyZone* safety_zone_;

  // Visualization objects
  std::vector<mrs_lib::StaticEdgesVisualization> static_edges_;
  std::vector<mrs_lib::IntEdgesVisualization>    int_edges_;
  std::vector<mrs_lib::VertexControl>            vertices_;
  std::vector<mrs_lib::CenterControl>            centers_;
  std::vector<mrs_lib::BoundsControl>            bounds_;


  ros::NodeHandle nh_;
  ros::Publisher  publisher_;

  // profiling
  mrs_lib::Profiler profiler_;
  bool              profiler_enabled_ = false;

  int status_timer_rate_   = 0;

  // safety area services
  ros::ServiceServer service_server_point_in_safety_area_3d_;
  ros::ServiceServer service_server_point_in_safety_area_2d_;
  ros::ServiceServer service_server_path_in_safety_area_3d_;
  ros::ServiceServer service_server_path_in_safety_area_2d_;
  
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

  bool isPointInSafetyArea3d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);

  bool isPointInSafetyArea2d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);

  bool isPathToPointInSafetyArea3d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res);

  bool isPathToPointInSafetyArea2d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res);

  // double getMaxZ(const std::string& frame_id);

  // double getMinZ(const std::string& frame_id);

public:
  virtual void onInit();

}; // class SafetyAreaManager

} // namespace safety_area_manager 

} // namespace mrs_uav_managers