#include "ros/ros.h"

#include "mrs_msgs/ReferenceStamped.h"
#include "mrs_msgs/ReferenceStampedSrv.h"
#include "mrs_msgs/PathToPointInSafetyArea.h"
#include "mrs_msgs/HwApiCapabilities.h"

#include "std_msgs/String.h"

#include "mrs_lib/mutex.h"
#include <mrs_lib/profiler.h>
#include "mrs_lib/transformer.h"
#include "mrs_lib/param_loader.h"
#include "mrs_lib/subscribe_handler.h"
#include "mrs_lib/safety_zone/safety_zone.h"

#include "nodelet/nodelet.h"

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

  std::mutex                           mutex_safety_area_min_z_;
  bool                                 use_safety_area_;
  bool                                 obstacle_polygons_enabled_ = false;
  bool                                 obstacle_points_enabled_   = false;
  double                               safety_area_min_z_ = 0;
  double                               safety_area_max_z_ = 0;
  std::string                          safety_area_frame_;
  std::string                          uav_name_;
  std::string                          body_frame_;
  std::unique_ptr<mrs_lib::SafetyZone> safety_zone_;

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

  // safety area min z servers
  ros::ServiceServer service_server_set_min_z_;
  ros::ServiceServer service_server_get_min_z_;
  ros::ServiceServer service_server_set_max_z_;
  ros::ServiceServer service_server_get_max_z_;
  
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiCapabilities> sh_hw_api_capabilities_;

  void preinitialize();

  void initialize();

  // this timer will check till we already got the hardware api diagnostics
  // then it will trigger the initialization of the controllers and finish
  // the initialization of the ControlManager
  ros::Timer timer_hw_api_capabilities_;
  void timerHwApiCapabilities(const ros::TimerEvent& event);

  bool isPointInSafetyArea3d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);

  bool isPointInSafetyArea2d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);

  bool isPathToPointInSafetyArea3d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res);

  bool isPathToPointInSafetyArea2d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res);

  double getMaxZ(const std::string& frame_id);

  double getMinZ(const std::string& frame_id);

public:
  virtual void onInit();

}; // class SafetyAreaManager

} // namespace safety_area_manager 

} // namespace mrs_uav_managers