#ifndef SAFETY_AREA_MANAGER_COMMON_HANDLERS_H
#define SAFETY_AREA_MANAGER_COMMON_HANDLERS_H

#include <mrs_lib/transformer.h>
#include <mrs_lib/safety_zone/safety_zone.h>
#include <mrs_lib/scope_timer.h>

namespace mrs_uav_managers
{

namespace safety_area_manager
{

/* safety area handler //{ */

typedef boost::function<bool(const mrs_msgs::ReferenceStamped &point)> isPointInSafetyArea3d_t;
typedef boost::function<bool(const mrs_msgs::ReferenceStamped &point)> isPointInSafetyArea2d_t;
typedef boost::function<double(const std::string &frame_id)>           getMaxZ_t;
typedef boost::function<double(const std::string &frame_id)>           getMinZ_t;

struct SafetyArea_t
{
  mrs_uav_managers::safety_area_manager::isPointInSafetyArea3d_t isPointInSafetyArea3d;
  mrs_uav_managers::safety_area_manager::isPointInSafetyArea2d_t isPointInSafetyArea2d;
  mrs_uav_managers::safety_area_manager::getMaxZ_t               getMaxZ;
  mrs_uav_managers::safety_area_manager::getMinZ_t               getMinZ;
  bool                                                           use_safety_area;
};

//}

/* scope timer handler //{ */

struct ScopeTimer_t
{
  bool                                       enabled;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> logger;
};

//}

struct CommonHandlers_t
{
  SafetyArea_t                                     safety_area;
  std::shared_ptr<mrs_lib::Transformer>            transformer;
  std::shared_ptr<mrs_lib::SafetyZone>             safety_zone;
  ScopeTimer_t                                     scope_timer;
  std::string                                      uav_name;
  ros::NodeHandle                                  parent_nh;
};

}  // namespace safety_area_manager 

}  // namespace mrs_uav_managers

#endif  // SAFETY_AREA_MANAGER_COMMON_HANDLERS_H
