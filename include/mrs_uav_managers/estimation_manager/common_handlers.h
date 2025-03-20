#ifndef ESTIMATION_MANAGER_COMMON_HANDLERS_H
#define ESTIMATION_MANAGER_COMMON_HANDLERS_H

#include <rclcpp/rclcpp.hpp>
#include <mrs_lib/transformer.h>
#include <mrs_lib/scope_timer.h>

namespace mrs_uav_managers
{

namespace estimation_manager
{

struct CommonFrames_t
{

  std::string fcu;
  std::string ns_fcu;
  std::string fcu_untilted;
  std::string ns_fcu_untilted;
  std::string rtk_antenna;
  std::string ns_rtk_antenna;
  std::string amsl;
  std::string ns_amsl;
};

struct WorldOrigin_t
{
  double x;
  double y;
};

struct DebugTopics_t
{
  bool input;
  bool output;
  bool state;
  bool covariance;
  bool innovation;
  bool diag;
  bool correction;
  bool corr_delay;
};

struct ScopeTimer_t
{
  bool                                       enabled;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> logger;
};

struct CommonHandlers_t
{

  std::string                           nodelet_name;
  std::string                           package_name;
  std::string                           uav_name;
  CommonFrames_t                        frames;
  std::shared_ptr<mrs_lib::Transformer> transformer;
  double                                desired_uav_state_rate;
  double                                desired_diagnostics_rate;
  WorldOrigin_t                         world_origin;
  ScopeTimer_t                          scope_timer;
  DebugTopics_t                         debug_topics;
};

}  // namespace estimation_manager

}  // namespace mrs_uav_managers

#endif  // COMMON_HANDLERS_H
