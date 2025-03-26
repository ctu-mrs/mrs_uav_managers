#ifndef CONTROL_MANAGER_COMMON_HANDLERS_H
#define CONTROL_MANAGER_COMMON_HANDLERS_H

#include <mrs_lib/transformer.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/quadratic_throttle_model.h>

namespace mrs_uav_managers
{

namespace control_manager
{

/* safety area handler //{ */

typedef std::function<bool(const mrs_msgs::msg::ReferenceStamped &point)> isPointInSafetyArea3d_t;
typedef std::function<bool(const mrs_msgs::msg::ReferenceStamped &point)> isPointInSafetyArea2d_t;
typedef std::function<double(const std::string &frame_id)>                getMaxZ_t;
typedef std::function<double(const std::string &frame_id)>                getMinZ_t;

struct SafetyArea_t
{
  mrs_uav_managers::control_manager::isPointInSafetyArea3d_t isPointInSafetyArea3d;
  mrs_uav_managers::control_manager::isPointInSafetyArea2d_t isPointInSafetyArea2d;
  mrs_uav_managers::control_manager::getMaxZ_t               getMaxZ;
  mrs_uav_managers::control_manager::getMinZ_t               getMinZ;
  bool                                                       use_safety_area;
};

//}

/* scope timer handler //{ */

struct ScopeTimer_t
{
  bool                                       enabled;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> logger;
};

//}

typedef std::function<double(void)> getMass_t;

struct DetailedModelParams_t
{
  Eigen::MatrixXd control_group_mixer;
  Eigen::MatrixXd force_torque_mixer;
  Eigen::Matrix3d inertia;
  double          prop_radius;
  double          arm_length;
  double          body_height;
};

struct ControlOutputModalities_t
{
  bool actuators             = false;
  bool control_group         = false;
  bool attitude_rate         = false;
  bool attitude              = false;
  bool acceleration_hdg_rate = false;
  bool acceleration_hdg      = false;
  bool velocity_hdg_rate     = false;
  bool velocity_hdg          = false;
  bool position              = false;
};

struct CommonHandlers_t
{
  SafetyArea_t                                     safety_area;
  std::shared_ptr<mrs_lib::Transformer>            transformer;
  ScopeTimer_t                                     scope_timer;
  getMass_t                                        getMass;
  double                                           g;
  mrs_lib::quadratic_throttle_model::MotorParams_t throttle_model;
  std::optional<DetailedModelParams_t>             detailed_model_params;
  ControlOutputModalities_t                        control_output_modalities;
  std::string                                      uav_name;
  rclcpp::Node::SharedPtr                          parent_node;
};

}  // namespace control_manager

}  // namespace mrs_uav_managers

#endif  // CONTROL_MANAGER_COMMON_HANDLERS_H
