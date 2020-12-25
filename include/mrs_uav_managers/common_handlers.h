#ifndef COMMON_HANDLERS_H
#define COMMON_HANDLERS_H

#include <mrs_lib/transformer.h>
#include <mrs_lib/quadratic_thrust_model.h>

namespace mrs_uav_managers
{

/* safety area handler //{ */

typedef boost::function<bool(const mrs_msgs::ReferenceStamped point)> isPointInSafetyArea3d_t;
typedef boost::function<bool(const mrs_msgs::ReferenceStamped point)> isPointInSafetyArea2d_t;
typedef boost::function<double(void)>                                 getMaxHeight_t;
typedef boost::function<double(void)>                                 getMinHeight_t;

struct SafetyArea_t
{
  mrs_uav_managers::isPointInSafetyArea3d_t isPointInSafetyArea3d;
  mrs_uav_managers::isPointInSafetyArea2d_t isPointInSafetyArea2d;
  mrs_uav_managers::getMaxHeight_t          getMaxHeight;
  mrs_uav_managers::getMinHeight_t          getMinHeight;
  std::string                               frame_id;
  bool                                      use_safety_area;
};

//}

/* obstacle bumper handler //{ */

typedef boost::function<bool(mrs_msgs::ReferenceStamped &point)> bumperValidatePoint_t;

struct Bumper_t
{
  bool                                    enabled;
  mrs_uav_managers::bumperValidatePoint_t bumperValidatePoint;
};

//}

typedef boost::function<double(void)> getMass_t;

struct CommonHandlers_t
{
  SafetyArea_t                                   safety_area;
  std::shared_ptr<mrs_lib::Transformer>          transformer;
  Bumper_t                                       bumper;
  getMass_t                                      getMass;
  mrs_lib::quadratic_thrust_model::MotorParams_t motor_params;
  double                                         g;
};


}  // namespace mrs_uav_managers

#endif  // COMMON_HANDLERS_H
