#ifndef CONTROL_MANAGER_PRIVATE_HANDLERS_H
#define CONTROL_MANAGER_PRIVATE_HANDLERS_H

#include <string>
#include <mrs_lib/param_loader.h>

namespace mrs_uav_managers
{

namespace control_manager
{

struct PrivateHandlers_t
{
  std::unique_ptr<mrs_lib::ParamLoader> param_loader;
  std::shared_ptr<mrs_lib::ParamLoader> parent_param_loader;
  std::string                           name_space;
  std::string                           runtime_name;
};

}  // namespace control_manager

}  // namespace mrs_uav_managers

#endif  // CONTROL_MANAGER_PRIVATE_HANDLERS_H
