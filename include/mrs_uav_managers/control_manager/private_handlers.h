#ifndef CONTROL_MANAGER_PRIVATE_HANDLERS_H
#define CONTROL_MANAGER_PRIVATE_HANDLERS_H

#include <string>

namespace mrs_uav_managers
{

namespace control_manager
{

typedef boost::function<bool(const std::string)> loadConfigFile_t;

struct PrivateHandlers_t
{
  loadConfigFile_t loadConfigFile;
  std::string      name_space;
  std::string      runtime_name;
};

}  // namespace control_manager

}  // namespace mrs_uav_managers

#endif  // CONTROL_MANAGER_PRIVATE_HANDLERS_H
