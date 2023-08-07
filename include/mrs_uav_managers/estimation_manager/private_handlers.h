#ifndef ESTIMATION_MANAGER_PRIVATE_HANDLERS_H
#define ESTIMATION_MANAGER_PRIVATE_HANDLERS_H

#include <string>

namespace mrs_uav_managers
{

namespace estimation_manager
{

typedef boost::function<bool(const std::string)> loadConfigFile_t;

struct PrivateHandlers_t
{
  loadConfigFile_t loadConfigFile;
  std::string      name_space;
  std::string      runtime_name;
};

}  // namespace estimation_manager

}  // namespace mrs_uav_managers

#endif  // ESTIMATION_MANAGER_PRIVATE_HANDLERS_H
