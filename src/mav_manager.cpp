#include <mrs_mav_manager/ControlManager.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

namespace mrs_mav_manager
{

class MavManager : public nodelet::Nodelet {

public:
  virtual void onInit();
  void         test(void);

private:
  ros::NodeHandle                 nh_;
  /* mrs_mav_manager::ControlManager control_manager_; */
};

// constructor
void MavManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getPrivateNodeHandle();

  ROS_INFO("[MavManager]: initilized");
}

void MavManager::test(void) {

  ROS_INFO("[MavManager]: pes steka");
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mav_manager::MavManager, nodelet::Nodelet)
