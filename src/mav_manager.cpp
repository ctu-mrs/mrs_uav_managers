#include <ros/ros.h>
#include <nodelet/nodelet.h>

//<reformat_checkpoint>
class MavManager : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
};

// constructor
void MavManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getPrivateNodeHandle();
}
