#include <ros/ros.h>

class MavManager {

public:

  MavManager();

private:

  ros::NodeHandle nh_;

};

// constructor
MavManager::MavManager() {

  nh_ = ros::NodeHandle("~");



}

int main(int argc, char **argv) {

  ros::init(argc, argv, "mav_manager");

  MavManager mav_manager;

  ros::spin();

  return 0;
};
