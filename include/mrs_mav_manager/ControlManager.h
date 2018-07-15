#ifndef CONTROLMANAGER_H
#define CONTROLMANAGER_H

#include <mrs_msgs/SwitchTracker.h>
#include <mrs_msgs/SwitchController.h>
#include <mrs_mav_manager/Controller.h>
#include <mrs_mav_manager/Tracker.h>
#include <mrs_msgs/TrackerStatus.h>
#include <mrs_msgs/Vec4.h>

#include <pluginlib/class_loader.h>

#include <nodelet/loader.h>

#include <mavros_msgs/AttitudeTarget.h>

#include <mutex>

#include <tf/transform_datatypes.h>

#include <thread>

#include <ros/package.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

namespace mrs_mav_manager
{

class ControlManager : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

private:
  pluginlib::ClassLoader<mrs_mav_manager::Tracker> *   tracker_loader;
  pluginlib::ClassLoader<mrs_mav_manager::Controller> *controller_loader;

  std::vector<std::string> tracker_names;
  std::vector<std::string> controller_names;

  std::vector<boost::shared_ptr<mrs_mav_manager::Tracker>>    tracker_list;
  std::vector<boost::shared_ptr<mrs_mav_manager::Controller>> controller_list;
  std::string                                                 null_tracker_name;
  std::mutex                                                  mutex_tracker_list;

  ros::Subscriber    subscriber_odometry;
  nav_msgs::Odometry odometry;
  std::mutex         mutex_odometry;
  bool               got_odometry = false;

  int active_tracker_idx    = 0;
  int active_controller_idx = 0;

  ros::Publisher publisher_attitude_cmd;
  ros::Publisher publisher_cmd_pose;
  ros::Publisher publisher_tracker_status;

  ros::ServiceServer service_switch_tracker;
  ros::ServiceServer service_switch_controller;
  ros::ServiceServer service_goto;
  ros::ServiceServer service_goto_relative;

  mrs_msgs::PositionCommand::ConstPtr last_position_cmd;
  mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd;

private:
  void callbackOdometry(const nav_msgs::OdometryConstPtr &msg);

  bool callbackSwitchTracker(mrs_msgs::SwitchTracker::Request &req, mrs_msgs::SwitchTracker::Response &res);
  bool callbackSwitchController(mrs_msgs::SwitchController::Request &req, mrs_msgs::SwitchController::Response &res);

  bool callbackGoto(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res);
  bool callbackGotoRelative(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res);

private:
  void        mainThread(void);
  std::thread main_thread;
};

}

#endif
