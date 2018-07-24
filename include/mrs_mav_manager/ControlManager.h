#ifndef CONTROLMANAGER_H
#define CONTROLMANAGER_H

#include <mrs_msgs/SwitchTracker.h>
#include <mrs_msgs/SwitchController.h>
#include <mrs_mav_manager/Controller.h>
#include <mrs_mav_manager/Tracker.h>
#include <mrs_msgs/TrackerStatus.h>

#include <pluginlib/class_loader.h>

#include <nodelet/loader.h>

#include <mavros_msgs/AttitudeTarget.h>

#include <mutex>

#include <tf/transform_datatypes.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_srvs/SetBool.h>

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

  std::string null_tracker_name_;
  std::string hover_tracker_name_;

  std::mutex mutex_tracker_list;
  std::mutex mutex_controller_list;

  ros::Subscriber    subscriber_odometry;
  nav_msgs::Odometry odometry;
  double             odometry_x;
  double             odometry_y;
  double             odometry_z;
  double             odometry_yaw;
  double             odometry_roll;
  double             odometry_pitch;
  std::mutex         mutex_odometry;
  bool               got_odometry = false;

  int  active_tracker_idx    = 0;
  int  active_controller_idx = 0;
  int  hover_tracker_idx     = 0;
  bool motors                = 0;

  ros::Publisher publisher_attitude_cmd;
  ros::Publisher publisher_cmd_pose;
  ros::Publisher publisher_tracker_status;

  ros::ServiceServer service_switch_tracker;
  ros::ServiceServer service_switch_controller;
  ros::ServiceServer service_goto;
  ros::ServiceServer service_goto_relative;
  ros::ServiceServer service_goto_altitude;
  ros::ServiceServer service_hover;
  ros::ServiceServer service_motors;

  mrs_msgs::PositionCommand::ConstPtr last_position_cmd;
  mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd;

private:
  double max_tilt_angle_;
  double max_control_error_;

private:
  void callbackOdometry(const nav_msgs::OdometryConstPtr &msg);

  bool callbackSwitchTracker(mrs_msgs::SwitchTracker::Request &req, mrs_msgs::SwitchTracker::Response &res);
  bool callbackSwitchController(mrs_msgs::SwitchController::Request &req, mrs_msgs::SwitchController::Response &res);

  bool callbackGoto(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res);
  bool callbackGotoRelative(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res);
  bool callbackGotoAltitude(mrs_msgs::Vec1::Request &req, mrs_msgs::Vec1::Response &res);

  bool callbackHover(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool callbackMotors(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

private:
  ros::Timer status_timer;
  void statusTimer(const ros::TimerEvent &event);

private:
  ros::Timer safety_timer;
  void safetyTimer(const ros::TimerEvent &event);
};
}

#endif
