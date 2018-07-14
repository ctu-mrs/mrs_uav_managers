#include <ros/package.h>
#include <ros/ros.h>

#include <mrs_msgs/SwitchTracker.h>
#include <mrs_msgs/SwitchController.h>
#include <mrs_mav_manager/Controller.h>
#include <mrs_mav_manager/Tracker.h>
#include <mrs_msgs/TrackerStatus.h>
#include <mrs_msgs/Vec4.h>

#include <pluginlib/class_loader.h>

#include <nodelet/loader.h>
#include <nodelet/nodelet.h>

#include <mavros_msgs/AttitudeTarget.h>

#include <mutex>

#include <tf/transform_datatypes.h>

#include <thread>

namespace mrs_mav_manager
{

class ControlManager : public nodelet::Nodelet {

public:
  virtual void onInit();

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

//{ onInit

void ControlManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getPrivateNodeHandle();

  ros::Time::waitForValid();

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_odometry = nh_.subscribe("odometry_in", 1, &ControlManager::callbackOdometry, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  publisher_attitude_cmd   = nh_.advertise<mavros_msgs::AttitudeTarget>("attitude_cmd_out", 1);
  publisher_cmd_pose       = nh_.advertise<nav_msgs::Odometry>("cmd_pose", 1);
  publisher_tracker_status = nh_.advertise<mrs_msgs::TrackerStatus>("tracker_status", 1);

  // --------------------------------------------------------------
  // |                          services                          |
  // --------------------------------------------------------------

  service_switch_tracker    = nh_.advertiseService("switch_tracker", &ControlManager::callbackSwitchTracker, this);
  service_switch_controller = nh_.advertiseService("switch_controller", &ControlManager::callbackSwitchController, this);
  service_goto              = nh_.advertiseService("goto", &ControlManager::callbackGoto, this);
  service_goto_relative     = nh_.advertiseService("goto_relative", &ControlManager::callbackGotoRelative, this);

  // --------------------------------------------------------------
  // |                        load trackers                       |
  // --------------------------------------------------------------

  nh_.getParam("trackers", tracker_names);
  nh_.getParam("null_tracker", null_tracker_name);
  tracker_names.insert(tracker_names.begin(), null_tracker_name);

  tracker_loader = new pluginlib::ClassLoader<mrs_mav_manager::Tracker>("mrs_mav_manager", "mrs_mav_manager::Tracker");

  for (unsigned long i = 0; i < tracker_names.size(); i++) {

    std::string tracker_name = tracker_names[i];

    try {
      ROS_INFO("[ControlManager]: Trying to load tracker %s", tracker_name.c_str());
      { tracker_list.push_back(tracker_loader->createInstance(tracker_name.c_str())); }
    }
    catch (pluginlib::CreateClassException &ex1) {
      ROS_ERROR("[ControlManager]: CreateClassException for tracker %s", tracker_name.c_str());
      ROS_ERROR("[ControlManager]: Error: %s", ex1.what());
      return;
    }
    catch (pluginlib::PluginlibException &ex) {
      ROS_ERROR("[ControlManager]: PluginlibException for tracker %s", tracker_name.c_str());
      ROS_ERROR("[ControlManager]: Error: %s", ex.what());
      return;
    }
  }

  ROS_INFO("[ControlManager]: trackers were loaded");

  for (unsigned long i = 0; i < tracker_list.size(); i++) {

    try {
      ROS_INFO("[ControlManager]: Initializing tracker %d: %s", (int)i, tracker_names[i].c_str());
      { tracker_list[i]->initialize(nh_); }
    }
    catch (std::runtime_error &ex) {
      ROS_ERROR("[ControlManager]: Exception caught during tracker initialization: %s", ex.what());
    }
  }

  ROS_INFO("[ControlManager]: trackers were activated");

  // --------------------------------------------------------------
  // |                      load controllers                      |
  // --------------------------------------------------------------

  nh_.getParam("controllers", controller_names);

  controller_loader = new pluginlib::ClassLoader<mrs_mav_manager::Controller>("mrs_mav_manager", "mrs_mav_manager::Controller");

  for (unsigned long i = 0; i < controller_names.size(); i++) {

    std::string controller_name = controller_names[i];

    try {
      ROS_INFO("[ControlManager]: Loading controller %s", controller_name.c_str());
      controller_list.push_back(controller_loader->createInstance(controller_name.c_str()));
    }
    catch (pluginlib::CreateClassException &ex1) {
      ROS_ERROR("[ControlManager]: CreateClassException for controller %s", controller_name.c_str());
      ROS_ERROR("[ControlManager]: Error: %s", ex1.what());
      return;
    }
    catch (pluginlib::PluginlibException &ex) {
      ROS_ERROR("[ControlManager]: PluginlibException for controller %s", controller_name.c_str());
      ROS_ERROR("[ControlManager]: Error: %s", ex.what());
      return;
    }
  }

  ROS_INFO("[ControlManager]: controllers were loaded");

  for (unsigned long i = 0; i < controller_list.size(); i++) {

    try {
      ROS_INFO("[ControlManager]: Initializing controller %d: %s", (int)i, controller_names[i].c_str());
      controller_list[i]->initialize(nh_);
    }
    catch (std::runtime_error &ex) {
      ROS_ERROR("[ControlManager]: Exception caught during controller initialization: %s", ex.what());
    }
  }

  ROS_INFO("[ControlManager]: controllers were initialized");

  // --------------------------------------------------------------
  // |           active the first controller on the list          |
  // --------------------------------------------------------------

  ROS_INFO("[ControlManager]: Activating the first controller on the list (%s)", controller_names[0].c_str());

  controller_list[active_controller_idx]->activate(last_attitude_cmd);

  // --------------------------------------------------------------
  // |                    start the main thread                   |
  // --------------------------------------------------------------

  main_thread = std::thread(&ControlManager::mainThread, this);

  ROS_INFO("[ControlManager]: initialized");
}

//}

//{ callbackOdometry

void ControlManager::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

  got_odometry = true;

  odometry = *msg;

  // --------------------------------------------------------------
  // |                     Update the trackers                    |
  // --------------------------------------------------------------

  nav_msgs::Odometry::ConstPtr        odometry_const_ptr(new nav_msgs::Odometry(odometry));
  mrs_msgs::PositionCommand::ConstPtr tracker_output_cmd;

  try {

    // for each tracker
    for (unsigned int i = 0; i < tracker_list.size(); i++) {

      if ((int)i == active_tracker_idx) {

        // if it is the active one, update and retrieve the command
        tracker_output_cmd = tracker_list[i]->update(odometry_const_ptr);

      } else {

        // if it is not the active one, just update without retrieving the commadn
        tracker_list[i]->update(odometry_const_ptr);
      }
    }

    if (mrs_msgs::PositionCommand::Ptr() != tracker_output_cmd) {

      last_position_cmd = tracker_output_cmd;

    } else if (active_tracker_idx > 0) {

      ROS_WARN_THROTTLE(1.0, "[ControlManager]: The tracker %s return empty command!", tracker_names[active_tracker_idx].c_str());
      // TODO: switch to failsave tracker, or stop outputting commands
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: TODO");

    } else if (active_tracker_idx == 0) {

      last_position_cmd = tracker_output_cmd;
    }
  }
  catch (std::runtime_error &exrun) {
    ROS_INFO("[ControlManager]: Exception while updateing trackers.");
    ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
  }

  tf::Quaternion desired_orientation;

  // publish the cmd_pose topic
  if (last_position_cmd != mrs_msgs::PositionCommand::Ptr()) {

    nav_msgs::Odometry cmd_pose;
    cmd_pose.header.stamp         = ros::Time::now();
    cmd_pose.header.frame_id      = "local_origin";
    cmd_pose.pose.pose.position   = tracker_output_cmd->position;
    cmd_pose.twist.twist.linear.x = tracker_output_cmd->velocity.x;
    cmd_pose.twist.twist.linear.y = tracker_output_cmd->velocity.y;
    cmd_pose.twist.twist.linear.z = tracker_output_cmd->velocity.z;
    desired_orientation           = tf::createQuaternionFromRPY(0, 0, tracker_output_cmd->yaw);
    desired_orientation.normalize();
    quaternionTFToMsg(desired_orientation, cmd_pose.pose.pose.orientation);
    publisher_cmd_pose.publish(cmd_pose);
  }

  // --------------------------------------------------------------
  // |                   Update the controllers                   |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand::ConstPtr controller_output_cmd;

  if (last_position_cmd != mrs_msgs::PositionCommand::Ptr()) {

    try {
      controller_output_cmd = controller_list[active_controller_idx]->update(odometry_const_ptr, last_position_cmd);
    }
    catch (std::runtime_error &exrun) {
      ROS_INFO("[ControlManager]: Exception while updating the active controller.");
      ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
    }
  }

  // --------------------------------------------------------------
  // |                 Publish the control command                |
  // --------------------------------------------------------------

  mavros_msgs::AttitudeTarget attitude_target;

  if (active_tracker_idx > 0 && controller_output_cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN_THROTTLE(1.0, "[ControlManager]: TrackerManager: the controller (%s) returned nil command! publishing zeros...",
                      controller_names[active_controller_idx].c_str());

    // convert the RPY to quaternion
    desired_orientation = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);
    desired_orientation.normalize();
    quaternionTFToMsg(desired_orientation, attitude_target.orientation);

    attitude_target.thrust = 0.0;

  } else if (controller_output_cmd != mrs_msgs::AttitudeCommand::Ptr()) {

    last_attitude_cmd = controller_output_cmd;

    // convert the RPY to quaternion
    desired_orientation = tf::createQuaternionFromRPY(controller_output_cmd->roll, controller_output_cmd->pitch, controller_output_cmd->yaw);
    desired_orientation.normalize();
    quaternionTFToMsg(desired_orientation, attitude_target.orientation);

    attitude_target.thrust = controller_output_cmd->thrust;
  }

  attitude_target.header.stamp    = ros::Time::now();
  attitude_target.header.frame_id = "local_origin";

  publisher_attitude_cmd.publish(attitude_target);
}

//}

//{ callbackSwitchTracker

bool ControlManager::callbackSwitchTracker(mrs_msgs::SwitchTracker::Request &req, mrs_msgs::SwitchTracker::Response &res) {

  char message[100];

  if (!got_odometry) {

    sprintf((char *)&message, "Can't switch tracker, missing odometry!");
    ROS_ERROR("[ControlManager]: %s", message);
    res.success = false;
    res.message = message;
    return true;
  }

  int new_tracker_idx = -1;

  for (unsigned int i = 0; i < tracker_names.size(); i++) {
    if (req.tracker.compare(tracker_names[i]) == 0) {
      new_tracker_idx = i;
    }
  }

  // check if the tracker exists
  if (new_tracker_idx < 0) {

    sprintf((char *)&message, "The tracker %s does not exist!", req.tracker.c_str());
    ROS_ERROR("[ControlManager]: %s", message);
    res.success = false;
    res.message = message;
    return true;
  }

  // check if the tracker is already active
  if (new_tracker_idx == active_tracker_idx) {

    sprintf((char *)&message, "The tracker %s is already active!", req.tracker.c_str());
    ROS_ERROR("[ControlManager]: %s", message);
    res.success = true;
    res.message = message;
    return true;
  }

  try {

    ROS_INFO("[ControlManager]: Activating tracker %s", tracker_names[new_tracker_idx].c_str());
    { tracker_list[new_tracker_idx]->activate(last_position_cmd); }
    sprintf((char *)&message, "Tracker %s has been activated", req.tracker.c_str());
    ROS_INFO("[ControlManager]: %s", message);
    res.success = true;

    // super important, switch which the active tracker idx
    try {
      { tracker_list[active_tracker_idx]->deactivate(); }
    }
    catch (std::runtime_error &exrun) {
      ROS_ERROR("[ControlManager]: Could not deactivate tracker %s", tracker_names[active_tracker_idx].c_str());
    }

    active_tracker_idx = new_tracker_idx;
  }
  catch (std::runtime_error &exrun) {
    ROS_ERROR("[ControlManager]: Error during activation of tracker %s", req.tracker.c_str());
    ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
  }

  res.message = message;
  return true;
}

bool ControlManager::callbackSwitchController(mrs_msgs::SwitchController::Request &req, mrs_msgs::SwitchController::Response &res) {

  char message[100];

  int new_controller_idx = -1;

  for (unsigned int i = 0; i < controller_names.size(); i++) {
    if (req.controller.compare(controller_names[i]) == 0) {
      new_controller_idx = i;
    }
  }

  // check if the controller exists
  if (new_controller_idx < 0) {

    sprintf((char *)&message, "The controller %s does not exist!", req.controller.c_str());
    ROS_ERROR("[ControlManager]: %s", message);
    res.success = false;
    res.message = message;
    return true;
  }

  // check if the controller is not active
  if (new_controller_idx == active_controller_idx) {

    sprintf((char *)&message, "The controller %s is already active!", req.controller.c_str());
    ROS_ERROR("[ControlManager]: %s", message);
    res.success = true;
    res.message = message;
    return true;
  }

  try {

    ROS_INFO("[ControlManager]: Activating controller %s", controller_names[new_controller_idx].c_str());
    { controller_list[new_controller_idx]->activate(last_attitude_cmd); }
    sprintf((char *)&message, "Controller %s has been activated", req.controller.c_str());
    ROS_INFO("[ControlManager]: %s", message);
    res.success = true;

    // super important, switch which the active controller idx
    try {
      { controller_list[active_controller_idx]->deactivate(); }
    }
    catch (std::runtime_error &exrun) {
      ROS_ERROR("[ControlManager]: Could not deactivate controller %s", controller_names[active_controller_idx].c_str());
    }

    active_controller_idx = new_controller_idx;
  }
  catch (std::runtime_error &exrun) {
    ROS_ERROR("[ControlManager]: Error during activation of controller %s", req.controller.c_str());
    ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
  }

  res.message = message;
  return true;
}

//}

//{ mainThread

void ControlManager::mainThread(void) {

  ROS_INFO("[ControlManager]: mainThread has started");
  ros::Rate r(100);

  mrs_msgs::TrackerStatus::Ptr tracker_status_ptr;
  mrs_msgs::TrackerStatus      tracker_status;

  while (ros::ok()) {

    tracker_status_ptr = tracker_list[active_tracker_idx]->status();

    tracker_status = mrs_msgs::TrackerStatus(*tracker_status_ptr);

    tracker_status.tracker = tracker_names[active_tracker_idx];
    tracker_status.stamp   = ros::Time::now();

    try {
      publisher_tracker_status.publish(tracker_status);
    }
    catch (...) {
      ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_tracker_status.getTopic().c_str());
    }

    r.sleep();
  }
}

bool ControlManager::callbackGoto(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res) {

  mrs_msgs::Vec4Response::ConstPtr tracker_response;
  char                             message[100];

  mrs_msgs::Vec4Request req_out;
  req_out.goal = req.goal;

  mutex_tracker_list.lock();
  {
    tracker_response = tracker_list[active_tracker_idx]->goTo(mrs_msgs::Vec4Request::ConstPtr(new mrs_msgs::Vec4Request(req_out)));

    if (tracker_response != mrs_msgs::Vec4Response::Ptr()) {
      res = *tracker_response;
    } else {
      sprintf((char *)&message, "The tracker '%s' does not implement 'goto'!", tracker_names[active_tracker_idx].c_str());
      res.message = message;
      res.success = false;
    }
  }
  mutex_tracker_list.unlock();

  return true;
}

bool ControlManager::callbackGotoRelative(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res) {

  mrs_msgs::Vec4Response::ConstPtr tracker_response;
  char                             message[100];

  mrs_msgs::Vec4Request req_out;
  req_out.goal = req.goal;

  mutex_tracker_list.lock();
  {
    tracker_response = tracker_list[active_tracker_idx]->goToRelative(mrs_msgs::Vec4Request::ConstPtr(new mrs_msgs::Vec4Request(req_out)));

    if (tracker_response != mrs_msgs::Vec4Response::Ptr()) {
      res = *tracker_response;
    } else {
      sprintf((char *)&message, "The tracker '%s' does not implement 'goto_relative'!", tracker_names[active_tracker_idx].c_str());
      res.message = message;
      res.success = false;
    }
  }
  mutex_tracker_list.unlock();

  return true;
}

//}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mav_manager::ControlManager, nodelet::Nodelet)
