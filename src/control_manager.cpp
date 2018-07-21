#include <mrs_mav_manager/ControlManager.h>

namespace mrs_mav_manager
{

//{ onInit()

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
  service_hover             = nh_.advertiseService("hover", &ControlManager::callbackHover, this);
  service_motors            = nh_.advertiseService("motors", &ControlManager::callbackMotors, this);

  // --------------------------------------------------------------
  // |                        load trackers                       |
  // --------------------------------------------------------------

  nh_.getParam("trackers", tracker_names);
  nh_.getParam("null_tracker", null_tracker_name_);
  nh_.getParam("hover_tracker", hover_tracker_name_);
  tracker_names.insert(tracker_names.begin(), null_tracker_name_);

  tracker_loader = new pluginlib::ClassLoader<mrs_mav_manager::Tracker>("mrs_mav_manager", "mrs_mav_manager::Tracker");

  for (unsigned long i = 0; i < tracker_names.size(); i++) {

    std::string tracker_name = tracker_names[i];

    if (hover_tracker_name_.compare(tracker_name) == 0) {
      hover_tracker_idx = i;
    }

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
      tracker_list[i]->initialize(nh_);
    }
    catch (std::runtime_error &ex) {
      ROS_ERROR("[ControlManager]: Exception caught during tracker initialization: %s", ex.what());
    }
  }

  // --------------------------------------------------------------
  // |                           params                           |
  // --------------------------------------------------------------

  nh_.param("safety/max_tilt_angle", max_tilt_angle_, -1.0);
  nh_.param("safety/max_control_error", max_control_error_, -1.0);

  if (max_tilt_angle_ < 0) {
    ROS_ERROR("[ControlManager]: max_tilt_angle was not specified!");
    ros::shutdown();
  }

  if (max_control_error_ < 0) {
    ROS_ERROR("[ControlManager]: max_control_error_ was not specified!");
    ros::shutdown();
  }

  max_tilt_angle_ = (max_tilt_angle_ / 180.0) * 3.141592;

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

  motors = false;

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  status_timer = nh_.createTimer(ros::Rate(10), &ControlManager::statusTimer, this);
  safety_timer = nh_.createTimer(ros::Rate(100), &ControlManager::safetyTimer, this);

  ROS_INFO("[ControlManager]: initialized");
}

//}

//{ callbackOdometry()

void ControlManager::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

  mutex_odometry.lock();
  {
    odometry = *msg;

    odometry_x = odometry.pose.pose.position.x;
    odometry_y = odometry.pose.pose.position.y;
    odometry_z = odometry.pose.pose.position.z;

    // calculate the euler angles
    tf::Quaternion quaternion_odometry;
    quaternionMsgToTF(odometry.pose.pose.orientation, quaternion_odometry);
    tf::Matrix3x3 m(quaternion_odometry);
    m.getRPY(odometry_roll, odometry_pitch, odometry_yaw);
  }
  mutex_odometry.unlock();

  got_odometry = true;

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
  // |                   Update the controller                    |
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
  if (active_tracker_idx == 0 || !motors) {

    if (!motors) {
      ROS_WARN_THROTTLE(1.0, "[ControlManager]: motors are off");
    } else {
      ROS_WARN_THROTTLE(1.0, "[ControlManager]: NullTracker is active, publishing zeros...");
    }

    desired_orientation = tf::createQuaternionFromRPY(odometry_roll, odometry_pitch, odometry_yaw);
    desired_orientation.normalize();
    quaternionTFToMsg(desired_orientation, attitude_target.orientation);

    attitude_target.thrust = 0.0;

    attitude_target.header.stamp    = ros::Time::now();
    attitude_target.header.frame_id = "local_origin";

    publisher_attitude_cmd.publish(attitude_target);

  } else if (active_tracker_idx > 0 && controller_output_cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN_THROTTLE(1.0, "[ControlManager]: the controller (%s) returned nil command! Not publishing anything...",
                      controller_names[active_controller_idx].c_str());

    // convert the RPY to quaternion
    desired_orientation = tf::createQuaternionFromRPY(odometry_roll, odometry_pitch, odometry_yaw);
    desired_orientation.normalize();
    quaternionTFToMsg(desired_orientation, attitude_target.orientation);

    attitude_target.thrust = 0.0;

    attitude_target.header.stamp    = ros::Time::now();
    attitude_target.header.frame_id = "local_origin";

    publisher_attitude_cmd.publish(attitude_target);

  } else if (controller_output_cmd != mrs_msgs::AttitudeCommand::Ptr()) {

    last_attitude_cmd = controller_output_cmd;

    // convert the RPY to quaternion
    desired_orientation = tf::createQuaternionFromRPY(controller_output_cmd->roll, controller_output_cmd->pitch, controller_output_cmd->yaw);
    desired_orientation.normalize();
    quaternionTFToMsg(desired_orientation, attitude_target.orientation);

    attitude_target.thrust = controller_output_cmd->thrust;

    attitude_target.header.stamp    = ros::Time::now();
    attitude_target.header.frame_id = "local_origin";

    publisher_attitude_cmd.publish(attitude_target);
  }
}

//}

//{ callbackSwitchTracker()

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
      tracker_list[active_tracker_idx]->deactivate();
      active_tracker_idx = new_tracker_idx;
    }
    catch (std::runtime_error &exrun) {
      ROS_ERROR("[ControlManager]: Could not deactivate tracker %s", tracker_names[active_tracker_idx].c_str());
    }
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
      controller_list[active_controller_idx]->deactivate();
      active_controller_idx = new_controller_idx;
    }
    catch (std::runtime_error &exrun) {
      ROS_ERROR("[ControlManager]: Could not deactivate controller %s", controller_names[active_controller_idx].c_str());
    }
  }
  catch (std::runtime_error &exrun) {
    ROS_ERROR("[ControlManager]: Error during activation of controller %s", req.controller.c_str());
    ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
  }

  res.message = message;
  return true;
}

//}

//{ statusTimer()

void ControlManager::statusTimer(const ros::TimerEvent &event) {

  // --------------------------------------------------------------
  // |                publishing the tracker status               |
  // --------------------------------------------------------------

  mrs_msgs::TrackerStatus::Ptr tracker_status_ptr;
  mrs_msgs::TrackerStatus      tracker_status;

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
}

//}

//{ safetyTimer()

void ControlManager::safetyTimer(const ros::TimerEvent &event) {

  if (!got_odometry || active_tracker_idx <= 0) {
    return;
  }

  if (!(last_position_cmd != mrs_msgs::PositionCommand::Ptr() && last_attitude_cmd != mrs_msgs::AttitudeCommand::Ptr())) {
    return;
  }

  char message[100];

  mutex_odometry.lock();
  mutex_tracker_list.lock();
  {
    double position_error_x = last_position_cmd->position.x - odometry_x;
    double position_error_y = last_position_cmd->position.y - odometry_y;
    double position_error_z = last_position_cmd->position.z - odometry_z;

    double control_error = sqrt(pow(position_error_x, 2) + pow(position_error_y, 2) + pow(position_error_z, 2));

    if (odometry_pitch > max_tilt_angle_ || odometry_roll > max_tilt_angle_ || control_error > max_control_error_) {

      // check if the controller is not active
      if (hover_tracker_idx != active_tracker_idx) {

        ROS_ERROR("[ControlManager]: Activating safety hover: max_tilt_angle_=%f, control_error=%f", max_tilt_angle_, control_error);

        try {

          ROS_INFO("[ControlManager]: Activating tracker %s", tracker_names[hover_tracker_idx].c_str());
          tracker_list[hover_tracker_idx]->activate(last_position_cmd);
          sprintf((char *)&message, "Tracker %s has been activated", hover_tracker_name_.c_str());
          ROS_INFO("[ControlManager]: %s", message);

          // super important, switch which the active tracker idx
          try {
            tracker_list[active_tracker_idx]->deactivate();
            active_tracker_idx = hover_tracker_idx;
          }
          catch (std::runtime_error &exrun) {
            ROS_ERROR("[ControlManager]: Could not deactivate tracker %s", tracker_names[active_tracker_idx].c_str());
          }
        }
        catch (std::runtime_error &exrun) {
          ROS_ERROR("[ControlManager]: Error during activation of tracker %s", hover_tracker_name_.c_str());
          ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
        }
      }
    }
  }
  mutex_tracker_list.unlock();
  mutex_odometry.unlock();
}

//}

//{ callbackGoto()

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

//}

//{ callbackGotoRelative()

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

//{ callbackHover()

bool ControlManager::callbackHover(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  std_srvs::TriggerResponse::ConstPtr tracker_response;
  char                                message[100];

  std_srvs::TriggerRequest req_out;

  mutex_tracker_list.lock();
  {
    tracker_response = tracker_list[active_tracker_idx]->hover(std_srvs::TriggerRequest::ConstPtr(new std_srvs::TriggerRequest(req_out)));

    if (tracker_response != std_srvs::TriggerResponse::Ptr()) {
      res = *tracker_response;
    } else {
      sprintf((char *)&message, "The tracker '%s' does not implement 'hover'!", tracker_names[active_tracker_idx].c_str());
      res.message = message;
      res.success = false;
    }
  }
  mutex_tracker_list.unlock();

  return true;
}

//}

//{ callbackMotors()

bool ControlManager::callbackMotors(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  motors = req.data;

  char message[100];
  sprintf((char *)&message, "Motors: %s", motors ? "ON" : "OFF");
  res.message = message;
  res.success = true;

  ROS_INFO("[ControlManager]: %s", message);

  return true;
}

//}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mav_manager::ControlManager, nodelet::Nodelet)
