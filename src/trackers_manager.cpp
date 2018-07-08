#include <ros/package.h>
#include <ros/ros.h>

#include <mrs_msgs/SwitchTracker.h>
#include <mrs_mav_manager/Controller.h>
#include <mrs_mav_manager/Tracker.h>

#include <pluginlib/class_loader.h>

#include <nodelet/loader.h>
#include <nodelet/nodelet.h>

#include <mavros_msgs/AttitudeTarget.h>

#include <mutex>

#include <tf/transform_datatypes.h>

namespace mrs_mav_manager
{

class TrackersManager : public nodelet::Nodelet {

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

  ros::Subscriber    subscriber_odometry;
  nav_msgs::Odometry odometry;
  std::mutex         mutex_odometry;
  bool               got_odometry = false;

  int active_tracker_idx    = 0;
  int active_controller_idx = 0;

  ros::Publisher     publisher_attitude_cmd;
  ros::Publisher     publisher_cmd_pose;
  ros::ServiceServer service_switch_tracker;

  mrs_msgs::PositionCommand::ConstPtr last_position_cmd_;

private:
  void callbackOdometry(const nav_msgs::OdometryConstPtr &msg);

  bool callbackSwitchTracker(mrs_msgs::SwitchTracker::Request &req, mrs_msgs::SwitchTracker::Response &res);
};

//{ onInit

void TrackersManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getPrivateNodeHandle();

  ros::Time::waitForValid();

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_odometry = nh_.subscribe("odometry_in", 1, &TrackersManager::callbackOdometry, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  publisher_attitude_cmd = nh_.advertise<mavros_msgs::AttitudeTarget>("attitude_cmd_out", 1);
  publisher_cmd_pose     = nh_.advertise<nav_msgs::Odometry>("cmd_pose", 1);

  // --------------------------------------------------------------
  // |                          services                          |
  // --------------------------------------------------------------

  service_switch_tracker = nh_.advertiseService("switch_tracker", &TrackersManager::callbackSwitchTracker, this);

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
      ROS_INFO("Trying to load: %s", tracker_name.c_str());
      tracker_list.push_back(tracker_loader->createInstance(tracker_name.c_str()));
      ROS_INFO("%d: %s", (int)i, tracker_name.c_str());
    }
    catch (pluginlib::CreateClassException &ex1) {
      ROS_ERROR("CreateClassException for tracker %s", tracker_name.c_str());
      ROS_INFO("Error: %s", ex1.what());
      return;
    }
    catch (pluginlib::PluginlibException &ex) {
      ROS_ERROR("PluginlibException for tracker %s", tracker_name.c_str());
      ROS_INFO("Error: %s", ex.what());
      ROS_ERROR("End of PluginlibException");
      return;
    }
  }

  ROS_INFO("Trackers are loaded");

  for (int i = 0; i < tracker_list.size(); i++) {

    try {
      ROS_INFO("Initializing tracker %d: %s", i, tracker_names[i].c_str());
      (*tracker_list[i]).Initialize(nh_);
    }
    catch (std::runtime_error &ex) {
      ROS_ERROR("Exception caught during tracker initialization: %s", ex.what());
    }
  }

  ROS_INFO("Trackers are activated");

  // --------------------------------------------------------------
  // |                      load controllers                      |
  // --------------------------------------------------------------

  nh_.getParam("controllers", controller_names);

  controller_loader = new pluginlib::ClassLoader<mrs_mav_manager::Controller>("mrs_mav_manager", "mrs_mav_manager::Controller");

  for (unsigned long i = 0; i < controller_names.size(); i++) {

    std::string controller_name = controller_names[i];

    try {
      ROS_INFO("Trying to load: %s", controller_name.c_str());
      controller_list.push_back(controller_loader->createInstance(controller_name.c_str()));
      ROS_INFO("%d: %s", (int)i, controller_name.c_str());
    }
    catch (pluginlib::CreateClassException &ex1) {
      ROS_ERROR("CreateClassException for controller %s", controller_name.c_str());
      ROS_INFO("Error: %s", ex1.what());
      return;
    }
    catch (pluginlib::PluginlibException &ex) {
      ROS_ERROR("PluginlibException for controller %s", controller_name.c_str());
      ROS_INFO("Error: %s", ex.what());
      ROS_ERROR("End of PluginlibException");
      return;
    }
  }

  ROS_INFO("Controllers are loaded");

  for (int i = 0; i < controller_list.size(); i++) {

    try {
      ROS_INFO("Initializing controller %d: %s", i, controller_names[i].c_str());
      (*controller_list[i]).Initialize(nh_);
    }
    catch (std::runtime_error &ex) {
      ROS_ERROR("Exception caught during controller initialization: %s", ex.what());
    }
  }

  ROS_INFO("Controllers are initialized");

  // --------------------------------------------------------------
  // |           active the first controller on the list          |
  // --------------------------------------------------------------

  ROS_INFO("Activating the first controller on the list (%s)", controller_names[0].c_str());

  (*controller_list[active_controller_idx]).Activate();
}

//}

//{ callbackOdometry

void TrackersManager::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

  got_odometry = true;

  mutex_odometry.lock();
  { odometry = *msg; }
  mutex_odometry.unlock();

  // --------------------------------------------------------------
  // |                     Update the trackers                    |
  // --------------------------------------------------------------

  nav_msgs::Odometry::ConstPtr        odometry_const_ptr(new nav_msgs::Odometry(odometry));
  mrs_msgs::PositionCommand::ConstPtr position_cmd_;

  try {

    // for each tracker
    for (int i = 0; i < tracker_list.size(); i++) {

      if (i == active_tracker_idx) {
        // if it is the active one, update and retrieve the command
        position_cmd_ = (*tracker_list[i]).update(odometry_const_ptr);
      } else {
        // if it is not the active one, just update without retrieving the commadn
        (*tracker_list[i]).update(odometry_const_ptr);
      }
    }

    if (mrs_msgs::PositionCommand::Ptr() != position_cmd_) {

      last_position_cmd_ = position_cmd_;

    } else if (active_tracker_idx > 0) {

      ROS_WARN_THROTTLE(1, "The tracker %s return empty command!", tracker_names[active_tracker_idx].c_str());

      // TODO: switch to failsave tracker, or stop outputting commands
      return;  // TODO figure out what to do

    } else if (active_tracker_idx == 0) {

      last_position_cmd_ = position_cmd_;
    }
  }
  catch (std::runtime_error &exrun) {
    ROS_INFO("Exception while updating trackers.");
    ROS_ERROR("Exeption: %s", exrun.what());
  }

  tf::Quaternion desired_orientation;

  // publish the cmd_pose topic
  if (last_position_cmd_ != mrs_msgs::PositionCommand::Ptr()) {

    nav_msgs::Odometry cmd_pose;
    cmd_pose.header.stamp       = ros::Time::now();
    cmd_pose.header.frame_id    = "local_origin";
    cmd_pose.pose.pose.position = position_cmd_->position;
    desired_orientation = tf::createQuaternionFromRPY(0, 0, position_cmd_->yaw);
    desired_orientation.normalize();
    quaternionTFToMsg(desired_orientation, cmd_pose.pose.pose.orientation);
    publisher_cmd_pose.publish(cmd_pose);
  }

  // --------------------------------------------------------------
  // |                   Update the controllers                   |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand::ConstPtr attitude_cmd_;

  if (last_position_cmd_ != mrs_msgs::PositionCommand::Ptr()) {

    try {
      /* ROS_INFO("Calling the '%s' controller", controller_names[active_controller_idx].c_str()); */
      attitude_cmd_ = (*controller_list[active_controller_idx]).update(odometry_const_ptr, last_position_cmd_);
    }
    catch (std::runtime_error &exrun) {
      ROS_INFO("Exception while updating the active controller.");
      ROS_ERROR("Exeption: %s", exrun.what());
    }
  }

  // --------------------------------------------------------------
  // |                 Publish the control command                |
  // --------------------------------------------------------------

  mavros_msgs::AttitudeTarget attitude_target;

  if (attitude_cmd_ == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN_THROTTLE(1.0, "TrackerManager: the controller (%s) returned nil command! publishing zeros...", controller_names[active_controller_idx].c_str());

    // convert the RPY to quaternion
    desired_orientation = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);
    desired_orientation.normalize();
    quaternionTFToMsg(desired_orientation, attitude_target.orientation);

    attitude_target.thrust = 0.0;

  } else {

    // convert the RPY to quaternion
    desired_orientation = tf::createQuaternionFromRPY(attitude_cmd_->roll, attitude_cmd_->pitch, attitude_cmd_->yaw);
    desired_orientation.normalize();
    quaternionTFToMsg(desired_orientation, attitude_target.orientation);

    attitude_target.thrust = attitude_cmd_->thrust;
  }

  attitude_target.header.stamp    = ros::Time::now();
  attitude_target.header.frame_id = "local_origin";

  publisher_attitude_cmd.publish(attitude_target);
}

//}

//{ callbackSwitchTracker

bool TrackersManager::callbackSwitchTracker(mrs_msgs::SwitchTracker::Request &req, mrs_msgs::SwitchTracker::Response &res) {

  char message[50];

  if (!got_odometry) {

    sprintf((char *)&message, "Can't switch tracker, missing odometry!");
    ROS_ERROR("%s", message);
    res.success = false;
    res.message = message;
    return true;
  }

  int new_tracker_idx = -1;

  for (int i = 0; i < tracker_names.size(); i++) {
    if (req.tracker.compare(tracker_names[i]) == 0) {
      new_tracker_idx = i;
    }
  }

  // check if the tracker exists
  if (new_tracker_idx < 0) {

    sprintf((char *)&message, "The tracker %s does not exist!", req.tracker.c_str());
    ROS_ERROR("%s", message);
    res.success = false;
    res.message = message;
    return true;
  }

  try {

    ROS_INFO("Activating tracker %s", tracker_names[new_tracker_idx].c_str());
    (*tracker_list[new_tracker_idx]).Activate(last_position_cmd_);
    ROS_INFO("Tracker %s has been activated", req.tracker.c_str());

    // super important, switch which the active tracker idx
    try {
      (*tracker_list[active_tracker_idx]).Deactivate();
    }
    catch (std::runtime_error &exrun) {
      ROS_ERROR("Could not deactivate %s", tracker_names[active_tracker_idx].c_str());
    }

    active_tracker_idx = new_tracker_idx;
  }
  catch (std::runtime_error &exrun) {
    ROS_INFO("Error during activation of %s tracker", req.tracker.c_str());
    ROS_INFO("Exeption: %s", exrun.what());
  }

  return true;
}

//}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mav_manager::TrackersManager, nodelet::Nodelet)
