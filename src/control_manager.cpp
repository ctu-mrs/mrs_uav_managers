/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_msgs/String.h>
#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <mrs_msgs/TrackerStatus.h>
#include <mrs_msgs/ControllerStatus.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ObstacleSectors.h>
#include <mrs_msgs/BoolStamped.h>

#include <mrs_lib/ConvexPolygon.h>
#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Utils.h>

#include <sensor_msgs/Joy.h>

#include <mrs_uav_manager/Controller.h>
#include <mrs_uav_manager/Tracker.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <std_srvs/SetBool.h>

#include <pluginlib/class_loader.h>

#include <nodelet/loader.h>

#include <mutex>
#include <eigen3/Eigen/Eigen>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//}

#define STRING_EQUAL 0
#define TAU 2 * M_PI
#define PWM_MIDDLE 1500.0
#define REF_X 0
#define REF_Y 1
#define REF_Z 2
#define REF_YAW 3

namespace mrs_uav_manager
{

namespace control_manager
{

/* //{ class ControlManager */

// state machine
typedef enum
{

  IDLE_STATE,
  LANDING_STATE,

} LandingStates_t;

const char *state_names[2] = {

    "IDLING", "LANDING"};

// state machine
typedef enum
{

  FCU_FRAME,
  RELATIVE_FRAME,
  ABSOLUTE_FRAME

} ReferenceFrameType_t;

/* class ControllerParams() //{ */

class ControllerParams {

public:
  ControllerParams(std::string address, double eland_threshold, double failsafe_threshold);

public:
  double      failsafe_threshold;
  double      eland_threshold;
  std::string address;
};

ControllerParams::ControllerParams(std::string address, double eland_threshold, double failsafe_threshold) {

  this->eland_threshold    = eland_threshold;
  this->failsafe_threshold = failsafe_threshold;
  this->address            = address;
}

//}

/* class TrackerParams() //{ */

class TrackerParams {

public:
  TrackerParams(std::string address);

public:
  std::string address;
};

TrackerParams::TrackerParams(std::string address) {

  this->address = address;
}

//}

class ControlManager : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized = false;
  std::string     uav_name_;

private:
private:
  pluginlib::ClassLoader<mrs_uav_manager::Tracker> *   tracker_loader;
  pluginlib::ClassLoader<mrs_uav_manager::Controller> *controller_loader;

  std::vector<std::string>             tracker_names;
  std::map<std::string, TrackerParams> trackers_;

  std::vector<std::string>                controller_names;
  std::map<std::string, ControllerParams> controllers_;

  std::vector<boost::shared_ptr<mrs_uav_manager::Tracker>>    tracker_list;
  std::vector<boost::shared_ptr<mrs_uav_manager::Controller>> controller_list;

  std::string null_tracker_name_;
  std::string ehover_tracker_name_;

  bool        joystick_enabled_ = false;
  std::string joystick_tracker_name_;
  std::string joystick_controller_name_;
  std::string joystick_fallback_tracker_name_;
  std::string joystick_fallback_controller_name_;

  std::string failsafe_controller_name_;
  std::string eland_controller_name_;

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
  ros::Time          odometry_last_time;
  double             odometry_max_missing_time_;

  ros::Subscriber    subscriber_pixhawk_odometry;
  nav_msgs::Odometry pixhawk_odometry;
  double             pixhawk_odometry_x;
  double             pixhawk_odometry_y;
  double             pixhawk_odometry_z;
  double             pixhawk_odometry_yaw;
  double             pixhawk_odometry_roll;
  double             pixhawk_odometry_pitch;
  std::mutex         mutex_pixhawk_odometry;
  bool               got_pixhawk_odometry = false;

  ros::Subscriber subscriber_max_height;
  double          max_height;
  bool            got_max_height = false;
  std::mutex      mutex_max_height;
  std::mutex      mutex_min_height;

  int active_tracker_idx               = 0;
  int active_controller_idx            = 0;
  int ehover_tracker_idx               = 0;
  int joystick_tracker_idx             = 0;
  int joystick_controller_idx          = 0;
  int failsafe_controller_idx          = 0;
  int joystick_fallback_controller_idx = 0;
  int joystick_fallback_tracker_idx    = 0;
  int null_tracker_idx                 = 0;
  int eland_controller_idx             = 0;

  void switchMotors(bool in);
  bool motors = false;

  double min_thrust_null_tracker_ = 0.0;

  int status_timer_rate_   = 0;
  int safety_timer_rate_   = 0;
  int elanding_timer_rate_ = 0;
  int failsafe_timer_rate_ = 0;
  int bumper_timer_rate_   = 0;

  ros::Publisher publisher_control_output;
  ros::Publisher publisher_position_cmd;
  ros::Publisher publisher_attitude_cmd;
  ros::Publisher publisher_thrust_force;
  ros::Publisher publisher_cmd_odom;
  ros::Publisher publisher_target_attitude;
  ros::Publisher publisher_tracker_status;
  ros::Publisher publisher_controller_status;
  ros::Publisher publisher_motors;
  ros::Publisher publisher_tilt_error;
  ros::Publisher publisher_mass_estimate;
  ros::Publisher publisher_control_error;
  ros::Publisher publisher_rviz;

  ros::ServiceServer service_server_switch_tracker;
  ros::ServiceServer service_server_switch_controller;
  ros::ServiceServer service_server_hover;
  ros::ServiceServer service_server_ehover;
  ros::ServiceServer service_server_failsafe;
  ros::ServiceServer service_server_failsafe_escalating;

  ros::ServiceServer service_server_motors;
  ros::ServiceServer service_server_arm;
  ros::ServiceServer service_server_enable_callbacks;
  ros::ServiceServer service_server_set_constraints;
  ros::ServiceServer service_server_use_joystick;

  ros::ServiceServer service_server_goto;
  ros::ServiceServer service_server_goto_fcu;
  ros::ServiceServer service_server_goto_relative;
  ros::ServiceServer service_server_goto_altitude;
  ros::ServiceServer service_server_set_yaw;
  ros::ServiceServer service_server_set_yaw_relative;

  ros::ServiceServer service_server_emergency_goto;
  ros::ServiceServer service_server_pirouette;
  ros::ServiceServer service_server_eland;

  ros::ServiceClient service_client_arm;
  ros::ServiceClient service_client_eland;
  ros::ServiceClient service_client_shutdown;

  ros::Subscriber subscriber_goto;
  ros::Subscriber subscriber_goto_fcu;
  ros::Subscriber subscriber_goto_relative;
  ros::Subscriber subscriber_goto_altitude;
  ros::Subscriber subscriber_set_yaw;
  ros::Subscriber subscriber_set_yaw_relative;

  mrs_msgs::PositionCommand::ConstPtr last_position_cmd;
  std::mutex                          mutex_last_position_cmd;

  mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd;
  std::mutex                          mutex_last_attitude_cmd;
  ros::Time                           controller_tracker_switch_time;
  std::mutex                          mutex_controller_tracker_switch_time;

  void shutdown();
  void setCallbacks(bool in);

private:
  ros::Subscriber    subscriber_mavros_state;
  mavros_msgs::State mavros_state;
  std::mutex         mutex_mavros_state;
  bool               got_mavros_state = false;
  bool               offboard_mode    = false;
  bool               armed            = false;

private:
  ros::Subscriber      subscriber_rc;
  mavros_msgs::RCIn    rc_channels;
  std::mutex           mutex_rc_channels;
  bool                 got_rc_channels = false;
  std::list<ros::Time> rc_channel_switch_time;
  std::mutex           mutex_rc_channel_switch_time;

  double rc_channel_pitch_, rc_channel_roll_, rc_channel_yaw_, rc_channel_thrust_;

private:
  void updateTrackers(void);
  void updateControllers(nav_msgs::Odometry odom_for_control);
  void publish(void);

private:
  mrs_uav_manager::MotorParams motor_params_;
  double                       g_;

private:
  double tilt_limit_eland_;
  double tilt_limit_disarm_;

  double failsafe_threshold_;
  double eland_threshold_;

private:
  bool       tilt_error_failsafe_enabled_ = false;
  double     tilt_error_threshold_;
  double     tilt_error_failsafe_min_height_;
  double     tilt_error;
  double     position_error_x_, position_error_y_, position_error_z_;
  double     velocity_error_x_, velocity_error_y_, velocity_error_z_;
  std::mutex mutex_tilt_error;
  std::mutex mutex_control_error;

private:
  mrs_lib::ConvexPolygon *      safety_area_polygon;
  bool                          use_safety_area_;
  double                        min_height;
  mrs_uav_manager::SafetyArea_t safety_area;

  bool   isPointInSafetyArea2d(const double x, const double y);
  bool   isPointInSafetyArea3d(const double x, const double y, const double z);
  double getMinHeight(void);
  double getMaxHeight(void);

private:
  void callbackOdometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackPixhawkOdometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackMaxHeight(const mrs_msgs::Float64StampedConstPtr &msg);

  bool callbackSwitchTracker(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);
  bool callbackSwitchController(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);

  bool callbackGoToService(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res);
  bool callbackGoToFcuService(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res);
  bool callbackGoToRelativeService(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res);
  bool callbackGoToAltitudeService(mrs_msgs::Vec1::Request &req, mrs_msgs::Vec1::Response &res);
  bool callbackSetYawService(mrs_msgs::Vec1::Request &req, mrs_msgs::Vec1::Response &res);
  bool callbackSetYawRelativeService(mrs_msgs::Vec1::Request &req, mrs_msgs::Vec1::Response &res);

  void callbackGoToTopic(const mrs_msgs::TrackerPointStampedConstPtr &msg);
  void callbackGoToFcuTopic(const mrs_msgs::TrackerPointStampedConstPtr &msg);
  void callbackGoToRelativeTopic(const mrs_msgs::TrackerPointStampedConstPtr &msg);
  void callbackGoToAltitudeTopic(const std_msgs::Float64ConstPtr &msg);
  void callbackSetYawTopic(const std_msgs::Float64ConstPtr &msg);
  void callbackSetYawRelativeTopic(const std_msgs::Float64ConstPtr &msg);

  bool callbackEmergencyGoToService(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res);

  void callbackMavrosState(const mavros_msgs::StateConstPtr &msg);
  void callbackRC(const mavros_msgs::RCInConstPtr &msg);
  bool isOffboard(void);

  bool ehover(std::string &message_out);
  bool hover(std::string &message_out);
  bool eland(std::string &message_out);
  bool failsafe();
  bool arming(bool input);

  bool callbackHoverService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackEHoverService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackFailsafe(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackFailsafeEscalating(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackEland(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool callbackMotors(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackArm(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  bool callbackEnableCallbacks(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackSetConstraints(mrs_msgs::TrackerConstraints::Request &req, mrs_msgs::TrackerConstraints::Response &res);

private:
  Eigen::Vector2d rotateVector(const Eigen::Vector2d vector_in, double angle);

private:
  bool callbacks_enabled = true;

private:
  ros::Timer status_timer;
  void       statusTimer(const ros::TimerEvent &event);

private:
  ros::Timer failsafe_timer;
  void       failsafeTimer(const ros::TimerEvent &event);
  bool       failsafe_triggered = false;

private:
  ros::Timer elanding_timer;
  void       elandingTimer(const ros::TimerEvent &event);
  bool       eland_triggered = false;

private:
  ros::Timer safety_timer;
  void       safetyTimer(const ros::TimerEvent &event);
  bool       running_safety_timer = false;
  double     reseting_odometry    = false;

private:
  ros::Timer pirouette_timer;
  void       pirouetteTimer(const ros::TimerEvent &event);

private:
  double    escalating_failsafe_timeout_;
  ros::Time escalating_failsafe_time;

private:
  ros::Timer bumper_timer;
  void       bumperTimer(const ros::TimerEvent &event);

private:
  ros::Subscriber subscriber_bumper;
  void            callbackBumper(const mrs_msgs::ObstacleSectorsConstPtr &msg);
  bool            got_bumper = false;

  mrs_msgs::ObstacleSectors bumper_data;
  std::mutex                mutex_bumper;

  bool bumper_enabled_           = false;
  bool bumper_hugging_enabled_   = false;
  bool bumper_repulsion_enabled_ = false;
  bool repulsing                 = false;
  uint repulsing_from;

  double bumper_horizontal_distance_;
  double bumper_vertical_distance_;

  double bumper_repulsion_horizontal_distance_;
  double bumper_repulsion_horizontal_offset_;
  double bumper_repulsion_vertical_distance_;
  double bumper_repulsion_vertical_offset_;

  bool bumperValidatePoint(double &x, double &y, double &z, ReferenceFrameType_t frame);
  int  bumperGetSectorId(const double x, const double y, const double z);
  bool bumperPushFromObstacle(void);

  Eigen::Vector3d local2fcu(const Eigen::Vector3d in);
  Eigen::Vector3d fcu2local(const Eigen::Vector3d in);

private:
  bool rc_eland_enabled_ = false;
  int  rc_eland_channel_;
  int  rc_eland_threshold_;
  bool rc_eland_triggered = false;

private:
  std::mutex mutex_joystick;

  ros::Subscriber  subscriber_joystick;
  void             callbackJoystick(const sensor_msgs::JoyConstPtr &msg);
  sensor_msgs::Joy joystick_data;
  bool             callbackUseJoystick([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  // joystick buttons
  int _channel_A_, _channel_B_, _channel_X_, _channel_Y_, _channel_start_, _channel_back_, _channel_LT_, _channel_RT_, _channel_L_joy_, _channel_R_joy_;

  // channel numbers and channel multipliers
  int _channel_pitch_, _channel_roll_, _channel_yaw_, _channel_thrust_;
  int _channel_mult_pitch_, _channel_mult_roll_, _channel_mult_yaw_, _channel_mult_thrust_;

  ros::Timer joystick_timer;
  void       joystickTimer(const ros::TimerEvent &event);
  double     joystick_timer_rate_;

  ros::Time joystick_tracker_press_time;
  bool      joytracker_start_pressed = false;

  ros::Time joystick_goto_press_time;
  bool      joystick_back_pressed = false;
  bool      joystick_goto_enabled = false;

  bool rc_goto_enabled_              = false;
  bool rc_goto_active_               = false;
  int  rc_joystic_channel_last_value = 0;
  int  rc_joystic_channel_;
  int  rc_joystic_n_switches_;
  int  rc_joystic_timeout_;

  bool      joystick_failsafe_pressed = false;
  ros::Time joystick_failsafe_press_time;

  bool      joystick_eland_pressed = false;
  ros::Time joystick_eland_press_time;

private:
  LandingStates_t current_state_landing  = IDLE_STATE;
  LandingStates_t previous_state_landing = IDLE_STATE;
  void            changeLandingState(LandingStates_t new_state);
  double          uav_mass_;
  double          elanding_cutoff_height_;
  double          elanding_cutoff_mass_factor_;
  double          landing_uav_mass_ = 0;

  double initial_body_disturbance_x_;
  double initial_body_disturbance_y_;

private:
  mrs_lib::Profiler *profiler;
  bool               profiler_enabled_ = false;

private:
  bool   automatic_pc_shutdown_enabled = false;
  double automatic_pc_shutdown_threshold;

private:
  bool       pirouette_enabled_ = false;
  double     pirouette_speed_;
  double     pirouette_timer_rate_;
  std::mutex mutex_pirouette_;
  double     pirouette_inital_yaw;
  double     pirouette_iterator;
  bool       callbackPirouette(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
};

//}

/* //{ onInit() */

void ControlManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  joystick_tracker_press_time    = ros::Time(0);
  joystick_failsafe_press_time   = ros::Time(0);
  joystick_eland_press_time      = ros::Time(0);
  escalating_failsafe_time       = ros::Time(0);
  controller_tracker_switch_time = ros::Time(0);

  ROS_INFO("[ControlManager]: initializing");

  last_attitude_cmd = mrs_msgs::AttitudeCommand::Ptr();
  last_position_cmd = mrs_msgs::PositionCommand::Ptr();

  // --------------------------------------------------------------
  // |                           params                           |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "ControlManager");

  param_loader.load_param("uav_name", uav_name_);

  param_loader.load_param("enable_profiler", profiler_enabled_);

  param_loader.load_param("safety/min_thrust_null_tracker", min_thrust_null_tracker_);
  param_loader.load_param("safety/ehover_tracker", ehover_tracker_name_);
  param_loader.load_param("safety/failsafe_controller", failsafe_controller_name_);
  param_loader.load_param("safety/eland_controller", eland_controller_name_);

  param_loader.load_param("safety/tilt_limit_eland", tilt_limit_eland_);
  tilt_limit_eland_ = (tilt_limit_eland_ / 180.0) * M_PI;
  param_loader.load_param("safety/tilt_limit_disarm", tilt_limit_disarm_);
  tilt_limit_disarm_ = (tilt_limit_disarm_ / 180.0) * M_PI;

  param_loader.load_param("status_timer_rate", status_timer_rate_);
  param_loader.load_param("safety/safety_timer_rate", safety_timer_rate_);
  param_loader.load_param("safety/elanding_timer_rate", elanding_timer_rate_);
  param_loader.load_param("safety/failsafe_timer_rate", failsafe_timer_rate_);

  param_loader.load_param("uav_mass", uav_mass_);
  param_loader.load_param("hover_thrust/a", motor_params_.hover_thrust_a);
  param_loader.load_param("hover_thrust/b", motor_params_.hover_thrust_b);
  param_loader.load_param("g", g_);

  param_loader.load_param("safety/elanding_cutoff_height", elanding_cutoff_height_);
  param_loader.load_param("safety/elanding_cutoff_mass_factor", elanding_cutoff_mass_factor_);

  param_loader.load_param("safety/odometry_max_missing_time", odometry_max_missing_time_);

  param_loader.load_param("safety/tilt_error_failsafe/enabled", tilt_error_failsafe_enabled_);
  param_loader.load_param("safety/tilt_error_failsafe/tilt_error_threshold", tilt_error_threshold_);
  param_loader.load_param("safety/tilt_error_failsafe/min_height", tilt_error_failsafe_min_height_);
  tilt_error_threshold_ = (tilt_error_threshold_ / 180.0) * M_PI;

  param_loader.load_param("safety/escalating_failsafe_timeout", escalating_failsafe_timeout_);

  param_loader.load_param("joystick/enabled", joystick_enabled_);
  param_loader.load_param("joystick/joystick_timer_rate", joystick_timer_rate_);
  param_loader.load_param("joystick/tracker", joystick_tracker_name_);
  param_loader.load_param("joystick/controller", joystick_controller_name_);
  param_loader.load_param("joystick/fallback/tracker", joystick_fallback_tracker_name_);
  param_loader.load_param("joystick/fallback/controller", joystick_fallback_controller_name_);

  param_loader.load_param("joystick/channels/A", _channel_A_);
  param_loader.load_param("joystick/channels/B", _channel_B_);
  param_loader.load_param("joystick/channels/X", _channel_X_);
  param_loader.load_param("joystick/channels/Y", _channel_Y_);
  param_loader.load_param("joystick/channels/start", _channel_start_);
  param_loader.load_param("joystick/channels/back", _channel_back_);
  param_loader.load_param("joystick/channels/LT", _channel_LT_);
  param_loader.load_param("joystick/channels/RT", _channel_RT_);
  param_loader.load_param("joystick/channels/L_joy", _channel_L_joy_);
  param_loader.load_param("joystick/channels/R_joy", _channel_R_joy_);

  // load channels
  param_loader.load_param("joystick/channels/pitch", _channel_pitch_);
  param_loader.load_param("joystick/channels/roll", _channel_roll_);
  param_loader.load_param("joystick/channels/yaw", _channel_yaw_);
  param_loader.load_param("joystick/channels/thrust", _channel_thrust_);

  // load channel multipliers
  param_loader.load_param("joystick/channel_multipliers/pitch", _channel_mult_pitch_);
  param_loader.load_param("joystick/channel_multipliers/roll", _channel_mult_roll_);
  param_loader.load_param("joystick/channel_multipliers/yaw", _channel_mult_yaw_);
  param_loader.load_param("joystick/channel_multipliers/thrust", _channel_mult_thrust_);

  param_loader.load_param("obstacle_bumper/enabled", bumper_enabled_);
  param_loader.load_param("obstacle_bumper/timer_rate", bumper_timer_rate_);
  param_loader.load_param("obstacle_bumper/horizontal_distance", bumper_horizontal_distance_);
  param_loader.load_param("obstacle_bumper/vertical_distance", bumper_vertical_distance_);

  param_loader.load_param("obstacle_bumper/obstacle_hugging/enabled", bumper_hugging_enabled_);

  param_loader.load_param("obstacle_bumper/repulsion/enabled", bumper_repulsion_enabled_);

  param_loader.load_param("obstacle_bumper/repulsion/horizontal_distance", bumper_repulsion_horizontal_distance_);
  param_loader.load_param("obstacle_bumper/repulsion/horizontal_offset", bumper_repulsion_horizontal_offset_);
  param_loader.load_param("obstacle_bumper/repulsion/vertical_distance", bumper_repulsion_vertical_distance_);
  param_loader.load_param("obstacle_bumper/repulsion/vertical_offset", bumper_repulsion_vertical_offset_);

  param_loader.load_param("safety/rc_eland/enabled", rc_eland_enabled_);
  param_loader.load_param("safety/rc_eland/channel_number", rc_eland_channel_);
  param_loader.load_param("safety/rc_eland/threshold", rc_eland_threshold_);

  param_loader.load_param("rc_joystick/enabled", rc_goto_enabled_);
  param_loader.load_param("rc_joystick/channel_number", rc_joystic_channel_);
  param_loader.load_param("rc_joystick/timeout", rc_joystic_timeout_);
  param_loader.load_param("rc_joystick/n_switches", rc_joystic_n_switches_);

  param_loader.load_param("rc_joystick/channels/pitch", rc_channel_pitch_);
  param_loader.load_param("rc_joystick/channels/roll", rc_channel_roll_);
  param_loader.load_param("rc_joystick/channels/yaw", rc_channel_yaw_);
  param_loader.load_param("rc_joystick/channels/thrust", rc_channel_thrust_);

  param_loader.load_param("automatic_pc_shutdown/enabled", automatic_pc_shutdown_enabled);
  param_loader.load_param("automatic_pc_shutdown/distance_threshold", automatic_pc_shutdown_threshold);

  param_loader.load_param("pirouette/speed", pirouette_speed_);
  param_loader.load_param("pirouette/timer_rate", pirouette_timer_rate_);

  // | ------------- load the body integrator values ------------ |

  param_loader.load_param("body_disturbance_x", initial_body_disturbance_x_);
  param_loader.load_param("body_disturbance_y", initial_body_disturbance_y_);

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  last_attitude_cmd = output_command;

  output_command->total_mass = uav_mass_;
  output_command->mass_difference = 0.0;

  output_command->disturbance_bx_b = initial_body_disturbance_x_;
  output_command->disturbance_by_b = initial_body_disturbance_y_;
  output_command->disturbance_wx_w = 0.0;
  output_command->disturbance_wy_w = 0.0;
  output_command->disturbance_bx_w = 0.0;
  output_command->disturbance_by_w = 0.0;

  output_command->thrust = min_thrust_null_tracker_;

  // --------------------------------------------------------------
  // |                        load trackers                       |
  // --------------------------------------------------------------

  param_loader.load_param("trackers", tracker_names);
  param_loader.load_param("null_tracker", null_tracker_name_);

  tracker_loader = new pluginlib::ClassLoader<mrs_uav_manager::Tracker>("mrs_uav_manager", "mrs_uav_manager::Tracker");

  for (unsigned long i = 0; i < tracker_names.size(); i++) {

    std::string tracker_name = tracker_names[i];

    // load the controller parameters
    std::string address;
    param_loader.load_param(tracker_name + "/address", address);

    TrackerParams new_tracker(address);
    trackers_.insert(std::pair<std::string, TrackerParams>(tracker_name, new_tracker));

    try {
      ROS_INFO("[ControlManager]: Trying to load tracker %s", new_tracker.address.c_str());
      tracker_list.push_back(tracker_loader->createInstance(new_tracker.address.c_str()));
    }
    catch (pluginlib::CreateClassException &ex1) {
      ROS_ERROR("[ControlManager]: CreateClassException for tracker %s", new_tracker.address.c_str());
      ROS_ERROR("[ControlManager]: Error: %s", ex1.what());
      ros::shutdown();
    }
    catch (pluginlib::PluginlibException &ex) {
      ROS_ERROR("[ControlManager]: PluginlibException for tracker %s", new_tracker.address.c_str());
      ROS_ERROR("[ControlManager]: Error: %s", ex.what());
      ros::shutdown();
    }
  }

  ROS_INFO("[ControlManager]: trackers were loaded");

  for (unsigned long i = 0; i < tracker_list.size(); i++) {

    std::map<std::string, TrackerParams>::iterator it;
    it = trackers_.find(tracker_names[i]);

    try {
      ROS_INFO("[ControlManager]: Initializing tracker %d: %s", (int)i, it->second.address.c_str());
      tracker_list[i]->initialize(nh_, &safety_area);
    }
    catch (std::runtime_error &ex) {
      ROS_ERROR("[ControlManager]: Exception caught during tracker initialization: %s", ex.what());
    }
  }

  ROS_INFO("[ControlManager]: trackers were activated");

  // --------------------------------------------------------------
  // |                      load controllers                      |
  // --------------------------------------------------------------

  param_loader.load_param("controllers", controller_names);

  controller_loader = new pluginlib::ClassLoader<mrs_uav_manager::Controller>("mrs_uav_manager", "mrs_uav_manager::Controller");

  // for each controller in the list
  for (unsigned long i = 0; i < controller_names.size(); i++) {

    std::string controller_name = controller_names[i];

    // load the controller parameters
    std::string address;
    double      eland_threshold, failsafe_threshold;
    param_loader.load_param(controller_name + "/address", address);
    param_loader.load_param(controller_name + "/eland_threshold", eland_threshold);
    param_loader.load_param(controller_name + "/failsafe_threshold", failsafe_threshold);

    if (eland_threshold == 0) {
      eland_threshold = 1e6;
    }

    if (failsafe_threshold == 0) {
      failsafe_threshold = 1e6;
    }

    ControllerParams new_controller(address, eland_threshold, failsafe_threshold);
    controllers_.insert(std::pair<std::string, ControllerParams>(controller_name, new_controller));

    try {
      ROS_INFO("[ControlManager]: Loading controller %s", new_controller.address.c_str());
      controller_list.push_back(controller_loader->createInstance(new_controller.address.c_str()));
    }
    catch (pluginlib::CreateClassException &ex1) {
      ROS_ERROR("[ControlManager]: CreateClassException for controller %s", new_controller.address.c_str());
      ROS_ERROR("[ControlManager]: Error: %s", ex1.what());
      ros::shutdown();
    }
    catch (pluginlib::PluginlibException &ex) {
      ROS_ERROR("[ControlManager]: PluginlibException for controller %s", new_controller.address.c_str());
      ROS_ERROR("[ControlManager]: Error: %s", ex.what());
      ros::shutdown();
    }
  }

  ROS_INFO("[ControlManager]: controllers were loaded");

  for (unsigned long i = 0; i < controller_list.size(); i++) {
    try {

      std::map<std::string, ControllerParams>::iterator it;
      it = controllers_.find(controller_names[i]);

      ROS_INFO("[ControlManager]: Initializing controller %d: %s", (int)i, it->second.address.c_str());
      controller_list[i]->initialize(nh_, motor_params_);
    }
    catch (std::runtime_error &ex) {
      ROS_ERROR("[ControlManager]: Exception caught during controller initialization: %s", ex.what());
    }
  }

  ROS_INFO("[ControlManager]: controllers were initialized");

  // --------------------------------------------------------------
  // |     check the existance of safety trackers/controllers     |
  // --------------------------------------------------------------

  // check if the hover_tracker is within the loaded trackers
  bool hover_tracker_check = false;
  for (unsigned long i = 0; i < tracker_names.size(); i++) {

    std::string tracker_name = tracker_names[i];

    if (tracker_name.compare(ehover_tracker_name_) == 0) {
      hover_tracker_check = true;
      ehover_tracker_idx  = i;
      break;
    }
  }
  if (!hover_tracker_check) {
    ROS_ERROR("[ControlManager]: the safety/hover_tracker (%s) is not within the loaded trackers", ehover_tracker_name_.c_str());
    ros::shutdown();
  }

  // check if the failsafe controller is within the loaded controllers
  bool failsafe_controller_check = false;
  for (unsigned long i = 0; i < controller_names.size(); i++) {

    std::string controller_name = controller_names[i];

    if (controller_name.compare(failsafe_controller_name_) == 0) {
      failsafe_controller_check = true;
      failsafe_controller_idx   = i;
      break;
    }
  }
  if (!failsafe_controller_check) {
    ROS_ERROR("[ControlManager]: the failsafe controller (%s) is not within the loaded controllers", failsafe_controller_name_.c_str());
    ros::shutdown();
  }

  // check if the eland controller is within the loaded controllers
  bool eland_controller_check = false;
  for (unsigned long i = 0; i < controller_names.size(); i++) {

    std::string controller_name = controller_names[i];

    if (controller_name.compare(eland_controller_name_) == 0) {
      eland_controller_check = true;
      eland_controller_idx   = i;
      break;
    }
  }
  if (!eland_controller_check) {
    ROS_ERROR("[ControlManager]: the eland controller (%s) is not within the loaded controllers", eland_controller_name_.c_str());
    ros::shutdown();
  }

  // --------------------------------------------------------------
  // |         check for the existance of the NullTracker         |
  // --------------------------------------------------------------

  // check if the hover_tracker is within the loaded trackers
  bool null_tracker_check = false;
  for (unsigned long i = 0; i < tracker_names.size(); i++) {

    std::string tracker_name = tracker_names[i];

    if (tracker_name.compare(null_tracker_name_) == 0) {
      null_tracker_check = true;
      null_tracker_idx   = i;
      break;
    }
  }
  if (!null_tracker_check) {
    ROS_ERROR("[ControlManager]: the null tracker (%s) is not within the loaded trackers", null_tracker_name_.c_str());
    ros::shutdown();
  }

  // --------------------------------------------------------------
  // |  check existance of controllers and trackers for joystick  |
  // --------------------------------------------------------------

  if (joystick_enabled_) {

    // check if the tracker for joystick control exists
    bool joystick_tracker_check = false;
    for (unsigned long i = 0; i < tracker_names.size(); i++) {

      std::string tracker_name = tracker_names[i];

      if (tracker_name.compare(joystick_tracker_name_) == 0) {
        joystick_tracker_check = true;
        joystick_tracker_idx   = i;
        break;
      }
    }
    if (!joystick_tracker_check) {
      ROS_ERROR("[ControlManager]: the joystick tracker (%s) is not within the loaded trackers", joystick_tracker_name_.c_str());
      ros::shutdown();
    }

    // check if the controller for joystick control exists
    bool joystic_controller_check = false;
    for (unsigned long i = 0; i < controller_names.size(); i++) {

      std::string controller_name = controller_names[i];

      if (controller_name.compare(joystick_controller_name_) == 0) {
        joystic_controller_check = true;
        joystick_controller_idx  = i;
        break;
      }
    }
    if (!joystic_controller_check) {
      ROS_ERROR("[ControlManager]: the failsafe controller (%s) is not within the loaded controllers", joystick_controller_name_.c_str());
      ros::shutdown();
    }

    // check if the fallback tracker for joystick control exists
    bool joystick_fallback_tracker_check = false;
    for (unsigned long i = 0; i < tracker_names.size(); i++) {

      std::string tracker_name = tracker_names[i];

      if (tracker_name.compare(joystick_fallback_tracker_name_) == 0) {
        joystick_fallback_tracker_check = true;
        joystick_fallback_tracker_idx   = i;
        break;
      }
    }
    if (!joystick_fallback_tracker_check) {
      ROS_ERROR("[ControlManager]: the joystick fallback tracker (%s) is not within the loaded trackers", joystick_fallback_tracker_name_.c_str());
      ros::shutdown();
    }

    // check if the fallback controller for joystick control exists
    bool joystic_fallback_controller_check = false;
    for (unsigned long i = 0; i < controller_names.size(); i++) {

      std::string controller_name = controller_names[i];

      if (controller_name.compare(joystick_fallback_controller_name_) == 0) {
        joystic_fallback_controller_check = true;
        joystick_fallback_controller_idx  = i;
        break;
      }
    }
    if (!joystic_fallback_controller_check) {
      ROS_ERROR("[ControlManager]: the failsafe fallback controller (%s) is not within the loaded controllers", joystick_fallback_controller_name_.c_str());
      ros::shutdown();
    }
  }

  // --------------------------------------------------------------
  // |                  activate the NullTracker                  |
  // --------------------------------------------------------------

  ROS_INFO("[ControlManager]: Activating the null tracker");

  tracker_list[null_tracker_idx]->activate(last_position_cmd);
  active_tracker_idx = null_tracker_idx;

  // --------------------------------------------------------------
  // |          activate the first controller on the list         |
  // --------------------------------------------------------------

  ROS_INFO("[ControlManager]: Activating the first controller on the list (%s)", controller_names[eland_controller_idx].c_str());

  controller_list[active_controller_idx]->activate(last_attitude_cmd);

  // update the time
  {
    std::scoped_lock lock(mutex_controller_tracker_switch_time);

    controller_tracker_switch_time = ros::Time::now();
  }

  motors = false;

  // --------------------------------------------------------------
  // |                         safety area                        |
  // --------------------------------------------------------------

  param_loader.load_param("safety_area/use_safety_area", use_safety_area_);
  param_loader.load_param("safety_area/min_height", min_height);
  param_loader.load_param("safety_area/max_height", max_height);

  if (use_safety_area_) {

    Eigen::MatrixXd safety_area_points = param_loader.load_matrix_dynamic2("safety_area/safety_area", -1, 2);

    try {

      safety_area_polygon = new mrs_lib::ConvexPolygon(safety_area_points);
    }
    catch (mrs_lib::ConvexPolygon::WrongNumberOfVertices) {

      ROS_ERROR("[ControlManager]: Exception caught. Wrong number of vertices was supplied to create the safety area.");
      ros::shutdown();
    }
    catch (mrs_lib::ConvexPolygon::PolygonNotConvexException) {

      ROS_ERROR("[ControlManager]: Exception caught. Polygon supplied to create the safety area is not convex.");
      ros::shutdown();
    }
    catch (mrs_lib::ConvexPolygon::WrongNumberOfColumns) {
      ROS_ERROR("[ControlManager]: Exception caught. Wrong number of columns was supplied to the safety area.");
      ros::shutdown();
    }
  }

  safety_area.use_safety_area       = use_safety_area_;
  safety_area.isPointInSafetyArea2d = boost::bind(&ControlManager::isPointInSafetyArea2d, this, _1, _2);
  safety_area.isPointInSafetyArea3d = boost::bind(&ControlManager::isPointInSafetyArea3d, this, _1, _2, _3);
  safety_area.getMaxHeight          = boost::bind(&ControlManager::getMaxHeight, this);
  safety_area.getMinHeight          = boost::bind(&ControlManager::getMinHeight, this);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler = new mrs_lib::Profiler(nh_, "ControlManager", profiler_enabled_);

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  publisher_control_output    = nh_.advertise<mavros_msgs::AttitudeTarget>("control_output_out", 1);
  publisher_position_cmd      = nh_.advertise<mrs_msgs::PositionCommand>("position_cmd_out", 1);
  publisher_attitude_cmd      = nh_.advertise<mrs_msgs::AttitudeCommand>("attitude_cmd_out", 1);
  publisher_thrust_force      = nh_.advertise<mrs_msgs::Float64Stamped>("thrust_force_out", 1);
  publisher_cmd_odom          = nh_.advertise<nav_msgs::Odometry>("cmd_odom_out", 1);
  publisher_target_attitude   = nh_.advertise<mavros_msgs::AttitudeTarget>("target_attitude_out", 1);
  publisher_tracker_status    = nh_.advertise<mrs_msgs::TrackerStatus>("tracker_status_out", 1);
  publisher_controller_status = nh_.advertise<mrs_msgs::ControllerStatus>("controller_status_out", 1);
  publisher_motors            = nh_.advertise<mrs_msgs::BoolStamped>("motors_out", 1);
  publisher_tilt_error        = nh_.advertise<std_msgs::Float64>("tilt_error_out", 1);
  publisher_mass_estimate     = nh_.advertise<std_msgs::Float64>("mass_estimate_out", 1);
  publisher_control_error     = nh_.advertise<nav_msgs::Odometry>("control_error_out", 1);
  publisher_rviz              = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array_out", 1);

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_odometry         = nh_.subscribe("odometry_in", 1, &ControlManager::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
  subscriber_pixhawk_odometry = nh_.subscribe("mavros_odometry_in", 1, &ControlManager::callbackPixhawkOdometry, this, ros::TransportHints().tcpNoDelay());
  odometry_last_time          = ros::Time(0);
  subscriber_max_height       = nh_.subscribe("max_height_in", 1, &ControlManager::callbackMaxHeight, this, ros::TransportHints().tcpNoDelay());
  subscriber_joystick         = nh_.subscribe("joystick_in", 1, &ControlManager::callbackJoystick, this, ros::TransportHints().tcpNoDelay());
  subscriber_bumper           = nh_.subscribe("bumper_in", 1, &ControlManager::callbackBumper, this, ros::TransportHints().tcpNoDelay());
  subscriber_mavros_state     = nh_.subscribe("mavros_state_in", 1, &ControlManager::callbackMavrosState, this, ros::TransportHints().tcpNoDelay());
  subscriber_rc               = nh_.subscribe("rc_in", 1, &ControlManager::callbackRC, this, ros::TransportHints().tcpNoDelay());

  // | -------------------- general services -------------------- |

  service_server_switch_tracker      = nh_.advertiseService("switch_tracker_in", &ControlManager::callbackSwitchTracker, this);
  service_server_switch_controller   = nh_.advertiseService("switch_controller_in", &ControlManager::callbackSwitchController, this);
  service_server_hover               = nh_.advertiseService("hover_in", &ControlManager::callbackHoverService, this);
  service_server_ehover              = nh_.advertiseService("ehover_in", &ControlManager::callbackEHoverService, this);
  service_server_failsafe            = nh_.advertiseService("failsafe_in", &ControlManager::callbackFailsafe, this);
  service_server_failsafe_escalating = nh_.advertiseService("failsafe_escalating_in", &ControlManager::callbackFailsafeEscalating, this);
  service_server_motors              = nh_.advertiseService("motors_in", &ControlManager::callbackMotors, this);
  service_server_arm                 = nh_.advertiseService("arm_in", &ControlManager::callbackArm, this);
  service_server_enable_callbacks    = nh_.advertiseService("enable_callbacks_in", &ControlManager::callbackEnableCallbacks, this);
  service_server_set_constraints     = nh_.advertiseService("set_constraints_in", &ControlManager::callbackSetConstraints, this);
  service_server_use_joystick        = nh_.advertiseService("use_joystick_in", &ControlManager::callbackUseJoystick, this);
  service_server_eland               = nh_.advertiseService("eland_in", &ControlManager::callbackEland, this);

  service_client_arm      = nh_.serviceClient<mavros_msgs::CommandBool>("arm_out");
  service_client_eland    = nh_.serviceClient<std_srvs::Trigger>("eland_out");
  service_client_shutdown = nh_.serviceClient<std_srvs::Trigger>("shutdown_out");

  // | ---------------- setpoint command services --------------- |

  service_server_goto             = nh_.advertiseService("goto_in", &ControlManager::callbackGoToService, this);
  service_server_goto_fcu         = nh_.advertiseService("goto_fcu_in", &ControlManager::callbackGoToFcuService, this);
  service_server_goto_relative    = nh_.advertiseService("goto_relative_in", &ControlManager::callbackGoToRelativeService, this);
  service_server_goto_altitude    = nh_.advertiseService("goto_altitude_in", &ControlManager::callbackGoToAltitudeService, this);
  service_server_set_yaw          = nh_.advertiseService("set_yaw_in", &ControlManager::callbackSetYawService, this);
  service_server_set_yaw_relative = nh_.advertiseService("set_yaw_relative_in", &ControlManager::callbackSetYawRelativeService, this);

  // | ----------------- setpoint command topics ---------------- |

  subscriber_goto             = nh_.subscribe("goto_in", 1, &ControlManager::callbackGoToTopic, this, ros::TransportHints().tcpNoDelay());
  subscriber_goto_fcu         = nh_.subscribe("goto_fcu_in", 1, &ControlManager::callbackGoToFcuTopic, this, ros::TransportHints().tcpNoDelay());
  subscriber_goto_relative    = nh_.subscribe("goto_relative_in", 1, &ControlManager::callbackGoToRelativeTopic, this, ros::TransportHints().tcpNoDelay());
  subscriber_goto_altitude    = nh_.subscribe("goto_altitude_in", 1, &ControlManager::callbackGoToAltitudeTopic, this, ros::TransportHints().tcpNoDelay());
  subscriber_set_yaw          = nh_.subscribe("set_yaw_in", 1, &ControlManager::callbackSetYawTopic, this, ros::TransportHints().tcpNoDelay());
  subscriber_set_yaw_relative = nh_.subscribe("set_yaw_relative_in", 1, &ControlManager::callbackSetYawRelativeTopic, this, ros::TransportHints().tcpNoDelay());

  // | --------------------- other services --------------------- |

  service_server_emergency_goto = nh_.advertiseService("emergency_goto_in", &ControlManager::callbackEmergencyGoToService, this);
  service_server_pirouette      = nh_.advertiseService("pirouette_in", &ControlManager::callbackPirouette, this);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  status_timer    = nh_.createTimer(ros::Rate(status_timer_rate_), &ControlManager::statusTimer, this);
  safety_timer    = nh_.createTimer(ros::Rate(safety_timer_rate_), &ControlManager::safetyTimer, this);
  bumper_timer    = nh_.createTimer(ros::Rate(bumper_timer_rate_), &ControlManager::bumperTimer, this);
  elanding_timer  = nh_.createTimer(ros::Rate(elanding_timer_rate_), &ControlManager::elandingTimer, this, false, false);
  failsafe_timer  = nh_.createTimer(ros::Rate(failsafe_timer_rate_), &ControlManager::failsafeTimer, this, false, false);
  pirouette_timer = nh_.createTimer(ros::Rate(pirouette_timer_rate_), &ControlManager::pirouetteTimer, this, false, false);
  joystick_timer  = nh_.createTimer(ros::Rate(joystick_timer_rate_), &ControlManager::joystickTimer, this);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[ControlManager]: Could not load all parameters!");
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[ControlManager]: initialized");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* //{ statusTimer() */

void ControlManager::statusTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("statusTimer", status_timer_rate_, 0.01, event);

  // --------------------------------------------------------------
  // |                publishing the tracker status               |
  // --------------------------------------------------------------

  {

    std::scoped_lock lock(mutex_tracker_list);

    mrs_msgs::TrackerStatus::Ptr tracker_status_ptr;
    mrs_msgs::TrackerStatus      tracker_status;

    tracker_status_ptr = tracker_list[active_tracker_idx]->getStatus();

    tracker_status = mrs_msgs::TrackerStatus(*tracker_status_ptr);

    tracker_status.tracker = tracker_names[active_tracker_idx];
    tracker_status.stamp   = ros::Time::now();

    try {
      publisher_tracker_status.publish(mrs_msgs::TrackerStatusConstPtr(new mrs_msgs::TrackerStatus(tracker_status)));
    }
    catch (...) {
      ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_tracker_status.getTopic().c_str());
    }
  }

  // --------------------------------------------------------------
  // |              publishing the controller status              |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_controller_list);

    mrs_msgs::ControllerStatus::Ptr controller_status_ptr;
    mrs_msgs::ControllerStatus      controller_status;

    controller_status_ptr = controller_list[active_controller_idx]->getStatus();

    controller_status = mrs_msgs::ControllerStatus(*controller_status_ptr);

    controller_status.controller = controller_names[active_controller_idx];
    controller_status.stamp      = ros::Time::now();

    try {
      publisher_controller_status.publish(mrs_msgs::ControllerStatusConstPtr(new mrs_msgs::ControllerStatus(controller_status)));
    }
    catch (...) {
      ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_controller_status.getTopic().c_str());
    }
  }

  // --------------------------------------------------------------
  // |                 publishing the motors state                |
  // --------------------------------------------------------------

  mrs_msgs::BoolStamped motors_out;
  motors_out.data  = motors;
  motors_out.stamp = ros::Time::now();

  try {
    publisher_motors.publish(motors_out);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", publisher_motors.getTopic().c_str());
  }

  // --------------------------------------------------------------
  // |                   publish the tilt error                   |
  // --------------------------------------------------------------
  {
    std::scoped_lock lock(mutex_tilt_error);

    std_msgs::Float64 tilt_error_out;
    tilt_error_out.data = (180.0 / M_PI) * tilt_error;

    try {
      publisher_tilt_error.publish(tilt_error_out);
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher_tilt_error.getTopic().c_str());
    }
  }

  // --------------------------------------------------------------
  // |                  publish the control error                 |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_control_error);

    nav_msgs::Odometry odom_out;

    odom_out.pose.pose.position.x = position_error_x_;
    odom_out.pose.pose.position.y = position_error_y_;
    odom_out.pose.pose.position.z = position_error_z_;

    odom_out.twist.twist.linear.x = velocity_error_x_;
    odom_out.twist.twist.linear.y = velocity_error_y_;
    odom_out.twist.twist.linear.z = velocity_error_z_;

    try {
      publisher_control_error.publish(odom_out);
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher_control_error.getTopic().c_str());
    }
  }

  // --------------------------------------------------------------
  // |                  publish the mass estimate                 |
  // --------------------------------------------------------------
  {
    std::scoped_lock lock(mutex_last_attitude_cmd);

    if (last_attitude_cmd != mrs_msgs::AttitudeCommand::Ptr()) {

      std_msgs::Float64 mass_estimate_out;
      mass_estimate_out.data = uav_mass_ + last_attitude_cmd->mass_difference;

      try {
        publisher_mass_estimate.publish(mass_estimate_out);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_mass_estimate.getTopic().c_str());
      }
    }
  }

  // --------------------------------------------------------------
  // |                  publish the rviz markers                  |
  // --------------------------------------------------------------
  {
    std::scoped_lock lock(mutex_last_attitude_cmd, mutex_odometry);

    if (last_attitude_cmd != mrs_msgs::AttitudeCommand::Ptr() && got_odometry) {

      visualization_msgs::MarkerArray msg_out;

      double id = 0;

      double multiplier = 1.0;

      Eigen::Quaterniond quat_eigen(odometry.pose.pose.orientation.w, odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y,
                                    odometry.pose.pose.orientation.z);

      Eigen::Vector3d      vec3d;
      geometry_msgs::Point point;

      /* world x disturbance //{ */
      {

        visualization_msgs::Marker marker;

        marker.header.frame_id = "local_origin";
        marker.header.stamp    = ros::Time::now();
        marker.ns              = "control_manager";
        marker.id              = id++;
        marker.type            = visualization_msgs::Marker::ARROW;
        marker.action          = visualization_msgs::Marker::ADD;

        /* position //{ */

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;

        //}

        /* orientation //{ */

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        //}

        /* origin //{ */
        point.x = odometry_x;
        point.y = odometry_y;
        point.z = odometry_z;

        marker.points.push_back(point);

        //}

        /* tip //{ */

        point.x = odometry_x + multiplier * last_attitude_cmd->disturbance_wx_w;
        point.y = odometry_y;
        point.z = odometry_z;

        marker.points.push_back(point);

        //}

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.a = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

        msg_out.markers.push_back(marker);
      }

      //}

      /* world y disturbance //{ */
      {

        visualization_msgs::Marker marker;

        marker.header.frame_id = "local_origin";
        marker.header.stamp    = ros::Time::now();
        marker.ns              = "control_manager";
        marker.id              = id++;
        marker.type            = visualization_msgs::Marker::ARROW;
        marker.action          = visualization_msgs::Marker::ADD;

        /* position //{ */

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;

        //}

        /* orientation //{ */

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        //}

        // defining points

        /* origin //{ */
        point.x = odometry_x;
        point.y = odometry_y;
        point.z = odometry_z;

        marker.points.push_back(point);

        //}

        /* tip //{ */

        point.x = odometry_x;
        point.y = odometry_y + multiplier * last_attitude_cmd->disturbance_wy_w;
        point.z = odometry_z;

        marker.points.push_back(point);

        //}

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.a = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

        msg_out.markers.push_back(marker);
      }

      //}

      /* body x disturbance //{ */
      {

        visualization_msgs::Marker marker;

        marker.header.frame_id = "local_origin";
        marker.header.stamp    = ros::Time::now();
        marker.ns              = "control_manager";
        marker.id              = id++;
        marker.type            = visualization_msgs::Marker::ARROW;
        marker.action          = visualization_msgs::Marker::ADD;

        /* position //{ */

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;

        //}

        /* orientation //{ */

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        //}

        /* origin //{ */

        point.x = odometry_x;
        point.y = odometry_y;
        point.z = odometry_z;

        marker.points.push_back(point);

        //}

        /* tip //{ */

        vec3d << multiplier * last_attitude_cmd->disturbance_bx_b, 0, 0;
        vec3d = quat_eigen * vec3d;

        point.x = odometry_x + vec3d[0];
        point.y = odometry_y + vec3d[1];
        point.z = odometry_z + vec3d[2];

        marker.points.push_back(point);

        //}

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

        msg_out.markers.push_back(marker);
      }

      //}

      /* body y disturbance //{ */
      {

        visualization_msgs::Marker marker;

        marker.header.frame_id = "local_origin";
        marker.header.stamp    = ros::Time::now();
        marker.ns              = "control_manager";
        marker.id              = id++;
        marker.type            = visualization_msgs::Marker::ARROW;
        marker.action          = visualization_msgs::Marker::ADD;

        /* position //{ */

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;

        //}

        /* orientation //{ */

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        //}

        /* origin //{ */

        point.x = odometry_x;
        point.y = odometry_y;
        point.z = odometry_z;

        marker.points.push_back(point);

        //}

        /* tip //{ */

        vec3d << 0, multiplier * last_attitude_cmd->disturbance_by_b, 0;
        vec3d = quat_eigen * vec3d;

        point.x = odometry_x + vec3d[0];
        point.y = odometry_y + vec3d[1];
        point.z = odometry_z + vec3d[2];

        marker.points.push_back(point);

        //}

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

        msg_out.markers.push_back(marker);
      }

      //}

      try {
        publisher_rviz.publish(msg_out);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_rviz.getTopic().c_str());
      }
    }
  }
}

//}

/* //{ safetyTimer() */

void ControlManager::safetyTimer(const ros::TimerEvent &event) {

  mrs_lib::ScopeUnset unset_running(running_safety_timer);

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("safetyTimer", safety_timer_rate_, 0.04, event);

  if (!got_odometry || !got_pixhawk_odometry || active_tracker_idx == null_tracker_idx) {
    return;
  }

  if (reseting_odometry) {
    ROS_WARN("[ControlManager]: safetyTimer tried to run while reseting odometry");
    return;
  }

  {
    std::scoped_lock lock(mutex_last_position_cmd, mutex_last_attitude_cmd);

    if (!(last_position_cmd != mrs_msgs::PositionCommand::Ptr() && last_attitude_cmd != mrs_msgs::AttitudeCommand::Ptr())) {
      return;
    }
  }

  // | -------------- eland and failsafe thresholds ------------- |

  {
    std::scoped_lock lock(mutex_controller_list);

    std::map<std::string, ControllerParams>::iterator it;
    it = controllers_.find(controller_names[active_controller_idx]);

    eland_threshold_    = it->second.eland_threshold;
    failsafe_threshold_ = it->second.failsafe_threshold;
  }

  // | -------- cacalculate control errors and tilt angle ------- |

  double tilt_angle;
  {
    std::scoped_lock lock(mutex_last_position_cmd, mutex_odometry, mutex_tilt_error, mutex_control_error);

    tilt_error = 0;

    // control errors
    position_error_x_ = last_position_cmd->position.x - odometry_x;
    position_error_y_ = last_position_cmd->position.y - odometry_y;
    position_error_z_ = last_position_cmd->position.z - odometry_z;

    velocity_error_x_ = last_position_cmd->velocity.x - odometry.twist.twist.linear.x;
    velocity_error_y_ = last_position_cmd->velocity.y - odometry.twist.twist.linear.y;
    velocity_error_z_ = last_position_cmd->velocity.z - odometry.twist.twist.linear.z;

    // tilt angle
    tf::Quaternion odometry_quaternion;
    quaternionMsgToTF(odometry.pose.pose.orientation, odometry_quaternion);

    // rotate the drone's z axis
    tf::Vector3 uav_z_in_world = tf::Transform(odometry_quaternion) * tf::Vector3(0, 0, 1);

    // calculate the angle between the drone's z axis and the world's z axis
    tilt_angle = acos(uav_z_in_world.dot(tf::Vector3(0, 0, 1)));

    // | ---------------- calculate the tilt error ---------------- |
    // to do that, we need the tilt reference, which might not be available
    {
      std::scoped_lock lock(mutex_last_attitude_cmd);

      // if the attitude_cmd exists (a controller is running)
      if (last_attitude_cmd != mrs_msgs::AttitudeCommand::Ptr()) {

        tf::Quaternion attitude_cmd_quaternion;

        // calculate the quaternion
        if (last_attitude_cmd->quater_attitude_set) {

          attitude_cmd_quaternion.setX(last_attitude_cmd->quter_attitude.x);
          attitude_cmd_quaternion.setY(last_attitude_cmd->quter_attitude.y);
          attitude_cmd_quaternion.setZ(last_attitude_cmd->quter_attitude.z);
          attitude_cmd_quaternion.setW(last_attitude_cmd->quter_attitude.w);

        } else if (last_attitude_cmd->euler_attitude_set) {

          // convert the RPY to quaternion
          attitude_cmd_quaternion =
              tf::createQuaternionFromRPY(last_attitude_cmd->euler_attitude.x, last_attitude_cmd->euler_attitude.y, last_attitude_cmd->euler_attitude.z);
        }

        if (last_attitude_cmd->quater_attitude_set || last_attitude_cmd->euler_attitude_set) {

          // calculate the desired drone's z axis in the world frame
          tf::Vector3 uav_z_in_world_desired = tf::Transform(attitude_cmd_quaternion) * tf::Vector3(0, 0, 1);

          // calculate the angle between the drone's z axis and the world's z axis
          tilt_error = acos(uav_z_in_world.dot(uav_z_in_world_desired));
        }
      }
    }
  }

  double control_error_ = sqrt(pow(position_error_x_, 2) + pow(position_error_y_, 2) + pow(position_error_z_, 2));

  // --------------------------------------------------------------
  // |   activate the failsafe controller in case of large error  |
  // --------------------------------------------------------------

  // | ------ copy the last tracker/controller switch time ------ |

  ros::Time tmp_controller_tracker_switch_time;
  {
    std::scoped_lock lock(mutex_controller_tracker_switch_time);

    tmp_controller_tracker_switch_time = controller_tracker_switch_time;
  }

  // | --------------------------------------------------------- |

  if (control_error_ > failsafe_threshold_ && !failsafe_triggered) {

    if ((ros::Time::now() - tmp_controller_tracker_switch_time).toSec() > 1.0) {

      if (!failsafe_triggered) {

        ROS_ERROR("[ControlManager]: Activating failsafe land: control_error_=%2.2f/%2.2f", control_error_, failsafe_threshold_);

        std::scoped_lock lock(mutex_controller_list, mutex_last_attitude_cmd);

        failsafe();
      }
    }
  }

  // --------------------------------------------------------------
  // |   activate emergancy land in case of medium control error  |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_odometry);

    if (tilt_angle > tilt_limit_eland_ || control_error_ > eland_threshold_) {

      if ((ros::Time::now() - tmp_controller_tracker_switch_time).toSec() > 1.0) {

        if (!failsafe_triggered && !eland_triggered) {

          ROS_ERROR("[ControlManager]: Activating emergancy land: tilt angle=%2.2f/%2.2f deg, control_error_=%2.2f/%2.2f", (180.0 / M_PI) * tilt_angle,
                    (180.0 / M_PI) * tilt_limit_eland_, control_error_, eland_threshold_);

          std::string message_out;
          eland(message_out);
        }
      }
    }
  }

  // --------------------------------------------------------------
  // |      activate failsafe when odometry stops publishing      |
  // --------------------------------------------------------------
  // to do that, we need to fire up safetyTimer, which will regularly trigger the controllers
  // in place of the odometryCallback()

  {
    std::scoped_lock lock(mutex_odometry);

    if ((ros::Time::now() - odometry_last_time).toSec() > odometry_max_missing_time_) {

      if (!failsafe_triggered) {

        ROS_ERROR_THROTTLE(1.0, "[ControlManager]: not receiving odometry for %.3f, initiating failsafe land.",
                           (ros::Time::now() - odometry_last_time).toSec());

        std::scoped_lock lock(mutex_controller_list, mutex_last_attitude_cmd);

        failsafe();
      }
    }
  }

  // --------------------------------------------------------------
  // |      disarm the drone when the tilt exceeds the limit      |
  // --------------------------------------------------------------
  if (tilt_angle > tilt_limit_disarm_) {

    ROS_ERROR("[ControlManager]: Tilt angle too large, disarming: tilt angle=%2.2f/%2.2f deg", (180.0 / M_PI) * tilt_angle, (180.0 / M_PI) * tilt_limit_eland_);

    arming(false);
  }

  // --------------------------------------------------------------
  // |     disarm the drone when tilt error exceeds the limit     |
  // --------------------------------------------------------------
  {
    std::scoped_lock lock(mutex_tilt_error, mutex_odometry, mutex_controller_list, mutex_controller_tracker_switch_time);

    if (tilt_error_failsafe_enabled_ &&
        odometry_z > tilt_error_failsafe_min_height_) {  // TODO the height conditions will not work when we start to fly under 0 height

      if (fabs(tilt_error) > tilt_error_threshold_) {

        if ((ros::Time::now() - controller_tracker_switch_time).toSec() > 1.0) {

          ROS_ERROR("[ControlManager]: Tilt error too large, disarming: tilt error=%2.2f/%2.2f deg", (180.0 / M_PI) * tilt_error,
                    (180.0 / M_PI) * tilt_error_threshold_);

          arming(false);

          failsafe_triggered = true;

        } else {

          ROS_ERROR("[ControlManager]: Tilt error too large (tilt error=%2.2f/%2.2f deg), however, controller/tracker just switched so its ok.",
                    (180.0 / M_PI) * tilt_error, (180.0 / M_PI) * tilt_error_threshold_);
        }
      }
    }
  }
}

//}

/* //{ elandingTimer() */

void ControlManager::elandingTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("elandingTimer", elanding_timer_rate_, 0.01, event);

  if (current_state_landing == IDLE_STATE) {

    return;

  } else if (current_state_landing == LANDING_STATE) {

    double last_thrust_cmd;

    {
      std::scoped_lock lock(mutex_last_attitude_cmd);

      if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {
        ROS_WARN_THROTTLE(1.0, "[ControlManager]: elandingTimer: last_attitude_cmd has not been initialized, returning");
        ROS_WARN_THROTTLE(1.0, "[ControlManager]: tip: the RC eland is probably triggered");
        return;
      }

      last_thrust_cmd = last_attitude_cmd->thrust;
    }

    // recalculate the mass based on the thrust
    double thrust_mass_estimate = pow((last_thrust_cmd - motor_params_.hover_thrust_b) / motor_params_.hover_thrust_a, 2) / g_;
    ROS_INFO("[ControlManager]: landing_uav_mass_: %f thrust_mass_estimate: %f", landing_uav_mass_, thrust_mass_estimate);

    // condition for automatic motor turn off
    if ((odometry_z < elanding_cutoff_height_) && ((thrust_mass_estimate < elanding_cutoff_mass_factor_ * landing_uav_mass_) || last_thrust_cmd < 0.01)) {

      // enable callbacks? ... NO

      switchMotors(false);

      // disarm the drone
      if (isOffboard()) {

        ROS_INFO("[ControlManager]: reached cutoff thrust, calling for disarm");
        mavros_msgs::CommandBool srv_out;
        service_client_arm.call(srv_out);

      } else {

        ROS_WARN("[ControlManager]: cannot disarm, not in OFFBOARD mode");
      }

      shutdown();

      changeLandingState(IDLE_STATE);

      ROS_WARN("[ControlManager]: emergancy landing finished");

      elanding_timer.stop();

      // we should NOT set eland_triggered=true
    }
  }
}

//}

/* //{ failsafeTimer() */

void ControlManager::failsafeTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("failsafeTimer", failsafe_timer_rate_, 0.01, event);

  {
    std::scoped_lock lock(mutex_pixhawk_odometry);

    updateControllers(pixhawk_odometry);
  }

  publish();

  double last_thrust;

  {
    std::scoped_lock lock(mutex_last_attitude_cmd);

    if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {
      ROS_WARN_THROTTLE(1.0, "[ControlManager]: failsafeTimer: last_attitude_cmd has not been initialized, returning");
      ROS_WARN_THROTTLE(1.0, "[ControlManager]: tip: the RC eland is probably triggered");
      return;
    }

    last_thrust = last_attitude_cmd->thrust;
  }

  double thrust_mass_estimate = pow((last_thrust - motor_params_.hover_thrust_b) / motor_params_.hover_thrust_a, 2) / g_;
  ROS_INFO_THROTTLE(1.0, "[ControlManager]: landing_uav_mass_: %f thrust_mass_estimate: %f", landing_uav_mass_, thrust_mass_estimate);

  // condition for automatic motor turn off
  if (((thrust_mass_estimate < elanding_cutoff_mass_factor_ * landing_uav_mass_) || last_thrust < 0.01)) {

    ROS_INFO("[ControlManager]: detecting zero thrust, disarming");

    arming(false);
  }
}

//}

/* //{ joystickTimer() */

void ControlManager::joystickTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  std::scoped_lock lock(mutex_joystick);

  mrs_lib::Routine profiler_routine = profiler->createRoutine("joystickTimer", status_timer_rate_, 0.01, event);

  // if start was pressed and held for > 3.0 s
  if (joytracker_start_pressed && (ros::Time::now() - joystick_tracker_press_time).toSec() > 3.0) {

    ROS_INFO("[ControlManager]: transitioning to joystick control: activating %s and %s", joystick_tracker_name_.c_str(), joystick_controller_name_.c_str());

    joytracker_start_pressed = false;

    mrs_msgs::StringRequest controller_srv;
    controller_srv.value = joystick_controller_name_;

    mrs_msgs::StringRequest tracker_srv;
    tracker_srv.value = joystick_tracker_name_;

    mrs_msgs::StringResponse response;

    callbackSwitchTracker(tracker_srv, response);
    callbackSwitchController(controller_srv, response);
  }

  // if RT+LT were pressed and held for > 1.0 s
  if (joystick_failsafe_pressed && (ros::Time::now() - joystick_failsafe_press_time).toSec() > 0.1) {

    ROS_INFO("[ControlManager]: activating failsafe by joystick");

    joystick_failsafe_pressed = false;

    failsafe();
  }

  // if joypads were pressed and held for > 1.0 s
  if (joystick_eland_pressed && (ros::Time::now() - joystick_eland_press_time).toSec() > 0.1) {

    ROS_INFO("[ControlManager]: activating eland by joystick");

    joystick_failsafe_pressed = false;

    std::string message_out;
    eland(message_out);
  }

  // if back was pressed and held for > 1.0 s
  if (joystick_back_pressed && (ros::Time::now() - joystick_goto_press_time).toSec() > 0.1) {

    // activate the joystick goto functionality
    joystick_goto_enabled = true;
  }

  // if the GOTO functionality is enabled...
  if (joystick_goto_enabled) {

    // create the reference

    mrs_msgs::Vec4::Request request;

    if (fabs(joystick_data.axes[_channel_pitch_]) >= 0.05 || fabs(joystick_data.axes[_channel_roll_]) >= 0.05 ||
        fabs(joystick_data.axes[_channel_yaw_]) >= 0.05 || fabs(joystick_data.axes[_channel_thrust_]) >= 0.05) {

      double speed = 1.0;

      request.goal[REF_X]   = _channel_mult_pitch_ * joystick_data.axes[_channel_pitch_] * speed;
      request.goal[REF_Y]   = _channel_mult_roll_ * joystick_data.axes[_channel_roll_] * speed;
      request.goal[REF_Z]   = _channel_mult_thrust_ * joystick_data.axes[_channel_thrust_];
      request.goal[REF_YAW] = _channel_mult_yaw_ * joystick_data.axes[_channel_yaw_];

      mrs_msgs::Vec4::Response response;

      callbackGoToFcuService(request, response);
    }
  }

  if (rc_goto_active_ && last_position_cmd != mrs_msgs::PositionCommand::Ptr()) {

    std::scoped_lock lock(mutex_rc_channels);

    // create the reference

    mrs_msgs::Vec4::Request request;

    double des_x   = 0;
    double des_y   = 0;
    double des_z   = 0;
    double des_yaw = 0;

    double speed = 2.0;

    bool nothing_to_do = true;

    if (uint(3) >= rc_channels.channels.size()) {

      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: RC eland channel number is out of range");

    } else {

      if (abs(rc_channels.channels[rc_channel_roll_] - PWM_MIDDLE) > 100) {
        des_y         = (-(rc_channels.channels[rc_channel_roll_] - PWM_MIDDLE) / 500.0) * speed;
        nothing_to_do = false;
      }

      if (abs(rc_channels.channels[rc_channel_thrust_] - PWM_MIDDLE) > 100) {
        des_z         = ((rc_channels.channels[rc_channel_thrust_] - PWM_MIDDLE) / 500.0) * speed;
        nothing_to_do = false;
      }

      if (abs(rc_channels.channels[rc_channel_pitch_] - PWM_MIDDLE) > 200) {
        des_x         = ((rc_channels.channels[rc_channel_pitch_] - PWM_MIDDLE) / 500.0) * speed;
        nothing_to_do = false;
      }

      if (abs(rc_channels.channels[rc_channel_yaw_] - PWM_MIDDLE) > 100) {
        des_yaw       = (-(rc_channels.channels[rc_channel_yaw_] - PWM_MIDDLE) / 500.0) * 1.0;
        nothing_to_do = false;
      }
    }

    if (!nothing_to_do) {

      request.goal[REF_X]   = des_x;
      request.goal[REF_Y]   = des_y;
      request.goal[REF_Z]   = des_z;
      request.goal[REF_YAW] = des_yaw;

      mrs_msgs::Vec4Response response;

      // disable callbacks of all trackers
      std_srvs::SetBoolRequest req_enable_callbacks;

      // enable the callbacks for the active tracker
      req_enable_callbacks.data = true;
      {
        std::scoped_lock lock(mutex_tracker_list);

        tracker_list[active_tracker_idx]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));
      }

      callbacks_enabled = true;

      // call the goto
      callbackGoToFcuService(request, response);

      callbacks_enabled = false;

      ROS_INFO("[ControlManager]: goto by rc by x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", request.goal[REF_X], request.goal[REF_Y], request.goal[REF_Z],
               request.goal[REF_YAW]);

      // disable the callbacks back again
      req_enable_callbacks.data = false;
      {
        std::scoped_lock lock(mutex_tracker_list);

        tracker_list[active_tracker_idx]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));
      }
    }
  }

  if (rc_goto_enabled_ && got_rc_channels) {

    std::scoped_lock lock(mutex_rc_channels);

    // prune the list of rc_channel_switches
    std::list<ros::Time>::iterator it;
    for (it = rc_channel_switch_time.begin(); it != rc_channel_switch_time.end();) {
      if ((ros::Time::now() - *it).toSec() > rc_joystic_timeout_) {
        it = rc_channel_switch_time.erase(it);
      } else {
        it++;
      }
    }

    if (int(rc_channel_switch_time.size()) >= rc_joystic_n_switches_) {

      if (rc_goto_active_ == false) {

        ROS_INFO("[ControlManager]: activating rc joystick");

        callbacks_enabled = false;

        std_srvs::SetBoolRequest req_goto_out;
        req_goto_out.data = false;

        std_srvs::SetBoolRequest req_enable_callbacks;
        req_enable_callbacks.data = callbacks_enabled;

        {
          std::scoped_lock lock(mutex_tracker_list);

          // disable callbacks of all trackers
          for (unsigned int i = 0; i < tracker_list.size(); i++) {
            tracker_list[i]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));
          }
        }
      } else if (rc_goto_active_ == true) {

        ROS_INFO("[ControlManager]: deactivating rc joystic");

        callbacks_enabled = true;

        std_srvs::SetBoolRequest req_goto_out;
        req_goto_out.data = true;

        std_srvs::SetBoolRequest req_enable_callbacks;
        req_enable_callbacks.data = callbacks_enabled;

        {
          std::scoped_lock lock(mutex_tracker_list);

          // enable callbacks of all trackers
          for (unsigned int i = 0; i < tracker_list.size(); i++) {
            tracker_list[i]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));
          }
        }
      }

      rc_goto_active_ = !rc_goto_active_;
      rc_channel_switch_time.clear();
    }
  }
}

//}

/* //{ bumperTimer() */

void ControlManager::bumperTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("bumperTimer", bumper_timer_rate_, 0.01, event);

  if (!bumper_enabled_ || !bumper_repulsion_enabled_) {
    return;
  }

  if ((ros::Time::now() - bumper_data.header.stamp).toSec() > 1.0) {
    return;
  }

  {
    std::scoped_lock lock(mutex_odometry);

    if (odometry_z < 0.5) {

      ROS_WARN_THROTTLE(0.5, "[ControlManager]: not using bumper repulsion, height < 0.5 m");
      return;
    }
  }

  // --------------------------------------------------------------
  // |                      bumper repulsion                      |
  // --------------------------------------------------------------

  ROS_INFO_THROTTLE(1.0, "[ControlManager]: bumperTimer spinning");

  bumperPushFromObstacle();
}

//}

/* //{ pirouetteTimer() */

void ControlManager::pirouetteTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("pirouetteTimer", pirouette_timer_rate_, 0.01, event);

  pirouette_iterator++;

  double pirouette_duration  = (2 * M_PI) / pirouette_speed_;
  double pirouette_n_steps   = pirouette_duration * pirouette_timer_rate_;
  double pirouette_step_size = (2 * M_PI) / pirouette_n_steps;

  if (rc_eland_triggered || failsafe_triggered || eland_triggered || (pirouette_iterator > pirouette_duration * pirouette_timer_rate_)) {

    pirouette_enabled_ = false;
    pirouette_timer.stop();

    setCallbacks(true);

    return;
  }

  // enable the callbacks for the active tracker
  std_srvs::SetBoolRequest req_enable_callbacks;
  req_enable_callbacks.data = true;
  tracker_list[active_tracker_idx]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));

  // call the goto
  mrs_msgs::Vec1Request req_goto_out;

  req_goto_out.goal = pirouette_inital_yaw + pirouette_iterator * pirouette_step_size;

  mrs_msgs::Vec1Response::ConstPtr tracker_response;
  tracker_response = tracker_list[active_tracker_idx]->setYaw(mrs_msgs::Vec1Request::ConstPtr(new mrs_msgs::Vec1Request(req_goto_out)));

  // disable the callbacks for the active tracker
  req_enable_callbacks.data = false;
  tracker_list[active_tracker_idx]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | --------------------- topic callbacks -------------------- |

/* //{ callbackOdometry() */

void ControlManager::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackOdometry");

  if (!got_max_height) {
    ROS_INFO("[ControlerManager]: the safety timer is in the middle of an iteration, waiting for it to finish");
    return;
  }

  // | -- prepare an OdometryConstPtr for trackers&controllers -- |

  nav_msgs::Odometry::ConstPtr odometry_const_ptr(new nav_msgs::Odometry(*msg));

  // | ----- check for change in odometry frame of reference ---- |

  if (got_odometry) {
    if (odometry.child_frame_id.compare(msg->child_frame_id) != STRING_EQUAL) {

      ROS_INFO("[ControlManager]: detecting switch of odometry frame");
      {
        std::scoped_lock lock(mutex_odometry);

        ROS_INFO("[ControlManager]: odometry before switch: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", odometry_x, odometry_y, odometry_z, odometry_yaw);
      }

      reseting_odometry = true;

      // we have to stop safety timer, otherwise it will interfere
      safety_timer.stop();
      // wait for the safety timer to stop if its running
      while (running_safety_timer) {
        ROS_INFO("[ControlManager]: waiting for safety timer to finish");
        ros::Duration wait(0.001);
        wait.sleep();
      }

      {
        std::scoped_lock lock(mutex_controller_list, mutex_tracker_list);

        tracker_list[active_tracker_idx]->switchOdometrySource(odometry_const_ptr);
        controller_list[active_controller_idx]->switchOdometrySource(odometry_const_ptr);
      }
    }
  }

  // --------------------------------------------------------------
  // |                      copy the odometry                     |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_odometry);

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

  got_odometry = true;

  odometry_last_time = ros::Time::now();

  if (!failsafe_triggered) {  // when failsafe is triggers, updateControllers() and publish() is called in failsafeTimer()

    // run the safety timer
    // in the case of large control errors, the safety mechanisms will be triggered before the controllers and trackers are updated...
    ros::TimerEvent safety_timer_event;
    safetyTimer(safety_timer_event);

    updateTrackers();

    {
      std::scoped_lock lock(mutex_odometry);

      updateControllers(odometry);
    }

    publish();
  }

  if (reseting_odometry) {

    safety_timer.start();
    reseting_odometry = false;

    std::scoped_lock lock(mutex_odometry);

    ROS_INFO("[ControlManager]: odometry after switch: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", odometry_x, odometry_y, odometry_z, odometry_yaw);
  }
}

//}

/* //{ callbackPixhawkOdometry() */

void ControlManager::callbackPixhawkOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackPixhawkOdometry");

  // --------------------------------------------------------------
  // |                      copy the odometry                     |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_pixhawk_odometry);

    pixhawk_odometry = *msg;

    pixhawk_odometry_x = pixhawk_odometry.pose.pose.position.x;
    pixhawk_odometry_y = pixhawk_odometry.pose.pose.position.y;
    pixhawk_odometry_z = pixhawk_odometry.pose.pose.position.z;

    // calculate the euler angles
    tf::Quaternion quaternion_odometry;
    quaternionMsgToTF(pixhawk_odometry.pose.pose.orientation, quaternion_odometry);
    tf::Matrix3x3 m(quaternion_odometry);
    m.getRPY(pixhawk_odometry_roll, pixhawk_odometry_pitch, pixhawk_odometry_yaw);
  }

  got_pixhawk_odometry = true;
}

//}

/* callbackMaxHeight() //{ */

void ControlManager::callbackMaxHeight(const mrs_msgs::Float64StampedConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackMaxHeight");

  {
    std::scoped_lock lock(mutex_max_height);

    got_max_height = true;
    max_height     = msg->value;
  }
}

//}

/* callbackJoystick() //{ */

void ControlManager::callbackJoystick(const sensor_msgs::JoyConstPtr &msg) {

  if (!is_initialized)
    return;

  std::scoped_lock lock(mutex_joystick);

  joystick_data = *msg;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackJoystick");

  // | ---- switching back to fallback tracker and controller --- |

  // if any of the A, B, X, Y buttons are pressed when flying with joystick, switch back to fallback controller and tracker
  if ((msg->buttons[_channel_A_] == 1 || msg->buttons[_channel_B_] == 1 || msg->buttons[_channel_X_] == 1 || msg->buttons[_channel_Y_] == 1) &&
      active_tracker_idx == joystick_tracker_idx && active_controller_idx == joystick_controller_idx) {

    ROS_INFO("[ControlManager]: switching from joystick to normal control");

    mrs_msgs::StringRequest controller_srv;
    controller_srv.value = joystick_fallback_controller_name_;

    mrs_msgs::StringRequest tracker_srv;
    tracker_srv.value = joystick_fallback_tracker_name_;

    mrs_msgs::StringResponse response;

    callbackSwitchTracker(tracker_srv, response);
    callbackSwitchController(controller_srv, response);

    joystick_goto_enabled = false;
  }

  // | ------- joystick control activation ------- |

  // if start button was pressed
  if (msg->buttons[_channel_start_] == 1) {

    if (!joytracker_start_pressed) {

      ROS_INFO("[ControlManager]: joystick start button pressed");

      joytracker_start_pressed    = true;
      joystick_tracker_press_time = ros::Time::now();
    }

  } else if (joytracker_start_pressed) {

    ROS_INFO("[ControlManager]: joystick start button released");

    joytracker_start_pressed    = false;
    joystick_tracker_press_time = ros::Time(0);
  }

  // | ---------------- Joystick goto activation ---------------- |

  // if back button was pressed
  if (msg->buttons[_channel_back_] == 1) {

    if (!joystick_back_pressed) {

      ROS_INFO("[ControlManager]: joystick back button pressed");

      joystick_back_pressed    = true;
      joystick_goto_press_time = ros::Time::now();
    }

  } else if (joystick_back_pressed) {

    ROS_INFO("[ControlManager]: joystick back button released");

    joystick_back_pressed    = false;
    joystick_goto_press_time = ros::Time(0);
  }

  // | ------------------------ Failsafes ----------------------- |

  // if LT and RT buttons are both pressed down
  if (msg->axes[_channel_LT_] < -0.99 && msg->axes[_channel_RT_] < -0.99) {

    if (!joystick_failsafe_pressed) {

      ROS_INFO("[ControlManager]: joystick Failsafe pressed");

      joystick_failsafe_pressed    = true;
      joystick_failsafe_press_time = ros::Time::now();
    }

  } else if (joystick_failsafe_pressed) {

    ROS_INFO("[ControlManager]: joystick Failsafe released");

    joystick_failsafe_pressed    = false;
    joystick_failsafe_press_time = ros::Time(0);
  }

  // if left and right joypads are both pressed down
  if (msg->buttons[_channel_L_joy_] == 1 && msg->buttons[_channel_R_joy_] == 1) {

    if (!joystick_eland_pressed) {

      ROS_INFO("[ControlManager]: joystick eland pressed");

      joystick_eland_pressed    = true;
      joystick_eland_press_time = ros::Time::now();
    }

  } else if (joystick_eland_pressed) {

    ROS_INFO("[ControlManager]: joystick eland released");

    joystick_eland_pressed    = false;
    joystick_eland_press_time = ros::Time(0);
  }
}

//}

/* callbackBumper() //{ */

void ControlManager::callbackBumper(const mrs_msgs::ObstacleSectorsConstPtr &msg) {

  if (!is_initialized)
    return;

  ROS_INFO_ONCE("[ControlManager]: getting bumper data");

  std::scoped_lock lock(mutex_bumper);

  got_bumper = true;

  bumper_data = *msg;
}

//}

/* //{ callbackMavrosState() */

void ControlManager::callbackMavrosState(const mavros_msgs::StateConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackMavrosState");

  std::scoped_lock lock(mutex_mavros_state);

  // | --------------------- save the state --------------------- |
  mavros_state     = *msg;
  got_mavros_state = true;

  // | ------ detect and print the changes in offboard mode ----- |
  if (mavros_state.mode.compare(std::string("OFFBOARD")) == STRING_EQUAL) {

    if (!offboard_mode) {
      offboard_mode = true;
      ROS_INFO("[ControlManager]: OFFBOARD mode ON");
    }

  } else {

    if (offboard_mode) {
      offboard_mode = false;
      ROS_INFO("[ControlManager]: OFFBOARD mode OFF");
    }
  }

  // | --------- detect and print the changes in arming --------- |
  if (mavros_state.armed == true) {

    if (!armed) {
      armed = true;
      ROS_INFO("[ControlManager]: vehicle ARMED");
    }

  } else {

    if (armed) {
      armed = false;
      ROS_INFO("[ControlManager]: vehicle DISARMED");
    }
  }
}

//}

/* //{ callbackRC() */

void ControlManager::callbackRC(const mavros_msgs::RCInConstPtr &msg) {

  if (!is_initialized)
    return;

  if (rc_eland_triggered) {
    return;
  }

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackRC");

  ROS_INFO_ONCE("[ControlManager]: getting RC channels");

  {
    std::scoped_lock lock(mutex_rc_channels);
    rc_channels = *msg;
  }

  got_rc_channels = true;

  // | ------------------- rc joystic control ------------------- |

  // when the switch change its position
  if (rc_goto_enabled_) {

    if (uint(rc_joystic_channel_) >= msg->channels.size()) {

      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: RC eland channel number is out of range");
      return;

    } else {

      if ((rc_joystic_channel_last_value < PWM_MIDDLE && rc_channels.channels[rc_joystic_channel_] > PWM_MIDDLE) ||
          (rc_joystic_channel_last_value > PWM_MIDDLE && rc_channels.channels[rc_joystic_channel_] < PWM_MIDDLE)) {

        // enter an event to the std vector
        std::scoped_lock lock(mutex_rc_channel_switch_time);

        rc_channel_switch_time.insert(rc_channel_switch_time.begin(), ros::Time::now());
      }
    }
  }

  // do not forget to update the last... variable
  rc_joystic_channel_last_value = rc_channels.channels[rc_joystic_channel_];

  // | ------------------------ rc eland ------------------------ |
  if (rc_eland_enabled_) {

    if (uint(rc_eland_channel_) >= msg->channels.size()) {

      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: RC eland channel number is out of range");
      return;

    } else {

      if (msg->channels[rc_eland_channel_] >= uint(rc_eland_threshold_) && !eland_triggered && !failsafe_triggered) {

        ROS_INFO("[ControlManager]: triggering eland by RC");

        rc_eland_triggered = true;

        std::string message_out;
        eland(message_out);
      }
    }
  }
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackSwitchTracker() */

bool ControlManager::callbackSwitchTracker(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

  if (!is_initialized)
    return false;

  char message[200];

  if (!got_odometry) {

    sprintf((char *)&message, "Can't switch tracker, missing odometry!");
    ROS_ERROR("[ControlManager]: %s", message);
    res.success = false;
    res.message = message;
    return true;
  }

  if (!got_pixhawk_odometry) {

    sprintf((char *)&message, "Can't switch tracker, missing PixHawk odometry!");
    ROS_ERROR("[ControlManager]: %s", message);
    res.success = false;
    res.message = message;
    return true;
  }

  int new_tracker_idx = -1;

  for (unsigned int i = 0; i < tracker_names.size(); i++) {
    if (req.value.compare(tracker_names[i]) == 0) {
      new_tracker_idx = i;
    }
  }

  // check if the tracker exists
  if (new_tracker_idx < 0) {

    sprintf((char *)&message, "The tracker %s does not exist!", req.value.c_str());
    ROS_ERROR("[ControlManager]: %s", message);
    res.success = false;
    res.message = message;
    return true;
  }

  // check if the tracker is already active
  if (new_tracker_idx == active_tracker_idx) {

    sprintf((char *)&message, "Not switching, the tracker %s is already active!", req.value.c_str());
    ROS_WARN("[ControlManager]: %s", message);
    res.success = true;
    res.message = message;
    return true;
  }

  {
    std::scoped_lock lock(mutex_last_attitude_cmd, mutex_last_position_cmd, mutex_tracker_list);

    try {

      ROS_INFO("[ControlManager]: Activating tracker %s", tracker_names[new_tracker_idx].c_str());

      if (!tracker_list[new_tracker_idx]->activate(last_position_cmd)) {

        sprintf((char *)&message, "Tracker %s was not activated", req.value.c_str());
        ROS_WARN("[ControlManager]: %s", message);
        res.success = false;

      } else {

        sprintf((char *)&message, "Tracker %s has been activated", req.value.c_str());
        ROS_INFO("[ControlManager]: %s", message);
        res.success = true;

        {
          std::scoped_lock lock(mutex_controller_tracker_switch_time);

          // update the time (used in failsafe)
          controller_tracker_switch_time = ros::Time::now();
        }

        // super important, switch which the active tracker idx
        try {

          ROS_INFO("[ControlManager]: deactivating %s", tracker_names[active_tracker_idx].c_str());
          tracker_list[active_tracker_idx]->deactivate();

          // if switching from null tracker, activate the active the controller
          if (tracker_names[active_tracker_idx].compare(null_tracker_name_) == 0) {

            ROS_INFO("[ControlManager]: activating %s due to switching from NullTracker", controller_names[active_controller_idx].c_str());
            {
              std::scoped_lock lock(mutex_controller_list);

              controller_list[active_controller_idx]->activate(last_attitude_cmd);

              {
                std::scoped_lock lock(mutex_controller_tracker_switch_time);

                // update the time (used in failsafe)
                controller_tracker_switch_time = ros::Time::now();
              }
            }

            // if switching to null tracker, deactivate the active controller
          } else if (tracker_names[new_tracker_idx].compare(null_tracker_name_) == 0) {

            ROS_INFO("[ControlManager]: deactivating %s due to switching to NullTracker", controller_names[active_controller_idx].c_str());
            {
              std::scoped_lock lock(mutex_controller_list);

              controller_list[active_controller_idx]->deactivate();
            }
          }

          active_tracker_idx = new_tracker_idx;
        }
        catch (std::runtime_error &exrun) {
          ROS_ERROR("[ControlManager]: Could not deactivate tracker %s", tracker_names[active_tracker_idx].c_str());
        }
      }
    }
    catch (std::runtime_error &exrun) {
      ROS_ERROR("[ControlManager]: Error during activation of tracker %s", req.value.c_str());
      ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
    }
  }

  res.message = message;

  return true;
}

//}

/* callbackSwitchController() //{ */

bool ControlManager::callbackSwitchController(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

  char message[200];

  int new_controller_idx = -1;

  for (unsigned int i = 0; i < controller_names.size(); i++) {
    if (req.value.compare(controller_names[i]) == STRING_EQUAL) {
      new_controller_idx = i;
    }
  }

  // check if the controller exists
  if (new_controller_idx < 0) {

    sprintf((char *)&message, "The controller %s does not exist!", req.value.c_str());
    ROS_ERROR("[ControlManager]: %s", message);
    res.success = false;
    res.message = message;
    return true;
  }

  // check if the controller is not active
  if (new_controller_idx == active_controller_idx) {

    sprintf((char *)&message, "Not switching, the controller %s is already active!", req.value.c_str());
    ROS_WARN("[ControlManager]: %s", message);
    res.success = true;
    res.message = message;
    return true;
  }

  {
    std::scoped_lock lock(mutex_last_attitude_cmd, mutex_controller_list);

    try {

      ROS_INFO("[ControlManager]: Activating controller %s", controller_names[new_controller_idx].c_str());
      if (!controller_list[new_controller_idx]->activate(last_attitude_cmd)) {

        sprintf((char *)&message, "Controller %s was not activated", req.value.c_str());
        ROS_WARN("[ControlManager]: %s", message);
        res.success = false;

      } else {

        sprintf((char *)&message, "Controller %s has been activated", req.value.c_str());
        ROS_INFO("[ControlManager]: %s", message);
        res.success = true;

        // TODO is this the right place?
        ROS_INFO("[ControlManager]: triggering hover after switching to a new controller, re-activating %s.", tracker_names[active_tracker_idx].c_str());

        // reactivate the current tracker
        // TODO this is not the most elegant way
        {
          std::scoped_lock lock(mutex_tracker_list);

          tracker_list[active_tracker_idx]->deactivate();
          tracker_list[active_tracker_idx]->activate(mrs_msgs::PositionCommand::Ptr());
        }

        {
          std::scoped_lock lock(mutex_controller_tracker_switch_time);

          // update the time (used in failsafe)
          controller_tracker_switch_time = ros::Time::now();
        }

        // super important, switch which the active controller idx
        try {

          controller_list[active_controller_idx]->deactivate();
          active_controller_idx = new_controller_idx;
        }
        catch (std::runtime_error &exrun) {
          ROS_ERROR("[ControlManager]: Could not deactivate controller %s", controller_names[active_controller_idx].c_str());
        }
      }
    }
    catch (std::runtime_error &exrun) {
      ROS_ERROR("[ControlManager]: Error during activation of controller %s", req.value.c_str());
      ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
    }
  }

  res.message = message;
  return true;
}

//}

/* //{ callbackEHover() */

bool ControlManager::callbackEHoverService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized)
    return false;

  res.success = ehover(res.message);

  return true;
}

//}

/* callbackFailsafe() //{ */

bool ControlManager::callbackFailsafe([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized)
    return false;

  std::scoped_lock lock(mutex_controller_list, mutex_last_attitude_cmd);

  res.success = failsafe();

  return true;
}

//}

/* callbackFailsafeEscalating() //{ */

bool ControlManager::callbackFailsafeEscalating([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized)
    return false;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackFailsafeEscalating");

  if ((ros::Time::now() - escalating_failsafe_time).toSec() < escalating_failsafe_timeout_) {

    res.message = "too soon for escalating failsafe";
    res.success = false;
    return true;
  }

  ROS_WARN("[ControlManager]: escalating failsafe triggered");

  if (!eland_triggered && !failsafe_triggered && motors) {

    res.success              = eland(res.message);
    res.message              = "triggering eland";
    escalating_failsafe_time = ros::Time::now();

  } else if (eland_triggered) {

    std::scoped_lock lock(mutex_controller_list, mutex_last_attitude_cmd);

    res.success              = failsafe();
    res.message              = "triggering failsafe";
    escalating_failsafe_time = ros::Time::now();

  } else if (failsafe_triggered) {

    escalating_failsafe_time = ros::Time::now();
    res.message              = "disarming";
    res.success              = arming(false);
  }

  return true;
}

//}

/* //{ callbackELand() */

bool ControlManager::callbackEland([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized)
    return false;

  res.success = eland(res.message);

  return true;
}

//}

/* //{ callbackMotors() */

bool ControlManager::callbackMotors(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized)
    return false;

  char message[200];

  {
    std::scoped_lock lock(mutex_odometry);

    if (!isPointInSafetyArea2d(odometry.pose.pose.position.x, odometry.pose.pose.position.y)) {

      sprintf((char *)&message, "Can't turn motors on, the UAV is outside of the safety area!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[ControlManager]: %s", message);
      return true;
    }
  }

  if (req.data && (failsafe_triggered || eland_triggered || rc_eland_triggered)) {
    sprintf((char *)&message, "cannot switch motors ON, we landed in emergancy.");
    res.message = message;
    res.success = false;
    ROS_ERROR("[ControlManager]: %s", message);
    return true;
  }

  {
    std::scoped_lock lock(mutex_mavros_state);

    if (!got_mavros_state || (ros::Time::now() - mavros_state.header.stamp).toSec() > 5.0) {
      sprintf((char *)&message, "Can't takeoff, missing mavros state!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[ControlManager]: %s", message);
      return true;
    }
  }

  if (bumper_enabled_) {
    if (!got_bumper) {
      sprintf((char *)&message, "Can't turn motors on, missing bumper data!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[ControlManager]: %s", message);
      return true;
    }
  }

  switchMotors(req.data);

  sprintf((char *)&message, "Motors: %s", motors ? "ON" : "OFF");
  res.message = message;
  res.success = true;

  ROS_INFO("[ControlManager]: %s", message);

  return true;
}

//}

/* callbackArm() //{ */

bool ControlManager::callbackArm(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  char message[200];

  if (req.data) {

    sprintf((char *)&message, "Not allowed to arm the drone.");
    res.message = message;
    res.success = false;
    ROS_ERROR("[ControlManager]: %s", message);

  } else {

    if (arming(false)) {

      sprintf((char *)&message, "Disarmed");
      res.message = message;
      res.success = true;
      ROS_INFO("[ControlManager]: %s", message);

    } else {

      sprintf((char *)&message, "Could not disarm");
      res.message = message;
      res.success = false;
      ROS_ERROR("[ControlManager]: %s", message);
    }
  }

  return true;
}

//}

/* //{ callbackEnableCallbacks() */

bool ControlManager::callbackEnableCallbacks(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized)
    return false;

  setCallbacks(req.data);

  char message[200];
  sprintf((char *)&message, "Callbacks: %s", motors ? "enabled" : "disabled");
  res.message = message;
  res.success = true;

  ROS_INFO("[ControlManager]: %s", message);

  return true;
}

//}

/* callbackSetConstraints() //{ */

bool ControlManager::callbackSetConstraints(mrs_msgs::TrackerConstraints::Request &req, mrs_msgs::TrackerConstraints::Response &res) {

  if (!is_initialized) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  mrs_msgs::TrackerConstraintsResponse::ConstPtr tracker_response;

  mrs_msgs::TrackerConstraintsRequest req_constraints;
  req_constraints = req;

  {
    std::scoped_lock lock(mutex_tracker_list);

    // for each tracker
    for (unsigned int i = 0; i < tracker_list.size(); i++) {

      // if it is the active one, update and retrieve the command
      tracker_response =
          tracker_list[i]->setConstraints(mrs_msgs::TrackerConstraintsRequest::ConstPtr(new mrs_msgs::TrackerConstraintsRequest(req_constraints)));
    }
  }

  res.message = "Setting constraints";
  res.success = true;

  return true;
}

//}

/* //{ callbackEmergencyGoToService() */

bool ControlManager::callbackEmergencyGoToService(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res) {

  if (!is_initialized)
    return false;

  callbacks_enabled = false;

  mrs_msgs::Vec4Response::ConstPtr tracker_response;
  char                             message[200];

  std_srvs::SetBoolRequest req_enable_callbacks;

  mrs_msgs::Vec4Request req_goto_out;
  req_goto_out.goal = req.goal;

  {
    std::scoped_lock lock(mutex_tracker_list);

    // disable callbacks of all trackers
    req_enable_callbacks.data = false;
    for (unsigned int i = 0; i < tracker_list.size(); i++) {
      tracker_list[i]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));
    }

    // enable the callbacks for the active tracker
    req_enable_callbacks.data = true;
    tracker_list[active_tracker_idx]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));

    // call the goto
    tracker_response = tracker_list[active_tracker_idx]->goTo(mrs_msgs::Vec4Request::ConstPtr(new mrs_msgs::Vec4Request(req_goto_out)));

    // disable the callbacks back again
    req_enable_callbacks.data = false;
    tracker_list[active_tracker_idx]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));

    if (tracker_response != mrs_msgs::Vec4Response::Ptr()) {
      res = *tracker_response;
    } else {
      sprintf((char *)&message, "The tracker '%s' does not implement 'goto' service!", tracker_names[active_tracker_idx].c_str());
      res.message = message;
      res.success = false;
    }
  }

  return true;
}

//}

/* callbackPirouette() //{ */

bool ControlManager::callbackPirouette([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized)
    return false;

  if (pirouette_enabled_) {
    res.success = false;
    res.message = "already active";
    return true;
  }

  pirouette_enabled_ = true;

  std::scoped_lock lock(mutex_odometry);

  setCallbacks(false);

  pirouette_inital_yaw = odometry_yaw;
  pirouette_iterator   = 0;
  pirouette_timer.start();

  res.success = true;
  res.message = "activated";

  return true;
}

//}

// | -------------- setpoint topics and services -------------- |

/* //{ callbackGoToService() */

bool ControlManager::callbackGoToService(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res) {

  if (!is_initialized) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  if (!callbacks_enabled) {
    ROS_WARN("[ControlManager]: not passing the goto service through, the callbacks are disabled");
    res.message = "callbacks are disabled";
    res.success = false;
    return true;
  }

  mrs_msgs::Vec4::Request request_in = req;

  // check number validity
  for (int i = 0; i < 4; i++) {
    if (!std::isfinite(request_in.goal[i])) {
      ROS_ERROR("NaN detected in variable \"request_in.goal[%d]\"!!!", i);
      res.message = "NaNs/infs in the goal!";
      res.success = false;
      return true;
    }
  }

  // check the obstacle bumper
  if (!bumperValidatePoint(request_in.goal[REF_X], request_in.goal[REF_Y], request_in.goal[REF_Z], ABSOLUTE_FRAME)) {
    ROS_ERROR("[ControlManager]: 'goto' service failed, potential collision with an obstacle!");
    res.message = "potential collision with an obstacle";
    res.success = false;
    return true;
  }

  if (!isPointInSafetyArea3d(request_in.goal[REF_X], request_in.goal[REF_Y], request_in.goal[REF_Z])) {
    ROS_ERROR("[ControlManager]: 'goto' service failed, the point is outside of the safety area!");
    res.message = "the point is outside of the safety area";
    res.success = false;
    return true;
  }

  mrs_msgs::Vec4Response::ConstPtr tracker_response;
  char                             message[200];

  // prepare the message for current tracker
  mrs_msgs::Vec4Request req_goto_out;
  req_goto_out.goal = request_in.goal;

  {
    std::scoped_lock lock(mutex_tracker_list);

    tracker_response = tracker_list[active_tracker_idx]->goTo(mrs_msgs::Vec4Request::ConstPtr(new mrs_msgs::Vec4Request(req_goto_out)));

    if (tracker_response != mrs_msgs::Vec4Response::Ptr()) {
      res = *tracker_response;
    } else {
      sprintf((char *)&message, "The tracker '%s' does not implement 'goto' service!", tracker_names[active_tracker_idx].c_str());
      res.message = message;
      res.success = false;
    }
  }

  return true;
}

//}

/* //{ callbackGoToTopic() */

void ControlManager::callbackGoToTopic(const mrs_msgs::TrackerPointStampedConstPtr &msg) {

  if (!is_initialized)
    return;

  if (!callbacks_enabled) {
    ROS_WARN("[ControlManager]: not passing the goto topic through, the callbacks are disabled");
    return;
  }

  if (!std::isfinite(msg->position.x)) {
    ROS_ERROR("NaN detected in variable \"msg->position.x\"!!!");
    return;
  }

  if (!std::isfinite(msg->position.y)) {
    ROS_ERROR("NaN detected in variable \"msg->position.y\"!!!");
    return;
  }

  if (!std::isfinite(msg->position.z)) {
    ROS_ERROR("NaN detected in variable \"msg->position.z\"!!!");
    return;
  }

  if (!std::isfinite(msg->position.yaw)) {
    ROS_ERROR("NaN detected in variable \"msg->position.yaw\"!!!");
    return;
  }

  // copy the original message so we can modify it
  double des_x   = msg->position.x;
  double des_y   = msg->position.y;
  double des_z   = msg->position.z;
  double des_yaw = msg->position.yaw;

  // prepare the message for current tracker
  mrs_msgs::TrackerPointStamped request;

  // check the obstacle bumper
  ReferenceFrameType_t frame_type;
  if (msg->header.frame_id.compare("fcu") == STRING_EQUAL) {
    frame_type = FCU_FRAME;
  } else {
    frame_type = ABSOLUTE_FRAME;
  }

  if (!bumperValidatePoint(des_x, des_y, des_z, frame_type)) {
    ROS_ERROR("[ControlManager]: 'goto' topic failed, potential collision with an obstacle!");
    return;
  }

  if (msg->header.frame_id.compare("fcu") == STRING_EQUAL) {

    // rotate it from the frame of the drone
    Eigen::Vector2d des(des_x, des_y);
    des = rotateVector(des, odometry_yaw);

    {
      std::scoped_lock lock(mutex_odometry);

      request.position.x   = des[0] + odometry_x;
      request.position.y   = des[1] + odometry_y;
      request.position.z   = des_z + odometry_z;
      request.position.yaw = des_yaw + odometry_yaw;
    }

  } else {

    request.position.x   = des_x;
    request.position.y   = des_y;
    request.position.z   = des_z;
    request.position.yaw = des_yaw;
  }

  {
    std::scoped_lock lock(mutex_last_position_cmd);

    if (!isPointInSafetyArea3d(request.position.x, request.position.y, request.position.z)) {
      ROS_ERROR("[ControlManager]: 'goto' topic failed, the point is outside of the safety area!");
      return;
    }
  }

  bool tracker_response;

  {
    std::scoped_lock lock(mutex_tracker_list);

    tracker_response = tracker_list[active_tracker_idx]->goTo(mrs_msgs::TrackerPointStamped::ConstPtr(new mrs_msgs::TrackerPointStamped(request)));
  }

  if (!tracker_response) {
    ROS_ERROR("[ControlManager]: The tracker '%s' does not implement 'goto' topic!", tracker_names[active_tracker_idx].c_str());
  }
}

//}

/* //{ callbackGoToFcuService() */

bool ControlManager::callbackGoToFcuService(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res) {

  if (!is_initialized) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  if (!callbacks_enabled) {
    ROS_WARN("[ControlManager]: not passing the goto service through, the callbacks are disabled");
    res.message = "callbacks are disabled";
    res.success = false;
    return true;
  }

  mrs_msgs::Vec4::Request request_in = req;

  // check number validity
  for (int i = 0; i < 4; i++) {
    if (!std::isfinite(request_in.goal[i])) {
      ROS_ERROR("NaN detected in variable \"request_in.goal[%d]\"!!!", i);
      res.message = "NaNs/infs in the goal!";
      res.success = false;
      return true;
    }
  }

  // check the obstacle bumper
  if (!bumperValidatePoint(request_in.goal[REF_X], request_in.goal[REF_Y], request_in.goal[REF_Z], FCU_FRAME)) {
    ROS_ERROR("[ControlManager]: 'goto_fcu' service failed, potential collision with an obstacle!");
    res.message = "potential collision with an obstacle";
    res.success = false;
    return true;
  }

  // prepare the message for current tracker
  mrs_msgs::Vec4Request request;
  Eigen::Vector2d       des(request_in.goal[REF_X], request_in.goal[REF_Y]);

  {
    std::scoped_lock lock(mutex_odometry);

    // rotate it from the frame of the drone
    des = rotateVector(des, odometry_yaw);

    request.goal[REF_X]   = des[REF_X] + odometry_x;
    request.goal[REF_Y]   = des[REF_Y] + odometry_y;
    request.goal[REF_Z]   = request_in.goal[REF_Z] + odometry_z;
    request.goal[REF_YAW] = request_in.goal[REF_YAW] + odometry_yaw;
  }

  // check the safety area
  if (!isPointInSafetyArea3d(request.goal[REF_X], request.goal[REF_Y], request.goal[REF_Z])) {
    ROS_ERROR("[ControlManager]: 'goto_fcu' service failed, the point is outside of the safety area!");
    res.message = "the point is outside of the safety area";
    res.success = false;
    return true;
  }

  mrs_msgs::Vec4Response::ConstPtr tracker_response;
  char                             message[200];

  {
    std::scoped_lock lock(mutex_tracker_list);

    tracker_response = tracker_list[active_tracker_idx]->goTo(mrs_msgs::Vec4Request::ConstPtr(new mrs_msgs::Vec4Request(request)));

    if (tracker_response != mrs_msgs::Vec4Response::Ptr()) {
      res = *tracker_response;
    } else {
      sprintf((char *)&message, "The tracker '%s' does not implement 'goto' service!", tracker_names[active_tracker_idx].c_str());
      res.message = message;
      res.success = false;
    }
  }

  return true;
}

//}

/* //{ callbackGoToFcuTopic() */

void ControlManager::callbackGoToFcuTopic(const mrs_msgs::TrackerPointStampedConstPtr &msg) {

  if (!is_initialized) {
    return;
  }

  if (!callbacks_enabled) {
    ROS_WARN("[ControlManager]: not passing the goto topic through, the callbacks are disabled");
    return;
  }

  if (!std::isfinite(msg->position.x)) {
    ROS_ERROR("NaN detected in variable \"msg->position.x\"!!!");
    return;
  }

  if (!std::isfinite(msg->position.y)) {
    ROS_ERROR("NaN detected in variable \"msg->position.y\"!!!");
    return;
  }

  if (!std::isfinite(msg->position.z)) {
    ROS_ERROR("NaN detected in variable \"msg->position.z\"!!!");
    return;
  }

  if (!std::isfinite(msg->position.yaw)) {
    ROS_ERROR("NaN detected in variable \"msg->position.yaw\"!!!");
    return;
  }

  // copy the original message so we can modify it
  double des_x   = msg->position.x;
  double des_y   = msg->position.y;
  double des_z   = msg->position.z;
  double des_yaw = msg->position.yaw;

  if (!bumperValidatePoint(des_x, des_y, des_z, FCU_FRAME)) {
    ROS_ERROR("[ControlManager]: 'goto_fcu' topic failed, potential collision with an obstacle!");
    return;
  }

  // prepare the message for current tracker
  mrs_msgs::TrackerPointStamped request;
  Eigen::Vector2d               des(des_x, des_y);

  {
    std::scoped_lock lock(mutex_odometry);
    // rotate it from the frame of the drone
    des = rotateVector(des, odometry_yaw);

    request.position.x   = des[0] + odometry_x;
    request.position.y   = des[1] + odometry_y;
    request.position.z   = des_z + odometry_z;
    request.position.yaw = des_yaw + odometry_yaw;
  }

  if (!isPointInSafetyArea3d(request.position.x, request.position.y, request.position.z)) {
    ROS_ERROR("[ControlManager]: 'goto_fcu' topic failed, the point is outside of the safety area!");
    return;
  }

  bool tracker_response;

  {
    std::scoped_lock lock(mutex_tracker_list);

    tracker_response = tracker_list[active_tracker_idx]->goTo(mrs_msgs::TrackerPointStamped::ConstPtr(new mrs_msgs::TrackerPointStamped(request)));

    if (!tracker_response) {
      ROS_ERROR("[ControlManager]: The tracker '%s' does not implement 'goto' topic!", tracker_names[active_tracker_idx].c_str());
    }
  }

  return;
}

//}

/* //{ callbackGoToRelativeService() */

bool ControlManager::callbackGoToRelativeService(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res) {

  if (!is_initialized) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  if (!callbacks_enabled) {
    ROS_WARN("[ControlManager]: not passing the goto_relative service through, the callbacks are disabled");
    res.message = "callbacks are disabled";
    res.success = false;
    return true;
  }

  mrs_msgs::Vec4::Request request_in = req;

  // check number validity
  for (int i = 0; i < 4; i++) {
    if (!std::isfinite(request_in.goal[i])) {
      ROS_ERROR("NaN detected in variable \"request_in.goal[%d]\"!!!", i);
      res.message = "NaNs/infs in the goal!";
      res.success = false;
      return true;
    }
  }

  // check the obstacle bumper
  if (!bumperValidatePoint(request_in.goal[REF_X], request_in.goal[REF_Y], request_in.goal[REF_Z], RELATIVE_FRAME)) {
    ROS_ERROR("[ControlManager]: 'goto_relative' service failed, potential collision with an obstacle!");
    res.message = "potential collision with an obstacle";
    res.success = false;
    return true;
  }

  {
    std::scoped_lock lock(mutex_last_position_cmd, mutex_odometry);

    if (!isPointInSafetyArea3d(last_position_cmd->position.x + request_in.goal[REF_X], last_position_cmd->position.y + request_in.goal[REF_Y],
                               last_position_cmd->position.z + request_in.goal[REF_Z])) {
      ROS_ERROR("[ControlManager]: 'goto_relative' service failed, the point is outside of the safety area!");
      res.message = "the point is outside of the safety area";
      res.success = false;
      return true;
    }
  }

  mrs_msgs::Vec4Response::ConstPtr tracker_response;
  char                             message[200];

  // prepare the message for current tracker
  mrs_msgs::Vec4Request req_goto_out;
  req_goto_out.goal = request_in.goal;

  {
    std::scoped_lock lock(mutex_tracker_list);

    tracker_response = tracker_list[active_tracker_idx]->goToRelative(mrs_msgs::Vec4Request::ConstPtr(new mrs_msgs::Vec4Request(req_goto_out)));

    if (tracker_response != mrs_msgs::Vec4Response::Ptr()) {
      res = *tracker_response;
    } else {
      sprintf((char *)&message, "The tracker '%s' does not implement 'goto_relative' service!", tracker_names[active_tracker_idx].c_str());
      res.message = message;
      res.success = false;
    }
  }

  return true;
}

//}

/* //{ callbackGoToRelativeTopic() */

void ControlManager::callbackGoToRelativeTopic(const mrs_msgs::TrackerPointStampedConstPtr &msg) {

  if (!is_initialized)
    return;

  if (!callbacks_enabled) {
    ROS_WARN("[ControlManager]: not passing the goto_relative topic through, the callbacks are disabled");
    return;
  }

  if (!std::isfinite(msg->position.x)) {
    ROS_ERROR("NaN detected in variable \"msg->position.x\"!!!");
    return;
  }

  if (!std::isfinite(msg->position.y)) {
    ROS_ERROR("NaN detected in variable \"msg->position.y\"!!!");
    return;
  }

  if (!std::isfinite(msg->position.z)) {
    ROS_ERROR("NaN detected in variable \"msg->position.z\"!!!");
    return;
  }

  if (!std::isfinite(msg->position.yaw)) {
    ROS_ERROR("NaN detected in variable \"msg->position.yaw\"!!!");
    return;
  }

  // copy the original message so we can modify it
  double des_x   = msg->position.x;
  double des_y   = msg->position.y;
  double des_z   = msg->position.z;
  double des_yaw = msg->position.yaw;

  // check the obstacle bumper
  ReferenceFrameType_t frame_type;
  if (msg->header.frame_id.compare("fcu") == STRING_EQUAL) {
    frame_type = FCU_FRAME;
  } else {
    frame_type = RELATIVE_FRAME;
  }

  if (!bumperValidatePoint(des_x, des_y, des_z, frame_type)) {
    ROS_ERROR("[ControlManager]: 'goto_relative' topic failed, potential collision with an obstacle!");
    return;
  }

  // prepare the message for current tracker
  mrs_msgs::TrackerPointStamped request;

  if (msg->header.frame_id.compare("fcu") == STRING_EQUAL) {

    // rotate it from the frame of the drone
    Eigen::Vector2d des(des_x, des_y);
    des = rotateVector(des, odometry_yaw);

    {
      std::scoped_lock lock(mutex_last_position_cmd, mutex_odometry);

      request.position.x   = des[0] + odometry_x - last_position_cmd->position.x;
      request.position.y   = des[1] + odometry_y - last_position_cmd->position.y;
      request.position.z   = des_z + odometry_z - last_position_cmd->position.z;
      request.position.yaw = des_yaw + odometry_yaw - last_position_cmd->yaw;
    }

  } else {

    request.position.x   = des_x;
    request.position.y   = des_y;
    request.position.z   = des_z;
    request.position.yaw = des_yaw;
  }

  {
    std::scoped_lock lock(mutex_last_position_cmd, mutex_odometry);

    if (!isPointInSafetyArea3d(last_position_cmd->position.x + request.position.x, last_position_cmd->position.y + request.position.y,
                               last_position_cmd->position.z + request.position.z)) {
      ROS_ERROR("[ControlManager]: 'goto_relative' topic failed, the point is outside of the safety area!");
      return;
    }
  }

  bool tracker_response;

  {
    std::scoped_lock lock(mutex_tracker_list);

    tracker_response = tracker_list[active_tracker_idx]->goToRelative(mrs_msgs::TrackerPointStamped::ConstPtr(new mrs_msgs::TrackerPointStamped(request)));
  }

  if (!tracker_response) {
    ROS_ERROR("[ControlManager]: The tracker '%s' does not implement 'goto_relative' topic!", tracker_names[active_tracker_idx].c_str());
  }
}

//}

/* //{ callbackGoToAltitudeService() */

bool ControlManager::callbackGoToAltitudeService(mrs_msgs::Vec1::Request &req, mrs_msgs::Vec1::Response &res) {

  if (!is_initialized) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  if (!callbacks_enabled) {
    ROS_WARN("[ControlManager]: not passing the goto_altitude service through, the callbacks are disabled");
    res.message = "callbacks are disabled";
    res.success = false;
    return true;
  }

  // check number validity
  if (!std::isfinite(req.goal)) {
    ROS_ERROR("NaN detected in variable \"req.goal\"!!!");
    res.message = "NaNs/infs in the goal!";
    res.success = false;
    return true;
  }

  {
    std::scoped_lock lock(mutex_last_position_cmd, mutex_odometry);

    if (!isPointInSafetyArea3d(last_position_cmd->position.x, last_position_cmd->position.y, req.goal)) {
      ROS_ERROR("[ControlManager]: 'goto_altitude' service failed, the point is outside of the safety area!");
      res.message = "the point is outside of the safety area";
      res.success = false;
      return true;
    }
  }

  mrs_msgs::Vec1Response::ConstPtr tracker_response;
  char                             message[200];

  // prepare the message for current tracker
  mrs_msgs::Vec1Request req_goto_out;
  req_goto_out.goal = req.goal;

  {
    std::scoped_lock lock(mutex_tracker_list);

    tracker_response = tracker_list[active_tracker_idx]->goToAltitude(mrs_msgs::Vec1Request::ConstPtr(new mrs_msgs::Vec1Request(req_goto_out)));

    if (tracker_response != mrs_msgs::Vec1Response::Ptr()) {
      res = *tracker_response;
    } else {
      sprintf((char *)&message, "The tracker '%s' does not implement 'goto_altitude' service!", tracker_names[active_tracker_idx].c_str());
      res.message = message;
      res.success = false;
    }
  }

  return true;
}

//}

/* //{ callbackGoToAltitudeTopic() */

void ControlManager::callbackGoToAltitudeTopic(const std_msgs::Float64ConstPtr &msg) {

  if (!is_initialized)
    return;

  if (!callbacks_enabled) {
    ROS_WARN("[ControlManager]: not passing the goto_altitude topic through, the callbacks are disabled");
    return;
  }

  if (!std::isfinite(msg->data)) {
    ROS_ERROR("NaN detected in variable \"msg->data\"!!!");
    return;
  }

  {
    std::scoped_lock lock(mutex_last_position_cmd, mutex_odometry);

    if (!isPointInSafetyArea3d(last_position_cmd->position.x, last_position_cmd->position.y, msg->data)) {
      ROS_ERROR("[ControlManager]: 'goto_altitude' topic failed, the point is outside of the safety area!");
      return;
    }
  }

  bool tracker_response;

  {
    std::scoped_lock lock(mutex_tracker_list);

    tracker_response = tracker_list[active_tracker_idx]->goToAltitude(std_msgs::Float64::ConstPtr(new std_msgs::Float64(*msg)));
  }

  if (!tracker_response) {
    ROS_ERROR("[ControlManager]: The tracker '%s' does not implement 'goto_altitude' topic!", tracker_names[active_tracker_idx].c_str());
  }
}

//}

/* //{ callbackSetYawService() */

bool ControlManager::callbackSetYawService(mrs_msgs::Vec1::Request &req, mrs_msgs::Vec1::Response &res) {

  if (!is_initialized) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  if (!callbacks_enabled) {
    ROS_WARN("[ControlManager]: not passing the set_yaw service through, the callbacks are disabled");
    res.message = "callbacks are disabled";
    res.success = false;
    return true;
  }

  // check number validity
  if (!std::isfinite(req.goal)) {
    ROS_ERROR("NaN detected in variable \"req.goal\"!!!");
    res.message = "NaNs/infs in the goal!";
    res.success = false;
    return true;
  }

  mrs_msgs::Vec1Response::ConstPtr tracker_response;
  char                             message[200];

  // prepare the message for current tracker
  mrs_msgs::Vec1Request req_goto_out;
  req_goto_out.goal = req.goal;

  {
    std::scoped_lock lock(mutex_tracker_list);

    tracker_response = tracker_list[active_tracker_idx]->setYaw(mrs_msgs::Vec1Request::ConstPtr(new mrs_msgs::Vec1Request(req_goto_out)));

    if (tracker_response != mrs_msgs::Vec1Response::Ptr()) {
      res = *tracker_response;
    } else {
      sprintf((char *)&message, "The tracker '%s' does not implement 'set_yaw' service!", tracker_names[active_tracker_idx].c_str());
      res.message = message;
      res.success = false;
    }
  }

  return true;
}

//}

/* //{ callbackSetYawTopic() */

void ControlManager::callbackSetYawTopic(const std_msgs::Float64ConstPtr &msg) {

  if (!is_initialized)
    return;

  if (!callbacks_enabled) {
    ROS_WARN("[ControlManager]: not passing the set_yaw topic through, the callbacks are disabled");
    return;
  }

  if (!std::isfinite(msg->data)) {
    ROS_ERROR("NaN detected in variable \"msg->data\"!!!");
    return;
  }

  bool tracker_response;

  {
    std::scoped_lock lock(mutex_tracker_list);

    tracker_response = tracker_list[active_tracker_idx]->setYaw(std_msgs::Float64::ConstPtr(new std_msgs::Float64(*msg)));
  }

  if (!tracker_response) {
    ROS_ERROR("[ControlManager]: The tracker '%s' does not implement 'set_yaw' topic!", tracker_names[active_tracker_idx].c_str());
  }
}

//}

/* //{ callbackSetYawRelativeService() */

bool ControlManager::callbackSetYawRelativeService(mrs_msgs::Vec1::Request &req, mrs_msgs::Vec1::Response &res) {

  if (!is_initialized) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  if (!callbacks_enabled) {
    ROS_WARN("[ControlManager]: not passing the set_yaw_relative service through, the callbacks are disabled");
    res.message = "callbacks are disabled";
    res.success = false;
    return true;
  }

  // check number validity
  if (!std::isfinite(req.goal)) {
    ROS_ERROR("NaN detected in variable \"req.goal\"!!!");
    res.message = "NaNs/infs in the goal!";
    res.success = false;
    return true;
  }

  mrs_msgs::Vec1Response::ConstPtr tracker_response;
  char                             message[200];

  // prepare the message for current tracker
  mrs_msgs::Vec1Request req_goto_out;
  req_goto_out.goal = req.goal;

  {
    std::scoped_lock lock(mutex_tracker_list);

    tracker_response = tracker_list[active_tracker_idx]->setYawRelative(mrs_msgs::Vec1Request::ConstPtr(new mrs_msgs::Vec1Request(req_goto_out)));

    if (tracker_response != mrs_msgs::Vec1Response::Ptr()) {
      res = *tracker_response;
    } else {
      sprintf((char *)&message, "The tracker '%s' does not implement 'set_yaw_relative' service!", tracker_names[active_tracker_idx].c_str());
      res.message = message;
      res.success = false;
    }
  }

  return true;
}

//}

/* //{ callbackSetYawRelativeTopic() */

void ControlManager::callbackSetYawRelativeTopic(const std_msgs::Float64ConstPtr &msg) {

  if (!is_initialized)
    return;

  if (!callbacks_enabled) {
    ROS_WARN("[ControlManager]: not passing the set_yaw_relative topic through, the callbacks are disabled");
    return;
  }

  if (!std::isfinite(msg->data)) {
    ROS_ERROR("NaN detected in variable \"msg->data\"!!!");
    return;
  }

  bool tracker_response;

  {
    std::scoped_lock lock(mutex_tracker_list);

    tracker_response = tracker_list[active_tracker_idx]->setYawRelative(std_msgs::Float64::ConstPtr(new std_msgs::Float64(*msg)));
  }

  if (!tracker_response) {
    ROS_ERROR("[ControlManager]: The tracker '%s' does not implement 'set_yaw_relative' topic!", tracker_names[active_tracker_idx].c_str());
  }
}

//}

/* //{ callbackHoverService() */

bool ControlManager::callbackHoverService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized)
    return false;

  res.success = hover(res.message);

  return true;
}

//}

/* callbackUseJoystick() //{ */

bool ControlManager::callbackUseJoystick([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  char message[400];

  mrs_msgs::StringRequest controller_srv;
  controller_srv.value = joystick_controller_name_;

  mrs_msgs::StringRequest tracker_srv;
  tracker_srv.value = joystick_tracker_name_;

  mrs_msgs::StringResponse response;

  callbackSwitchTracker(tracker_srv, response);

  if (!response.success) {

    sprintf((char *)&message, "Switching to %s was unsuccessfull: %s", joystick_tracker_name_.c_str(), response.message.c_str());
    res.success = false;
    res.message = message;

    ROS_ERROR("[ControlManager]: %s", message);

    return true;
  }

  callbackSwitchController(controller_srv, response);

  if (!response.success) {

    sprintf((char *)&message, "Switching to %s was unsuccessfull: %s", joystick_controller_name_.c_str(), response.message.c_str());
    res.success = false;
    res.message = message;

    // switch back to hover tracker
    tracker_srv.value = ehover_tracker_name_;
    callbackSwitchTracker(tracker_srv, response);

    // switch back to safety controller
    controller_srv.value = controller_names[0];
    callbackSwitchController(controller_srv, response);

    ROS_ERROR("[ControlManager]: %s", message);

    return true;
  }

  sprintf((char *)&message, "Switched to joystick control");
  res.success = true;
  res.message = message;

  ROS_INFO("[ControlManager]: %s", message);

  return true;
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* local2fcu() //{ */

Eigen::Vector3d ControlManager::local2fcu(const Eigen::Vector3d in) {

  // get uav rotational transform
  tf::Quaternion quaternion_odometry;
  {
    std::scoped_lock lock(mutex_odometry);

    quaternionMsgToTF(odometry.pose.pose.orientation, quaternion_odometry);
  }

  // rotate the point to world and add the UAV position
  // TODO we should do this using tfs
  tf::Vector3 out = tf::Transform(quaternion_odometry) * tf::Vector3(in[0], in[1], in[2]);
  out += tf::Vector3(odometry_x, odometry_y, odometry_z);

  return Eigen::Vector3d(out[0], out[1], out[2]);
}

//}

/* fcu2local() //{ */

Eigen::Vector3d ControlManager::fcu2local(const Eigen::Vector3d in) {

  // get uav rotational transform
  tf::Quaternion quaternion_odometry;
  {
    std::scoped_lock lock(mutex_odometry);

    quaternionMsgToTF(odometry.pose.pose.orientation, quaternion_odometry);
  }

  quaternion_odometry = quaternion_odometry.inverse();

  // rotate the point to world and add the UAV position
  // TODO we should do this using tfs
  tf::Vector3 out = tf::Vector3(in[0], in[1], in[2]) - tf::Vector3(odometry_x, odometry_y, odometry_z);
  out             = tf::Transform(quaternion_odometry) * out;

  return Eigen::Vector3d(out[0], out[1], out[2]);
}

//}

/* isOffboard() //{ */

bool ControlManager::isOffboard(void) {

  if (got_mavros_state && (ros::Time::now() - mavros_state.header.stamp).toSec() < 1.0 && mavros_state.mode.compare(std::string("OFFBOARD")) == STRING_EQUAL) {

    return true;
  }

  return false;
}

//}

/* shutdown() //{ */

void ControlManager::shutdown() {

  if (automatic_pc_shutdown_enabled) {

    std::scoped_lock lock(mutex_odometry);

    double distance_to_origin = sqrt(pow(odometry.pose.pose.position.x, 2.0) + pow(odometry.pose.pose.position.y, 2.0));

    if (distance_to_origin > automatic_pc_shutdown_threshold) {

      ROS_INFO("[ControlManager]: Calling service for shutdown (DARPA-specific)");

      std_srvs::Trigger shutdown_out;
      service_client_shutdown.call(shutdown_out);
    }
  }
}

//}

/* setCallbacks() //{ */

void ControlManager::setCallbacks(bool in) {

  callbacks_enabled = in;

  std_srvs::SetBoolRequest req_enable_callbacks;
  req_enable_callbacks.data = callbacks_enabled;

  {
    std::scoped_lock lock(mutex_tracker_list);

    // set callbacks to all trackers
    for (unsigned int i = 0; i < tracker_list.size(); i++) {
      tracker_list[i]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));
    }
  }
}

//}

// | ----------------------- safety area ---------------------- |

/* //{ isInSafetyArea3d() */

bool ControlManager::isPointInSafetyArea3d(const double x, const double y, const double z) {

  if (!use_safety_area_) {
    return true;
  }

  std::scoped_lock lock(mutex_max_height, mutex_min_height);

  if (safety_area_polygon->isPointIn(x, y) && z >= min_height && z <= max_height) {
    return true;
  }

  return false;
}

//}

/* //{ isInSafetyArea2d() */

bool ControlManager::isPointInSafetyArea2d(const double x, const double y) {

  if (!use_safety_area_) {
    return true;
  }

  if (safety_area_polygon->isPointIn(x, y)) {
    return true;
  }

  return false;
}

//}

/* //{ getMaxHeight() */

double ControlManager::getMaxHeight(void) {

  std::scoped_lock lock(mutex_max_height);

  return max_height;
}

//}

/* //{ getMinHeight() */

double ControlManager::getMinHeight(void) {

  std::scoped_lock lock(mutex_min_height);

  return min_height;
}

//}

// | --------------------- obstacle bumper -------------------- |

/* bumperValidatePoint() //{ */

// everything here happens in FCU
bool ControlManager::bumperValidatePoint(double &x, double &y, double &z, ReferenceFrameType_t frame) {

  if (!bumper_enabled_) {
    return true;
  }

  if (!got_bumper) {
    return true;
  }

  if ((ros::Time::now() - bumper_data.header.stamp).toSec() > 1.0) {
    return true;
  }

  double fcu_x, fcu_y, fcu_z;
  /* reference to FCU //{ */

  // express the point in FCU
  switch (frame) {

    // leave it as it is
    case FCU_FRAME: {
      fcu_x = x;
      fcu_y = y;
      fcu_z = z;

      break;
    }

    // rotate by yaw
    case RELATIVE_FRAME: {

      Eigen::Vector2d temp = Eigen::Vector2d(x, y);

      mrs_msgs::Vec4Request req_goto_out;
      {
        std::scoped_lock lock(mutex_odometry);
        temp = rotateVector(temp, -odometry_yaw);
      }

      fcu_x = temp[0];
      fcu_y = temp[1];
      fcu_z = z;

      break;
    }

    // offset and rotate by yaw
    case ABSOLUTE_FRAME: {

      Eigen::Vector2d temp;

      mrs_msgs::Vec4Request req_goto_out;
      {
        std::scoped_lock lock(mutex_odometry);
        temp = Eigen::Vector2d(x - odometry_x, y - odometry_y);
        temp = rotateVector(temp, -odometry_yaw);
      }

      fcu_x = temp[0];
      fcu_y = temp[1];
      fcu_z = z - odometry_z;

      break;
    }
  }

  //}

  // get the id of the sector, where the reference is
  int horizontal_vector_idx = bumperGetSectorId(fcu_x, fcu_y, fcu_z);
  int vertical_vector_idx   = fcu_z < 0 ? bumper_data.n_horizontal_sectors : bumper_data.n_horizontal_sectors + 1;

  // calculate the horizontal distance to the point
  double horizontal_point_distance = sqrt(pow(fcu_x, 2.0) + pow(fcu_y, 2.0));
  double vertical_point_distance   = fabs(fcu_z);

  // check whether we measure in that direction
  if (bumper_data.sectors[horizontal_vector_idx] == bumper_data.OBSTACLE_NO_DATA) {

    ROS_WARN("[ControlManager]: Bumper: the fcu reference x: %2.2f, y: %2.2f, z: %2.2f (sector %d) is not valid, we do not measure in that direction", fcu_x,
             fcu_y, fcu_z, horizontal_vector_idx);
    return false;
  }

  if (bumper_data.sectors[horizontal_vector_idx] == bumper_data.OBSTACLE_NOT_DETECTED &&
      bumper_data.sectors[vertical_vector_idx] == bumper_data.OBSTACLE_NOT_DETECTED) {

    return true;
  }

  if (horizontal_point_distance <= (bumper_data.sectors[horizontal_vector_idx] - bumper_horizontal_distance_) &&
      (fabs(fcu_z) <= 0.1 || vertical_point_distance <= (bumper_data.sectors[vertical_vector_idx] - bumper_vertical_distance_))) {

    return true;
  }

  // if the obstacle is too close and hugging can't be done, we can't fly, return false
  if (horizontal_point_distance > 0.1 &&
      (bumper_data.sectors[horizontal_vector_idx] > 0 && bumper_data.sectors[horizontal_vector_idx] <= bumper_horizontal_distance_)) {

    ROS_WARN("[ControlManager]: Bumper: the fcu reference x: %2.2f, y: %2.2f, z: %2.2f (sector %d) is not valid, obstacle is too close (horizontally)", fcu_x,
             fcu_y, fcu_z, horizontal_vector_idx);
    return false;
  }

  // if the obstacle is too close and hugging can't be done, we can't fly, return false
  if (vertical_point_distance > 0.1 &&
      (bumper_data.sectors[vertical_vector_idx] > 0 && bumper_data.sectors[vertical_vector_idx] <= bumper_vertical_distance_)) {

    ROS_WARN("[ControlManager]: Bumper: the fcu reference x: %2.2f, y: %2.2f, z: %2.2f is not valid, obstacle is too close (vertically)", fcu_x, fcu_y, fcu_z);
    return false;
  }

  // otherwise, if hugging enabled, fix the coordinates
  if (bumper_hugging_enabled_) {

    // heading of the point in drone frame
    double point_heading_horizontal = atan2(fcu_y, fcu_x);
    double point_heading_vertical   = fcu_z > 0 ? 1.0 : -1.0;

    double new_x = fcu_x;
    double new_y = fcu_y;
    double new_z = fcu_z;

    if (bumper_data.sectors[horizontal_vector_idx] > 0 &&
        horizontal_point_distance >= (bumper_data.sectors[horizontal_vector_idx] - bumper_horizontal_distance_)) {

      new_x = cos(point_heading_horizontal) * (bumper_data.sectors[horizontal_vector_idx] - bumper_horizontal_distance_);
      new_y = sin(point_heading_horizontal) * (bumper_data.sectors[horizontal_vector_idx] - bumper_horizontal_distance_);

      ROS_WARN(
          "[ControlManager]: Bumper: the fcu reference x: %2.2f, y: %2.2f (sector %d) is not valid, distance %2.2f < (%2.2f - %2.2f)., HUGGING IT it "
          "to x: %2.2f, y: %2.2f",
          fcu_x, fcu_y, horizontal_vector_idx, horizontal_point_distance, bumper_data.sectors[horizontal_vector_idx], bumper_horizontal_distance_, new_x,
          new_y);
    }

    if (bumper_data.sectors[vertical_vector_idx] > 0 && vertical_point_distance >= (bumper_data.sectors[vertical_vector_idx] - bumper_vertical_distance_)) {

      new_z = point_heading_vertical * (bumper_data.sectors[vertical_vector_idx] - bumper_vertical_distance_);

      ROS_WARN("[ControlManager]: Bumper: the fcu reference z: %2.2f is not valid, distance %2.2f < (%2.2f - %2.2f)., HUGGING IT it z: %2.2f", fcu_z,
               vertical_point_distance, bumper_data.sectors[vertical_vector_idx], bumper_vertical_distance_, new_z);
    }

    // express the point back in the original FRAME
    /* FCU to original frame conversion //{ */

    switch (frame) {

      // leave it as it is
      case FCU_FRAME: {

        x = new_x;
        y = new_y;
        z = new_z;

        break;
      }

      // rotate by yaw
      case RELATIVE_FRAME: {

        Eigen::Vector2d temp = Eigen::Vector2d(new_x, new_y);

        mrs_msgs::Vec4Request req_goto_out;
        {
          std::scoped_lock lock(mutex_odometry);
          temp = rotateVector(temp, odometry_yaw);
        }

        x = temp[0];
        y = temp[1];
        z = new_z;

        break;
      }

      // offset and rotate by yaw
      case ABSOLUTE_FRAME: {

        Eigen::Vector2d temp;

        mrs_msgs::Vec4Request req_goto_out;
        {
          std::scoped_lock lock(mutex_odometry);
          temp = Eigen::Vector2d(new_x, new_y);
          temp = rotateVector(temp, odometry_yaw);
          temp += Eigen::Vector2d(odometry_x, odometry_y);
        }

        x = temp[0];
        y = temp[1];
        z = new_z + odometry_z;

        break;
      }
    }

    //}

    return true;

  } else {

    return false;
  }
}

//}

/* bumperPushFromObstacle() //{ */

bool ControlManager::bumperPushFromObstacle(void) {

  if (!bumper_enabled_) {
    return true;
  }

  if (!bumper_repulsion_enabled_) {
    return true;
  }

  if (!got_bumper) {
    return true;
  }

  std::scoped_lock lock(mutex_bumper);

  double sector_size = TAU / double(bumper_data.n_horizontal_sectors);

  double direction;
  double min_distance                  = 10e9;
  double repulsion_distance            = 10e9;
  double horizontal_collision_detected = false;

  double vertical_collision_detected = false;

  for (uint i = 0; i < bumper_data.n_horizontal_sectors; i++) {

    if (bumper_data.sectors[i] < 0) {
      continue;
    }

    double wall_locked_horizontal = false;

    // if the sector is under critical distance
    if (bumper_data.sectors[i] <= bumper_repulsion_horizontal_distance_ && bumper_data.sectors[i] < repulsion_distance) {

      // check for locking between the oposite walls
      // get the desired direction of motion
      double oposite_direction  = double(i) * sector_size + M_PI;
      int    oposite_sector_idx = bumperGetSectorId(cos(oposite_direction), sin(oposite_direction), 0);

      if (bumper_data.sectors[oposite_sector_idx] > 0 && ((bumper_data.sectors[i] + bumper_data.sectors[oposite_sector_idx]) <=
                                                          (2 * bumper_repulsion_horizontal_distance_ + 2 * bumper_repulsion_horizontal_offset_))) {

        wall_locked_horizontal = true;

        if (fabs(bumper_data.sectors[i] - bumper_data.sectors[oposite_sector_idx]) <= 2 * bumper_repulsion_horizontal_offset_) {

          ROS_INFO_THROTTLE(1.0, "[ControlManager]: bumper locked between two walls");
          continue;
        }
      }

      // get the id of the oposite sector
      direction = oposite_direction;

      /* int oposite_sector_idx = (i + bumper_data.n_horizontal_sectors / 2) % bumper_data.n_horizontal_sectors; */

      ROS_WARN_THROTTLE(1.0, "[ControlManager]: found potential collision (sector %d vs. %d), obstacle distance: %2.2f, repulsing", i, oposite_sector_idx,
                        bumper_data.sectors[i]);

      ROS_INFO_THROTTLE(1.0, "[ControlManager]: oposite direction: %2.2f", oposite_direction);

      if (wall_locked_horizontal) {
        if (bumper_data.sectors[i] < bumper_data.sectors[oposite_sector_idx]) {
          repulsion_distance = bumper_repulsion_horizontal_offset_;
        } else {
          repulsion_distance = -bumper_repulsion_horizontal_offset_;
        }
      } else {
        repulsion_distance = bumper_repulsion_horizontal_distance_ + bumper_repulsion_horizontal_offset_ - bumper_data.sectors[i];
      }

      min_distance = bumper_data.sectors[i];

      repulsing_from = i;

      horizontal_collision_detected = true;
    }
  }

  bool   collision_above             = false;
  bool   collision_below             = false;
  bool   wall_locked_vertical        = false;
  double vertical_repulsion_distance = 0;

  // check for vertical collision down
  if (bumper_data.sectors[bumper_data.n_horizontal_sectors] > 0 &&
      bumper_data.sectors[bumper_data.n_horizontal_sectors] <= bumper_repulsion_vertical_distance_) {

    ROS_INFO_THROTTLE(1.0, "[ControlManager]: bumper: potential collision below");
    collision_above             = true;
    vertical_collision_detected = true;
    vertical_repulsion_distance = bumper_repulsion_vertical_distance_ - bumper_data.sectors[bumper_data.n_horizontal_sectors];
  }

  // check for vertical collision up
  if (bumper_data.sectors[bumper_data.n_horizontal_sectors + 1] > 0 &&
      bumper_data.sectors[bumper_data.n_horizontal_sectors + 1] <= bumper_repulsion_vertical_distance_) {

    ROS_INFO_THROTTLE(1.0, "[ControlManager]: bumper: potential collision above");
    collision_below             = true;
    vertical_collision_detected = true;
    vertical_repulsion_distance = -(bumper_repulsion_vertical_distance_ - bumper_data.sectors[bumper_data.n_horizontal_sectors + 1]);
  }

  // check the up/down wall locking
  if (collision_above && collision_below) {

    if (((bumper_data.sectors[bumper_data.n_horizontal_sectors] + bumper_data.sectors[bumper_data.n_horizontal_sectors + 1]) <=
         (2 * bumper_repulsion_vertical_distance_ + 2 * bumper_repulsion_vertical_offset_))) {

      wall_locked_vertical = true;

      vertical_repulsion_distance = (-bumper_data.sectors[bumper_data.n_horizontal_sectors] + bumper_data.sectors[bumper_data.n_horizontal_sectors + 1]) / 2.0;

      /* // should we repulse up or down? */
      /* if (bumper_data.sectors[bumper_data.n_horizontal_sectors] < bumper_data.sectors[bumper_data.n_horizontal_sectors+1]) { */
      /*   vertical_repulsion_distance = bumper_repulsion_vertical_offset_; */
      /* } else { */
      /*   vertical_repulsion_distance = -bumper_repulsion_vertical_offset_; */
      /* } */

      if (fabs(bumper_data.sectors[bumper_data.n_horizontal_sectors] - bumper_data.sectors[bumper_data.n_horizontal_sectors + 1]) <=
          2 * bumper_repulsion_vertical_offset_) {

        ROS_INFO_THROTTLE(1.0, "[ControlManager]: bumper locked between the floor and ceiling");
        vertical_collision_detected = false;
      }
    }
  }

  // if potential collision was detected and we should start the repulsing
  if (horizontal_collision_detected || vertical_collision_detected) {

    ROS_INFO("[ControlManager]: repulsing was initiated");

    repulsing = true;

    callbacks_enabled = false;

    mrs_msgs::Vec4Response::ConstPtr tracker_response;

    std_srvs::SetBoolRequest req_enable_callbacks;

    Eigen::Vector2d des = Eigen::Vector2d(cos(direction) * repulsion_distance, sin(direction) * repulsion_distance);

    mrs_msgs::Vec4Request req_goto_out;
    {
      std::scoped_lock lock(mutex_odometry);

      // rotate it from the frame of the drone
      des = rotateVector(des, odometry_yaw);

      if (horizontal_collision_detected) {
        req_goto_out.goal[REF_X] = des[0];
        req_goto_out.goal[REF_Y] = des[1];
      }

      if (vertical_collision_detected) {
        req_goto_out.goal[REF_Z] = vertical_repulsion_distance;
      }

      req_goto_out.goal[REF_YAW] = 0;
    }

    {
      std::scoped_lock lock(mutex_tracker_list);

      // disable callbacks of all trackers
      req_enable_callbacks.data = false;
      for (unsigned int i = 0; i < tracker_list.size(); i++) {
        tracker_list[i]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));
      }

      // enable the callbacks for the active tracker
      req_enable_callbacks.data = true;
      tracker_list[active_tracker_idx]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));

      // call the goto
      tracker_response = tracker_list[active_tracker_idx]->goToRelative(mrs_msgs::Vec4Request::ConstPtr(new mrs_msgs::Vec4Request(req_goto_out)));

      // disable the callbacks back again
      req_enable_callbacks.data = false;
      tracker_list[active_tracker_idx]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));

      // TODO check responses?
    }
  }

  // if repulsing and the distance is safe once again
  if ((repulsing && !horizontal_collision_detected && !vertical_collision_detected)) {

    ROS_INFO("[ControlManager]: repulsing was stopped");

    std_srvs::SetBoolRequest req_enable_callbacks;

    {
      std::scoped_lock lock(mutex_tracker_list);

      // enable callbacks of all trackers
      req_enable_callbacks.data = true;
      for (unsigned int i = 0; i < tracker_list.size(); i++) {
        tracker_list[i]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(new std_srvs::SetBoolRequest(req_enable_callbacks)));
      }

      // TODO check responses?
    }

    callbacks_enabled = true;

    repulsing = false;
  }

  return false;
}

//}

/* bumperGetSectorId() //{ */

int ControlManager::bumperGetSectorId(const double x, const double y, [[maybe_unused]] const double z) {

  std::scoped_lock lock(mutex_odometry);  // TODO check if it can be here

  // heading of the point in drone frame
  double point_heading_horizontal = atan2(y, x);

  point_heading_horizontal += TAU;

  // if point_heading_horizontal is greater then 2*M_PI mod it
  if (fabs(point_heading_horizontal) >= TAU) {
    point_heading_horizontal = fmod(point_heading_horizontal, TAU);
  }

  // heading of the right edge of the first sector
  double sector_size = TAU / double(bumper_data.n_horizontal_sectors);

  // calculate the idx
  int idx = floor((point_heading_horizontal + (sector_size / 2.0)) / sector_size);

  if (uint(idx) > bumper_data.n_horizontal_sectors - 1) {
    idx -= bumper_data.n_horizontal_sectors;
  }

  return idx;
}

//}

// | ------------------------ elanding ------------------------ |

/* //{ changeLandingState() */

void ControlManager::changeLandingState(LandingStates_t new_state) {

  previous_state_landing = current_state_landing;
  current_state_landing  = new_state;

  switch (current_state_landing) {

    case IDLE_STATE:
      break;
    case LANDING_STATE: {

      elanding_timer.start();
      eland_triggered = true;

      if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {
        landing_uav_mass_ = uav_mass_;
      } else {
        landing_uav_mass_ = uav_mass_ + last_attitude_cmd->mass_difference;
      }
    }

    break;
  }

  ROS_INFO("[ControlManager]: Switching emergancy landing state %s -> %s", state_names[previous_state_landing], state_names[current_state_landing]);
}

//}

/* hover() //{ */

bool ControlManager::hover(std::string &message_out) {

  if (!is_initialized) {

    message_out = std::string("ControlManager is not initialized");
    return false;
  }

  std::scoped_lock lock(mutex_tracker_list);

  std_srvs::TriggerResponse::ConstPtr tracker_response;
  char                                message[200];

  std_srvs::TriggerRequest hover_out;

  tracker_response = tracker_list[active_tracker_idx]->hover(std_srvs::TriggerRequest::ConstPtr(new std_srvs::TriggerRequest(hover_out)));

  if (tracker_response != std_srvs::TriggerResponse::Ptr()) {

    message_out = tracker_response->message;
    return tracker_response->success;

  } else {

    sprintf((char *)&message, "The tracker '%s' does not implement 'goto' service!", tracker_names[active_tracker_idx].c_str());
    message_out = std::string(message);
    return false;
  }
}

//}

/* //{ ehover() */

bool ControlManager::ehover(std::string &message_out) {

  if (!is_initialized)
    return false;

  std::scoped_lock lock(mutex_tracker_list, mutex_last_position_cmd, mutex_last_attitude_cmd, mutex_controller_list);

  char message[200];
  bool success = false;

  try {

    // check if the tracker is not active
    if (ehover_tracker_idx == active_tracker_idx) {

      sprintf((char *)&message, "Not switching, the tracker %s is already active!", ehover_tracker_name_.c_str());
      ROS_WARN("[ControlManager]: %s", message);

    } else {

      ROS_INFO("[ControlManager]: Activating tracker %s", tracker_names[ehover_tracker_idx].c_str());
      tracker_list[ehover_tracker_idx]->activate(last_position_cmd);
      sprintf((char *)&message, "Tracker %s has been activated", ehover_tracker_name_.c_str());
      ROS_INFO("[ControlManager]: %s", message);

      {
        std::scoped_lock lock(mutex_controller_tracker_switch_time);

        // update the time (used in failsafe)
        controller_tracker_switch_time = ros::Time::now();
      }

      // super important, switch the active tracker idx
      try {

        tracker_list[active_tracker_idx]->deactivate();
        active_tracker_idx = ehover_tracker_idx;

        success = true;
      }
      catch (std::runtime_error &exrun) {

        sprintf((char *)&message, "[ControlManager]: Could not deactivate tracker %s", tracker_names[active_tracker_idx].c_str());
        ROS_ERROR("[ControlManager]: %s", message);

        message_out = std::string(message);
        success     = false;
      }
    }
  }
  catch (std::runtime_error &exrun) {

    sprintf((char *)&message, "[ControlManager]: Error during activation of tracker %s", ehover_tracker_name_.c_str());
    ROS_ERROR("[ControlManager]: %s", message);
    ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());

    message_out = std::string(message);
    success     = false;
  }

  try {

    ROS_INFO("[ControlManager]: Activating controller %s", controller_names[eland_controller_idx].c_str());

    // check if the controller is not active
    if (eland_controller_idx == active_controller_idx) {

      sprintf((char *)&message, "Not switching, the controller %s is already active!", eland_controller_name_.c_str());
      ROS_WARN("[ControlManager]: %s", message);

    } else {

      controller_list[eland_controller_idx]->activate(last_attitude_cmd);
      sprintf((char *)&message, "Controller %s has been activated", controller_names[eland_controller_idx].c_str());
      ROS_INFO("[ControlManager]: %s", message);

      {
        std::scoped_lock lock(mutex_controller_tracker_switch_time);

        // update the time (used in failsafe)
        controller_tracker_switch_time = ros::Time::now();
      }

      try {

        // deactivate the old controller
        controller_list[active_controller_idx]->deactivate();
        active_controller_idx = eland_controller_idx;  // super important

        success = true;
      }
      catch (std::runtime_error &exrun) {

        sprintf((char *)&message, "[ControlManager]: Could not deactivate controller %s", tracker_names[active_tracker_idx].c_str());
        ROS_ERROR("[ControlManager]: %s", message);

        message_out = std::string(message);
        success     = false;
      }
    }
  }
  catch (std::runtime_error &exrun) {

    sprintf((char *)&message, "[ControlManager]: Error during activation of controller %s", ehover_tracker_name_.c_str());
    ROS_ERROR("[ControlManager]: %s", message);
    ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());

    message_out = std::string(message);
    success     = false;
  }

  if (success) {
    sprintf((char *)&message, "[ControlManager]: ehover activated.");
    message_out = std::string(message);
  }

  return success;
}

//}

/* eland() //{ */

bool ControlManager::eland(std::string &message_out) {

  if (!is_initialized)
    return false;

  if (eland_triggered || failsafe_triggered) {
    return false;
  }

  std::scoped_lock lock(mutex_tracker_list, mutex_controller_list, mutex_last_attitude_cmd, mutex_last_position_cmd);

  char message[200];
  bool success = false;

  try {

    // check if the tracker is not active
    if (ehover_tracker_idx == active_tracker_idx) {

      sprintf((char *)&message, "Not switching, the tracker %s is already active!", ehover_tracker_name_.c_str());
      ROS_WARN("[ControlManager]: %s", message);

    } else {

      ROS_INFO("[ControlManager]: Activating tracker %s", tracker_names[ehover_tracker_idx].c_str());
      tracker_list[ehover_tracker_idx]->activate(last_position_cmd);
      sprintf((char *)&message, "Tracker %s has been activated", ehover_tracker_name_.c_str());
      ROS_INFO("[ControlManager]: %s", message);

      {
        std::scoped_lock lock(mutex_controller_tracker_switch_time);

        // update the time (used in failsafe)
        controller_tracker_switch_time = ros::Time::now();
      }

      // super important, switch the active tracker idx
      try {

        tracker_list[active_tracker_idx]->deactivate();
        active_tracker_idx = ehover_tracker_idx;

        success = true;
      }
      catch (std::runtime_error &exrun) {

        sprintf((char *)&message, "[ControlManager]: Could not deactivate tracker %s", tracker_names[active_tracker_idx].c_str());
        ROS_ERROR("[ControlManager]: %s", message);

        message_out = std::string(message);
        success     = false;
      }
    }
  }
  catch (std::runtime_error &exrun) {

    sprintf((char *)&message, "[ControlManager]: Error during activation of tracker %s", ehover_tracker_name_.c_str());
    ROS_ERROR("[ControlManager]: %s", message);
    ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());

    message_out = std::string(message);
    success     = false;
  }

  try {

    ROS_INFO("[ControlManager]: Activating controller %s", controller_names[eland_controller_idx].c_str());

    // check if the controller is not active
    if (eland_controller_idx == active_controller_idx) {

      sprintf((char *)&message, "Not switching, the controller %s is already active!", eland_controller_name_.c_str());
      ROS_WARN("[ControlManager]: %s", message);

    } else {

      controller_list[eland_controller_idx]->activate(last_attitude_cmd);
      sprintf((char *)&message, "Controller %s has been activated", controller_names[eland_controller_idx].c_str());
      ROS_INFO("[ControlManager]: %s", message);

      {
        std::scoped_lock lock(mutex_controller_tracker_switch_time);

        // update the time (used in failsafe)
        controller_tracker_switch_time = ros::Time::now();
      }

      try {

        controller_list[active_controller_idx]->deactivate();
        active_controller_idx = eland_controller_idx;  // super important

        success = true;
      }
      catch (std::runtime_error &exrun) {

        sprintf((char *)&message, "[ControlManager]: Could not deactivate controller %s", tracker_names[active_tracker_idx].c_str());
        ROS_ERROR("[ControlManager]: %s", message);

        message_out = std::string(message);
        success     = false;
      }
    }
  }
  catch (std::runtime_error &exrun) {

    sprintf((char *)&message, "[ControlManager]: Error during activation of controller %s", ehover_tracker_name_.c_str());
    ROS_ERROR("[ControlManager]: %s", message);
    ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());

    message_out = std::string(message);
    success     = false;
  }

  std_srvs::Trigger eland_out;
  service_client_eland.call(eland_out);

  if (eland_out.response.success) {

    changeLandingState(LANDING_STATE);
    sprintf((char *)&message, "[ControlManager]: eland activated.");
    message_out = std::string(message);

  } else {

    sprintf((char *)&message, "[ControlManager]: Error during activation of eland: %s", eland_out.response.message.c_str());
    ROS_ERROR("[ControlManager]: %s", message);

    message_out = std::string(message);
    success     = false;
  }

  return success;
}

//}

/* failsafe() //{ */

// needs locking:
//  mutex_controller_list
//  mutex_last_attitude_cmd
bool ControlManager::failsafe() {

  if (!is_initialized)
    return false;

  if (failsafe_triggered) {
    return false;
  }

  if (failsafe_controller_idx != active_controller_idx) {

    try {

      ROS_INFO("[ControlManager]: Activating controller %s", failsafe_controller_name_.c_str());
      controller_list[failsafe_controller_idx]->activate(last_attitude_cmd);

      {
        std::scoped_lock lock(mutex_controller_tracker_switch_time);

        // update the time (used in failsafe)
        controller_tracker_switch_time = ros::Time::now();
      }

      failsafe_triggered = true;
      elanding_timer.stop();

      if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {
        landing_uav_mass_ = uav_mass_;
      } else {
        landing_uav_mass_ = uav_mass_ + last_attitude_cmd->mass_difference;
      }

      eland_triggered = false;
      failsafe_timer.start();

      ROS_INFO("[ControlManager]: Controller %s has been activated", failsafe_controller_name_.c_str());

      // super important, switch the active controller idx
      try {

        controller_list[active_controller_idx]->deactivate();
        active_controller_idx = failsafe_controller_idx;
      }
      catch (std::runtime_error &exrun) {
        ROS_ERROR("[ControlManager]: Could not deactivate controller %s", controller_names[active_tracker_idx].c_str());
      }
    }
    catch (std::runtime_error &exrun) {
      ROS_ERROR("[ControlManager]: Error during activation of controller %s", failsafe_controller_name_.c_str());
      ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
    }
  }

  return true;
}

//}

/* arming() //{ */

bool ControlManager::arming(bool input) {

  mavros_msgs::CommandBool srv_out;
  srv_out.request.value = input;

  // if disarming, switch motors off
  if (!input) {
    switchMotors(false);
  }

  failsafe_timer.stop();
  elanding_timer.stop();

  // we cannot disarm if the drone is not in offboard mode
  {
    std::scoped_lock lock(mutex_mavros_state);

    if (isOffboard()) {

      ROS_INFO("[ControlManager]: calling for disarming");

      service_client_arm.call(srv_out);

      shutdown();

      return srv_out.response.success;

    } else {

      ROS_WARN("[ControlManager]: cannot disarm, not in OFFBOARD mode.");
      return false;
    }
  }
}

//}

/* switchMotors() //{ */

void ControlManager::switchMotors(bool input) {

  ROS_INFO("[ControlManager]: switching motors %s", input ? "ON" : "OFF");

  // set 'enable motors' to the desired value
  motors = input;

  // if switching motors off, switch to NullTracker
  if (!motors) {

    // request
    mrs_msgs::StringRequest request;
    request.value = null_tracker_name_;

    // response (not used)
    mrs_msgs::StringResponse response;

    ROS_INFO("[ControlManager]: switching to NullTracker after switching motors off");

    callbackSwitchTracker(request, response);

    // request
    request.value = controller_names[eland_controller_idx];

    ROS_INFO("[ControlManager]: switching to %s after switching motors off", request.value.c_str());

    callbackSwitchController(request, response);
  }
}

//}

/* updateTrackers() //{ */

void ControlManager::updateTrackers(void) {

  // --------------------------------------------------------------
  // |                     Update the trackers                    |
  // --------------------------------------------------------------

  std::scoped_lock lock(mutex_odometry, mutex_last_position_cmd, mutex_tracker_list);

  mrs_msgs::PositionCommand::ConstPtr tracker_output_cmd;

  nav_msgs::Odometry::ConstPtr odometry_const_ptr(new nav_msgs::Odometry(odometry));

  try {

    // for each tracker
    for (unsigned int i = 0; i < tracker_list.size(); i++) {

      if ((int)i == active_tracker_idx) {

        // if it is the active one, update and retrieve the command
        tracker_output_cmd = tracker_list[i]->update(odometry_const_ptr);

      } else {

        // if it is not the active one, just update without retrieving the command
        tracker_list[i]->update(odometry_const_ptr);
      }
    }

    if (mrs_msgs::PositionCommand::Ptr() != tracker_output_cmd) {

      last_position_cmd = tracker_output_cmd;

    } else if (active_tracker_idx != null_tracker_idx) {

      ROS_WARN_THROTTLE(1.0, "[ControlManager]: The tracker %s returned empty command!", tracker_names[active_tracker_idx].c_str());

      std::scoped_lock lock(mutex_controller_list, mutex_last_attitude_cmd);

      failsafe();

    } else if (active_tracker_idx == null_tracker_idx) {

      last_position_cmd = tracker_output_cmd;
    }
  }
  catch (std::runtime_error &exrun) {
    ROS_INFO("[ControlManager]: Exception while updateing trackers.");
    ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
  }
}

//}

/* updateControllers() //{ */

void ControlManager::updateControllers(nav_msgs::Odometry odom_for_control) {

  // --------------------------------------------------------------
  // |                   Update the controller                    |
  // --------------------------------------------------------------

  std::scoped_lock lock(mutex_last_position_cmd, mutex_controller_list);

  nav_msgs::Odometry::ConstPtr odometry_const_ptr(new nav_msgs::Odometry(odom_for_control));

  mrs_msgs::AttitudeCommand::ConstPtr controller_output_cmd;

  if (last_position_cmd != mrs_msgs::PositionCommand::Ptr()) {

    try {

      // for each controller
      for (unsigned int i = 0; i < controller_list.size(); i++) {

        if ((int)i == active_controller_idx) {

          // if it is the active one, update and retrieve the command
          controller_output_cmd = controller_list[active_controller_idx]->update(odometry_const_ptr, last_position_cmd);

        } else {

          // if it is not the active one, just update without retrieving the command
          controller_list[i]->update(odometry_const_ptr, last_position_cmd);
        }
      }

      // in normal sitation, the controller returns a valid command
      if (controller_output_cmd != mrs_msgs::AttitudeCommand::Ptr()) {

        std::scoped_lock lock(mutex_last_attitude_cmd);

        last_attitude_cmd = controller_output_cmd;

        // but it can return an empty command
        // which means we should trigger the failsafe landing
      } else {

        ROS_WARN("[ControlManager]: triggering failsafe, the controller returned null");

        std::scoped_lock lock(mutex_last_attitude_cmd);
        failsafe();
      }
    }
    catch (std::runtime_error &exrun) {

      ROS_INFO("[ControlManager]: Exception while updating the active controller.");
      ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());

      ROS_WARN("[ControlManager]: triggering failsafe due to an exception in the controller");

      std::scoped_lock lock(mutex_last_attitude_cmd);
      failsafe();
    }
  }
}

//}

/* publish() //{ */

void ControlManager::publish(void) {

  std::scoped_lock lock(mutex_last_attitude_cmd, mutex_last_position_cmd);

  tf::Quaternion desired_orientation;

  // --------------------------------------------------------------
  // |                  publish the position cmd                  |
  // --------------------------------------------------------------

  if (last_position_cmd != mrs_msgs::PositionCommand::Ptr()) {

    // publish the odom topic (position command for debugging, e.g. rviz)
    nav_msgs::Odometry cmd_odom;

    cmd_odom.header.stamp         = ros::Time::now();
    cmd_odom.header.frame_id      = "local_origin";
    cmd_odom.pose.pose.position   = last_position_cmd->position;
    cmd_odom.twist.twist.linear.x = last_position_cmd->velocity.x;
    cmd_odom.twist.twist.linear.y = last_position_cmd->velocity.y;
    cmd_odom.twist.twist.linear.z = last_position_cmd->velocity.z;

    // | -------------- prepared desired orientation -------------- |

    if (last_attitude_cmd != mrs_msgs::AttitudeCommand::Ptr()) {

      // when controlling with quaternion or attitude rates, the quaternion should be filled in
      if (last_attitude_cmd->mode_mask == last_attitude_cmd->MODE_QUATER_ATTITUDE || last_attitude_cmd->mode_mask == last_attitude_cmd->MODE_ATTITUDE_RATE) {

        desired_orientation.setX(last_attitude_cmd->quter_attitude.x);
        desired_orientation.setY(last_attitude_cmd->quter_attitude.y);
        desired_orientation.setZ(last_attitude_cmd->quter_attitude.z);
        desired_orientation.setW(last_attitude_cmd->quter_attitude.w);

      } else if (last_attitude_cmd->mode_mask == last_attitude_cmd->MODE_EULER_ATTITUDE) {  // when controlling with euler attitude, convert it to quaternion

        desired_orientation =
            tf::createQuaternionFromRPY(last_attitude_cmd->euler_attitude.x, last_attitude_cmd->euler_attitude.y, last_attitude_cmd->euler_attitude.z);
      } else {

        // else use just the yaw from position command
        desired_orientation = tf::createQuaternionFromRPY(0, 0, last_position_cmd->yaw);
      }

    } else {

      // else use just the yaw from position command
      desired_orientation = tf::createQuaternionFromRPY(0, 0, last_position_cmd->yaw);
    }

    desired_orientation.normalize();

    // fill in the desired orientation
    quaternionTFToMsg(desired_orientation, cmd_odom.pose.pose.orientation);

    try {
      publisher_cmd_odom.publish(nav_msgs::OdometryConstPtr(new nav_msgs::Odometry(cmd_odom)));
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher_cmd_odom.getTopic().c_str());
    }

    // publish the full command structure
    try {
      publisher_position_cmd.publish(mrs_msgs::PositionCommandConstPtr(last_position_cmd));  // the last_position_cmd is already a ConstPtr
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher_position_cmd.getTopic().c_str());
    }
  }

  // --------------------------------------------------------------
  // |                 Publish the control command                |
  // --------------------------------------------------------------

  mavros_msgs::AttitudeTarget attitude_target;
  attitude_target.header.stamp    = ros::Time::now();
  attitude_target.header.frame_id = "base_link";

  bool should_publish = false;

  if (!motors) {

    ROS_WARN_THROTTLE(1.0, "[ControlManager]: motors are off");

    should_publish = false;

  } else if (active_tracker_idx == null_tracker_idx) {

    ROS_WARN_THROTTLE(1.0, "[ControlManager]: NullTracker is active, publishing zeros...");

    desired_orientation = tf::createQuaternionFromRPY(odometry_roll, odometry_pitch, odometry_yaw);
    desired_orientation.normalize();
    quaternionTFToMsg(desired_orientation, attitude_target.orientation);

    attitude_target.thrust = min_thrust_null_tracker_;

    should_publish = true;

  } else if (active_tracker_idx != null_tracker_idx && last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN_THROTTLE(1.0, "[ControlManager]: the controller (%s) returned nil command! Not publishing anything...",
                      controller_names[active_controller_idx].c_str());

    // convert the RPY to quaternion
    desired_orientation = tf::createQuaternionFromRPY(odometry_roll, odometry_pitch, odometry_yaw);
    desired_orientation.normalize();
    quaternionTFToMsg(desired_orientation, attitude_target.orientation);

    attitude_target.thrust = min_thrust_null_tracker_;

    should_publish = true;

  } else if (last_attitude_cmd != mrs_msgs::AttitudeCommand::Ptr()) {

    attitude_target.thrust = last_attitude_cmd->thrust;

    if (last_attitude_cmd->mode_mask == last_attitude_cmd->MODE_EULER_ATTITUDE) {

      // convert the RPY to quaternion
      desired_orientation =
          tf::createQuaternionFromRPY(last_attitude_cmd->euler_attitude.x, last_attitude_cmd->euler_attitude.y, last_attitude_cmd->euler_attitude.z);

      desired_orientation.normalize();
      quaternionTFToMsg(desired_orientation, attitude_target.orientation);

      attitude_target.body_rate.x = 0.0;
      attitude_target.body_rate.y = 0.0;
      attitude_target.body_rate.z = 0.0;

      attitude_target.type_mask = attitude_target.IGNORE_YAW_RATE | attitude_target.IGNORE_ROLL_RATE | attitude_target.IGNORE_PITCH_RATE;

    } else if (last_attitude_cmd->mode_mask == last_attitude_cmd->MODE_QUATER_ATTITUDE) {

      attitude_target.orientation = last_attitude_cmd->quter_attitude;

      attitude_target.body_rate.x = 0.0;
      attitude_target.body_rate.y = 0.0;
      attitude_target.body_rate.z = 0.0;

      attitude_target.type_mask = attitude_target.IGNORE_YAW_RATE | attitude_target.IGNORE_ROLL_RATE | attitude_target.IGNORE_PITCH_RATE;

    } else if (last_attitude_cmd->mode_mask == last_attitude_cmd->MODE_ATTITUDE_RATE) {

      attitude_target.body_rate.x = last_attitude_cmd->attitude_rate.x;
      attitude_target.body_rate.y = last_attitude_cmd->attitude_rate.y;
      attitude_target.body_rate.z = last_attitude_cmd->attitude_rate.z;

      attitude_target.orientation = last_attitude_cmd->quter_attitude;

      attitude_target.type_mask = attitude_target.IGNORE_ATTITUDE;

      // when controlling with angular rates, PixHawk does not publish the
      // target_attitude topic anymore, thus we need do it here
      try {
        publisher_target_attitude.publish(mavros_msgs::AttitudeTargetConstPtr(new mavros_msgs::AttitudeTarget(attitude_target)));
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_control_output.getTopic().c_str());
      }
    }

    should_publish = true;
  } else {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: not publishing a control command");
  }

  if (should_publish) {

    // test the output
    if (!std::isfinite(attitude_target.orientation.x)) {
      ROS_ERROR("NaN detected in variable \"attitude_target.orientation.x\"!!!");
      return;
    }
    if (!std::isfinite(attitude_target.orientation.y)) {
      ROS_ERROR("NaN detected in variable \"attitude_target.orientation.y\"!!!");
      return;
    }
    if (!std::isfinite(attitude_target.orientation.z)) {
      ROS_ERROR("NaN detected in variable \"attitude_target.orientation.z\"!!!");
      return;
    }
    if (!std::isfinite(attitude_target.orientation.w)) {
      ROS_ERROR("NaN detected in variable \"attitude_target.orientation.w\"!!!");
      return;
    }
    if (!std::isfinite(attitude_target.thrust)) {
      ROS_ERROR("NaN detected in variable \"attitude_target.thrust\"!!!");
      return;
    }

    try {

      publisher_control_output.publish(mavros_msgs::AttitudeTargetConstPtr(new mavros_msgs::AttitudeTarget(attitude_target)));
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher_control_output.getTopic().c_str());
    }
  }

  // | --------- publish the attitude_cmd for debugging --------- |

  if (last_attitude_cmd != mrs_msgs::AttitudeCommand::Ptr()) {
    try {
      publisher_attitude_cmd.publish(mrs_msgs::AttitudeCommandConstPtr(last_attitude_cmd));  // the control command is already a ConstPtr
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher_attitude_cmd.getTopic().c_str());
    }
  }

  // | ------------ publish the desired thrust force ------------ |

  if (last_attitude_cmd != mrs_msgs::AttitudeCommand::Ptr()) {

    mrs_msgs::Float64Stamped thrust_out;
    thrust_out.header.stamp = ros::Time::now();
    thrust_out.value        = (pow((last_attitude_cmd->thrust - motor_params_.hover_thrust_b) / motor_params_.hover_thrust_a, 2) / g_) * 10.0;

    try {
      publisher_thrust_force.publish(thrust_out);
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher_thrust_force.getTopic().c_str());
    }
  }
}

//}

/* rotateVector() //{ */

Eigen::Vector2d ControlManager::rotateVector(const Eigen::Vector2d vector_in, double angle) {

  Eigen::Rotation2D<double> rot2(angle);

  return rot2.toRotationMatrix() * vector_in;
}

//}

}  // namespace control_manager

}  // namespace mrs_uav_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_manager::control_manager::ControlManager, nodelet::Nodelet)
