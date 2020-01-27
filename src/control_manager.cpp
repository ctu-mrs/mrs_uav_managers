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
#include <mrs_msgs/BumperStatus.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/TrackerConstraints.h>
#include <mrs_msgs/ControlError.h>
#include <mrs_msgs/Float64Srv.h>
#include <mrs_msgs/GetFloat64.h>

#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <mrs_lib/SafetyZone/SafetyZone.h>
#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Utils.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>

#include <mrs_uav_manager/Controller.h>
#include <mrs_uav_manager/Tracker.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <std_srvs/SetBool.h>

#include <pluginlib/class_loader.h>

#include <nodelet/loader.h>

#include <eigen3/Eigen/Eigen>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <mrs_msgs/ReferenceStamped.h>

#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/ReferenceStampedSrvRequest.h>
#include <mrs_msgs/ReferenceStampedSrvResponse.h>

#include <mrs_msgs/TransformReferenceSrv.h>
#include <mrs_msgs/TransformReferenceSrvRequest.h>
#include <mrs_msgs/TransformReferenceSrvResponse.h>

#include <mrs_msgs/TransformPoseSrv.h>
#include <mrs_msgs/TransformPoseSrvRequest.h>
#include <mrs_msgs/TransformPoseSrvResponse.h>

#include <mrs_msgs/TransformVector3Srv.h>
#include <mrs_msgs/TransformVector3SrvRequest.h>
#include <mrs_msgs/TransformVector3SrvResponse.h>

#include <mrs_msgs/Float64StampedSrv.h>
#include <mrs_msgs/Float64StampedSrvRequest.h>
#include <mrs_msgs/Float64StampedSrvResponse.h>

#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/Vec4Request.h>
#include <mrs_msgs/Vec4Response.h>

#include <mrs_msgs/Vec1.h>
#include <mrs_msgs/Vec1Request.h>
#include <mrs_msgs/Vec1Response.h>

#include <mrs_msgs/TrackerTrajectory.h>

//}

/* defines //{ */

#define STRING_EQUAL 0
#define TAU 2 * M_PI
#define PWM_MIDDLE 1500.0
#define PWM_MIN 1000.0
#define PWM_MAX 2000.0
#define PWM_RANGE PWM_MAX - PWM_MIN
#define REF_X 0
#define REF_Y 1
#define REF_Z 2
#define REF_YAW 3
#define ELAND_STR "eland"
#define ESCALATING_FAILSAFE_STR "escalating_failsafe"
#define FAILSAFE_STR "failsafe"

//}

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

const char* state_names[2] = {

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
  ControllerParams(std::string address, std::string name_space, double eland_threshold, double failsafe_threshold, double odometry_innovation_threshold);

public:
  double      failsafe_threshold;
  double      eland_threshold;
  double      odometry_innovation_threshold;
  std::string address;
  std::string name_space;
};

ControllerParams::ControllerParams(std::string address, std::string name_space, double eland_threshold, double failsafe_threshold,
                                   double odometry_innovation_threshold) {

  this->eland_threshold               = eland_threshold;
  this->odometry_innovation_threshold = odometry_innovation_threshold;
  this->failsafe_threshold            = failsafe_threshold;
  this->address                       = address;
  this->name_space                    = name_space;
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
  // node handle stuff
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  std::string     _uav_name_;

  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_manager::Tracker>> tracker_loader_;  // pluginlib loader of dynamically loaded trackers
  std::vector<std::string>                                          _tracker_names_;  // list of tracker names
  std::map<std::string, TrackerParams>                              trackers_;        // map between tracker names and tracker param
  std::vector<boost::shared_ptr<mrs_uav_manager::Tracker>>          tracker_list_;    // list of trackers, routines are callable from this
  std::mutex                                                        mutex_tracker_list_;

  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_manager::Controller>> controller_loader_;  // pluginlib loader of dynamically loaded controllers
  std::vector<std::string>                                             _controller_names_;  // list of controller names
  std::map<std::string, ControllerParams>                              controllers_;        // map between controller names and controller params
  std::vector<boost::shared_ptr<mrs_uav_manager::Controller>>          controller_list_;    // list of controllers, routines are callable from this
  std::mutex                                                           mutex_controller_list_;

  std::tuple<bool, std::string> switchController(const std::string controller_name);
  std::tuple<bool, std::string> switchTracker(const std::string tracker_name);

  // transfomer
  std::shared_ptr<mrs_lib::Transformer> transformer_;

  // defines the type of state input: odometry or uav_state mesasge types
  int _state_input_;

  // names of important trackers
  std::string _null_tracker_name_;     // null tracker is active when UAV is not in the air
  std::string _ehover_tracker_name_;   // ehover tracker is used for emergency hovering
  std::string _landoff_tracker_name_;  // landoff is used for landing and takeoff

  // names of important controllers
  std::string _failsafe_controller_name_;  // controller used for feed-forward failsafe
  std::string _eland_controller_name_;     // controller used for emergancy landing

  // joystick control
  bool        _joystick_enabled_ = false;
  int         _joystick_mode_;
  std::string _joystick_tracker_name_;
  std::string _joystick_controller_name_;
  std::string _joystick_fallback_tracker_name_;
  std::string _joystick_fallback_controller_name_;

  // should disarm after emergancy landing?
  bool _eland_disarm_enabled_ = false;

  // uav_state (odometry) subscriber gets the state estimate for control
  ros::Subscriber    subscriber_odometry_;
  ros::Subscriber    subscriber_uav_state_;
  mrs_msgs::UavState uav_state_;
  bool               got_uav_state_ = false;
  ros::Time          uav_state_last_time_;          // when was the last time we got the state estimate?
  double             _uav_state_max_missing_time_;  // how long should we tolerate missing state estimate?
  double             uav_roll_;
  double             uav_pitch_;
  double             uav_yaw_;
  std::mutex         mutex_uav_state_;

  // pixhawk odom is used to initialize the failsafe routine
  ros::Subscriber    subscriber_pixhawk_odometry_;
  nav_msgs::Odometry pixhawk_odometry_;
  bool               got_pixhawk_odometry_ = false;
  std::mutex         mutex_pixhawk_odometry_;

  // subscriber for mavros' GPS data
  ros::Subscriber subscriber_mavros_gps_;

  // max height is a dynamically set safety area height
  ros::Subscriber subscriber_max_height_;
  double          max_height_external_;
  double          max_height_safety_area_;
  double          _max_height_;
  bool            got_max_height_ = false;
  std::mutex      mutex_max_height_external_;
  std::mutex      mutex_min_height_;

  // odometry innovation is published by the odometry node
  // it is used to issue eland if the estimator's input is too wonky
  ros::Subscriber    subscriber_odometry_innovation_;
  nav_msgs::Odometry odometry_innovation_;
  std::mutex         mutex_odometry_innovation_;
  bool               got_odometry_innovation_ = false;

  // resolves simplified frame names
  std::string resolveFrameName(const std::string in);

  // check for invalid values in the result from trackers
  bool validatePositionCommand(const mrs_msgs::PositionCommand::ConstPtr position_command);
  bool validateAttitudeCommand(const mrs_msgs::AttitudeCommand::ConstPtr attitude_command);
  bool validateOdometry(const nav_msgs::OdometryConstPtr odometry);
  bool validateUavState(const mrs_msgs::UavStateConstPtr odometry);

  double RCChannelToRange(double rc_value, double range, double deadband);

  // contains handlers that are shared with trackers and controllers
  // safety area, tf transformer and bumper
  std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers_;

  // keeping track of currently active controllers and trackers
  int active_tracker_idx_    = 0;
  int active_controller_idx_ = 0;

  // indeces of some notable trackers
  int _ehover_tracker_idx_               = 0;
  int _landoff_tracker_idx_              = 0;
  int _joystick_tracker_idx_             = 0;
  int _joystick_controller_idx_          = 0;
  int _failsafe_controller_idx_          = 0;
  int _joystick_fallback_controller_idx_ = 0;
  int _joystick_fallback_tracker_idx_    = 0;
  int _null_tracker_idx_                 = 0;
  int _eland_controller_idx_             = 0;
  int _partial_landing_controller_idx_   = 0;

  // partial landing remembers what controller and tracker was used before
  int partial_landing_previous_tracker_idx_    = 0;
  int partial_landing_previous_controller_idx_ = 0;

  // motors on/off enables the control output from the control manager
  void switchMotors(bool in);
  bool motors_ = false;

  void setOdometryCallbacks(const bool input);

  // what thrust should be output when null tracker is active?
  double _min_thrust_null_tracker_ = 0.0;

  // rates of all the timers
  int _status_timer_rate_          = 0;
  int _safety_timer_rate_          = 0;
  int _elanding_timer_rate_        = 0;
  int _partial_landing_timer_rate_ = 0;
  int _failsafe_timer_rate_        = 0;
  int _bumper_timer_rate_          = 0;

  // publishers
  ros::Publisher publisher_control_output_;
  ros::Publisher publisher_position_cmd_;
  ros::Publisher publisher_attitude_cmd_;
  ros::Publisher publisher_thrust_force_;
  ros::Publisher publisher_cmd_odom_;
  ros::Publisher publisher_diagnostics_;
  ros::Publisher publisher_motors_;
  ros::Publisher publisher_tilt_error_;
  ros::Publisher publisher_mass_estimate_;
  ros::Publisher publisher_control_error_;
  ros::Publisher publisher_safety_area_markers_;
  ros::Publisher publisher_safety_area_coordinates_markers_;
  ros::Publisher publisher_disturbances_markers_;
  ros::Publisher publisher_bumper_status_;
  ros::Publisher publisher_mpc_trajectory_;
  ros::Publisher publisher_current_constraints_;

  // service servers
  ros::ServiceServer service_server_switch_tracker_;
  ros::ServiceServer service_server_switch_controller_;
  ros::ServiceServer service_server_reset_tracker_;
  ros::ServiceServer service_server_hover_;
  ros::ServiceServer service_server_ehover_;
  ros::ServiceServer service_server_failsafe_;
  ros::ServiceServer service_server_failsafe_escalating_;
  ros::ServiceServer service_server_motors_;
  ros::ServiceServer service_server_arm_;
  ros::ServiceServer service_server_enable_callbacks_;
  ros::ServiceServer service_server_set_constraints_;
  ros::ServiceServer service_server_use_joystick_;
  ros::ServiceServer service_server_emergency_reference_;
  ros::ServiceServer service_server_pirouette_;
  ros::ServiceServer service_server_eland_;
  ros::ServiceServer service_server_partial_landing_;

  // human callbable services for references
  ros::ServiceServer service_server_goto_;
  ros::ServiceServer service_server_goto_fcu_;
  ros::ServiceServer service_server_goto_relative_;
  ros::ServiceServer service_server_goto_altitude_;
  ros::ServiceServer service_server_set_yaw_;
  ros::ServiceServer service_server_set_yaw_relative_;

  // the main reference
  ros::ServiceServer service_server_reference_;
  ros::Subscriber    subscriber_reference_;

  // transform service servers
  ros::ServiceServer service_server_transform_reference_;
  ros::ServiceServer service_server_transform_pose_;
  ros::ServiceServer service_server_transform_vector3_;

  // bumper service servers
  ros::ServiceServer service_server_bumper_enabler_;
  ros::ServiceServer service_server_bumper_repulsion_enabler_;

  // service clients
  ros::ServiceClient service_client_arm_;
  ros::ServiceClient service_client_eland_;
  ros::ServiceClient service_client_land_;
  ros::ServiceClient service_client_shutdown_;
  ros::ServiceClient service_client_set_odometry_callbacks_;

  // min client
  ros::ServiceServer service_server_set_min_height_;
  ros::ServiceServer service_server_get_min_height_;

  // the last result of an active tracker
  mrs_msgs::PositionCommand::ConstPtr last_position_cmd_;
  std::mutex                          mutex_last_position_cmd_;

  // the last result of an active controller
  mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd_;
  std::mutex                          mutex_last_attitude_cmd_;

  // the time of last switching of a tracker or a controller
  ros::Time  controller_tracker_switch_time_;
  std::mutex mutex_controller_tracker_switch_time_;

  // mavros state tell us about the inner pixhwak state
  ros::Subscriber    subscriber_mavros_state_;
  mavros_msgs::State mavros_state_;
  std::mutex         mutex_mavros_state_;
  bool               got_mavros_state_ = false;
  bool               offboard_mode_    = false;
  bool               armed_            = false;

  // listening to the RC channels as told by pixhawk
  ros::Subscriber      subscriber_rc_;
  mavros_msgs::RCIn    rc_channels_;
  std::mutex           mutex_rc_channels_;
  bool                 got_rc_channels_ = false;
  std::list<ros::Time> rc_channel_switch_time_;
  std::mutex           mutex_rc_channel_switch_time_;

  // the RC channel mapping of the main 4 control signals
  double _rc_channel_pitch_, _rc_channel_roll_, _rc_channel_yaw_, _rc_channel_thrust_;

  // this is called to update the trackers and to receive position control command from the active one
  void updateTrackers(void);

  // this is called to update the controllers and to receive attitude control command from the active one
  void updateControllers(mrs_msgs::UavState uav_state_for_control);

  // this publishes the control commands
  void publish(void);

  // parameters of the motor model, magnitude of gravity
  mrs_uav_manager::MotorParams _motor_params_;
  double                       _g_;

  // thrust mass estimation during eland
  double    thrust_mass_estimate_;
  bool      thrust_under_threshold_ = false;
  ros::Time thrust_mass_estimate_first_time_;

  // failsafe when tilt error is too large
  bool   _tilt_error_failsafe_enabled_ = false;
  double _tilt_error_threshold_;

  // elanding when tilt error is too large
  double _tilt_limit_eland_;  // tilt error for triggering eland

  // disarming when tilt error is too large
  double _tilt_limit_disarm_;  // tilt error for triggering disarm

  // elanding when yaw error is too large
  bool   _yaw_error_eland_enabled_ = false;
  double _yaw_error_eland_threshold_;

  // keeping track of control errors
  double     tilt_error_;
  double     yaw_error_;
  double     position_error_x_, position_error_y_, position_error_z_;
  double     velocity_error_x_, velocity_error_y_, velocity_error_z_;
  std::mutex mutex_attitude_error_;
  std::mutex mutex_control_error_;

  // control error for triggering failsafe, eland, etc.
  // this filled with the current controllers failsafe threshold
  double _failsafe_threshold_;             // control error for triggering failsafe
  double _eland_threshold_;                // control error for triggering eland
  double _odometry_innovation_threshold_;  // innovation size for triggering eland

  // safety area
  std::unique_ptr<mrs_lib::SafetyZone> safety_zone_;
  bool                                 _use_safety_area_ = false;
  std::string                          _safety_area_frame_;
  double                               min_height_;
  bool                                 _obstacle_points_enabled_   = false;
  bool                                 _obstacle_polygons_enabled_ = false;

  // safety area routines
  // those and passed to trackers using the common_handlers object
  bool   isPointInSafetyArea2d(const mrs_msgs::ReferenceStamped point);
  bool   isPointInSafetyArea3d(const mrs_msgs::ReferenceStamped point);
  bool   isPathToPointInSafetyArea2d(const mrs_msgs::ReferenceStamped from, const mrs_msgs::ReferenceStamped to);
  double getMinHeight(void);
  double getMaxHeight(void);

  // getting info callbacks
  void callbackOdometry(const nav_msgs::OdometryConstPtr& msg);
  void callbackUavState(const mrs_msgs::UavStateConstPtr& msg);
  void callbackOdometryInnovation(const nav_msgs::OdometryConstPtr& msg);
  void callbackPixhawkOdometry(const nav_msgs::OdometryConstPtr& msg);
  void callbackMavrosGps(const sensor_msgs::NavSatFixConstPtr& msg);
  void callbackMaxHeight(const mrs_msgs::Float64StampedConstPtr& msg);
  void callbackMavrosState(const mavros_msgs::StateConstPtr& msg);
  void callbackRC(const mavros_msgs::RCInConstPtr& msg);

  // switching controller and tracker services
  bool callbackSwitchTracker(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);
  bool callbackSwitchController(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);
  bool callbackTrackerResetStatic(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // reference callbacks
  void callbackReferenceTopic(const mrs_msgs::ReferenceStampedConstPtr& msg);
  bool callbackGoToService(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res);
  bool callbackGoToFcuService(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res);
  bool callbackGoToRelativeService(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res);
  bool callbackGoToAltitudeService(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res);
  bool callbackSetYawService(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res);
  bool callbackSetYawRelativeService(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res);
  bool callbackReferenceService(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);
  bool callbackEmergencyReferenceService(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);

  // safety callbacks
  bool callbackHoverService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackEHoverService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackFailsafe(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackFailsafeEscalating(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackEland(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackMotors(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool callbackArm(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool callbackEnableCallbacks(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool callbackBumperEnableService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool callbackBumperEnableRepulsionService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  bool callbackSetMinHeight(mrs_msgs::Float64Srv::Request& req, mrs_msgs::Float64Srv::Response& res);
  bool callbackGetMinHeight(mrs_msgs::GetFloat64::Request& req, mrs_msgs::GetFloat64::Response& res);

  // transformation callbacks
  bool callbackTransformReference(mrs_msgs::TransformReferenceSrv::Request& req, mrs_msgs::TransformReferenceSrv::Response& res);
  bool callbackTransformPose(mrs_msgs::TransformPoseSrv::Request& req, mrs_msgs::TransformPoseSrv::Response& res);
  bool callbackTransformVector3(mrs_msgs::TransformVector3Srv::Request& req, mrs_msgs::TransformVector3Srv::Response& res);

  // sets constraints to all trackers
  bool callbackSetConstraints(mrs_msgs::TrackerConstraintsSrv::Request& req, mrs_msgs::TrackerConstraintsSrv::Response& res);

  // land and sit while applying thrust
  bool callbackPartialLanding(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // constraints management
  bool       got_constraints_ = false;
  std::mutex mutex_constraints_;
  void       setConstraints(mrs_msgs::TrackerConstraintsSrvRequest constraints);
  bool       enforceControllersConstraints(mrs_msgs::TrackerConstraintsSrvRequest& constraints);

  mrs_msgs::TrackerConstraintsSrvRequest current_constraints_;
  mrs_msgs::TrackerConstraintsSrvRequest sanitized_constraints_;

  // yaw manipulation
  double sanitizeYaw(const double yaw_in);
  double angleDist(const double in1, const double in2);

  // are callbacks enabled to trackers?
  bool callbacks_enabled_ = true;

  // timer for regular status publishing
  ros::Timer status_timer_;
  void       statusTimer(const ros::TimerEvent& event);

  // timer for issuing the failsafe landing
  ros::Timer failsafe_timer_;
  void       failsafeTimer(const ros::TimerEvent& event);
  bool       failsafe_triggered_ = false;

  // oneshot timer for running controllers and trackers
  ros::Timer control_timer_;
  void       controlTimerOneshot(const ros::TimerEvent& event);
  bool       running_control_timer_ = false;

  // timer for issuing emergancy landing
  ros::Timer elanding_timer_;
  void       elandingTimer(const ros::TimerEvent& event);
  bool       eland_triggered_ = false;

  // timer for issuing partial landing
  ros::Timer partial_landing_timer_;
  void       partialLandingTimer(const ros::TimerEvent& event);
  bool       partial_landing_triggered_ = false;

  // timer for regular checking of controller errors
  ros::Timer safety_timer_;
  void       safetyTimer(const ros::TimerEvent& event);
  bool       running_safety_timer_        = false;
  double     odometry_switch_in_progress_ = false;

  // timer for issuing the pirouette
  ros::Timer pirouette_timer_;
  void       pirouetteTimer(const ros::TimerEvent& event);

  // timer for checking the bumper collisions
  ros::Timer bumper_timer_;
  void       bumperTimer(const ros::TimerEvent& event);

  // obstacle bumper
  ros::Subscriber subscriber_bumper_;
  void            callbackBumper(const mrs_msgs::ObstacleSectorsConstPtr& msg);
  bool            got_bumper_ = false;

  mrs_msgs::ObstacleSectors bumper_data_;
  std::mutex                mutex_bumper_data_;

  bool bumper_enabled_           = false;
  bool _bumper_hugging_enabled_  = false;
  bool bumper_repulsion_enabled_ = false;
  bool repulsing_                = false;
  uint repulsing_from_;

  double _bumper_horizontal_distance_;
  double _bumper_vertical_distance_;

  double _bumper_repulsion_horizontal_distance_;
  double _bumper_repulsion_horizontal_offset_;
  double _bumper_repulsion_vertical_distance_;
  double _bumper_repulsion_vertical_offset_;

  bool bumperValidatePoint(mrs_msgs::ReferenceStamped& point);
  int  bumperGetSectorId(const double x, const double y, const double z);
  bool bumperPushFromObstacle(void);

  // escalating failsafe (eland -> failsafe -> disarm)
  double    _escalating_failsafe_timeout_;
  ros::Time escalating_failsafe_time_;

  // rc control
  bool        _rc_eland_enabled_ = false;
  int         _rc_eland_channel_;
  int         _rc_eland_threshold_;
  std::string _rc_eland_action_;
  bool        rc_eland_triggered_ = false;

  // joystick control
  std::mutex mutex_joystick_;

  ros::Subscriber  subscriber_joystick_;
  void             callbackJoystick(const sensor_msgs::JoyConstPtr& msg);
  sensor_msgs::Joy joystick_data_;
  bool             callbackUseJoystick([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // joystick buttons mappings
  int _channel_A_, _channel_B_, _channel_X_, _channel_Y_, _channel_start_, _channel_back_, _channel_LT_, _channel_RT_, _channel_L_joy_, _channel_R_joy_;

  // channel numbers and channel multipliers
  int _channel_pitch_, _channel_roll_, _channel_yaw_, _channel_thrust_;
  int _channel_mult_pitch_, _channel_mult_roll_, _channel_mult_yaw_, _channel_mult_thrust_;

  ros::Timer joystick_timer_;
  void       joystickTimer(const ros::TimerEvent& event);
  double     _joystick_timer_rate_;

  double _joystick_carrot_distance_;

  ros::Time joystick_start_press_time_;
  bool      joystick_start_pressed_ = false;

  ros::Time joystick_back_press_time_;
  bool      joystick_back_pressed_ = false;
  bool      joystick_goto_enabled_ = false;

  bool      joystick_failsafe_pressed_ = false;
  ros::Time joystick_failsafe_press_time_;

  bool      joystick_eland_pressed_ = false;
  ros::Time joystick_eland_press_time_;

  bool   _rc_goto_enabled_               = false;
  bool   rc_goto_active_                 = false;
  int    rc_joystick_channel_last_value_ = 0;
  int    _rc_joystick_channel_;
  int    _rc_joystick_n_switches_;
  double _rc_joystick_carrot_distance_;
  int    _rc_joystick_timeout_;

  // emergancy landing state machine
  LandingStates_t current_state_landing_  = IDLE_STATE;
  LandingStates_t previous_state_landing_ = IDLE_STATE;
  std::mutex      mutex_landing_state_machine_;
  void            changeLandingState(LandingStates_t new_state);
  double          _uav_mass_;
  double          _elanding_cutoff_mass_factor_;
  double          _elanding_cutoff_timeout_;
  double          landing_uav_mass_ = 0;

  // partial landing state machine
  LandingStates_t current_state_partial_landing_  = IDLE_STATE;
  LandingStates_t previous_state_partial_landing_ = IDLE_STATE;
  void            changePartialLandingState(LandingStates_t new_state);
  bool            _partial_landing_enabled_ = false;
  double          _partial_landing_cutoff_timeout_;
  double          _partial_landing_mass_factor_;
  std::string     _partial_landing_controller_name_;
  std::mutex      mutex_partial_landing_state_machine_;

  // initial body disturbance loaded from params
  double _initial_body_disturbance_x_;
  double _initial_body_disturbance_y_;

  // profiling
  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // automatic pc shutdown (DARPA specific)
  bool   _automatic_pc_shutdown_enabled_ = false;
  double _automatic_pc_shutdown_threshold_;

  // pirouette
  bool       _pirouette_enabled_ = false;
  double     _pirouette_speed_;
  double     _pirouette_timer_rate_;
  std::mutex mutex_pirouette_;
  double     pirouette_inital_yaw_;
  double     pirouette_iterator_;
  bool       callbackPirouette(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // diagnostics publishing
  void       publishDiagnostics(void);
  std::mutex mutex_diagnostics_;

  // standalone function handlers
  void shutdown();
  void setCallbacks(bool in);
  bool ehover(std::string& message_out);
  bool hover(std::string& message_out);
  bool eland(std::string& message_out);
  bool partialLanding(std::string& message_out);
  bool failsafe();
  bool escalatingFailsafe(std::string& message_out);
  bool arming(bool input);
  bool isOffboard(void);
};

//}

/* //{ onInit() */

void ControlManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  joystick_start_press_time_      = ros::Time(0);
  joystick_failsafe_press_time_   = ros::Time(0);
  joystick_eland_press_time_      = ros::Time(0);
  escalating_failsafe_time_       = ros::Time(0);
  controller_tracker_switch_time_ = ros::Time(0);

  ROS_INFO("[ControlManager]: initializing");

  last_attitude_cmd_ = mrs_msgs::AttitudeCommand::Ptr();
  last_position_cmd_ = mrs_msgs::PositionCommand::Ptr();

  // --------------------------------------------------------------
  // |                           params                           |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "ControlManager");

  param_loader.load_param("uav_name", _uav_name_);

  param_loader.load_param("enable_profiler", _profiler_enabled_);

  param_loader.load_param("state_input", _state_input_);

  if (!(_state_input_ == 0 || _state_input_ == 1)) {
    ROS_ERROR("[ControlManager]: the state_input parameter has to be in {0, 1}");
    ros::shutdown();
  }

  param_loader.load_param("safety/min_thrust_null_tracker", _min_thrust_null_tracker_);
  param_loader.load_param("safety/ehover_tracker", _ehover_tracker_name_);
  param_loader.load_param("safety/failsafe_controller", _failsafe_controller_name_);

  param_loader.load_param("safety/eland/controller", _eland_controller_name_);
  param_loader.load_param("safety/eland/cutoff_mass_factor", _elanding_cutoff_mass_factor_);
  param_loader.load_param("safety/eland/cutoff_timeout", _elanding_cutoff_timeout_);
  param_loader.load_param("safety/eland/timer_rate", _elanding_timer_rate_);
  param_loader.load_param("safety/eland/disarm", _eland_disarm_enabled_);

  param_loader.load_param("safety/escalating_failsafe/timeout", _escalating_failsafe_timeout_);

  param_loader.load_param("partial_land/enabled", _partial_landing_enabled_);
  param_loader.load_param("partial_land/mass_factor_trigger", _partial_landing_mass_factor_);
  param_loader.load_param("partial_land/cutoff_timeout", _partial_landing_cutoff_timeout_);
  param_loader.load_param("partial_land/controller", _partial_landing_controller_name_);
  param_loader.load_param("partial_land/timer_rate", _partial_landing_timer_rate_);

  param_loader.load_param("safety/tilt_limit_eland", _tilt_limit_eland_);
  _tilt_limit_eland_ = (_tilt_limit_eland_ / 180.0) * M_PI;
  param_loader.load_param("safety/tilt_limit_disarm", _tilt_limit_disarm_);
  _tilt_limit_disarm_ = (_tilt_limit_disarm_ / 180.0) * M_PI;
  param_loader.load_param("safety/yaw_limit_eland", _yaw_error_eland_threshold_);
  _yaw_error_eland_threshold_ = (_yaw_error_eland_threshold_ / 180.0) * M_PI;

  param_loader.load_param("status_timer_rate", _status_timer_rate_);
  param_loader.load_param("safety/safety_timer_rate", _safety_timer_rate_);
  param_loader.load_param("safety/failsafe_timer_rate", _failsafe_timer_rate_);

  param_loader.load_param("uav_mass", _uav_mass_);
  param_loader.load_param("hover_thrust/a", _motor_params_.hover_thrust_a);
  param_loader.load_param("hover_thrust/b", _motor_params_.hover_thrust_b);
  param_loader.load_param("g", _g_);

  param_loader.load_param("safety/odometry_max_missing_time", _uav_state_max_missing_time_);

  param_loader.load_param("safety/tilt_error_failsafe/enabled", _tilt_error_failsafe_enabled_);
  param_loader.load_param("safety/tilt_error_failsafe/tilt_error_threshold", _tilt_error_threshold_);
  _tilt_error_threshold_ = (_tilt_error_threshold_ / 180.0) * M_PI;

  param_loader.load_param("joystick/enabled", _joystick_enabled_);
  param_loader.load_param("joystick/mode", _joystick_mode_);
  param_loader.load_param("joystick/carrot_distance", _joystick_carrot_distance_);
  param_loader.load_param("joystick/joystick_timer_rate", _joystick_timer_rate_);
  param_loader.load_param("joystick/attitude_control/tracker", _joystick_tracker_name_);
  param_loader.load_param("joystick/attitude_control/controller", _joystick_controller_name_);
  param_loader.load_param("joystick/attitude_control/fallback/tracker", _joystick_fallback_tracker_name_);
  param_loader.load_param("joystick/attitude_control/fallback/controller", _joystick_fallback_controller_name_);

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
  param_loader.load_param("obstacle_bumper/timer_rate", _bumper_timer_rate_);
  param_loader.load_param("obstacle_bumper/horizontal_distance", _bumper_horizontal_distance_);
  param_loader.load_param("obstacle_bumper/vertical_distance", _bumper_vertical_distance_);

  param_loader.load_param("obstacle_bumper/obstacle_hugging/enabled", _bumper_hugging_enabled_);

  param_loader.load_param("obstacle_bumper/repulsion/enabled", bumper_repulsion_enabled_);

  param_loader.load_param("obstacle_bumper/repulsion/horizontal_distance", _bumper_repulsion_horizontal_distance_);
  param_loader.load_param("obstacle_bumper/repulsion/horizontal_offset", _bumper_repulsion_horizontal_offset_);
  param_loader.load_param("obstacle_bumper/repulsion/vertical_distance", _bumper_repulsion_vertical_distance_);
  param_loader.load_param("obstacle_bumper/repulsion/vertical_offset", _bumper_repulsion_vertical_offset_);

  param_loader.load_param("safety/rc_eland/enabled", _rc_eland_enabled_);
  param_loader.load_param("safety/rc_eland/channel_number", _rc_eland_channel_);
  param_loader.load_param("safety/rc_eland/threshold", _rc_eland_threshold_);
  param_loader.load_param("safety/rc_eland/action", _rc_eland_action_);

  // check the values of RC eland action
  if (_rc_eland_action_.compare(ELAND_STR) != STRING_EQUAL && _rc_eland_action_.compare(ESCALATING_FAILSAFE_STR) != STRING_EQUAL &&
      _rc_eland_action_.compare(FAILSAFE_STR) != STRING_EQUAL) {
    ROS_ERROR("[ControlManager]: the rc_eland/action parameter (%s) is not correct, requires {%s, %s, %s}", _rc_eland_action_.c_str(), ELAND_STR,
              ESCALATING_FAILSAFE_STR, FAILSAFE_STR);
    ros::shutdown();
  }

  param_loader.load_param("rc_joystick/enabled", _rc_goto_enabled_);
  param_loader.load_param("rc_joystick/channel_number", _rc_joystick_channel_);
  param_loader.load_param("rc_joystick/timeout", _rc_joystick_timeout_);
  param_loader.load_param("rc_joystick/n_switches", _rc_joystick_n_switches_);
  param_loader.load_param("rc_joystick/carrot_distance", _rc_joystick_carrot_distance_);

  param_loader.load_param("rc_joystick/channels/pitch", _rc_channel_pitch_);
  param_loader.load_param("rc_joystick/channels/roll", _rc_channel_roll_);
  param_loader.load_param("rc_joystick/channels/yaw", _rc_channel_yaw_);
  param_loader.load_param("rc_joystick/channels/thrust", _rc_channel_thrust_);

  param_loader.load_param("automatic_pc_shutdown/enabled", _automatic_pc_shutdown_enabled_);
  param_loader.load_param("automatic_pc_shutdown/distance_threshold", _automatic_pc_shutdown_threshold_);

  param_loader.load_param("pirouette/speed", _pirouette_speed_);
  param_loader.load_param("pirouette/timer_rate", _pirouette_timer_rate_);

  // | ------------- load the body integrator values ------------ |

  param_loader.load_param("body_disturbance_x", _initial_body_disturbance_x_);
  param_loader.load_param("body_disturbance_y", _initial_body_disturbance_y_);

  mrs_msgs::AttitudeCommand::Ptr output_command(std::make_unique<mrs_msgs::AttitudeCommand>());
  last_attitude_cmd_ = output_command;

  output_command->total_mass      = _uav_mass_;
  output_command->mass_difference = 0.0;

  output_command->disturbance_bx_b = _initial_body_disturbance_x_;
  output_command->disturbance_by_b = _initial_body_disturbance_y_;
  output_command->disturbance_wx_w = 0.0;
  output_command->disturbance_wy_w = 0.0;
  output_command->disturbance_bx_w = 0.0;
  output_command->disturbance_by_w = 0.0;

  output_command->thrust = _min_thrust_null_tracker_;

  output_command->controller = "none";

  // --------------------------------------------------------------
  // |         common handler for trackers and controllers        |
  // --------------------------------------------------------------

  common_handlers_ = std::make_shared<mrs_uav_manager::CommonHandlers_t>();

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>("ControlManager", _uav_name_);

  // bind transformer so trackers and controllers can use
  common_handlers_->transformer = transformer_;

  // | ----------------------- safety area ---------------------- |

  param_loader.load_param("safety_area/use_safety_area", _use_safety_area_);
  param_loader.load_param("safety_area/frame_name", _safety_area_frame_);
  param_loader.load_param("safety_area/min_height", min_height_);
  param_loader.load_param("safety_area/max_height", _max_height_);

  if (_use_safety_area_) {
    Eigen::MatrixXd border_points = param_loader.load_matrix_dynamic2("safety_area/safety_area", -1, 2);

    param_loader.load_param("safety_area/polygon_obstacles/enabled", _obstacle_polygons_enabled_);
    std::vector<Eigen::MatrixXd> polygon_obstacle_points;
    if (_obstacle_polygons_enabled_) {
      polygon_obstacle_points = param_loader.load_matrix_array2("safety_area/polygon_obstacles", std::vector<Eigen::MatrixXd>{});
    } else {
      polygon_obstacle_points = std::vector<Eigen::MatrixXd>();
    }

    param_loader.load_param("safety_area/point_obstacles/enabled", _obstacle_points_enabled_);
    std::vector<Eigen::MatrixXd> point_obstacle_points;

    if (_obstacle_points_enabled_) {

      point_obstacle_points = param_loader.load_matrix_array2("safety_area/point_obstacles", std::vector<Eigen::MatrixXd>{});

      if (_safety_area_frame_ == "latlon_origin") {
        for (unsigned int i = 0; i < point_obstacle_points.size(); i++) {

          Eigen::MatrixXd temp = point_obstacle_points[i];
          temp(0, 2) *= 8.9832e-06;
          point_obstacle_points[i] = temp;
        }
      }

    } else {
      point_obstacle_points = std::vector<Eigen::MatrixXd>();
    }

    // TODO: remove this when param loader supports proper loading
    for (auto& matrix : polygon_obstacle_points) {
      matrix.transposeInPlace();
    }

    try {
      safety_zone_ = std::make_unique<mrs_lib::SafetyZone>(border_points, polygon_obstacle_points, point_obstacle_points);
    }
    catch (mrs_lib::SafetyZone::BorderError) {
      ROS_ERROR("[ControlManager]: Exception caught. Wrong configruation for the safety zone border polygon.");
      ros::shutdown();
    }
    catch (mrs_lib::SafetyZone::PolygonObstacleError) {
      ROS_ERROR("[ControlManager]: Exception caught. Wrong configuration for one of the safety zone polygon obstacles.");
      ros::shutdown();
    }
    catch (mrs_lib::SafetyZone::PointObstacleError) {
      ROS_ERROR("[ControlManager]: Exception caught. Wrong configuration for one of the safety zone point obstacles.");
      ros::shutdown();
    }
  }

  common_handlers_->safety_area.use_safety_area       = _use_safety_area_;
  common_handlers_->safety_area.frame_id              = _safety_area_frame_;
  common_handlers_->safety_area.isPointInSafetyArea2d = boost::bind(&ControlManager::isPointInSafetyArea2d, this, _1);
  common_handlers_->safety_area.isPointInSafetyArea3d = boost::bind(&ControlManager::isPointInSafetyArea3d, this, _1);
  common_handlers_->safety_area.getMinHeight          = boost::bind(&ControlManager::getMinHeight, this);
  common_handlers_->safety_area.getMaxHeight          = boost::bind(&ControlManager::getMaxHeight, this);

  common_handlers_->bumper.bumperValidatePoint = boost::bind(&ControlManager::bumperValidatePoint, this, _1);
  common_handlers_->bumper.enabled             = bumper_enabled_;

  // --------------------------------------------------------------
  // |                        load trackers                       |
  // --------------------------------------------------------------

  param_loader.load_param("trackers", _tracker_names_);
  param_loader.load_param("null_tracker", _null_tracker_name_);
  param_loader.load_param("landing_takeoff_tracker", _landoff_tracker_name_);

  tracker_loader_ = std::make_unique<pluginlib::ClassLoader<mrs_uav_manager::Tracker>>("mrs_uav_manager", "mrs_uav_manager::Tracker");

  for (unsigned long i = 0; i < _tracker_names_.size(); i++) {

    std::string tracker_name = _tracker_names_[i];

    // load the controller parameters
    std::string address;
    param_loader.load_param(tracker_name + "/address", address);

    TrackerParams new_tracker(address);
    trackers_.insert(std::pair<std::string, TrackerParams>(tracker_name, new_tracker));

    try {
      ROS_INFO("[ControlManager]: Trying to load tracker %s", new_tracker.address.c_str());
      tracker_list_.push_back(tracker_loader_->createInstance(new_tracker.address.c_str()));
    }
    catch (pluginlib::CreateClassException& ex1) {
      ROS_ERROR("[ControlManager]: CreateClassException for tracker %s", new_tracker.address.c_str());
      ROS_ERROR("[ControlManager]: Error: %s", ex1.what());
      ros::shutdown();
    }
    catch (pluginlib::PluginlibException& ex) {
      ROS_ERROR("[ControlManager]: PluginlibException for tracker %s", new_tracker.address.c_str());
      ROS_ERROR("[ControlManager]: Error: %s", ex.what());
      ros::shutdown();
    }
  }

  ROS_INFO("[ControlManager]: trackers were loaded");

  for (unsigned long i = 0; i < tracker_list_.size(); i++) {

    std::map<std::string, TrackerParams>::iterator it;
    it = trackers_.find(_tracker_names_[i]);

    try {
      ROS_INFO("[ControlManager]: Initializing tracker %d: %s", (int)i, it->second.address.c_str());
      tracker_list_[i]->initialize(nh_, _uav_name_, common_handlers_);
    }
    catch (std::runtime_error& ex) {
      ROS_ERROR("[ControlManager]: Exception caught during tracker initialization: %s", ex.what());
    }
  }

  ROS_INFO("[ControlManager]: trackers were activated");

  // --------------------------------------------------------------
  // |                      load controllers                      |
  // --------------------------------------------------------------

  param_loader.load_param("controllers", _controller_names_);

  controller_loader_ = std::make_unique<pluginlib::ClassLoader<mrs_uav_manager::Controller>>("mrs_uav_manager", "mrs_uav_manager::Controller");

  // for each controller in the list
  for (unsigned long i = 0; i < _controller_names_.size(); i++) {

    std::string controller_name = _controller_names_[i];

    // load the controller parameters
    std::string address;
    std::string name_space;
    double      eland_threshold, failsafe_threshold, odometry_innovation_threshold;
    param_loader.load_param(controller_name + "/address", address);
    param_loader.load_param(controller_name + "/namespace", name_space);
    param_loader.load_param(controller_name + "/eland_threshold", eland_threshold);
    param_loader.load_param(controller_name + "/failsafe_threshold", failsafe_threshold);
    param_loader.load_param(controller_name + "/odometry_innovation_threshold", odometry_innovation_threshold);

    if (eland_threshold == 0) {
      eland_threshold = 1e6;
    }

    if (failsafe_threshold == 0) {
      failsafe_threshold = 1e6;
    }

    if (odometry_innovation_threshold == 0) {
      odometry_innovation_threshold = 1e6;
    }

    ControllerParams new_controller(address, name_space, eland_threshold, failsafe_threshold, odometry_innovation_threshold);
    controllers_.insert(std::pair<std::string, ControllerParams>(controller_name, new_controller));

    try {
      ROS_INFO("[ControlManager]: Loading controller %s", new_controller.address.c_str());
      controller_list_.push_back(controller_loader_->createInstance(new_controller.address.c_str()));
    }
    catch (pluginlib::CreateClassException& ex1) {
      ROS_ERROR("[ControlManager]: CreateClassException for controller %s", new_controller.address.c_str());
      ROS_ERROR("[ControlManager]: Error: %s", ex1.what());
      ros::shutdown();
    }
    catch (pluginlib::PluginlibException& ex) {
      ROS_ERROR("[ControlManager]: PluginlibException for controller %s", new_controller.address.c_str());
      ROS_ERROR("[ControlManager]: Error: %s", ex.what());
      ros::shutdown();
    }
  }

  ROS_INFO("[ControlManager]: controllers were loaded");

  for (unsigned long i = 0; i < controller_list_.size(); i++) {
    try {

      std::map<std::string, ControllerParams>::iterator it;
      it = controllers_.find(_controller_names_[i]);

      ROS_INFO("[ControlManager]: Initializing controller %d: %s", (int)i, it->second.address.c_str());
      controller_list_[i]->initialize(nh_, _controller_names_[i], it->second.name_space, _motor_params_, _uav_mass_, _g_);
    }
    catch (std::runtime_error& ex) {
      ROS_ERROR("[ControlManager]: Exception caught during controller initialization: %s", ex.what());
    }
  }

  ROS_INFO("[ControlManager]: controllers were initialized");

  // --------------------------------------------------------------
  // |     check the existance of safety trackers/controllers     |
  // --------------------------------------------------------------

  // check if the hover_tracker is within the loaded trackers
  bool hover_tracker_check = false;
  for (unsigned long i = 0; i < _tracker_names_.size(); i++) {

    std::string tracker_name = _tracker_names_[i];

    if (tracker_name.compare(_ehover_tracker_name_) == 0) {
      hover_tracker_check  = true;
      _ehover_tracker_idx_ = i;
      break;
    }
  }
  if (!hover_tracker_check) {
    ROS_ERROR("[ControlManager]: the safety/hover_tracker (%s) is not within the loaded trackers", _ehover_tracker_name_.c_str());
    ros::shutdown();
  }

  // check if the failsafe controller is within the loaded controllers
  bool failsafe_controller_check = false;
  for (unsigned long i = 0; i < _controller_names_.size(); i++) {

    std::string controller_name = _controller_names_[i];

    if (controller_name.compare(_failsafe_controller_name_) == 0) {
      failsafe_controller_check = true;
      _failsafe_controller_idx_ = i;
      break;
    }
  }
  if (!failsafe_controller_check) {
    ROS_ERROR("[ControlManager]: the failsafe controller (%s) is not within the loaded controllers", _failsafe_controller_name_.c_str());
    ros::shutdown();
  }

  // check if the eland controller is within the loaded controllers
  bool eland_controller_check = false;
  for (unsigned long i = 0; i < _controller_names_.size(); i++) {

    std::string controller_name = _controller_names_[i];

    if (controller_name.compare(_eland_controller_name_) == 0) {
      eland_controller_check = true;
      _eland_controller_idx_ = i;
      break;
    }
  }
  if (!eland_controller_check) {
    ROS_ERROR("[ControlManager]: the eland controller (%s) is not within the loaded controllers", _eland_controller_name_.c_str());
    ros::shutdown();
  }

  // check if the partial landing controller is within the loaded controllers
  if (_partial_landing_enabled_) {

    bool partial_landing_controller_check = false;
    for (unsigned long i = 0; i < _controller_names_.size(); i++) {

      std::string controller_name = _controller_names_[i];

      if (controller_name.compare(_partial_landing_controller_name_) == 0) {
        partial_landing_controller_check = true;
        _partial_landing_controller_idx_ = i;
        break;
      }
    }
    if (!partial_landing_controller_check) {
      ROS_ERROR("[ControlManager]: the partial landing controller (%s) is not within the loaded controllers", _partial_landing_controller_name_.c_str());
      ros::shutdown();
    }
  }

  // --------------------------------------------------------------
  // |           check the existance of landoff tracker           |
  // --------------------------------------------------------------

  // check if the landoff_tracker is within the loaded trackers
  bool landoff_tracker_check = false;
  for (unsigned long i = 0; i < _tracker_names_.size(); i++) {

    std::string tracker_name = _tracker_names_[i];

    if (tracker_name.compare(_landoff_tracker_name_) == 0) {
      landoff_tracker_check = true;
      _landoff_tracker_idx_ = i;
      break;
    }
  }
  if (!landoff_tracker_check) {
    ROS_ERROR("[ControlManager]: the landoff tracker (%s) is not within the loaded trackers", _landoff_tracker_name_.c_str());
    ros::shutdown();
  }

  // --------------------------------------------------------------
  // |         check for the existance of the NullTracker         |
  // --------------------------------------------------------------

  // check if the hover_tracker is within the loaded trackers
  bool null_tracker_check = false;
  for (unsigned long i = 0; i < _tracker_names_.size(); i++) {

    std::string tracker_name = _tracker_names_[i];

    if (tracker_name.compare(_null_tracker_name_) == 0) {
      null_tracker_check = true;
      _null_tracker_idx_ = i;
      break;
    }
  }
  if (!null_tracker_check) {
    ROS_ERROR("[ControlManager]: the null tracker (%s) is not within the loaded trackers", _null_tracker_name_.c_str());
    ros::shutdown();
  }

  // --------------------------------------------------------------
  // |  check existance of controllers and trackers for joystick  |
  // --------------------------------------------------------------

  if (_joystick_enabled_) {

    // check if the tracker for joystick control exists
    bool joystick_tracker_check = false;
    for (unsigned long i = 0; i < _tracker_names_.size(); i++) {

      std::string tracker_name = _tracker_names_[i];

      if (tracker_name.compare(_joystick_tracker_name_) == 0) {
        joystick_tracker_check = true;
        _joystick_tracker_idx_ = i;
        break;
      }
    }
    if (!joystick_tracker_check) {
      ROS_ERROR("[ControlManager]: the joystick tracker (%s) is not within the loaded trackers", _joystick_tracker_name_.c_str());
      ros::shutdown();
    }

    // check if the controller for joystick control exists
    bool joystick_controller_check = false;
    for (unsigned long i = 0; i < _controller_names_.size(); i++) {

      std::string controller_name = _controller_names_[i];

      if (controller_name.compare(_joystick_controller_name_) == 0) {
        joystick_controller_check = true;
        _joystick_controller_idx_ = i;
        break;
      }
    }
    if (!joystick_controller_check) {
      ROS_ERROR("[ControlManager]: the joystick controller (%s) is not within the loaded controllers", _joystick_controller_name_.c_str());
      ros::shutdown();
    }

    // check if the fallback tracker for joystick control exists
    bool joystick_fallback_tracker_check = false;
    for (unsigned long i = 0; i < _tracker_names_.size(); i++) {

      std::string tracker_name = _tracker_names_[i];

      if (tracker_name.compare(_joystick_fallback_tracker_name_) == 0) {
        joystick_fallback_tracker_check = true;
        _joystick_fallback_tracker_idx_ = i;
        break;
      }
    }
    if (!joystick_fallback_tracker_check) {
      ROS_ERROR("[ControlManager]: the joystick fallback tracker (%s) is not within the loaded trackers", _joystick_fallback_tracker_name_.c_str());
      ros::shutdown();
    }

    // check if the fallback controller for joystick control exists
    bool joystick_fallback_controller_check = false;
    for (unsigned long i = 0; i < _controller_names_.size(); i++) {

      std::string controller_name = _controller_names_[i];

      if (controller_name.compare(_joystick_fallback_controller_name_) == 0) {
        joystick_fallback_controller_check = true;
        _joystick_fallback_controller_idx_ = i;
        break;
      }
    }
    if (!joystick_fallback_controller_check) {
      ROS_ERROR("[ControlManager]: the joystick fallback controller (%s) is not within the loaded controllers", _joystick_fallback_controller_name_.c_str());
      ros::shutdown();
    }
  }

  // --------------------------------------------------------------
  // |                  activate the NullTracker                  |
  // --------------------------------------------------------------

  ROS_INFO("[ControlManager]: Activating the null tracker");

  tracker_list_[_null_tracker_idx_]->activate(last_position_cmd_);
  active_tracker_idx_ = _null_tracker_idx_;

  // --------------------------------------------------------------
  // |    activate the eland controller as the first controller   |
  // --------------------------------------------------------------

  ROS_INFO("[ControlManager]: Activating the the eland controller (%s) as the first controller", _controller_names_[_eland_controller_idx_].c_str());

  controller_list_[_eland_controller_idx_]->activate(last_attitude_cmd_);
  active_controller_idx_ = _eland_controller_idx_;

  // update the time
  {
    std::scoped_lock lock(mutex_controller_tracker_switch_time_);

    controller_tracker_switch_time_ = ros::Time::now();
  }

  motors_ = false;

  // --------------------------------------------------------------
  // |                          profiler_                          |
  // --------------------------------------------------------------

  profiler_ = mrs_lib::Profiler(nh_, "ControlManager", _profiler_enabled_);

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  publisher_control_output_                  = nh_.advertise<mavros_msgs::AttitudeTarget>("control_output_out", 1);
  publisher_position_cmd_                    = nh_.advertise<mrs_msgs::PositionCommand>("position_cmd_out", 1);
  publisher_attitude_cmd_                    = nh_.advertise<mrs_msgs::AttitudeCommand>("attitude_cmd_out", 1);
  publisher_thrust_force_                    = nh_.advertise<mrs_msgs::Float64Stamped>("thrust_force_out", 1);
  publisher_cmd_odom_                        = nh_.advertise<nav_msgs::Odometry>("cmd_odom_out", 1);
  publisher_diagnostics_                     = nh_.advertise<mrs_msgs::ControlManagerDiagnostics>("diagnostics_out", 1);
  publisher_motors_                          = nh_.advertise<mrs_msgs::BoolStamped>("motors_out", 1);
  publisher_tilt_error_                      = nh_.advertise<mrs_msgs::Float64>("tilt_error_out", 1);
  publisher_mass_estimate_                   = nh_.advertise<std_msgs::Float64>("mass_estimate_out", 1);
  publisher_control_error_                   = nh_.advertise<mrs_msgs::ControlError>("control_error_out", 1);
  publisher_safety_area_markers_             = nh_.advertise<visualization_msgs::MarkerArray>("safety_area_markers_out", 1);
  publisher_safety_area_coordinates_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("safety_area_coordinates_markers_out", 1);
  publisher_disturbances_markers_            = nh_.advertise<visualization_msgs::MarkerArray>("disturbances_markers_out", 1);
  publisher_bumper_status_                   = nh_.advertise<mrs_msgs::BumperStatus>("bumper_status_out", 1);
  publisher_mpc_trajectory_                  = nh_.advertise<mrs_msgs::TrackerTrajectory>("mpc_trajectory_out", 1);
  publisher_current_constraints_             = nh_.advertise<mrs_msgs::TrackerConstraints>("current_constraints_out", 1);

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  if (_state_input_ == 0) {
    subscriber_uav_state_ = nh_.subscribe("uav_state_in", 1, &ControlManager::callbackUavState, this, ros::TransportHints().tcpNoDelay());
  } else if (_state_input_ == 1) {
    subscriber_odometry_ = nh_.subscribe("odometry_in", 1, &ControlManager::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
  }

  subscriber_pixhawk_odometry_ = nh_.subscribe("mavros_odometry_in", 1, &ControlManager::callbackPixhawkOdometry, this, ros::TransportHints().tcpNoDelay());
  subscriber_mavros_gps_       = nh_.subscribe("mavros_gps_in", 1, &ControlManager::callbackMavrosGps, this, ros::TransportHints().tcpNoDelay());
  subscriber_max_height_       = nh_.subscribe("max_height_in", 1, &ControlManager::callbackMaxHeight, this, ros::TransportHints().tcpNoDelay());
  subscriber_joystick_         = nh_.subscribe("joystick_in", 1, &ControlManager::callbackJoystick, this, ros::TransportHints().tcpNoDelay());
  subscriber_bumper_           = nh_.subscribe("bumper_sectors_in", 1, &ControlManager::callbackBumper, this, ros::TransportHints().tcpNoDelay());
  subscriber_mavros_state_     = nh_.subscribe("mavros_state_in", 1, &ControlManager::callbackMavrosState, this, ros::TransportHints().tcpNoDelay());
  subscriber_rc_               = nh_.subscribe("rc_in", 1, &ControlManager::callbackRC, this, ros::TransportHints().tcpNoDelay());
  subscriber_odometry_innovation_ =
      nh_.subscribe("odometry_innovation_in", 1, &ControlManager::callbackOdometryInnovation, this, ros::TransportHints().tcpNoDelay());

  uav_state_last_time_ = ros::Time(0);

  // | -------------------- general services -------------------- |

  service_server_switch_tracker_           = nh_.advertiseService("switch_tracker_in", &ControlManager::callbackSwitchTracker, this);
  service_server_switch_controller_        = nh_.advertiseService("switch_controller_in", &ControlManager::callbackSwitchController, this);
  service_server_reset_tracker_            = nh_.advertiseService("tracker_reset_static_in", &ControlManager::callbackTrackerResetStatic, this);
  service_server_hover_                    = nh_.advertiseService("hover_in", &ControlManager::callbackHoverService, this);
  service_server_ehover_                   = nh_.advertiseService("ehover_in", &ControlManager::callbackEHoverService, this);
  service_server_failsafe_                 = nh_.advertiseService("failsafe_in", &ControlManager::callbackFailsafe, this);
  service_server_failsafe_escalating_      = nh_.advertiseService("failsafe_escalating_in", &ControlManager::callbackFailsafeEscalating, this);
  service_server_motors_                   = nh_.advertiseService("motors_in", &ControlManager::callbackMotors, this);
  service_server_arm_                      = nh_.advertiseService("arm_in", &ControlManager::callbackArm, this);
  service_server_enable_callbacks_         = nh_.advertiseService("enable_callbacks_in", &ControlManager::callbackEnableCallbacks, this);
  service_server_set_constraints_          = nh_.advertiseService("set_constraints_in", &ControlManager::callbackSetConstraints, this);
  service_server_use_joystick_             = nh_.advertiseService("use_joystick_in", &ControlManager::callbackUseJoystick, this);
  service_server_eland_                    = nh_.advertiseService("eland_in", &ControlManager::callbackEland, this);
  service_server_partial_landing_          = nh_.advertiseService("partial_land_in", &ControlManager::callbackPartialLanding, this);
  service_server_transform_reference_      = nh_.advertiseService("transform_reference_in", &ControlManager::callbackTransformReference, this);
  service_server_transform_pose_           = nh_.advertiseService("transform_pose_in", &ControlManager::callbackTransformPose, this);
  service_server_transform_vector3_        = nh_.advertiseService("transform_vector3_in", &ControlManager::callbackTransformVector3, this);
  service_server_bumper_enabler_           = nh_.advertiseService("bumper_in", &ControlManager::callbackBumperEnableService, this);
  service_server_bumper_repulsion_enabler_ = nh_.advertiseService("bumper_repulsion_in", &ControlManager::callbackBumperEnableRepulsionService, this);
  service_server_set_min_height_           = nh_.advertiseService("set_min_height_in", &ControlManager::callbackSetMinHeight, this);
  service_server_get_min_height_           = nh_.advertiseService("get_min_height_in", &ControlManager::callbackGetMinHeight, this);

  service_client_arm_                    = nh_.serviceClient<mavros_msgs::CommandBool>("arm_out");
  service_client_eland_                  = nh_.serviceClient<std_srvs::Trigger>("eland_out");
  service_client_land_                   = nh_.serviceClient<std_srvs::Trigger>("land_out");
  service_client_shutdown_               = nh_.serviceClient<std_srvs::Trigger>("shutdown_out");
  service_client_set_odometry_callbacks_ = nh_.serviceClient<std_srvs::SetBool>("set_odometry_callbacks_out");

  // | ---------------- setpoint command services --------------- |

  // human callable
  service_server_goto_             = nh_.advertiseService("goto_in", &ControlManager::callbackGoToService, this);
  service_server_goto_fcu_         = nh_.advertiseService("goto_fcu_in", &ControlManager::callbackGoToFcuService, this);
  service_server_goto_relative_    = nh_.advertiseService("goto_relative_in", &ControlManager::callbackGoToRelativeService, this);
  service_server_goto_altitude_    = nh_.advertiseService("goto_altitude_in", &ControlManager::callbackGoToAltitudeService, this);
  service_server_set_yaw_          = nh_.advertiseService("set_yaw_in", &ControlManager::callbackSetYawService, this);
  service_server_set_yaw_relative_ = nh_.advertiseService("set_yaw_relative_in", &ControlManager::callbackSetYawRelativeService, this);

  service_server_reference_ = nh_.advertiseService("reference_in", &ControlManager::callbackReferenceService, this);

  subscriber_reference_ = nh_.subscribe("reference_in", 1, &ControlManager::callbackReferenceTopic, this, ros::TransportHints().tcpNoDelay());

  // | --------------------- other services --------------------- |

  service_server_emergency_reference_ = nh_.advertiseService("emergency_reference_in", &ControlManager::callbackEmergencyReferenceService, this);
  service_server_pirouette_           = nh_.advertiseService("pirouette_in", &ControlManager::callbackPirouette, this);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  status_timer_          = nh_.createTimer(ros::Rate(_status_timer_rate_), &ControlManager::statusTimer, this);
  safety_timer_          = nh_.createTimer(ros::Rate(_safety_timer_rate_), &ControlManager::safetyTimer, this);
  bumper_timer_          = nh_.createTimer(ros::Rate(_bumper_timer_rate_), &ControlManager::bumperTimer, this);
  elanding_timer_        = nh_.createTimer(ros::Rate(_elanding_timer_rate_), &ControlManager::elandingTimer, this, false, false);
  partial_landing_timer_ = nh_.createTimer(ros::Rate(_partial_landing_timer_rate_), &ControlManager::partialLandingTimer, this, false, false);
  failsafe_timer_        = nh_.createTimer(ros::Rate(_failsafe_timer_rate_), &ControlManager::failsafeTimer, this, false, false);
  pirouette_timer_       = nh_.createTimer(ros::Rate(_pirouette_timer_rate_), &ControlManager::pirouetteTimer, this, false, false);
  joystick_timer_        = nh_.createTimer(ros::Rate(_joystick_timer_rate_), &ControlManager::joystickTimer, this);
  control_timer_         = nh_.createTimer(ros::Duration(0), &ControlManager::controlTimerOneshot, this, true, false);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[ControlManager]: Could not load all parameters!");
    ros::shutdown();
  }

  is_initialized_ = true;

  ROS_INFO("[ControlManager]: initialized");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* //{ statusTimer() */

void ControlManager::statusTimer(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  // copy member variables
  auto uav_state           = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_attitude_cmd   = mrs_lib::get_mutexed(mutex_last_attitude_cmd_, last_attitude_cmd_);
  auto max_height_external = mrs_lib::get_mutexed(mutex_max_height_external_, max_height_external_);
  auto yaw_error           = mrs_lib::get_mutexed(mutex_attitude_error_, yaw_error_);
  auto [position_error_x, position_error_y, position_error_z] =
      mrs_lib::get_mutexed(mutex_control_error_, position_error_x_, position_error_y_, position_error_z_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);
  auto min_height            = mrs_lib::get_mutexed(mutex_min_height_, min_height_);

  double max_height = _max_height_ > max_height_external ? max_height_external_ : _max_height_;

  double uav_x, uav_y, uav_z;
  uav_x = uav_state.pose.position.x;
  uav_y = uav_state.pose.position.y;
  uav_z = uav_state.pose.position.z;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("statusTimer", _status_timer_rate_, 0.01, event);

  // --------------------------------------------------------------
  // |                   publish the diagnostics                  |
  // --------------------------------------------------------------

  publishDiagnostics();

  // --------------------------------------------------------------
  // |                 publishing the motors_ state                |
  // --------------------------------------------------------------

  mrs_msgs::BoolStamped motors_out;
  motors_out.data  = motors_;
  motors_out.stamp = ros::Time::now();

  try {
    publisher_motors_.publish(motors_out);
  }
  catch (...) {
    ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_motors_.getTopic().c_str());
  }

  // --------------------------------------------------------------
  // |                   publish the tilt error                   |
  // --------------------------------------------------------------
  {
    std::scoped_lock lock(mutex_attitude_error_);

    mrs_msgs::Float64 tilt_error_out;
    tilt_error_out.value = (180.0 / M_PI) * tilt_error_;

    try {
      publisher_tilt_error_.publish(tilt_error_out);
    }
    catch (...) {
      ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_tilt_error_.getTopic().c_str());
    }
  }

  // --------------------------------------------------------------
  // |                  publish the control error                 |
  // --------------------------------------------------------------

  {
    mrs_msgs::ControlError msg_out;

    msg_out.header.stamp    = ros::Time::now();
    msg_out.header.frame_id = uav_state.header.frame_id;

    msg_out.position_errors.x    = position_error_x;
    msg_out.position_errors.y    = position_error_y;
    msg_out.position_errors.z    = position_error_z;
    msg_out.total_position_error = sqrt(pow(position_error_x, 2) + pow(position_error_y, 2) + pow(position_error_z, 2));
    msg_out.yaw_error            = yaw_error;

    std::map<std::string, ControllerParams>::iterator it;
    it                                  = controllers_.find(_controller_names_[active_controller_idx]);
    msg_out.position_eland_threshold    = it->second.eland_threshold;
    msg_out.position_failsafe_threshold = it->second.failsafe_threshold;

    try {
      publisher_control_error_.publish(msg_out);
    }
    catch (...) {
      ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_control_error_.getTopic().c_str());
    }
  }

  // --------------------------------------------------------------
  // |                  publish the mass estimate                 |
  // --------------------------------------------------------------

  if (last_attitude_cmd != mrs_msgs::AttitudeCommand::Ptr()) {

    std_msgs::Float64 mass_estimate_out;
    mass_estimate_out.data = _uav_mass_ + last_attitude_cmd->mass_difference;

    try {
      publisher_mass_estimate_.publish(mass_estimate_out);
    }
    catch (...) {
      ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_mass_estimate_.getTopic().c_str());
    }
  }

  // --------------------------------------------------------------
  // |               publish the safety area markers              |
  // --------------------------------------------------------------

  if (_use_safety_area_) {

    visualization_msgs::MarkerArray safety_area_marker_array;
    visualization_msgs::MarkerArray safety_area_coordinates_marker_array;

    mrs_lib::Polygon border = safety_zone_->getBorder();

    std::vector<geometry_msgs::Point> border_points_bot_original = border.getPointMessageVector(min_height);
    std::vector<geometry_msgs::Point> border_points_top_original = border.getPointMessageVector(max_height);

    std::vector<geometry_msgs::Point> border_points_bot_transformed = border_points_bot_original;
    std::vector<geometry_msgs::Point> border_points_top_transformed = border_points_bot_original;

    mrs_msgs::ReferenceStamped temp_ref;
    temp_ref.header.frame_id = _safety_area_frame_;

    mrs_lib::TransformStamped tf;

    if (auto ret = transformer_->getTransform(_safety_area_frame_, "local_origin", ros::Time(0))) {

      // if we fail in transforming the area at some point
      // do not publish it at all
      bool tf_success = true;

      mrs_lib::TransformStamped tf = ret.value();

      /* transform area points to local origin //{ */

      // transform border bottom points to local origin
      for (size_t i = 0; i < border_points_bot_original.size(); i++) {

        temp_ref.header.frame_id      = _safety_area_frame_;
        temp_ref.header.stamp         = ros::Time(0);
        temp_ref.reference.position.x = border_points_bot_original[i].x;
        temp_ref.reference.position.y = border_points_bot_original[i].y;
        temp_ref.reference.position.z = border_points_bot_original[i].z;

        if (auto ret = transformer_->transform(tf, temp_ref)) {

          temp_ref = ret.value();

          border_points_bot_transformed[i].x = temp_ref.reference.position.x;
          border_points_bot_transformed[i].y = temp_ref.reference.position.y;
          border_points_bot_transformed[i].z = temp_ref.reference.position.z;

        } else {
          tf_success = false;
        }
      }

      // transform border top points to local origin
      for (size_t i = 0; i < border_points_top_original.size(); i++) {

        temp_ref.header.frame_id      = _safety_area_frame_;
        temp_ref.header.stamp         = ros::Time(0);
        temp_ref.reference.position.x = border_points_top_original[i].x;
        temp_ref.reference.position.y = border_points_top_original[i].y;
        temp_ref.reference.position.z = border_points_top_original[i].z;

        if (auto ret = transformer_->transform(tf, temp_ref)) {

          temp_ref = ret.value();

          border_points_top_transformed[i].x = temp_ref.reference.position.x;
          border_points_top_transformed[i].y = temp_ref.reference.position.y;
          border_points_top_transformed[i].z = temp_ref.reference.position.z;

        } else {
          tf_success = false;
        }
      }

      //}

      visualization_msgs::Marker safety_area_marker;

      safety_area_marker.header.frame_id = _uav_name_ + "/local_origin";
      safety_area_marker.type            = visualization_msgs::Marker::LINE_LIST;
      safety_area_marker.color.a         = 1;
      safety_area_marker.scale.x         = 0.2;
      safety_area_marker.color.r         = 1;
      safety_area_marker.color.g         = 0;
      safety_area_marker.color.b         = 0;

      safety_area_marker.pose.orientation.x = 0;
      safety_area_marker.pose.orientation.y = 0;
      safety_area_marker.pose.orientation.z = 0;
      safety_area_marker.pose.orientation.w = 1;

      visualization_msgs::Marker safety_area_coordinates_marker;

      safety_area_coordinates_marker.header.frame_id = _uav_name_ + "/local_origin";
      safety_area_coordinates_marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
      safety_area_coordinates_marker.color.a         = 1;
      safety_area_coordinates_marker.scale.z         = 1.0;
      safety_area_coordinates_marker.color.r         = 0;
      safety_area_coordinates_marker.color.g         = 0;
      safety_area_coordinates_marker.color.b         = 0;

      safety_area_coordinates_marker.id = 0;

      safety_area_coordinates_marker.pose.orientation.x = 0;
      safety_area_coordinates_marker.pose.orientation.y = 0;
      safety_area_coordinates_marker.pose.orientation.z = 0;
      safety_area_coordinates_marker.pose.orientation.w = 1;

      /* adding safety area points //{ */

      // bottom border
      for (size_t i = 0; i < border_points_bot_transformed.size(); i++) {

        safety_area_marker.points.push_back(border_points_bot_transformed[i]);
        safety_area_marker.points.push_back(border_points_bot_transformed[(i + 1) % border_points_bot_transformed.size()]);

        std::stringstream ss;

        if (_safety_area_frame_ == "latlon_origin") {
          ss << "idx: " << i << std::endl
             << std::setprecision(6) << std::fixed << "lat: " << border_points_bot_original[i].x << std::endl
             << "lon: " << border_points_bot_original[i].y;
        } else {
          ss << "idx: " << i << std::endl
             << std::setprecision(1) << std::fixed << "lat: " << border_points_bot_original[i].x << std::endl
             << "lon: " << border_points_bot_original[i].y;
        }

        safety_area_coordinates_marker.color.r = 0;
        safety_area_coordinates_marker.color.g = 0;
        safety_area_coordinates_marker.color.b = 0;

        safety_area_coordinates_marker.pose.position = border_points_bot_transformed[i];
        safety_area_coordinates_marker.text          = ss.str();
        safety_area_coordinates_marker.id++;

        safety_area_coordinates_marker_array.markers.push_back(safety_area_coordinates_marker);
      }

      // top border + top/bot edges
      for (size_t i = 0; i < border_points_top_transformed.size(); i++) {

        safety_area_marker.points.push_back(border_points_top_transformed[i]);
        safety_area_marker.points.push_back(border_points_top_transformed[(i + 1) % border_points_top_transformed.size()]);

        safety_area_marker.points.push_back(border_points_bot_transformed[i]);
        safety_area_marker.points.push_back(border_points_top_transformed[i]);

        std::stringstream ss;

        if (_safety_area_frame_ == "latlon_origin") {
          ss << "idx: " << i << std::endl
             << std::setprecision(6) << std::fixed << "lat: " << border_points_bot_original[i].x << std::endl
             << "lon: " << border_points_bot_original[i].y;
        } else {
          ss << "idx: " << i << std::endl
             << std::setprecision(1) << std::fixed << "lat: " << border_points_bot_original[i].x << std::endl
             << "lon: " << border_points_bot_original[i].y;
        }

        safety_area_coordinates_marker.color.r = 1;
        safety_area_coordinates_marker.color.g = 1;
        safety_area_coordinates_marker.color.b = 1;

        safety_area_coordinates_marker.pose.position = border_points_top_transformed[i];
        safety_area_coordinates_marker.text          = ss.str();
        safety_area_coordinates_marker.id++;

        safety_area_coordinates_marker_array.markers.push_back(safety_area_coordinates_marker);
      }

      //}

      /* adding polygon obstacles points //{ */

      std::vector<mrs_lib::Polygon> polygon_obstacles = safety_zone_->getObstacles();

      for (auto polygon : polygon_obstacles) {

        std::vector<geometry_msgs::Point> points_bot = polygon.getPointMessageVector(min_height);
        std::vector<geometry_msgs::Point> points_top = polygon.getPointMessageVector(max_height);

        // transform border bottom points to local origin
        for (size_t i = 0; i < points_bot.size(); i++) {

          temp_ref.header.frame_id      = _safety_area_frame_;
          temp_ref.header.stamp         = ros::Time(0);
          temp_ref.reference.position.x = points_bot[i].x;
          temp_ref.reference.position.y = points_bot[i].y;
          temp_ref.reference.position.z = points_bot[i].z;

          if (auto ret = transformer_->transform(tf, temp_ref)) {

            temp_ref = ret.value();

            points_bot[i].x = temp_ref.reference.position.x;
            points_bot[i].y = temp_ref.reference.position.y;
            points_bot[i].z = temp_ref.reference.position.z;

          } else {
            tf_success = false;
          }
        }

        // transform border top points to local origin
        for (size_t i = 0; i < points_top.size(); i++) {

          temp_ref.header.frame_id      = _safety_area_frame_;
          temp_ref.header.stamp         = ros::Time(0);
          temp_ref.reference.position.x = points_top[i].x;
          temp_ref.reference.position.y = points_top[i].y;
          temp_ref.reference.position.z = points_top[i].z;

          if (auto ret = transformer_->transform(tf, temp_ref)) {

            temp_ref = ret.value();

            points_top[i].x = temp_ref.reference.position.x;
            points_top[i].y = temp_ref.reference.position.y;
            points_top[i].z = temp_ref.reference.position.z;

          } else {
            tf_success = false;
          }
        }

        // bottom points
        for (size_t i = 0; i < points_bot.size(); i++) {

          safety_area_marker.points.push_back(points_bot[i]);
          safety_area_marker.points.push_back(points_bot[(i + 1) % points_bot.size()]);
        }

        // top points + top/bot edges
        for (size_t i = 0; i < points_bot.size(); i++) {

          safety_area_marker.points.push_back(points_top[i]);
          safety_area_marker.points.push_back(points_top[(i + 1) % points_top.size()]);

          safety_area_marker.points.push_back(points_bot[i]);
          safety_area_marker.points.push_back(points_top[i]);
        }
      }

      //}

      /* adding point-obstacle points //{ */

      std::vector<mrs_lib::PointObstacle> point_obstacles = safety_zone_->getPointObstacles();

      for (auto point : point_obstacles) {

        std::vector<geometry_msgs::Point> points_bot = point.getPointMessageVector(min_height);
        std::vector<geometry_msgs::Point> points_top = point.getPointMessageVector(max_height);

        // transform bottom points to local origin
        for (size_t i = 0; i < points_bot.size(); i++) {

          temp_ref.header.frame_id      = _safety_area_frame_;
          temp_ref.header.stamp         = ros::Time(0);
          temp_ref.reference.position.x = points_bot[i].x;
          temp_ref.reference.position.y = points_bot[i].y;
          temp_ref.reference.position.z = points_bot[i].z;

          if (auto ret = transformer_->transform(tf, temp_ref)) {

            temp_ref        = ret.value();
            points_bot[i].x = temp_ref.reference.position.x;
            points_bot[i].y = temp_ref.reference.position.y;
            points_bot[i].z = temp_ref.reference.position.z;

          } else {
            tf_success = false;
          }
        }

        // transform top points to local origin
        for (size_t i = 0; i < points_top.size(); i++) {

          temp_ref.header.frame_id      = _safety_area_frame_;
          temp_ref.header.stamp         = ros::Time(0);
          temp_ref.reference.position.x = points_top[i].x;
          temp_ref.reference.position.y = points_top[i].y;
          temp_ref.reference.position.z = points_top[i].z;

          if (auto ret = transformer_->transform(tf, temp_ref)) {

            temp_ref = ret.value();

            points_top[i].x = temp_ref.reference.position.x;
            points_top[i].y = temp_ref.reference.position.y;
            points_top[i].z = temp_ref.reference.position.z;

          } else {
            tf_success = false;
          }
        }

        // botom points
        for (size_t i = 0; i < points_bot.size(); i++) {

          safety_area_marker.points.push_back(points_bot[i]);
          safety_area_marker.points.push_back(points_bot[(i + 1) % points_bot.size()]);
        }

        // top points + bot/top edges
        for (size_t i = 0; i < points_top.size(); i++) {

          safety_area_marker.points.push_back(points_top[i]);
          safety_area_marker.points.push_back(points_top[(i + 1) % points_top.size()]);

          safety_area_marker.points.push_back(points_bot[i]);
          safety_area_marker.points.push_back(points_top[i]);
        }
      }

      //}

      if (tf_success) {

        safety_area_marker_array.markers.push_back(safety_area_marker);

        try {
          publisher_safety_area_markers_.publish(safety_area_marker_array);
        }
        catch (...) {
          ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_safety_area_markers_.getTopic().c_str());
        }

        try {
          publisher_safety_area_coordinates_markers_.publish(safety_area_coordinates_marker_array);
        }
        catch (...) {
          ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_safety_area_markers_.getTopic().c_str());
        }
      }

    } else {
      ROS_WARN_THROTTLE(1.0, "[ControlManager]: missing TFs, cannot publish safety area markers");
    }
  }

  // --------------------------------------------------------------
  // |              publish the disturbances markers              |
  // --------------------------------------------------------------

  if (last_attitude_cmd != mrs_msgs::AttitudeCommand::Ptr() && got_uav_state_) {

    visualization_msgs::MarkerArray msg_out;

    double id = 0;

    double multiplier = 1.0;

    Eigen::Quaterniond quat_eigen(uav_state.pose.orientation.w, uav_state.pose.orientation.x, uav_state.pose.orientation.y, uav_state.pose.orientation.z);

    Eigen::Vector3d      vec3d;
    geometry_msgs::Point point;

    /* world x disturbance //{ */
    {

      visualization_msgs::Marker marker;

      marker.header.frame_id = uav_state.header.frame_id;
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
      point.x = uav_x;
      point.y = uav_y;
      point.z = uav_z;

      marker.points.push_back(point);

      //}

      /* tip //{ */

      point.x = uav_x + multiplier * last_attitude_cmd->disturbance_wx_w;
      point.y = uav_y;
      point.z = uav_z;

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

      marker.header.frame_id = uav_state.header.frame_id;
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
      point.x = uav_x;
      point.y = uav_y;
      point.z = uav_z;

      marker.points.push_back(point);

      //}

      /* tip //{ */

      point.x = uav_x;
      point.y = uav_y + multiplier * last_attitude_cmd->disturbance_wy_w;
      point.z = uav_z;

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

      marker.header.frame_id = uav_state.header.frame_id;
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

      point.x = uav_x;
      point.y = uav_y;
      point.z = uav_z;

      marker.points.push_back(point);

      //}

      /* tip //{ */

      vec3d << multiplier * last_attitude_cmd->disturbance_bx_b, 0, 0;
      vec3d = quat_eigen * vec3d;

      point.x = uav_x + vec3d[0];
      point.y = uav_y + vec3d[1];
      point.z = uav_z + vec3d[2];

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

      marker.header.frame_id = uav_state.header.frame_id;
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

      point.x = uav_x;
      point.y = uav_y;
      point.z = uav_z;

      marker.points.push_back(point);

      //}

      /* tip //{ */

      vec3d << 0, multiplier * last_attitude_cmd->disturbance_by_b, 0;
      vec3d = quat_eigen * vec3d;

      point.x = uav_x + vec3d[0];
      point.y = uav_y + vec3d[1];
      point.z = uav_z + vec3d[2];

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
      publisher_disturbances_markers_.publish(msg_out);
    }
    catch (...) {
      ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_disturbances_markers_.getTopic().c_str());
    }
  }

  // --------------------------------------------------------------
  // |               publish the current constraints              |
  // --------------------------------------------------------------

  if (got_constraints_) {

    auto sanitized_constraints = mrs_lib::get_mutexed(mutex_constraints_, sanitized_constraints_);

    mrs_msgs::TrackerConstraints constraints = sanitized_constraints.constraints;

    try {
      publisher_current_constraints_.publish(constraints);
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher_current_constraints_.getTopic().c_str());
    }
  }
}

//}

/* //{ safetyTimer() */

void ControlManager::safetyTimer(const ros::TimerEvent& event) {

  mrs_lib::ScopeUnset unset_running(running_safety_timer_);

  if (!is_initialized_)
    return;

  // copy member variables
  auto last_attitude_cmd                         = mrs_lib::get_mutexed(mutex_last_attitude_cmd_, last_attitude_cmd_);
  auto last_position_cmd                         = mrs_lib::get_mutexed(mutex_last_position_cmd_, last_position_cmd_);
  auto [uav_state, uav_state_last_time, uav_yaw] = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_, uav_state_last_time_, uav_yaw_);
  auto odometry_innovation                       = mrs_lib::get_mutexed(mutex_odometry_innovation_, odometry_innovation_);
  auto controller_tracker_switch_time            = mrs_lib::get_mutexed(mutex_controller_tracker_switch_time_, controller_tracker_switch_time_);
  auto active_controller_idx                     = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);
  auto active_tracker_idx                        = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("safetyTimer", _safety_timer_rate_, 0.04, event);

  if (!got_uav_state_ || !got_odometry_innovation_ || !got_pixhawk_odometry_ || active_tracker_idx == _null_tracker_idx_) {
    return;
  }

  if (odometry_switch_in_progress_) {
    ROS_WARN("[ControlManager]: safetyTimer tried to run while odometry switch in progress");
    return;
  }

  // | -------------- eland and failsafe thresholds ------------- |

  std::map<std::string, ControllerParams>::iterator it;
  it = controllers_.find(_controller_names_[active_controller_idx]);

  _eland_threshold_               = it->second.eland_threshold;
  _failsafe_threshold_            = it->second.failsafe_threshold;
  _odometry_innovation_threshold_ = it->second.odometry_innovation_threshold;

  // | --------- calculate control errors and tilt angle -------- |

  // This means that the failsafeTimer only does its work when Controllers and Trackers produce valid output.
  // Cases when the commands are not valid should be handle in updateControllers() and updateTrackers() methods.
  if (last_position_cmd == mrs_msgs::PositionCommand::Ptr() || last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {
    return;
  }

  {
    std::scoped_lock lock(mutex_attitude_error_);

    tilt_error_ = 0;
    yaw_error_  = 0;
  }

  // control errors
  {
    std::scoped_lock lock(mutex_control_error_);

    position_error_x_ = last_position_cmd->position.x - uav_state.pose.position.x;
    position_error_y_ = last_position_cmd->position.y - uav_state.pose.position.y;
    position_error_z_ = last_position_cmd->position.z - uav_state.pose.position.z;

    velocity_error_x_ = last_position_cmd->velocity.x - uav_state.velocity.linear.x;
    velocity_error_y_ = last_position_cmd->velocity.y - uav_state.velocity.linear.y;
    velocity_error_z_ = last_position_cmd->velocity.z - uav_state.velocity.linear.z;
  }

  // tilt angle
  tf::Quaternion odometry_quaternion;
  quaternionMsgToTF(uav_state.pose.orientation, odometry_quaternion);

  // rotate the drone's z axis
  tf::Vector3 uav_z_in_world = tf::Transform(odometry_quaternion) * tf::Vector3(0, 0, 1);

  // calculate the angle between the drone's z axis and the world's z axis
  double tilt_angle = acos(uav_z_in_world.dot(tf::Vector3(0, 0, 1)));

  // | ------------ calculate the tilt and yaw error ------------ |

  // | --------------------- the tilt error --------------------- |
  tf::Quaternion attitude_cmd_quaternion;

  // calculate the quaternion
  if (last_attitude_cmd->quater_attitude_set) {

    attitude_cmd_quaternion.setX(last_attitude_cmd->quater_attitude.x);
    attitude_cmd_quaternion.setY(last_attitude_cmd->quater_attitude.y);
    attitude_cmd_quaternion.setZ(last_attitude_cmd->quater_attitude.z);
    attitude_cmd_quaternion.setW(last_attitude_cmd->quater_attitude.w);

  } else if (last_attitude_cmd->euler_attitude_set) {

    // convert the RPY to quaternion
    attitude_cmd_quaternion =
        tf::createQuaternionFromRPY(last_attitude_cmd->euler_attitude.x, last_attitude_cmd->euler_attitude.y, last_attitude_cmd->euler_attitude.z);
  }

  if (last_attitude_cmd->quater_attitude_set || last_attitude_cmd->euler_attitude_set) {

    // calculate the desired drone's z axis in the world frame
    tf::Vector3 uav_z_in_world_desired = tf::Transform(attitude_cmd_quaternion) * tf::Vector3(0, 0, 1);

    // calculate the angle between the drone's z axis and the world's z axis
    {
      std::scoped_lock lock(mutex_attitude_error_);

      tilt_error_ = acos(uav_z_in_world.dot(uav_z_in_world_desired));
    }
  }

  // | ---------------------- the yaw error --------------------- |
  if (last_attitude_cmd->euler_attitude_set) {

    {
      std::scoped_lock lock(mutex_attitude_error_);

      yaw_error_ = last_attitude_cmd->euler_attitude.z - uav_yaw;
    }

  } else if (last_attitude_cmd->quater_attitude_set) {

    // calculate the euler angles
    tf::Quaternion quater_attitude_cmd;
    quaternionMsgToTF(last_attitude_cmd->quater_attitude, quater_attitude_cmd);
    tf::Matrix3x3 m(quater_attitude_cmd);
    double        attitude_cmd_roll, attitude_cmd_pitch, attitude_cmd_yaw;
    m.getRPY(attitude_cmd_roll, attitude_cmd_pitch, attitude_cmd_yaw);

    {
      std::scoped_lock lock(mutex_attitude_error_);

      yaw_error_ = angleDist(attitude_cmd_yaw, uav_yaw);
    }
  }

  // do not have to mutex the position error, since I am filling it in this function
  double control_error = sqrt(pow(position_error_x_, 2) + pow(position_error_y_, 2) + pow(position_error_z_, 2));

  // --------------------------------------------------------------
  // |   activate the failsafe controller in case of large error  |
  // --------------------------------------------------------------

  if (control_error > _failsafe_threshold_ && !failsafe_triggered_) {

    if ((ros::Time::now() - controller_tracker_switch_time).toSec() > 1.0) {

      if (!failsafe_triggered_) {

        ROS_ERROR("[ControlManager]: Activating failsafe land: control_error=%0.2f/%0.2f m (x: %.2f, y: %.2f, z: %.2f)", control_error, _failsafe_threshold_,
                  position_error_x_, position_error_y_, position_error_z_);

        failsafe();
      }
    }
  }

  // --------------------------------------------------------------
  // |     activate emergency land in case of large innovation    |
  // --------------------------------------------------------------

  double last_innovation = sqrt(pow(odometry_innovation.pose.pose.position.x, 2.0) + pow(odometry_innovation.pose.pose.position.y, 2.0) +
                                pow(odometry_innovation.pose.pose.position.z, 2.0));

  if (last_innovation > _odometry_innovation_threshold_) {

    if ((ros::Time::now() - controller_tracker_switch_time).toSec() > 1.0) {

      if (!failsafe_triggered_ && !eland_triggered_) {

        ROS_ERROR("[ControlManager]: Activating emergency land: odometry innovation too large: %.2f/%.2f (x: %.2f, y: %.2f, z: %.2f)", last_innovation,
                  _odometry_innovation_threshold_, odometry_innovation.pose.pose.position.x, odometry_innovation.pose.pose.position.y,
                  odometry_innovation.pose.pose.position.z);

        std::string message_out;
        eland(message_out);
      }
    }
  }

  // --------------------------------------------------------------
  // |   activate emergency land in case of medium control error  |
  // --------------------------------------------------------------

  // | ------------------- tilt control error ------------------- |
  if (tilt_angle > _tilt_limit_eland_) {

    if ((ros::Time::now() - controller_tracker_switch_time).toSec() > 1.0) {

      if (!failsafe_triggered_ && !eland_triggered_) {

        ROS_ERROR("[ControlManager]: Activating emergency land: tilt angle error %.2f/%.2f deg", (180.0 / M_PI) * tilt_angle,
                  (180.0 / M_PI) * _tilt_limit_eland_);

        std::string message_out;
        eland(message_out);
      }
    }
  }

  // | ----------------- position control error ----------------- |
  if (control_error > _eland_threshold_) {

    if ((ros::Time::now() - controller_tracker_switch_time).toSec() > 1.0) {

      if (!failsafe_triggered_ && !eland_triggered_) {

        ROS_ERROR("[ControlManager]: Activating emergency land: position error %.2f/%.2f m (x: %.2f, y: %.2f, z: %.2f)", control_error, _eland_threshold_,
                  position_error_x_, position_error_y_, position_error_z_);

        std::string message_out;
        eland(message_out);
      }
    }
  }

  // | -------------------- yaw control error ------------------- |
  // do not have to mutex the yaw_error_ here since I am filling it in this function
  if (yaw_error_ > _yaw_error_eland_threshold_) {

    if ((ros::Time::now() - controller_tracker_switch_time).toSec() > 1.0) {

      if (!failsafe_triggered_ && !eland_triggered_) {

        ROS_ERROR("[ControlManager]: Activating emergency land: yaw error %.2f/%.2f deg", (180.0 / M_PI) * yaw_error_,
                  (180.0 / M_PI) * _yaw_error_eland_threshold_);

        std::string message_out;
        eland(message_out);
      }
    }
  }

  // --------------------------------------------------------------
  // |      activate failsafe when odometry stops publishing      |
  // --------------------------------------------------------------
  // to do that, we need to fire up safetyTimer, which will regularly trigger the controllers
  // in place of the odometryCallback()

  if ((ros::Time::now() - uav_state_last_time).toSec() > _uav_state_max_missing_time_) {

    if (!failsafe_triggered_) {

      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: not receiving odometry for %.3f, initiating failsafe land.", (ros::Time::now() - uav_state_last_time).toSec());

      failsafe();
    }
  }

  // --------------------------------------------------------------
  // |      disarm the drone when the tilt exceeds the limit      |
  // --------------------------------------------------------------
  if (tilt_angle > _tilt_limit_disarm_) {

    ROS_ERROR("[ControlManager]: Tilt angle too large, disarming: tilt angle=%0.2f/%0.2f deg", (180.0 / M_PI) * tilt_angle,
              (180.0 / M_PI) * _tilt_limit_eland_);

    arming(false);
  }

  // --------------------------------------------------------------
  // |     disarm the drone when tilt error exceeds the limit     |
  // --------------------------------------------------------------

  if (_tilt_error_failsafe_enabled_) {

    // do not have to mutex the tilt error, since I am filling it in this function
    if (fabs(tilt_error_) > _tilt_error_threshold_) {

      if ((ros::Time::now() - controller_tracker_switch_time).toSec() > 1.0) {

        ROS_ERROR("[ControlManager]: Tilt error too large, disarming: tilt error=%0.2f/%0.2f deg", (180.0 / M_PI) * tilt_error_,
                  (180.0 / M_PI) * _tilt_error_threshold_);

        arming(false);

        failsafe_triggered_ = true;

      } else {

        ROS_ERROR("[ControlManager]: Tilt error too large (tilt error=%0.2f/%0.2f deg), however, controller/tracker just switched so its ok.",
                  (180.0 / M_PI) * tilt_error_, (180.0 / M_PI) * _tilt_error_threshold_);
      }
    }
  }

  // | --------- dropping out of OFFBOARD in mid flight --------- |

  // if we are not in offboard and the drone is in mid air (NullTracker is not active)
  if (!offboard_mode_ && active_tracker_idx != _null_tracker_idx_) {

    ROS_ERROR("[ControlManager]: we fell out of OFFBOARD in mid air, switching motors off");

    switchMotors(false);
  }
}

//}

/* //{ elandingTimer() */

void ControlManager::elandingTimer(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("elandingTimer", _elanding_timer_rate_, 0.01, event);

  // copy member variables
  auto last_attitude_cmd = mrs_lib::get_mutexed(mutex_last_attitude_cmd_, last_attitude_cmd_);

  if (current_state_landing_ == IDLE_STATE) {

    return;

  } else if (current_state_landing_ == LANDING_STATE) {

    if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {
      ROS_WARN_THROTTLE(1.0, "[ControlManager]: elandingTimer: last_attitude_cmd has not been initialized, returning");
      ROS_WARN_THROTTLE(1.0, "[ControlManager]: tip: the RC eland is probably triggered");
      return;
    }

    // recalculate the mass based on the thrust
    thrust_mass_estimate_ = pow((last_attitude_cmd->thrust - _motor_params_.hover_thrust_b) / _motor_params_.hover_thrust_a, 2) / _g_;
    ROS_INFO_THROTTLE(1.0, "[ControlManager]: landing: initial mass: %.2f thrust mass estimate: %.2f", landing_uav_mass_, thrust_mass_estimate_);

    // condition for automatic motor turn off
    if (((thrust_mass_estimate_ < _elanding_cutoff_mass_factor_ * landing_uav_mass_) || last_attitude_cmd->thrust < 0.01)) {

      if (!thrust_under_threshold_) {

        thrust_mass_estimate_first_time_ = ros::Time::now();
        thrust_under_threshold_          = true;
      }

      ROS_INFO_THROTTLE(0.1, "[ControlManager]: thrust is under cutoff factor for %.2f s", (ros::Time::now() - thrust_mass_estimate_first_time_).toSec());

    } else {

      thrust_mass_estimate_first_time_ = ros::Time::now();
      thrust_under_threshold_          = false;
    }

    if (thrust_under_threshold_ && ((ros::Time::now() - thrust_mass_estimate_first_time_).toSec() > _elanding_cutoff_timeout_)) {

      // enable callbacks? ... NO

      ROS_INFO("[ControlManager]: reached cutoff thrust, setting motors OFF");
      switchMotors(false);

      // disarm the drone
      if (_eland_disarm_enabled_) {

        ROS_INFO("[ControlManager]: calling for disarm");
        arming(false);
      }

      shutdown();

      changeLandingState(IDLE_STATE);

      ROS_WARN("[ControlManager]: emergency landing finished");

      elanding_timer_.stop();

      // we should NOT set eland_triggered_=true
    }
  }
}

//}

/* //{ partialLandingTimer() */

void ControlManager::partialLandingTimer(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("partialLandingTimer", _partial_landing_timer_rate_, 0.01, event);

  // copy member variables
  auto last_attitude_cmd = mrs_lib::get_mutexed(mutex_last_attitude_cmd_, last_attitude_cmd_);

  if (current_state_partial_landing_ == IDLE_STATE) {

    return;

  } else if (current_state_partial_landing_ == LANDING_STATE) {

    if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {
      ROS_WARN_THROTTLE(1.0, "[ControlManager]: partialLandingTimer: last_attitude_cmd has not been initialized, returning");
      ROS_WARN_THROTTLE(1.0, "[ControlManager]: tip: the RC eland is probably triggered");
      return;
    }

    // recalculate the mass based on the thrust
    thrust_mass_estimate_ = pow((last_attitude_cmd->thrust - _motor_params_.hover_thrust_b) / _motor_params_.hover_thrust_a, 2) / _g_;
    ROS_INFO_THROTTLE(1.0, "[ControlManager]: landing_uav_mass_: %.2f thrust_mass_estimate_: %.2f", landing_uav_mass_, thrust_mass_estimate_);

    // condition for automatic motor turn off
    if (((thrust_mass_estimate_ < _partial_landing_mass_factor_ * _uav_mass_) || last_attitude_cmd->thrust < 0.01)) {

      if (!thrust_under_threshold_) {

        thrust_mass_estimate_first_time_ = ros::Time::now();
        thrust_under_threshold_          = true;
      }

      ROS_INFO_THROTTLE(0.1, "[ControlManager]: thrust is under cutoff factor for %.2f s", (ros::Time::now() - thrust_mass_estimate_first_time_).toSec());

    } else {

      thrust_mass_estimate_first_time_ = ros::Time::now();
      thrust_under_threshold_          = false;
    }

    if (thrust_under_threshold_ && ((ros::Time::now() - thrust_mass_estimate_first_time_).toSec() > _partial_landing_cutoff_timeout_)) {

      // enable callbacks? ... NO

      ROS_INFO("[ControlManager]: reached cutoff thrust, switching to partial landing controller");

      changePartialLandingState(IDLE_STATE);

      mrs_msgs::AttitudeCommand::Ptr new_attitude_cmd(std::make_unique<mrs_msgs::AttitudeCommand>());
      new_attitude_cmd->mass_difference = landing_uav_mass_ - _uav_mass_;
      new_attitude_cmd->total_mass      = landing_uav_mass_;
      new_attitude_cmd->thrust = sqrt(_partial_landing_mass_factor_ * _uav_mass_ * _g_) * _motor_params_.hover_thrust_a + _motor_params_.hover_thrust_b;
      new_attitude_cmd->disturbance_bx_b = last_attitude_cmd->disturbance_bx_b;
      new_attitude_cmd->disturbance_by_b = last_attitude_cmd->disturbance_by_b;
      new_attitude_cmd->disturbance_bx_w = last_attitude_cmd->disturbance_bx_w;
      new_attitude_cmd->disturbance_by_w = last_attitude_cmd->disturbance_by_w;
      new_attitude_cmd->disturbance_wx_w = last_attitude_cmd->disturbance_wx_w;
      new_attitude_cmd->disturbance_wy_w = last_attitude_cmd->disturbance_wy_w;

      {
        std::scoped_lock lock(mutex_last_attitude_cmd_);

        last_attitude_cmd_ = new_attitude_cmd;
      }

      partial_landing_previous_controller_idx_ = active_controller_idx_;

      switchController(_partial_landing_controller_name_);

      ROS_WARN("[ControlManager]: partial landing finished");

      partial_landing_triggered_ = false;

      partial_landing_timer_.stop();
    }
  }
}

//}

/* //{ failsafeTimer() */

void ControlManager::failsafeTimer(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("failsafeTimer", _failsafe_timer_rate_, 0.01, event);

  // copy member variables
  auto last_attitude_cmd = mrs_lib::get_mutexed(mutex_last_attitude_cmd_, last_attitude_cmd_);
  auto pixhawk_odometry  = mrs_lib::get_mutexed(mutex_pixhawk_odometry_, pixhawk_odometry_);

  mrs_msgs::UavState pixhawk_odom_uav_state;
  pixhawk_odom_uav_state.header   = pixhawk_odometry.header;
  pixhawk_odom_uav_state.pose     = pixhawk_odometry.pose.pose;
  pixhawk_odom_uav_state.velocity = pixhawk_odometry.twist.twist;

  updateControllers(pixhawk_odom_uav_state);

  publish();

  if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {
    ROS_WARN_THROTTLE(1.0, "[ControlManager]: failsafeTimer: last_attitude_cmd has not been initialized, returning");
    ROS_WARN_THROTTLE(1.0, "[ControlManager]: tip: the RC eland is probably triggered");
    return;
  }

  double thrust_mass_estimate_ = pow((last_attitude_cmd_->thrust - _motor_params_.hover_thrust_b) / _motor_params_.hover_thrust_a, 2) / _g_;
  ROS_INFO_THROTTLE(1.0, "[ControlManager]: failsafe: initial mass: %.2f thrust_mass_estimate_: %.2f", landing_uav_mass_, thrust_mass_estimate_);

  // condition for automatic motor turn off
  if (((thrust_mass_estimate_ < _elanding_cutoff_mass_factor_ * landing_uav_mass_))) {

    if (!thrust_under_threshold_) {

      thrust_mass_estimate_first_time_ = ros::Time::now();
      thrust_under_threshold_          = true;
    }

    ROS_INFO_THROTTLE(0.1, "[ControlManager]: thrust is under cutoff factor for %.2f s", (ros::Time::now() - thrust_mass_estimate_first_time_).toSec());

  } else {

    thrust_mass_estimate_first_time_ = ros::Time::now();
    thrust_under_threshold_          = false;
  }

  // condition for automatic motor turn off
  if (thrust_under_threshold_ && ((ros::Time::now() - thrust_mass_estimate_first_time_).toSec() > _elanding_cutoff_timeout_)) {

    ROS_INFO("[ControlManager]: detecting zero thrust, disarming");

    arming(false);
  }
}

//}

/* //{ joystickTimer() */

void ControlManager::joystickTimer(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("joystickTimer", _status_timer_rate_, 0.01, event);

  // copy member variables
  auto rc_channels = mrs_lib::get_mutexed(mutex_rc_channels_, rc_channels_);

  std::scoped_lock lock(mutex_joystick_);

  // if start was pressed and held for > 3.0 s
  if (joystick_start_pressed_ && joystick_start_press_time_ != ros::Time(0) && (ros::Time::now() - joystick_start_press_time_).toSec() > 3.0) {

    joystick_start_press_time_ = ros::Time(0);

    ROS_INFO("[ControlManager]: transitioning to joystick control: activating %s and %s", _joystick_tracker_name_.c_str(), _joystick_controller_name_.c_str());

    joystick_start_pressed_ = false;

    switchTracker(_joystick_tracker_name_);
    switchController(_joystick_controller_name_);
  }

  // if RT+LT were pressed and held for > 0.1 s
  if (joystick_failsafe_pressed_ && joystick_failsafe_press_time_ != ros::Time(0) && (ros::Time::now() - joystick_failsafe_press_time_).toSec() > 0.1) {

    joystick_failsafe_press_time_ = ros::Time(0);

    ROS_INFO("[ControlManager]: activating failsafe by joystick");

    joystick_failsafe_pressed_ = false;

    failsafe();
  }

  // if joypads were pressed and held for > 0.1 s
  if (joystick_eland_pressed_ && joystick_eland_press_time_ != ros::Time(0) && (ros::Time::now() - joystick_eland_press_time_).toSec() > 0.1) {

    joystick_eland_press_time_ = ros::Time(0);

    ROS_INFO("[ControlManager]: activating eland by joystick");

    joystick_failsafe_pressed_ = false;

    std::string message_out;
    eland(message_out);
  }

  // if back was pressed and held for > 0.1 s
  if (joystick_back_pressed_ && joystick_back_press_time_ != ros::Time(0) && (ros::Time::now() - joystick_back_press_time_).toSec() > 0.1) {

    joystick_back_press_time_ = ros::Time(0);

    // activate/deactivate the joystick goto functionality
    joystick_goto_enabled_ = !joystick_goto_enabled_;

    ROS_INFO("[ControlManager]: joystick controll %s", joystick_goto_enabled_ ? "activated" : "deactivated");
  }

  // if the GOTO functionality is enabled...
  if (joystick_goto_enabled_) {

    // create the reference

    mrs_msgs::Vec4::Request request;

    if (fabs(joystick_data_.axes[_channel_pitch_]) >= 0.05 || fabs(joystick_data_.axes[_channel_roll_]) >= 0.05 ||
        fabs(joystick_data_.axes[_channel_yaw_]) >= 0.05 || fabs(joystick_data_.axes[_channel_thrust_]) >= 0.05) {

      if (_joystick_mode_ == 0) {

        request.goal[REF_X]   = _channel_mult_pitch_ * joystick_data_.axes[_channel_pitch_] * _joystick_carrot_distance_;
        request.goal[REF_Y]   = _channel_mult_roll_ * joystick_data_.axes[_channel_roll_] * _joystick_carrot_distance_;
        request.goal[REF_Z]   = _channel_mult_thrust_ * joystick_data_.axes[_channel_thrust_];
        request.goal[REF_YAW] = _channel_mult_yaw_ * joystick_data_.axes[_channel_yaw_];

        mrs_msgs::Vec4::Response response;

        callbackGoToFcuService(request, response);

      } else if (_joystick_mode_ == 1) {

        mrs_msgs::TrackerTrajectory trajectory;

        trajectory.fly_now         = true;
        trajectory.header.frame_id = "fcu_untilted";
        trajectory.use_yaw         = true;

        mrs_msgs::TrackerPoint point;
        point.x   = 0;
        point.y   = 0;
        point.z   = 0;
        point.yaw = 0;

        trajectory.points.push_back(point);

        double speed = 1.0;

        for (int i = 0; i < 50; i++) {

          point.x += _channel_mult_pitch_ * joystick_data_.axes[_channel_pitch_] * (speed * 0.2);
          point.y += _channel_mult_roll_ * joystick_data_.axes[_channel_roll_] * (speed * 0.2);
          point.z += _channel_mult_thrust_ * joystick_data_.axes[_channel_thrust_] * (speed * 0.2);
          point.yaw = _channel_mult_yaw_ * joystick_data_.axes[_channel_yaw_];

          trajectory.points.push_back(point);
        }

        try {
          publisher_mpc_trajectory_.publish(trajectory);
        }
        catch (...) {
          ROS_ERROR("Exception caught during publishing topic %s.", publisher_mpc_trajectory_.getTopic().c_str());
        }
      }
    }
  }

  if (rc_goto_active_ && last_position_cmd_ != mrs_msgs::PositionCommand::Ptr()) {

    // create the reference

    mrs_msgs::Vec4::Request request;

    double des_x   = 0;
    double des_y   = 0;
    double des_z   = 0;
    double des_yaw = 0;

    bool nothing_to_do = true;

    if (rc_channels.channels.size() < uint(4)) {

      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: RC control channel numbers are out of range (the # of channels in rc/in is %d)",
                         uint(rc_channels.channels.size()));
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: tip: this could be caused by the RC failsafe not being configured!");

    } else {

      double tmp_x   = RCChannelToRange(rc_channels.channels[_rc_channel_pitch_], _rc_joystick_carrot_distance_, 0.1);
      double tmp_y   = -RCChannelToRange(rc_channels.channels[_rc_channel_roll_], _rc_joystick_carrot_distance_, 0.1);
      double tmp_z   = RCChannelToRange(rc_channels.channels[_rc_channel_thrust_], _rc_joystick_carrot_distance_, 0.3);
      double tmp_yaw = -RCChannelToRange(rc_channels.channels[_rc_channel_yaw_], 1.0, 0.1);

      if (abs(tmp_x) > 1e-3) {
        des_x         = tmp_x;
        nothing_to_do = false;
      }

      if (abs(tmp_y) > 1e-3) {
        des_y         = tmp_y;
        nothing_to_do = false;
      }

      if (abs(tmp_z) > 1e-3) {
        des_z         = tmp_z;
        nothing_to_do = false;
      }

      if (abs(tmp_yaw) > 1e-3) {
        des_yaw       = tmp_yaw;
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
        std::scoped_lock lock(mutex_tracker_list_);

        tracker_list_[active_tracker_idx_]->enableCallbacks(
            std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
      }

      callbacks_enabled_ = true;

      // call the goto
      callbackGoToFcuService(request, response);

      callbacks_enabled_ = false;

      ROS_INFO("[ControlManager]: goto by rc by x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", request.goal[REF_X], request.goal[REF_Y], request.goal[REF_Z],
               request.goal[REF_YAW]);

      // disable the callbacks back again
      req_enable_callbacks.data = false;
      {
        std::scoped_lock lock(mutex_tracker_list_);

        tracker_list_[active_tracker_idx_]->enableCallbacks(
            std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
      }
    }
  }

  if (_rc_goto_enabled_ && got_rc_channels_) {

    // prune the list of rc_channel_switches
    std::list<ros::Time>::iterator it;
    for (it = rc_channel_switch_time_.begin(); it != rc_channel_switch_time_.end();) {
      if ((ros::Time::now() - *it).toSec() > _rc_joystick_timeout_) {
        it = rc_channel_switch_time_.erase(it);
      } else {
        it++;
      }
    }

    if (int(rc_channel_switch_time_.size()) >= _rc_joystick_n_switches_) {

      if (rc_goto_active_ == false) {

        ROS_INFO("[ControlManager]: activating rc joystick");

        callbacks_enabled_ = false;

        std_srvs::SetBoolRequest req_goto_out;
        req_goto_out.data = false;

        std_srvs::SetBoolRequest req_enable_callbacks;
        req_enable_callbacks.data = callbacks_enabled_;

        {
          std::scoped_lock lock(mutex_tracker_list_);

          // disable callbacks of all trackers
          for (unsigned int i = 0; i < tracker_list_.size(); i++) {
            tracker_list_[i]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
          }
        }

      } else if (rc_goto_active_ == true) {

        ROS_INFO("[ControlManager]: deactivating rc joystick");

        callbacks_enabled_ = true;

        std_srvs::SetBoolRequest req_goto_out;
        req_goto_out.data = true;

        std_srvs::SetBoolRequest req_enable_callbacks;
        req_enable_callbacks.data = callbacks_enabled_;

        {
          std::scoped_lock lock(mutex_tracker_list_);

          // enable callbacks of all trackers
          for (unsigned int i = 0; i < tracker_list_.size(); i++) {
            tracker_list_[i]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
          }
        }
      }

      rc_goto_active_ = !rc_goto_active_;
      rc_channel_switch_time_.clear();
    }
  }
}

//}

/* //{ bumperTimer() */

void ControlManager::bumperTimer(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("bumperTimer", _bumper_timer_rate_, 0.01, event);

  // copy member variables
  auto active_tracker_idx = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);
  auto bumper_data        = mrs_lib::get_mutexed(mutex_bumper_data_, bumper_data_);

  if (!bumper_enabled_ || !bumper_repulsion_enabled_) {
    return;
  }

  // do not use the bumper, unless with non-special tracker
  if (active_tracker_idx == _ehover_tracker_idx_ || active_tracker_idx == _null_tracker_idx_ || active_tracker_idx == _landoff_tracker_idx_) {
    return;
  }

  if (!got_uav_state_) {
    return;
  }

  if ((ros::Time::now() - bumper_data.header.stamp).toSec() > 1.0) {
    return;
  }

  // --------------------------------------------------------------
  // |                      bumper repulsion                      |
  // --------------------------------------------------------------

  bumperPushFromObstacle();
}

//}

/* //{ pirouetteTimer() */

void ControlManager::pirouetteTimer(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("pirouetteTimer", _pirouette_timer_rate_, 0.01, event);

  pirouette_iterator_++;

  double pirouette_duration  = (2 * M_PI) / _pirouette_speed_;
  double pirouette_n_steps   = pirouette_duration * _pirouette_timer_rate_;
  double pirouette_step_size = (2 * M_PI) / pirouette_n_steps;

  if (rc_eland_triggered_ || failsafe_triggered_ || eland_triggered_ || (pirouette_iterator_ > pirouette_duration * _pirouette_timer_rate_)) {

    _pirouette_enabled_ = false;
    pirouette_timer_.stop();

    setCallbacks(true);

    return;
  }

  {
    std::scoped_lock lock(mutex_tracker_list_);

    // enable the callbacks for the active tracker
    std_srvs::SetBoolRequest req_enable_callbacks;
    req_enable_callbacks.data = true;
    tracker_list_[active_tracker_idx_]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));

    // call the goto
    mrs_msgs::Float64SrvRequest req_goto_out;

    req_goto_out.value = pirouette_inital_yaw_ + pirouette_iterator_ * pirouette_step_size;

    mrs_msgs::Float64SrvResponse::ConstPtr tracker_response;
    tracker_response =
        tracker_list_[active_tracker_idx_]->setYaw(mrs_msgs::Float64SrvRequest::ConstPtr(std::make_unique<mrs_msgs::Float64SrvRequest>(req_goto_out)));

    // disable the callbacks for the active tracker
    req_enable_callbacks.data = false;
    tracker_list_[active_tracker_idx_]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
  }
}

//}

/* controlTimer() //{ */

void ControlManager::controlTimerOneshot([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  mrs_lib::ScopeUnset unset_running(running_control_timer_);

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("controlTimerOneshot");

  // copy member variables
  auto uav_state             = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto sanitized_constraints = mrs_lib::get_mutexed(mutex_constraints_, sanitized_constraints_);

  if (!failsafe_triggered_) {  // when failsafe is triggered, updateControllers() and publish() is called in failsafeTimer()

    // run the safety timer
    // in the case of large control errors, the safety mechanisms will be triggered before the controllers and trackers are updated...
    while (running_safety_timer_) {
      ROS_DEBUG("[ControlManager]: waiting for safety timer to finish");
      ros::Duration wait(0.001);
      wait.sleep();
    }

    ros::TimerEvent safety_timer_event;
    safetyTimer(safety_timer_event);

    updateTrackers();

    updateControllers(uav_state);

    if (got_constraints_) {

      // update the constraints to trackers, if need to

      if (enforceControllersConstraints(sanitized_constraints)) {
        setConstraints(sanitized_constraints);

        {
          std::scoped_lock lock(mutex_constraints_);

          sanitized_constraints_ = sanitized_constraints;
        }
      }
    }

    publish();
  }

  if (odometry_switch_in_progress_) {

    safety_timer_.start();
    odometry_switch_in_progress_ = false;

    {
      std::scoped_lock lock(mutex_uav_state_);

      ROS_INFO("[ControlManager]: odometry after switch: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", uav_state.pose.position.x, uav_state.pose.position.y,
               uav_state.pose.position.z, uav_yaw_);
    }
  }
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | --------------------- topic callbacks -------------------- |

/* //{ callbackOdometry() */

void ControlManager::callbackOdometry(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackOdometry");

  // | --------------------- check for nans --------------------- |

  if (!validateOdometry(msg)) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: odometry contains invalid values, throwing it away");
    return;
  }

  // | ---------------------- frame switch ---------------------- |

  /* Odometry frame switch //{ */

  // | -- prepare an OdometryConstPtr for trackers & controllers -- |

  mrs_msgs::UavState uav_state_odom;

  uav_state_odom.header   = msg->header;
  uav_state_odom.pose     = msg->pose.pose;
  uav_state_odom.velocity = msg->twist.twist;

  mrs_msgs::UavState::ConstPtr uav_state_const_ptr(std::make_unique<mrs_msgs::UavState>(uav_state_odom));

  // | ----- check for change in odometry frame of reference ---- |

  if (got_uav_state_) {
    if (msg->header.frame_id.compare(uav_state_.header.frame_id) != STRING_EQUAL) {

      ROS_INFO("[ControlManager]: detecting switch of odometry frame");
      {
        std::scoped_lock lock(mutex_uav_state_);

        ROS_INFO("[ControlManager]: odometry before switch: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", uav_state_.pose.position.x, uav_state_.pose.position.y,
                 uav_state_.pose.position.z, uav_yaw_);
      }

      odometry_switch_in_progress_ = true;

      // we have to stop safety timer, otherwise it will interfere
      safety_timer_.stop();
      // wait for the safety timer to stop if its running
      while (running_safety_timer_) {
        ROS_DEBUG("[ControlManager]: waiting for safety timer to finish");
        ros::Duration wait(0.001);
        wait.sleep();
      }

      // we have to also for the oneshot control timer to finish
      while (running_control_timer_) {
        ROS_DEBUG("[ControlManager]: waiting for control timer to finish");
        ros::Duration wait(0.001);
        wait.sleep();
      }

      {
        std::scoped_lock lock(mutex_controller_list_, mutex_tracker_list_);

        tracker_list_[active_tracker_idx_]->switchOdometrySource(uav_state_const_ptr);
        controller_list_[active_controller_idx_]->switchOdometrySource(uav_state_const_ptr);
      }
    }
  }

  //}

  // --------------------------------------------------------------
  // |                      copy the odometry                     |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = mrs_msgs::UavState();

    uav_state_.header   = msg->header;
    uav_state_.pose     = msg->pose.pose;
    uav_state_.velocity = msg->twist.twist;

    // calculate the euler angles
    tf::Quaternion uav_attitude;
    quaternionMsgToTF(msg->pose.pose.orientation, uav_attitude);
    tf::Matrix3x3 m(uav_attitude);
    m.getRPY(uav_roll_, uav_pitch_, uav_yaw_);

    uav_state_last_time_ = ros::Time::now();

    transformer_->setCurrentControlFrame(msg->header.frame_id);

    got_uav_state_ = true;
  }

  // run the control loop asynchronously in an OneShotTimer
  // but only if its not already running
  if (!running_control_timer_) {
    control_timer_.stop();
    control_timer_.start();
  }
}

//}

/* //{ callbackUavState() */

void ControlManager::callbackUavState(const mrs_msgs::UavStateConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackUavState");

  // | --------------------- check for nans --------------------- |

  if (!validateUavState(msg)) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: uav_state contains invalid values, throwing it away");
    return;
  }

  // | ---------------------- frame switch ---------------------- |

  /* frame switch //{ */

  // | -- prepare an OdometryConstPtr for trackers & controllers -- |

  mrs_msgs::UavState::ConstPtr uav_state_const_ptr(std::make_unique<mrs_msgs::UavState>(*msg));

  // | ----- check for change in odometry frame of reference ---- |

  if (got_uav_state_) {
    if (msg->estimator_iteration != uav_state_.estimator_iteration) {

      ROS_INFO("[ControlManager]: detecting switch of odometry frame");
      {
        std::scoped_lock lock(mutex_uav_state_);

        ROS_INFO("[ControlManager]: odometry before switch: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", uav_state_.pose.position.x, uav_state_.pose.position.y,
                 uav_state_.pose.position.z, uav_yaw_);
      }

      odometry_switch_in_progress_ = true;

      // we have to stop safety timer, otherwise it will interfere
      safety_timer_.stop();
      // wait for the safety timer to stop if its running
      while (running_safety_timer_) {
        ROS_DEBUG("[ControlManager]: waiting for safety timer to finish");
        ros::Duration wait(0.001);
        wait.sleep();
      }

      // we have to also for the oneshot control timer to finish
      while (running_control_timer_) {
        ROS_DEBUG("[ControlManager]: waiting for control timer to finish");
        ros::Duration wait(0.001);
        wait.sleep();
      }

      {
        std::scoped_lock lock(mutex_controller_list_, mutex_tracker_list_);

        tracker_list_[active_tracker_idx_]->switchOdometrySource(uav_state_const_ptr);
        controller_list_[active_controller_idx_]->switchOdometrySource(uav_state_const_ptr);
      }
    }
  }

  //}

  // --------------------------------------------------------------
  // |           copy the UavState message for later use          |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = *msg;

    // calculate the euler angles
    tf::Quaternion uav_quaternion;
    quaternionMsgToTF(uav_state_.pose.orientation, uav_quaternion);
    tf::Matrix3x3 m(uav_quaternion);
    m.getRPY(uav_roll_, uav_pitch_, uav_yaw_);

    uav_state_last_time_ = ros::Time::now();

    transformer_->setCurrentControlFrame(msg->header.frame_id);

    got_uav_state_ = true;
  }

  // run the control loop asynchronously in an OneShotTimer
  // but only if its not already running
  if (!running_control_timer_) {
    control_timer_.stop();
    control_timer_.start();
  }
}

//}

/* //{ callbackOdometryInnovation() */

void ControlManager::callbackOdometryInnovation(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackOdometryInnovation");

  {
    std::scoped_lock lock(mutex_odometry_innovation_);

    odometry_innovation_ = *msg;

    got_odometry_innovation_ = true;
  }
}

//}

/* //{ callbackPixhawkOdometry() */

void ControlManager::callbackPixhawkOdometry(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackPixhawkOdometry");

  // --------------------------------------------------------------
  // |                      copy the odometry                     |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_pixhawk_odometry_);

    pixhawk_odometry_ = *msg;

    got_pixhawk_odometry_ = true;
  }
}

//}

/* //{ callbackMavrosGps() */

void ControlManager::callbackMavrosGps(const sensor_msgs::NavSatFixConstPtr& msg) {

  if (!is_initialized_)
    return;

  transformer_->setCurrentLatLon(msg->latitude, msg->longitude);
}

//}

/* callbackMaxHeight() //{ */

void ControlManager::callbackMaxHeight(const mrs_msgs::Float64StampedConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackMaxHeight");

  {
    std::scoped_lock lock(mutex_max_height_external_);

    max_height_external_ = msg->value;

    got_max_height_ = true;
  }
}

//}

/* callbackJoystick() //{ */

void ControlManager::callbackJoystick(const sensor_msgs::JoyConstPtr& msg) {

  if (!is_initialized_)
    return;

  // copy member variables
  auto active_tracker_idx    = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackJoystick");

  std::scoped_lock lock(mutex_joystick_);

  joystick_data_ = *msg;

  // | ---- switching back to fallback tracker and controller --- |

  // if any of the A, B, X, Y buttons are pressed when flying with joystick, switch back to fallback controller and tracker
  if ((msg->buttons[_channel_A_] == 1 || msg->buttons[_channel_B_] == 1 || msg->buttons[_channel_X_] == 1 || msg->buttons[_channel_Y_] == 1) &&
      active_tracker_idx == _joystick_tracker_idx_ && active_controller_idx == _joystick_controller_idx_) {

    ROS_INFO("[ControlManager]: switching from joystick to normal control");

    switchTracker(_joystick_fallback_tracker_name_);
    switchController(_joystick_fallback_controller_name_);

    joystick_goto_enabled_ = false;
  }

  // | ------- joystick control activation ------- |

  // if start button was pressed
  if (msg->buttons[_channel_start_] == 1) {

    if (!joystick_start_pressed_) {

      ROS_INFO("[ControlManager]: joystick start button pressed");

      joystick_start_pressed_    = true;
      joystick_start_press_time_ = ros::Time::now();
    }

  } else if (joystick_start_pressed_) {

    ROS_INFO("[ControlManager]: joystick start button released");

    joystick_start_pressed_    = false;
    joystick_start_press_time_ = ros::Time(0);
  }

  // | ---------------- Joystick goto activation ---------------- |

  // if back button was pressed
  if (msg->buttons[_channel_back_] == 1) {

    if (!joystick_back_pressed_) {

      ROS_INFO("[ControlManager]: joystick back button pressed");

      joystick_back_pressed_    = true;
      joystick_back_press_time_ = ros::Time::now();
    }

  } else if (joystick_back_pressed_) {

    ROS_INFO("[ControlManager]: joystick back button released");

    joystick_back_pressed_    = false;
    joystick_back_press_time_ = ros::Time(0);
  }

  // | ------------------------ Failsafes ----------------------- |

  // if LT and RT buttons are both pressed down
  if (msg->axes[_channel_LT_] < -0.99 && msg->axes[_channel_RT_] < -0.99) {

    if (!joystick_failsafe_pressed_) {

      ROS_INFO("[ControlManager]: joystick Failsafe pressed");

      joystick_failsafe_pressed_    = true;
      joystick_failsafe_press_time_ = ros::Time::now();
    }

  } else if (joystick_failsafe_pressed_) {

    ROS_INFO("[ControlManager]: joystick Failsafe released");

    joystick_failsafe_pressed_    = false;
    joystick_failsafe_press_time_ = ros::Time(0);
  }

  // if left and right joypads are both pressed down
  if (msg->buttons[_channel_L_joy_] == 1 && msg->buttons[_channel_R_joy_] == 1) {

    if (!joystick_eland_pressed_) {

      ROS_INFO("[ControlManager]: joystick eland pressed");

      joystick_eland_pressed_    = true;
      joystick_eland_press_time_ = ros::Time::now();
    }

  } else if (joystick_eland_pressed_) {

    ROS_INFO("[ControlManager]: joystick eland released");

    joystick_eland_pressed_    = false;
    joystick_eland_press_time_ = ros::Time(0);
  }
}

//}

/* callbackBumper() //{ */

void ControlManager::callbackBumper(const mrs_msgs::ObstacleSectorsConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackBumper");

  ROS_INFO_ONCE("[ControlManager]: getting bumper data");

  {
    std::scoped_lock lock(mutex_bumper_data_);

    bumper_data_ = *msg;

    got_bumper_ = true;
  }
}

//}

/* //{ callbackMavrosState() */

void ControlManager::callbackMavrosState(const mavros_msgs::StateConstPtr& msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackMavrosState");

  std::scoped_lock lock(mutex_mavros_state_);

  // | --------------------- save the state --------------------- |
  mavros_state_     = *msg;
  got_mavros_state_ = true;

  // | ------ detect and print the changes in offboard mode ----- |
  if (mavros_state_.mode.compare(std::string("OFFBOARD")) == STRING_EQUAL) {

    if (!offboard_mode_) {
      offboard_mode_ = true;
      ROS_INFO("[ControlManager]: OFFBOARD mode ON");
    }

  } else {

    if (offboard_mode_) {
      offboard_mode_ = false;
      ROS_INFO("[ControlManager]: OFFBOARD mode OFF");
    }
  }

  // | --------- detect and print the changes in arming --------- |
  if (mavros_state_.armed == true) {

    if (!armed_) {
      armed_ = true;
      ROS_INFO("[ControlManager]: vehicle ARMED");
    }

  } else {

    if (armed_) {
      armed_ = false;
      ROS_INFO("[ControlManager]: vehicle DISARMED");
    }
  }
}

//}

/* //{ callbackRC() */

void ControlManager::callbackRC(const mavros_msgs::RCInConstPtr& msg) {

  if (!is_initialized_)
    return;

  if (rc_eland_triggered_) {
    return;
  }

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackRC");

  ROS_INFO_ONCE("[ControlManager]: getting RC channels");

  {
    std::scoped_lock lock(mutex_rc_channels_);

    rc_channels_ = *msg;

    got_rc_channels_ = true;
  }

  // | ------------------- rc joystic control ------------------- |

  // when the switch change its position
  if (_rc_goto_enabled_) {

    if (uint(_rc_joystick_channel_) >= msg->channels.size()) {

      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: RC joystick activation channel number (%d) is out of range [0-%d]", uint(_rc_joystick_channel_),
                         uint(msg->channels.size()));

    } else {

      // detect the switch of a switch on the RC
      if ((rc_joystick_channel_last_value_ < PWM_MIDDLE && msg->channels[_rc_joystick_channel_] > PWM_MIDDLE) ||
          (rc_joystick_channel_last_value_ > PWM_MIDDLE && msg->channels[_rc_joystick_channel_] < PWM_MIDDLE)) {

        // enter an event to the std vector
        std::scoped_lock lock(mutex_rc_channel_switch_time_);

        rc_channel_switch_time_.insert(rc_channel_switch_time_.begin(), ros::Time::now());
      }

      // do not forget to update the last... variable
      rc_joystick_channel_last_value_ = msg->channels[_rc_joystick_channel_];
    }
  }

  // | ------------------------ rc eland ------------------------ |
  if (_rc_eland_enabled_) {

    if (uint(_rc_eland_channel_) >= msg->channels.size()) {

      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: RC eland channel number (%d) is out of range [0-%d]", _rc_eland_channel_, uint(msg->channels.size()));

    } else {

      if (_rc_eland_action_.compare(ELAND_STR) == STRING_EQUAL) {

        if (msg->channels[_rc_eland_channel_] >= uint(_rc_eland_threshold_) && !eland_triggered_ && !failsafe_triggered_ && !rc_eland_triggered_) {

          ROS_WARN("[ControlManager]: triggering eland by RC");

          rc_eland_triggered_ = true;

          std::string message_out;
          eland(message_out);
        }
      } else if (_rc_eland_action_.compare(ESCALATING_FAILSAFE_STR) == STRING_EQUAL) {

        std::string message_out;
        escalatingFailsafe(message_out);

      } else if (_rc_eland_action_.compare(FAILSAFE_STR) == STRING_EQUAL) {

        if (!failsafe_triggered_) {

          ROS_WARN("[ControlManager]: triggering failsafe by RC");

          failsafe();
        }
      }
    }
  }
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackSwitchTracker() */

bool ControlManager::callbackSwitchTracker(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res) {

  if (!is_initialized_)
    return false;

  std::stringstream message;

  if (failsafe_triggered_ || eland_triggered_ || rc_eland_triggered_) {

    message << "Cannot switch tracker, eland or failsafe active.";

    res.message = message.str();
    res.success = false;

    ROS_WARN_STREAM("[ControlManager]: " << message.str());

    return true;
  }

  auto [success, response] = switchTracker(req.value);

  res.success = success;
  res.message = response;

  return true;
}

//}

/* callbackSwitchController() //{ */

bool ControlManager::callbackSwitchController(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res) {

  if (!is_initialized_)
    return false;

  std::stringstream message;

  if (failsafe_triggered_ || eland_triggered_ || rc_eland_triggered_) {

    message << "Cannot switch controller, eland or failsafe active.";

    res.message = message.str();
    res.success = false;

    ROS_WARN_STREAM("[ControlManager]: " << message.str());

    return true;
  }

  auto [success, response] = switchController(req.value);

  res.success = success;
  res.message = response;

  return true;
}

//}

/* //{ callbackSwitchTracker() */

bool ControlManager::callbackTrackerResetStatic([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  std::stringstream message;

  if (failsafe_triggered_ || eland_triggered_ || rc_eland_triggered_) {

    message << "Cannot reset tracker, eland or failsafe active.";

    res.message = message.str();
    res.success = false;

    ROS_WARN_STREAM("[ControlManager]: " << message.str());

    return true;
  }

  // reactivate the current tracker
  {
    std::scoped_lock lock(mutex_tracker_list_);

    std::string tracker_name = _tracker_names_[active_tracker_idx_];

    bool succ = tracker_list_[active_tracker_idx_]->resetStatic();

    if (succ) {
      message << "The tracker " << tracker_name << " was reset";
      ROS_INFO_STREAM("[ControlManager]: " << message.str());
    } else {
      message << "The tracker " << tracker_name << " reset failed!";
      ROS_ERROR_STREAM("[ControlManager]: " << message.str());
    }
  }

  res.message = message.str();
  res.success = true;

  return true;
}

//}

/* //{ callbackEHover() */

bool ControlManager::callbackEHoverService([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  std::stringstream message;

  if (failsafe_triggered_ || eland_triggered_ || rc_eland_triggered_) {

    message << "Cannot switch controller, eland or failsafe active.";

    res.message = message.str();
    res.success = false;

    ROS_WARN_STREAM("[ControlManager]: " << message.str());

    return true;
  }

  res.success = ehover(res.message);

  return true;
}

//}

/* callbackFailsafe() //{ */

bool ControlManager::callbackFailsafe([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  std::stringstream message;

  if (failsafe_triggered_) {

    message << "Cannot activate failsafe, it is already active.";

    res.message = message.str();
    res.success = false;

    ROS_INFO_STREAM("[ControlManager]: " << message.str());

    return true;
  }

  res.success = failsafe();

  return true;
}

//}

/* callbackFailsafeEscalating() //{ */

bool ControlManager::callbackFailsafeEscalating([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  res.success = escalatingFailsafe(res.message);

  return true;
}

//}

/* //{ callbackELand() */

bool ControlManager::callbackEland([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  res.success = eland(res.message);

  return true;
}

//}

/* //{ callbackPartialLanding() */

bool ControlManager::callbackPartialLanding([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  std::stringstream message;

  if (failsafe_triggered_ || eland_triggered_ || rc_eland_triggered_) {

    message << "Cannot activate partial landing, eland or failsafe active.";

    res.message = message.str();
    res.success = false;

    ROS_ERROR_STREAM("[ControlManager]: " << message.str());

    return true;
  }

  res.success = partialLanding(res.message);

  return true;
}

//}

/* //{ callbackMotors() */

bool ControlManager::callbackMotors(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  if (!is_initialized_)
    return false;

  // copy member variables
  auto uav_state    = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto mavros_state = mrs_lib::get_mutexed(mutex_mavros_state_, mavros_state_);

  char message[400];

  mrs_msgs::ReferenceStamped current_coord;
  current_coord.header.frame_id      = uav_state.header.frame_id;
  current_coord.reference.position.x = uav_state.pose.position.x;
  current_coord.reference.position.y = uav_state.pose.position.y;

  if (!isPointInSafetyArea2d(current_coord)) {

    sprintf((char*)&message, "Can't switch motors on, the UAV is outside of the safety area!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[ControlManager]: %s", message);
    return true;
  }

  if (req.data && (failsafe_triggered_ || eland_triggered_ || rc_eland_triggered_)) {
    sprintf((char*)&message, "cannot switch motors ON, we landed in emergency.");
    res.message = message;
    res.success = false;
    ROS_ERROR("[ControlManager]: %s", message);
    return true;
  }

  if (!got_mavros_state_ || (ros::Time::now() - mavros_state.header.stamp).toSec() > 5.0) {
    sprintf((char*)&message, "Can't switch motors ON, missing mavros state!");
    res.message = message;
    res.success = false;
    ROS_ERROR("[ControlManager]: %s", message);
    return true;
  }

  if (bumper_enabled_) {
    if (!got_bumper_) {
      sprintf((char*)&message, "Can't switch motors on, missing bumper data!");
      res.message = message;
      res.success = false;
      ROS_ERROR("[ControlManager]: %s", message);
      return true;
    }
  }

  switchMotors(req.data);

  sprintf((char*)&message, "Motors: %s", motors_ ? "ON" : "OFF");
  res.message = message;
  res.success = true;

  ROS_INFO("[ControlManager]: %s", message);

  return true;
}

//}

/* callbackArm() //{ */

bool ControlManager::callbackArm(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  std::stringstream message;

  if (failsafe_triggered_ || eland_triggered_ || rc_eland_triggered_) {

    message << "Cannot " << (req.data ? "arm" : "disarm") << ", eland or failsafe active";

    res.message = message.str();
    res.success = false;

    ROS_ERROR_STREAM("[ControlManager]: " << message.str());

    return true;
  }

  if (req.data) {

    message << "Not allowed to arm the drone.";
    res.success = false;
    ROS_ERROR_STREAM("[ControlManager]: " << message.str());

  } else {

    if (arming(false)) {

      message << "Disarmed";
      res.success = true;
      ROS_INFO_STREAM("[ControlManager]: " << message.str());

    } else {

      message << "Could not disarm";
      res.success = false;
      ROS_ERROR_STREAM("[ControlManager]: " << message.str());
    }
  }

  res.message = message.str();

  return true;
}

//}

/* //{ callbackEnableCallbacks() */

bool ControlManager::callbackEnableCallbacks(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  if (!is_initialized_)
    return false;

  setCallbacks(req.data);

  char message[400];
  sprintf((char*)&message, "Callbacks: %s", motors_ ? "enabled" : "disabled");
  res.message = message;
  res.success = true;

  ROS_INFO("[ControlManager]: %s", message);

  return true;
}

//}

/* callbackSetConstraints() //{ */

bool ControlManager::callbackSetConstraints(mrs_msgs::TrackerConstraintsSrv::Request& req, mrs_msgs::TrackerConstraintsSrv::Response& res) {

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  // copy member variables
  auto sanitized_constraints = mrs_lib::get_mutexed(mutex_constraints_, sanitized_constraints_);

  {
    std::scoped_lock lock(mutex_constraints_);

    current_constraints_   = req;
    sanitized_constraints_ = req;
    got_constraints_       = true;

    enforceControllersConstraints(sanitized_constraints_);

    sanitized_constraints = sanitized_constraints_;
  }

  setConstraints(sanitized_constraints);

  res.message = "Setting constraints";
  res.success = true;

  return true;
}

//}

/* //{ callbackEmergencyReferenceService() */

bool ControlManager::callbackEmergencyReferenceService(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res) {

  if (!is_initialized_)
    return false;

  callbacks_enabled_ = false;

  mrs_msgs::ReferenceSrvResponse::ConstPtr tracker_response;
  char                                     message[400];

  std_srvs::SetBoolRequest req_enable_callbacks;

  mrs_msgs::ReferenceSrvRequest req_goto_out;
  req_goto_out.reference = req.reference;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    // disable callbacks of all trackers
    req_enable_callbacks.data = false;
    for (unsigned int i = 0; i < tracker_list_.size(); i++) {
      tracker_list_[i]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
    }

    // enable the callbacks for the active tracker
    req_enable_callbacks.data = true;
    tracker_list_[active_tracker_idx_]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));

    // call the goto
    tracker_response =
        tracker_list_[active_tracker_idx_]->goTo(mrs_msgs::ReferenceSrvRequest::ConstPtr(std::make_unique<mrs_msgs::ReferenceSrvRequest>(req_goto_out)));

    // disable the callbacks back again
    req_enable_callbacks.data = false;
    tracker_list_[active_tracker_idx_]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));

    if (tracker_response != mrs_msgs::ReferenceSrvResponse::Ptr()) {
      res.message = tracker_response->message;
      res.success = tracker_response->success;
    } else {
      sprintf((char*)&message, "The tracker '%s' does not implement 'goto' service!", _tracker_names_[active_tracker_idx_].c_str());
      res.message = message;
      res.success = false;
    }
  }

  return true;
}

//}

/* callbackPirouette() //{ */

bool ControlManager::callbackPirouette([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  // copy member variables
  auto uav_yaw = mrs_lib::get_mutexed(mutex_uav_state_, uav_yaw_);

  if (_pirouette_enabled_) {
    res.success = false;
    res.message = "already active";
    return true;
  }

  std::stringstream message;

  if (failsafe_triggered_ || eland_triggered_ || rc_eland_triggered_) {

    message << "Cannot activate pirouette, eland or failsafe active.";

    res.message = message.str();
    res.success = false;

    ROS_ERROR_STREAM("[ControlManager]: " << message.str());

    return true;
  }

  _pirouette_enabled_ = true;

  setCallbacks(false);

  pirouette_inital_yaw_ = uav_yaw;
  pirouette_iterator_   = 0;
  pirouette_timer_.start();

  res.success = true;
  res.message = "activated";

  return true;
}

//}

/* callbackUseJoystick() //{ */

bool ControlManager::callbackUseJoystick([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  std::stringstream message;

  {
    auto [success, response] = switchTracker(_joystick_tracker_name_);

    if (!success) {

      message << "Switching to " << _joystick_tracker_name_ << " was unsuccessfull: " << response;
      ROS_ERROR_STREAM("[ControlManager]: " << message.str());

      res.success = false;
      res.message = message.str();

      return true;
    }
  }

  auto [success, response] = switchController(_joystick_controller_name_);

  if (!success) {

    message << "Switching to " << _joystick_controller_name_ << " was unsuccessfull: " << response;
    ROS_ERROR_STREAM("[ControlManager]: " << message.str());

    res.success = false;
    res.message = message.str();

    // switch back to hover tracker
    switchTracker(_ehover_tracker_name_);

    // switch back to safety controller
    switchController(_eland_controller_name_);

    ROS_ERROR_STREAM("[ControlManager]: " << message.str());

    return true;
  }

  message << "Switched to joystick control";

  res.success = true;
  res.message = message.str();

  ROS_INFO_STREAM("[ControlManager]: " << message.str());

  return true;
}

//}

/* //{ callbackHoverService() */

bool ControlManager::callbackHoverService([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  res.success = hover(res.message);

  return true;
}

//}

/* //{ callbackTransformReference() */

bool ControlManager::callbackTransformReference(mrs_msgs::TransformReferenceSrv::Request& req, mrs_msgs::TransformReferenceSrv::Response& res) {

  if (!is_initialized_)
    return false;

  // transform the reference to the current frame
  mrs_msgs::ReferenceStamped transformed_reference = req.reference;

  if (auto ret = transformer_->transformSingle(req.frame_id, transformed_reference)) {

    res.reference = ret.value();
    res.message   = "transformation successful";
    res.success   = true;
    return true;

  } else {

    res.message = "the reference could not be transformed";
    res.success = false;
    return true;
  }

  return true;
}

//}

/* //{ callbackTransformPose() */

bool ControlManager::callbackTransformPose(mrs_msgs::TransformPoseSrv::Request& req, mrs_msgs::TransformPoseSrv::Response& res) {

  if (!is_initialized_)
    return false;

  // transform the reference to the current frame
  geometry_msgs::PoseStamped transformed_pose = req.pose;

  if (auto ret = transformer_->transformSingle(req.frame_id, transformed_pose)) {

    res.pose    = ret.value();
    res.message = "transformation successful";
    res.success = true;
    return true;

  } else {

    res.message = "the pose could not be transformed";
    res.success = false;
    return true;
  }

  return true;
}

//}

/* //{ callbackTransformVector3() */

bool ControlManager::callbackTransformVector3(mrs_msgs::TransformVector3Srv::Request& req, mrs_msgs::TransformVector3Srv::Response& res) {

  if (!is_initialized_)
    return false;

  // transform the reference to the current frame
  geometry_msgs::Vector3Stamped transformed_vector3 = req.vector;

  if (auto ret = transformer_->transformSingle(req.frame_id, transformed_vector3)) {

    res.vector  = ret.value();
    res.message = "transformation successful";
    res.success = true;
    return true;

  } else {

    res.message = "the twist could not be transformed";
    res.success = false;
    return true;
  }

  return true;
}

//}

/* //{ callbackBumperEnableService() */

bool ControlManager::callbackBumperEnableService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  if (!is_initialized_)
    return false;

  bumper_enabled_ = req.data;

  char message[400];

  sprintf((char*)&message, "bumper %s", bumper_enabled_ ? "ENALBED" : "DISABLED");

  ROS_INFO("[ControlManager]: %s", message);

  res.success = true;
  res.message = message;

  return true;
}

//}

/* //{ callbackBumperEnableRepulsionService() */

bool ControlManager::callbackBumperEnableRepulsionService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  if (!is_initialized_)
    return false;

  bumper_repulsion_enabled_ = req.data;

  char message[400];

  sprintf((char*)&message, "bumper repulsion %s", bumper_repulsion_enabled_ ? "ENALBED" : "DISABLED");

  ROS_INFO("[ControlManager]: %s", message);

  res.success = true;
  res.message = message;

  return true;
}

//}

/* //{ callbackSetMinHeight() */

bool ControlManager::callbackSetMinHeight(mrs_msgs::Float64Srv::Request& req, mrs_msgs::Float64Srv::Response& res) {

  if (!is_initialized_)
    return false;

  double min_height = req.value;

  std::stringstream message;

  message << "The minimal height set to " << min_height_ << ".";

  ROS_INFO_STREAM("[ControlManager]: " << message.str());

  mrs_lib::set_mutexed(mutex_min_height_, min_height, min_height_);

  res.success = true;
  res.message = message.str();

  return true;
}

//}

/* //{ callbackGetMinHeight() */

bool ControlManager::callbackGetMinHeight([[maybe_unused]] mrs_msgs::GetFloat64::Request& req, mrs_msgs::GetFloat64::Response& res) {

  if (!is_initialized_)
    return false;

  auto min_height = mrs_lib::get_mutexed(mutex_min_height_, min_height_);

  res.success = true;
  res.value   = min_height;

  return true;
}

//}

// | -------------- setpoint topics and services -------------- |

/* //{ callbackReferenceService() */

bool ControlManager::callbackReferenceService(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res) {

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  if (!callbacks_enabled_) {
    ROS_WARN("[ControlManager]: not passing the goto service through, the callbacks are disabled");
    res.message = "callbacks are disabled";
    res.success = false;
    return true;
  }

  if (!std::isfinite(req.reference.position.x)) {
    ROS_ERROR("[ControlManager]: NaN detected in variable \"req.reference.position.x\"!!!");
    res.message = "NaNs/infs in the goal!";
    res.success = false;
    return true;
  }

  if (!std::isfinite(req.reference.position.y)) {
    ROS_ERROR("[ControlManager]: NaN detected in variable \"req.reference.position.y\"!!!");
    res.message = "NaNs/infs in the goal!";
    res.success = false;
    return true;
  }

  if (!std::isfinite(req.reference.position.z)) {
    ROS_ERROR("[ControlManager]: NaN detected in variable \"req.reference.position.z\"!!!");
    res.message = "NaNs/infs in the goal!";
    res.success = false;
    return true;
  }

  if (!std::isfinite(req.reference.yaw)) {
    ROS_ERROR("[ControlManager]: NaN detected in variable \"req.reference.yaw\"!!!");
    res.message = "NaNs/infs in the goal!";
    res.success = false;
    return true;
  }

  // copy member variables
  auto uav_state         = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_position_cmd = mrs_lib::get_mutexed(mutex_last_position_cmd_, last_position_cmd_);

  // transform the reference to the current frame
  mrs_msgs::ReferenceStamped original_reference;
  original_reference.header    = req.header;
  original_reference.reference = req.reference;

  auto ret = transformer_->transformSingle(uav_state.header.frame_id, original_reference);

  if (!ret) {

    ROS_WARN("[ControlManager]: the reference could not be transformed.");
    res.message = "the reference could not be transformed";
    res.success = false;
    return true;
  }

  mrs_msgs::ReferenceStamped transformed_reference = ret.value();

  // check the obstacle bumper
  if (!bumperValidatePoint(transformed_reference)) {
    ROS_ERROR("[ControlManager]: 'set_reference' service failed, potential collision with an obstacle!");
    res.message = "potential collision with an obstacle";
    res.success = false;
    return true;
  }

  if (!isPointInSafetyArea3d(transformed_reference)) {
    ROS_ERROR("[ControlManager]: 'set_reference' service failed, the point is outside of the safety area!");
    res.message = "the point is outside of the safety area";
    res.success = false;
    return true;
  }

  if (last_position_cmd != mrs_msgs::PositionCommand::Ptr()) {

    mrs_msgs::ReferenceStamped from_point;
    from_point.header.frame_id      = uav_state.header.frame_id;
    from_point.reference.position.x = last_position_cmd->position.x;
    from_point.reference.position.y = last_position_cmd->position.y;

    if (!isPathToPointInSafetyArea2d(from_point, transformed_reference)) {
      ROS_ERROR("[ControlManager]: 'set_reference' service failed, the path is going outside the safety area!");
      res.message = "the path is going outside the safety area";
      res.success = false;
      return true;
    }
  }

  mrs_msgs::ReferenceSrvResponse::ConstPtr tracker_response;
  char                                     message[400];

  // prepare the message for current tracker
  mrs_msgs::ReferenceSrvRequest req_goto_out;
  req_goto_out.reference = transformed_reference.reference;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    tracker_response =
        tracker_list_[active_tracker_idx_]->goTo(mrs_msgs::ReferenceSrvRequest::ConstPtr(std::make_unique<mrs_msgs::ReferenceSrvRequest>(req_goto_out)));

    if (tracker_response != mrs_msgs::ReferenceSrvResponse::Ptr()) {
      res.success = tracker_response->success;
      res.message = tracker_response->message;
    } else {
      sprintf((char*)&message, "The tracker '%s' does not implement 'goto' service!", _tracker_names_[active_tracker_idx_].c_str());
      res.message = message;
      res.success = false;
    }
  }

  return true;
}

//}

/* //{ callbackReferenceTopic() */

void ControlManager::callbackReferenceTopic(const mrs_msgs::ReferenceStampedConstPtr& msg) {

  if (!is_initialized_)
    return;

  if (!callbacks_enabled_) {
    ROS_WARN("[ControlManager]: not passing the goto topic through, the callbacks are disabled");
    return;
  }

  if (!std::isfinite(msg->reference.position.x)) {
    ROS_ERROR("[ControlManager]: NaN detected in variable \"msg->reference.position.x\"!!!");
    return;
  }

  if (!std::isfinite(msg->reference.position.y)) {
    ROS_ERROR("[ControlManager]: NaN detected in variable \"msg->reference.position.y\"!!!");
    return;
  }

  if (!std::isfinite(msg->reference.position.z)) {
    ROS_ERROR("[ControlManager]: NaN detected in variable \"msg->reference.position.z\"!!!");
    return;
  }

  if (!std::isfinite(msg->reference.yaw)) {
    ROS_ERROR("[ControlManager]: NaN detected in variable \"msg->reference.yaw\"!!!");
    return;
  }

  // copy member variables
  auto uav_state         = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_position_cmd = mrs_lib::get_mutexed(mutex_last_position_cmd_, last_position_cmd_);

  // copy the original message so we can modify it

  auto ret = transformer_->transformSingle(uav_state.header.frame_id, *msg);

  if (!ret) {

    ROS_WARN("[ControlManager]: the reference could not be transformed.");
    return;
  }

  mrs_msgs::ReferenceStamped transformed_reference = ret.value();

  if (!bumperValidatePoint(transformed_reference)) {
    ROS_ERROR("[ControlManager]: 'goto' topic failed, potential collision with an obstacle!");
    return;
  }

  if (!isPointInSafetyArea3d(transformed_reference)) {
    ROS_ERROR("[ControlManager]: 'goto' topic failed, the point is outside of the safety area!");
    return;
  }

  if (last_position_cmd != mrs_msgs::PositionCommand::Ptr()) {

    mrs_msgs::ReferenceStamped current_coord;
    current_coord.header.frame_id      = uav_state.header.frame_id;
    current_coord.reference.position.x = last_position_cmd->position.x;
    current_coord.reference.position.y = last_position_cmd->position.y;

    if (!isPathToPointInSafetyArea2d(current_coord, transformed_reference)) {
      ROS_ERROR("[ControlManager]: 'goto' topic failed, the path is going outside the safety area!");
      return;
    }
  }

  bool tracker_response;

  mrs_msgs::Reference reference_out = transformed_reference.reference;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    tracker_response = tracker_list_[active_tracker_idx_]->goTo(mrs_msgs::Reference::ConstPtr(std::make_unique<mrs_msgs::Reference>(reference_out)));

    if (!tracker_response) {
      ROS_ERROR("[ControlManager]: The tracker '%s' does not implement 'goto' topic!", _tracker_names_[active_tracker_idx_].c_str());
    }
  }
}

//}

// human callable services

/* //{ callbackGoToService() */

bool ControlManager::callbackGoToService(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {

  // copy member variables
  auto uav_state         = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_position_cmd = mrs_lib::get_mutexed(mutex_last_position_cmd_, last_position_cmd_);

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  if (!callbacks_enabled_) {
    ROS_WARN("[ControlManager]: not passing the goto service through, the callbacks are disabled");
    res.message = "callbacks are disabled";
    res.success = false;
    return true;
  }

  mrs_msgs::Vec4::Request request_in = req;

  // check number validity
  for (int i = 0; i < 4; i++) {
    if (!std::isfinite(request_in.goal[i])) {
      ROS_ERROR("[ControlManager]: NaN detected in variable \"request_in.goal[%d]\"!!!", i);
      res.message = "NaNs/infs in the goal!";
      res.success = false;
      return true;
    }
  }

  mrs_msgs::ReferenceStamped des_reference;
  des_reference.header.frame_id      = "";
  des_reference.reference.position.x = req.goal[REF_X];
  des_reference.reference.position.y = req.goal[REF_Y];
  des_reference.reference.position.z = req.goal[REF_Z];
  des_reference.reference.yaw        = req.goal[REF_YAW];

  // check the obstacle bumper
  if (!bumperValidatePoint(des_reference)) {
    ROS_ERROR("[ControlManager]: 'goto' service failed, potential collision with an obstacle!");
    res.message = "potential collision with an obstacle";
    res.success = false;
    return true;
  }

  if (!isPointInSafetyArea3d(des_reference)) {
    ROS_ERROR("[ControlManager]: 'goto' service failed, the point is outside of the safety area!");
    res.message = "the point is outside of the safety area";
    res.success = false;
    return true;
  }

  if (last_position_cmd != mrs_msgs::PositionCommand::Ptr()) {

    mrs_msgs::ReferenceStamped current_coord;
    current_coord.header.frame_id      = uav_state.header.frame_id;
    current_coord.reference.position.x = last_position_cmd->position.x;
    current_coord.reference.position.y = last_position_cmd->position.y;
    current_coord.reference.position.z = last_position_cmd->position.z;

    if (!isPathToPointInSafetyArea2d(current_coord, des_reference)) {
      ROS_ERROR("[ControlManager]: 'goto' service failed, the path is going outside the safety area!");
      res.message = "the path is going outside the safety area";
      res.success = false;
      return true;
    }
  }

  mrs_msgs::ReferenceSrvResponse::ConstPtr tracker_response;
  char                                     message[400];

  // prepare the message for current tracker
  mrs_msgs::ReferenceSrvRequest req_goto_out;
  req_goto_out.reference = des_reference.reference;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    tracker_response =
        tracker_list_[active_tracker_idx_]->goTo(mrs_msgs::ReferenceSrvRequest::ConstPtr(std::make_unique<mrs_msgs::ReferenceSrvRequest>(req_goto_out)));

    if (tracker_response != mrs_msgs::ReferenceSrvResponse::Ptr()) {
      res.success = tracker_response->success;
      res.message = tracker_response->message;
    } else {
      sprintf((char*)&message, "The tracker '%s' does not implement 'goto' service!", _tracker_names_[active_tracker_idx_].c_str());
      res.message = message;
      res.success = false;
    }
  }

  return true;
}

//}

/* //{ callbackGoToFcuService() */

bool ControlManager::callbackGoToFcuService(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {

  // copy member variables
  auto uav_state         = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_position_cmd = mrs_lib::get_mutexed(mutex_last_position_cmd_, last_position_cmd_);

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  if (!callbacks_enabled_) {
    ROS_WARN("[ControlManager]: not passing the goto service through, the callbacks are disabled");
    res.message = "callbacks are disabled";
    res.success = false;
    return true;
  }

  mrs_msgs::Vec4::Request request_in = req;

  // check number validity
  for (int i = 0; i < 4; i++) {
    if (!std::isfinite(request_in.goal[i])) {
      ROS_ERROR("[ControlManager]: NaN detected in variable \"request_in.goal[%d]\"!!!", i);
      res.message = "NaNs/infs in the goal!";
      res.success = false;
      return true;
    }
  }

  mrs_msgs::ReferenceStamped des_reference;
  des_reference.header.frame_id      = "fcu_untilted";
  des_reference.reference.position.x = req.goal[REF_X];
  des_reference.reference.position.y = req.goal[REF_Y];
  des_reference.reference.position.z = req.goal[REF_Z];
  des_reference.reference.yaw        = req.goal[REF_YAW];

  // check the obstacle bumper
  if (!bumperValidatePoint(des_reference)) {
    ROS_ERROR("[ControlManager]: 'goto_fcu' service failed, potential collision with an obstacle!");
    res.message = "potential collision with an obstacle";
    res.success = false;
    return true;
  }

  auto ret = transformer_->transformSingle(uav_state.header.frame_id, des_reference);

  if (!ret) {

    ROS_WARN("[ControlManager]: the reference could not be transformed.");
    res.message = "the reference could not be transformed";
    res.success = false;
    return true;
  }

  mrs_msgs::ReferenceStamped transformed_reference = ret.value();

  // check the safety area
  if (!isPointInSafetyArea3d(transformed_reference)) {
    ROS_ERROR("[ControlManager]: 'goto_fcu' service failed, the point is outside of the safety area!");
    res.message = "the point is outside of the safety area";
    res.success = false;
    return true;
  }

  if (last_position_cmd != mrs_msgs::PositionCommand::Ptr()) {

    mrs_msgs::ReferenceStamped current_coord;
    current_coord.header.frame_id      = uav_state.header.frame_id;
    current_coord.reference.position.x = last_position_cmd->position.x;
    current_coord.reference.position.y = last_position_cmd->position.y;

    if (!isPathToPointInSafetyArea2d(current_coord, transformed_reference)) {
      ROS_ERROR("[ControlManager]: 'goto_fcu' service failed, the path is going outside the safety area!");
      res.message = "the path is going outside the safety area";
      res.success = false;
      return true;
    }
  }

  mrs_msgs::ReferenceSrvResponse::ConstPtr tracker_response;
  char                                     message[400];

  mrs_msgs::ReferenceSrvRequest request_out;
  request_out.reference = transformed_reference.reference;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    tracker_response =
        tracker_list_[active_tracker_idx_]->goTo(mrs_msgs::ReferenceSrvRequest::ConstPtr(std::make_unique<mrs_msgs::ReferenceSrvRequest>(request_out)));

    if (tracker_response != mrs_msgs::ReferenceSrvResponse::Ptr()) {
      res.success = tracker_response->success;
      res.message = tracker_response->message;
    } else {
      sprintf((char*)&message, "The tracker '%s' does not implement 'goto' service!", _tracker_names_[active_tracker_idx_].c_str());
      res.message = message;
      res.success = false;
    }
  }

  return true;
}

//}

/* //{ callbackGoToRelativeService() */

bool ControlManager::callbackGoToRelativeService(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {

  // copy member variables
  auto uav_state         = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_position_cmd = mrs_lib::get_mutexed(mutex_last_position_cmd_, last_position_cmd_);

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  if (!callbacks_enabled_) {
    ROS_WARN("[ControlManager]: not passing the goto_relative service through, the callbacks are disabled");
    res.message = "callbacks are disabled";
    res.success = false;
    return true;
  }

  mrs_msgs::Vec4::Request request_in = req;

  // check number validity
  for (int i = 0; i < 4; i++) {
    if (!std::isfinite(request_in.goal[i])) {
      ROS_ERROR("[ControlManager]: NaN detected in variable \"request_in.goal[%d]\"!!!", i);
      res.message = "NaNs/infs in the goal!";
      res.success = false;
      return true;
    }
  }

  mrs_msgs::ReferenceStamped des_reference;
  des_reference.header.frame_id      = "";
  des_reference.reference.position.x = last_position_cmd->position.x + request_in.goal[REF_X];
  des_reference.reference.position.y = last_position_cmd->position.y + request_in.goal[REF_Y];
  des_reference.reference.position.z = last_position_cmd->position.z + request_in.goal[REF_Z];

  // check the obstacle bumper
  if (!bumperValidatePoint(des_reference)) {
    ROS_ERROR("[ControlManager]: 'goto_relative' service failed, potential collision with an obstacle!");
    res.message = "potential collision with an obstacle";
    res.success = false;
    return true;
  }

  if (last_position_cmd != mrs_msgs::PositionCommand::Ptr()) {

    if (!isPointInSafetyArea3d(des_reference)) {
      ROS_ERROR("[ControlManager]: 'goto_relative' service failed, the point is outside of the safety area!");
      res.message = "the point is outside of the safety area";
      res.success = false;
      return true;
    }

    mrs_msgs::ReferenceStamped current_coord;
    current_coord.header.frame_id      = uav_state.header.frame_id;
    current_coord.reference.position.x = last_position_cmd->position.x;
    current_coord.reference.position.y = last_position_cmd->position.y;

    if (!isPathToPointInSafetyArea2d(current_coord, des_reference)) {
      ROS_ERROR("[ControlManager]: 'goto_relative' service failed, the path is going outside the safety area!");
      res.message = "the path is going outside the safety area";
      res.success = false;
      return true;
    }

  } else {

    ROS_ERROR("[ControlManager]: 'goto_relative' service failed, last_position_cmd is not valid!");
    res.message = "last_position_cmd is not valid";
    res.success = false;
    return true;
  }

  mrs_msgs::ReferenceSrvResponse::ConstPtr tracker_response;
  char                                     message[400];

  // prepare the message for current tracker
  mrs_msgs::ReferenceSrvRequest req_goto_out;
  req_goto_out.reference.position.x = request_in.goal[REF_X];
  req_goto_out.reference.position.y = request_in.goal[REF_Y];
  req_goto_out.reference.position.z = request_in.goal[REF_Z];
  req_goto_out.reference.yaw        = request_in.goal[REF_YAW];

  {
    std::scoped_lock lock(mutex_tracker_list_);

    tracker_response = tracker_list_[active_tracker_idx_]->goToRelative(
        mrs_msgs::ReferenceSrvRequest::ConstPtr(std::make_unique<mrs_msgs::ReferenceSrvRequest>(req_goto_out)));

    if (tracker_response != mrs_msgs::ReferenceSrvResponse::Ptr()) {
      res.success = tracker_response->success;
      res.message = tracker_response->message;
    } else {
      sprintf((char*)&message, "The tracker '%s' does not implement 'goto_relative' service!", _tracker_names_[active_tracker_idx_].c_str());
      res.message = message;
      res.success = false;
    }
  }

  return true;
}

//}

/* //{ callbackGoToAltitudeService() */

bool ControlManager::callbackGoToAltitudeService(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res) {

  // copy member variables
  auto uav_state         = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_position_cmd = mrs_lib::get_mutexed(mutex_last_position_cmd_, last_position_cmd_);

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  if (!callbacks_enabled_) {
    ROS_WARN("[ControlManager]: not passing the goto_altitude service through, the callbacks are disabled");
    res.message = "callbacks are disabled";
    res.success = false;
    return true;
  }

  // check number validity
  if (!std::isfinite(req.goal)) {
    ROS_ERROR("[ControlManager]: NaN detected in variable \"req.goal\"!!!");
    res.message = "NaNs/infs in the goal!";
    res.success = false;
    return true;
  }

  if (last_position_cmd != mrs_msgs::PositionCommand::Ptr()) {

    mrs_msgs::ReferenceStamped des_reference;
    des_reference.header.frame_id      = uav_state.header.frame_id;
    des_reference.reference.position.x = last_position_cmd->position.x;
    des_reference.reference.position.y = last_position_cmd->position.y;
    des_reference.reference.position.z = req.goal;

    if (!isPointInSafetyArea3d(des_reference)) {
      ROS_ERROR("[ControlManager]: 'goto_altitude' service failed, the point is outside of the safety area!");
      res.message = "the point is outside of the safety area";
      res.success = false;
      return true;
    }

  } else {

    ROS_ERROR("[ControlManager]: 'goto_altitude' service failed, last_position_cmd is not valid!");
    res.message = "last_position_cmd is not valid";
    res.success = false;
    return true;
  }

  mrs_msgs::Float64SrvResponse::ConstPtr tracker_response;
  char                                   message[400];

  // prepare the message for current tracker
  mrs_msgs::Float64SrvRequest req_goto_out;
  req_goto_out.value = req.goal;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    tracker_response =
        tracker_list_[active_tracker_idx_]->goToAltitude(mrs_msgs::Float64SrvRequest::ConstPtr(std::make_unique<mrs_msgs::Float64SrvRequest>(req_goto_out)));

    if (tracker_response != mrs_msgs::Float64SrvResponse::Ptr()) {
      res.success = tracker_response->success;
      res.message = tracker_response->message;
    } else {
      sprintf((char*)&message, "The tracker '%s' does not implement 'goto_altitude' service!", _tracker_names_[active_tracker_idx_].c_str());
      res.message = message;
      res.success = false;
    }
  }

  return true;
}

//}

/* //{ callbackSetYawService() */

bool ControlManager::callbackSetYawService(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res) {

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  if (!callbacks_enabled_) {
    ROS_WARN("[ControlManager]: not passing the set_yaw service through, the callbacks are disabled");
    res.message = "callbacks are disabled";
    res.success = false;
    return true;
  }

  // check number validity
  if (!std::isfinite(req.goal)) {
    ROS_ERROR("[ControlManager]: NaN detected in variable \"req.goal\"!!!");
    res.message = "NaNs/infs in the goal!";
    res.success = false;
    return true;
  }

  mrs_msgs::Float64SrvResponse::ConstPtr tracker_response;
  char                                   message[400];

  // prepare the message for current tracker
  mrs_msgs::Float64SrvRequest req_goto_out;
  req_goto_out.value = req.goal;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    tracker_response =
        tracker_list_[active_tracker_idx_]->setYaw(mrs_msgs::Float64SrvRequest::ConstPtr(std::make_unique<mrs_msgs::Float64SrvRequest>(req_goto_out)));

    if (tracker_response != mrs_msgs::Float64SrvResponse::Ptr()) {
      res.success = tracker_response->success;
      res.message = tracker_response->message;
    } else {
      sprintf((char*)&message, "The tracker '%s' does not implement 'set_yaw' service!", _tracker_names_[active_tracker_idx_].c_str());
      res.message = message;
      res.success = false;
    }
  }

  return true;
}

//}

/* //{ callbackSetYawRelativeService() */

bool ControlManager::callbackSetYawRelativeService(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res) {

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  if (!callbacks_enabled_) {
    ROS_WARN("[ControlManager]: not passing the set_yaw_relative service through, the callbacks are disabled");
    res.message = "callbacks are disabled";
    res.success = false;
    return true;
  }

  // check number validity
  if (!std::isfinite(req.goal)) {
    ROS_ERROR("[ControlManager]: NaN detected in variable \"req.value\"!!!");
    res.message = "NaNs/infs in the goal!";
    res.success = false;
    return true;
  }

  mrs_msgs::Float64SrvResponse::ConstPtr tracker_response;
  char                                   message[400];

  // prepare the message for current tracker
  mrs_msgs::Float64SrvRequest req_goto_out;
  req_goto_out.value = req.goal;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    tracker_response =
        tracker_list_[active_tracker_idx_]->setYawRelative(mrs_msgs::Float64SrvRequest::ConstPtr(std::make_unique<mrs_msgs::Float64SrvRequest>(req_goto_out)));

    if (tracker_response != mrs_msgs::Float64SrvResponse::Ptr()) {
      res.success = tracker_response->success;
      res.message = tracker_response->message;
    } else {
      sprintf((char*)&message, "The tracker '%s' does not implement 'set_yaw_relative' service!", _tracker_names_[active_tracker_idx_].c_str());
      res.message = message;
      res.success = false;
    }
  }

  return true;
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* isOffboard() //{ */

bool ControlManager::isOffboard(void) {

  // copy member variables
  auto mavros_state = mrs_lib::get_mutexed(mutex_mavros_state_, mavros_state_);

  if (got_mavros_state_ && (ros::Time::now() - mavros_state.header.stamp).toSec() < 1.0 && mavros_state.mode.compare(std::string("OFFBOARD")) == STRING_EQUAL) {

    return true;
  }

  return false;
}

//}

/* shutdown() //{ */

void ControlManager::shutdown() {

  // copy member variables
  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  if (_automatic_pc_shutdown_enabled_) {

    double distance_to_origin = sqrt(pow(uav_state.pose.position.x, 2.0) + pow(uav_state.pose.position.y, 2.0));

    if (distance_to_origin > _automatic_pc_shutdown_threshold_) {

      ROS_INFO("[ControlManager]: Calling service for shutdown (DARPA-specific)");

      std_srvs::Trigger shutdown_out;
      service_client_shutdown_.call(shutdown_out);
    }
  }
}

//}

/* setCallbacks() //{ */

void ControlManager::setCallbacks(bool in) {

  callbacks_enabled_ = in;

  std_srvs::SetBoolRequest req_enable_callbacks;
  req_enable_callbacks.data = callbacks_enabled_;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    // set callbacks to all trackers
    for (unsigned int i = 0; i < tracker_list_.size(); i++) {
      tracker_list_[i]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
    }
  }
}

//}

/* publishDiagnostics() //{ */

void ControlManager::publishDiagnostics(void) {

  std::scoped_lock lock(mutex_diagnostics_);

  mrs_msgs::ControlManagerDiagnostics diagnostics_msg;

  diagnostics_msg.stamp = ros::Time::now();

  // | ----------------- fill the tracker status ---------------- |

  {
    std::scoped_lock lock(mutex_tracker_list_);

    mrs_msgs::TrackerStatus tracker_status;

    tracker_status = tracker_list_[active_tracker_idx_]->getStatus();

    tracker_status.tracker = _tracker_names_[active_tracker_idx_];

    diagnostics_msg.tracker_status = tracker_status;
  }

  // | --------------- fill the controller status --------------- |

  {
    std::scoped_lock lock(mutex_controller_list_);

    mrs_msgs::ControllerStatus controller_status;

    controller_status = controller_list_[active_controller_idx_]->getStatus();

    controller_status.controller = _controller_names_[active_controller_idx_];

    diagnostics_msg.controller_status = controller_status;
  }

  try {
    publisher_diagnostics_.publish(diagnostics_msg);
  }
  catch (...) {
    ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_diagnostics_.getTopic().c_str());
  }
}

//}

/* setConstraints() //{ */

void ControlManager::setConstraints(mrs_msgs::TrackerConstraintsSrvRequest constraints) {

  mrs_msgs::TrackerConstraintsSrvResponse::ConstPtr tracker_response;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    // for each tracker
    for (unsigned int i = 0; i < tracker_list_.size(); i++) {

      // if it is the active one, update and retrieve the command
      tracker_response = tracker_list_[i]->setConstraints(
          mrs_msgs::TrackerConstraintsSrvRequest::ConstPtr(std::make_unique<mrs_msgs::TrackerConstraintsSrvRequest>(constraints)));
    }
  }
}

//}

/* enforceControllerConstraints() //{ */

bool ControlManager::enforceControllersConstraints(mrs_msgs::TrackerConstraintsSrvRequest& constraints) {

  // copy member variables
  auto last_attitude_cmd     = mrs_lib::get_mutexed(mutex_last_attitude_cmd_, last_attitude_cmd_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);

  bool enforcing = false;

  if (last_attitude_cmd != mrs_msgs::AttitudeCommand::Ptr()) {
    if (last_attitude_cmd->controller_enforcing_constraints) {

      std::scoped_lock lock(mutex_tracker_list_);

      // enforce horizontal speed
      if (last_attitude_cmd->horizontal_speed_constraint < constraints.constraints.horizontal_speed) {
        constraints.constraints.horizontal_speed = last_attitude_cmd->horizontal_speed_constraint;

        enforcing = true;
      }

      // enforce horizontal acceleration
      if (last_attitude_cmd->horizontal_acc_constraint < constraints.constraints.horizontal_acceleration) {
        constraints.constraints.horizontal_acceleration = last_attitude_cmd->horizontal_acc_constraint;

        enforcing = true;
      }

      // enforce vertical ascending speed
      if (last_attitude_cmd->vertical_asc_speed_constraint < constraints.constraints.vertical_ascending_speed) {
        constraints.constraints.vertical_ascending_speed = last_attitude_cmd->vertical_asc_speed_constraint;

        enforcing = true;
      }

      // enforce vertical ascending acceleration
      if (last_attitude_cmd->vertical_asc_acc_constraint < constraints.constraints.vertical_ascending_acceleration) {
        constraints.constraints.vertical_ascending_acceleration = last_attitude_cmd->vertical_asc_acc_constraint;

        enforcing = true;
      }

      // enforce vertical descending speed
      if (last_attitude_cmd->vertical_desc_speed_constraint < constraints.constraints.vertical_descending_speed) {
        constraints.constraints.vertical_descending_speed = last_attitude_cmd->vertical_desc_speed_constraint;

        enforcing = true;
      }

      // enforce vertical descending acceleration
      if (last_attitude_cmd->vertical_desc_acc_constraint < constraints.constraints.vertical_descending_acceleration) {
        constraints.constraints.vertical_descending_acceleration = last_attitude_cmd->vertical_desc_acc_constraint;

        enforcing = true;
      }
    }
  }

  if (enforcing) {
    ROS_WARN_THROTTLE(1.0, "[ControlManager]: %s is enforcing constraints over ConstraintManager", _controller_names_[active_controller_idx].c_str());
  }

  return enforcing;
}

//}

// | ----------------------- safety area ---------------------- |

/* //{ isInSafetyArea3d() */

bool ControlManager::isPointInSafetyArea3d(const mrs_msgs::ReferenceStamped point) {

  if (!_use_safety_area_) {
    return true;
  }

  // copy member variables
  auto min_height          = mrs_lib::get_mutexed(mutex_min_height_, min_height_);
  auto max_height_external = mrs_lib::get_mutexed(mutex_max_height_external_, max_height_external_);

  auto ret = transformer_->transformSingle(_safety_area_frame_, point);

  if (!ret) {

    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: SafetyArea: Could not transform reference to the current control frame");

    return false;
  }

  mrs_msgs::ReferenceStamped point_transformed = ret.value();

  // what is lower, the max height from the safety area, or the max height from odometry?
  double max_height = _max_height_ > max_height_external ? max_height_external_ : _max_height_;

  if (safety_zone_->isPointValid(point_transformed.reference.position.x, point_transformed.reference.position.y) &&
      point_transformed.reference.position.z >= min_height && point_transformed.reference.position.z <= max_height) {
    return true;
  }

  return false;
}

//}

/* //{ isInSafetyArea2d() */

bool ControlManager::isPointInSafetyArea2d(const mrs_msgs::ReferenceStamped point) {

  if (!_use_safety_area_) {
    return true;
  }

  auto ret = transformer_->transformSingle(_safety_area_frame_, point);

  if (!ret) {

    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: SafetyArea: Could not transform reference to the current control frame");

    return false;
  }

  mrs_msgs::ReferenceStamped point_transformed = ret.value();

  return safety_zone_->isPointValid(point_transformed.reference.position.x, point_transformed.reference.position.y);
}

//}

/* //{ isPathToPointInSafetyArea2d() */

bool ControlManager::isPathToPointInSafetyArea2d(const mrs_msgs::ReferenceStamped start, const mrs_msgs::ReferenceStamped end) {

  if (!_use_safety_area_) {
    return true;
  }

  mrs_msgs::ReferenceStamped start_transformed, end_transformed;

  {
    auto ret = transformer_->transformSingle(_safety_area_frame_, start);

    if (!ret) {

      ROS_ERROR("[ControlManager]: SafetyArea: Could not transform the first point in the path");

      return false;
    }

    start_transformed = ret.value();
  }

  {
    auto ret = transformer_->transformSingle(_safety_area_frame_, end);

    if (!ret) {

      ROS_ERROR("[ControlManager]: SafetyArea: Could not transform the first point in the path");

      return false;
    }

    end_transformed = ret.value();
  }

  return safety_zone_->isPathValid(start_transformed.reference.position.x, start_transformed.reference.position.y, end_transformed.reference.position.x,
                                   end_transformed.reference.position.y);
}

//}

/* //{ getMaxHeight() */

double ControlManager::getMaxHeight(void) {

  return mrs_lib::get_mutexed(mutex_max_height_external_, max_height_external_);
}

//}

/* //{ getMinHeight() */

double ControlManager::getMinHeight(void) {

  return mrs_lib::get_mutexed(mutex_min_height_, min_height_);
}

//}

// | --------------------- obstacle bumper -------------------- |

/* bumperValidatePoint() //{ */

// everything here happens in FCU
bool ControlManager::bumperValidatePoint(mrs_msgs::ReferenceStamped& point) {

  if (!bumper_enabled_) {
    return true;
  }

  if (!got_bumper_) {
    return true;
  }

  // copy member variables
  auto bumper_data = mrs_lib::get_mutexed(mutex_bumper_data_, bumper_data_);

  if ((ros::Time::now() - bumper_data.header.stamp).toSec() > 1.0) {
    return true;
  }

  auto ret = transformer_->transformSingle("fcu_untilted", point);

  if (!ret) {

    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: Bumper: cannot transform reference to fcu frame");

    return false;
  }

  mrs_msgs::ReferenceStamped point_fcu = ret.value();

  double fcu_x = point_fcu.reference.position.x;
  double fcu_y = point_fcu.reference.position.y;
  double fcu_z = point_fcu.reference.position.z;

  // get the id of the sector, where the reference is
  int horizontal_vector_idx = bumperGetSectorId(fcu_x, fcu_y, fcu_z);
  int vertical_vector_idx   = fcu_z < 0 ? bumper_data.n_horizontal_sectors : bumper_data.n_horizontal_sectors + 1;

  // calculate the horizontal distance to the point
  double horizontal_point_distance = sqrt(pow(fcu_x, 2.0) + pow(fcu_y, 2.0));
  double vertical_point_distance   = fabs(fcu_z);

  // check whether we measure in that direction
  if (bumper_data.sectors[horizontal_vector_idx] == bumper_data.OBSTACLE_NO_DATA) {

    ROS_WARN_THROTTLE(1.0,
                      "[ControlManager]: Bumper: the fcu reference x: %0.2f, y: %0.2f, z: %0.2f (sector %d) is not valid, we do not measure in that direction",
                      fcu_x, fcu_y, fcu_z, horizontal_vector_idx);
    return false;
  }

  if (bumper_data.sectors[horizontal_vector_idx] == bumper_data.OBSTACLE_NOT_DETECTED &&
      bumper_data.sectors[vertical_vector_idx] == bumper_data.OBSTACLE_NOT_DETECTED) {

    return true;
  }

  if (horizontal_point_distance <= (bumper_data.sectors[horizontal_vector_idx] - _bumper_horizontal_distance_) &&
      (fabs(fcu_z) <= 0.1 || vertical_point_distance <= (bumper_data.sectors[vertical_vector_idx] - _bumper_vertical_distance_))) {

    return true;
  }

  // if the obstacle is too close and hugging can't be done, we can't fly, return false
  if (horizontal_point_distance > 0.1 &&
      (bumper_data.sectors[horizontal_vector_idx] > 0 && bumper_data.sectors[horizontal_vector_idx] <= _bumper_horizontal_distance_)) {

    ROS_WARN_THROTTLE(1.0,
                      "[ControlManager]: Bumper: the fcu reference x: %0.2f, y: %0.2f, z: %0.2f (sector %d) is not valid, obstacle is too close (horizontally)",
                      fcu_x, fcu_y, fcu_z, horizontal_vector_idx);

    mrs_msgs::BumperStatus bumper_status;
    bumper_status.modifying_reference = true;
    try {
      publisher_bumper_status_.publish(bumper_status);
    }
    catch (...) {
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: Exception caught during publishing topic %s.", publisher_bumper_status_.getTopic().c_str());
    }

    return false;
  }

  // if the obstacle is too close and hugging can't be done, we can't fly, return false
  if (vertical_point_distance > 0.1 &&
      (bumper_data.sectors[vertical_vector_idx] > 0 && bumper_data.sectors[vertical_vector_idx] <= _bumper_vertical_distance_)) {

    ROS_WARN_THROTTLE(1.0, "[ControlManager]: Bumper: the fcu reference x: %0.2f, y: %0.2f, z: %0.2f is not valid, obstacle is too close (vertically)", fcu_x,
                      fcu_y, fcu_z);

    mrs_msgs::BumperStatus bumper_status;
    bumper_status.modifying_reference = true;
    try {
      publisher_bumper_status_.publish(bumper_status);
    }
    catch (...) {
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: Exception caught during publishing topic %s.", publisher_bumper_status_.getTopic().c_str());
    }

    return false;
  }

  // otherwise, if hugging enabled, fix the coordinates
  if (_bumper_hugging_enabled_) {

    // heading of the point in drone frame
    double point_heading_horizontal = atan2(fcu_y, fcu_x);
    double point_heading_vertical   = fcu_z > 0 ? 1.0 : -1.0;

    double new_x = fcu_x;
    double new_y = fcu_y;
    double new_z = fcu_z;

    if (bumper_data.sectors[horizontal_vector_idx] > 0 &&
        horizontal_point_distance >= (bumper_data.sectors[horizontal_vector_idx] - _bumper_horizontal_distance_)) {

      new_x = cos(point_heading_horizontal) * (bumper_data.sectors[horizontal_vector_idx] - _bumper_horizontal_distance_);
      new_y = sin(point_heading_horizontal) * (bumper_data.sectors[horizontal_vector_idx] - _bumper_horizontal_distance_);

      // horizontal_point_distance                  = uav distance to the reference
      // bumper_data.sectors[horizontal_vector_idx] = bumper data
      // bumper_horizontal_distance                 = the bumper limit

      ROS_WARN_THROTTLE(
          1.0,
          "[ControlManager]: Bumper: the fcu reference [%0.2f, %0.2f] (sector %d) is not valid, distance %0.2f >= (%0.2f - %0.2f)., HUGGING IT it "
          "to x: %0.2f, y: %0.2f",
          fcu_x, fcu_y, horizontal_vector_idx, horizontal_point_distance, bumper_data.sectors[horizontal_vector_idx], _bumper_horizontal_distance_, new_x,
          new_y);

      point_fcu.reference.position.x = new_x;
      point_fcu.reference.position.y = new_y;

      mrs_msgs::BumperStatus bumper_status;
      bumper_status.modifying_reference = true;
      try {
        publisher_bumper_status_.publish(bumper_status);
      }
      catch (...) {
        ROS_ERROR_THROTTLE(1.0, "[ControlManager]: Exception caught during publishing topic %s.", publisher_bumper_status_.getTopic().c_str());
      }
    }

    if (bumper_data.sectors[vertical_vector_idx] > 0 && vertical_point_distance >= (bumper_data.sectors[vertical_vector_idx] - _bumper_vertical_distance_)) {

      new_z = point_heading_vertical * (bumper_data.sectors[vertical_vector_idx] - _bumper_vertical_distance_);

      ROS_WARN_THROTTLE(1.0, "[ControlManager]: Bumper: the fcu reference z: %0.2f is not valid, distance %0.2f > (%0.2f - %0.2f)., HUGGING IT it z: %0.2f",
                        fcu_z, vertical_point_distance, bumper_data.sectors[vertical_vector_idx], _bumper_vertical_distance_, new_z);

      point_fcu.reference.position.z = new_z;

      mrs_msgs::BumperStatus bumper_status;
      bumper_status.modifying_reference = true;
      try {
        publisher_bumper_status_.publish(bumper_status);
      }
      catch (...) {
        ROS_ERROR_THROTTLE(1.0, "[ControlManager]: Exception caught during publishing topic %s.", publisher_bumper_status_.getTopic().c_str());
      }
    }

    // express the point back in the original FRAME
    auto ret = transformer_->transformSingle(point.header.frame_id, point);

    if (!ret) {

      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: Bumper: cannot transform reference back to original frame");

      return false;
    }

    point = ret.value();

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

  if (!got_bumper_) {
    return true;
  }

  // copy member variables
  auto bumper_data = mrs_lib::get_mutexed(mutex_bumper_data_, bumper_data_);
  auto uav_state   = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  double sector_size = TAU / double(bumper_data.n_horizontal_sectors);

  double direction;
  double min_distance                  = 10e9;
  double repulsion_distance            = 10e9;
  double horizontal_collision_detected = false;

  double vertical_collision_detected = false;

  for (int i = 0; i < int(bumper_data.n_horizontal_sectors); i++) {

    if (bumper_data.sectors[i] < 0) {
      continue;
    }

    double wall_locked_horizontal = false;

    // if the sector is under critical distance
    if (bumper_data.sectors[i] <= _bumper_repulsion_horizontal_distance_ && bumper_data.sectors[i] < repulsion_distance) {

      // check for locking between the oposite walls
      // get the desired direction of motion
      double oposite_direction  = double(i) * sector_size + M_PI;
      int    oposite_sector_idx = bumperGetSectorId(cos(oposite_direction), sin(oposite_direction), 0);

      if (bumper_data.sectors[oposite_sector_idx] > 0 && ((bumper_data.sectors[i] + bumper_data.sectors[oposite_sector_idx]) <=
                                                          (2 * _bumper_repulsion_horizontal_distance_ + 2 * _bumper_repulsion_horizontal_offset_))) {

        wall_locked_horizontal = true;

        if (fabs(bumper_data.sectors[i] - bumper_data.sectors[oposite_sector_idx]) <= 2 * _bumper_repulsion_horizontal_offset_) {

          ROS_INFO_THROTTLE(1.0, "[ControlManager]: bumper locked between two walls");
          continue;
        }
      }

      // get the id of the oposite sector
      direction = oposite_direction;

      /* int oposite_sector_idx = (i + bumper_data.n_horizontal_sectors / 2) % bumper_data.n_horizontal_sectors; */

      ROS_WARN_THROTTLE(1.0, "[ControlManager]: found potential collision (sector %d vs. %d), obstacle distance: %0.2f, repulsing", i, oposite_sector_idx,
                        bumper_data.sectors[i]);

      ROS_INFO_THROTTLE(1.0, "[ControlManager]: oposite direction: %0.2f", oposite_direction);

      if (wall_locked_horizontal) {
        if (bumper_data.sectors[i] < bumper_data.sectors[oposite_sector_idx]) {
          repulsion_distance = _bumper_repulsion_horizontal_offset_;
        } else {
          repulsion_distance = -_bumper_repulsion_horizontal_offset_;
        }
      } else {
        repulsion_distance = _bumper_repulsion_horizontal_distance_ + _bumper_repulsion_horizontal_offset_ - bumper_data.sectors[i];
      }

      min_distance = bumper_data.sectors[i];

      repulsing_from_ = i;

      horizontal_collision_detected = true;
    }
  }

  bool   collision_above             = false;
  bool   collision_below             = false;
  bool   wall_locked_vertical        = false;
  double vertical_repulsion_distance = 0;

  // check for vertical collision down
  if (bumper_data.sectors[bumper_data.n_horizontal_sectors] > 0 &&
      bumper_data.sectors[bumper_data.n_horizontal_sectors] <= _bumper_repulsion_vertical_distance_) {

    ROS_INFO_THROTTLE(1.0, "[ControlManager]: bumper: potential collision below");
    collision_above             = true;
    vertical_collision_detected = true;
    vertical_repulsion_distance = _bumper_repulsion_vertical_distance_ - bumper_data.sectors[bumper_data.n_horizontal_sectors];
  }

  // check for vertical collision up
  if (bumper_data.sectors[bumper_data.n_horizontal_sectors + 1] > 0 &&
      bumper_data.sectors[bumper_data.n_horizontal_sectors + 1] <= _bumper_repulsion_vertical_distance_) {

    ROS_INFO_THROTTLE(1.0, "[ControlManager]: bumper: potential collision above");
    collision_below             = true;
    vertical_collision_detected = true;
    vertical_repulsion_distance = -(_bumper_repulsion_vertical_distance_ - bumper_data.sectors[bumper_data.n_horizontal_sectors + 1]);
  }

  // check the up/down wall locking
  if (collision_above && collision_below) {

    if (((bumper_data.sectors[bumper_data.n_horizontal_sectors] + bumper_data.sectors[bumper_data.n_horizontal_sectors + 1]) <=
         (2 * _bumper_repulsion_vertical_distance_ + 2 * _bumper_repulsion_vertical_offset_))) {

      wall_locked_vertical = true;

      vertical_repulsion_distance = (-bumper_data.sectors[bumper_data.n_horizontal_sectors] + bumper_data.sectors[bumper_data.n_horizontal_sectors + 1]) / 2.0;

      if (fabs(bumper_data.sectors[bumper_data.n_horizontal_sectors] - bumper_data.sectors[bumper_data.n_horizontal_sectors + 1]) <=
          2 * _bumper_repulsion_vertical_offset_) {

        ROS_INFO_THROTTLE(1.0, "[ControlManager]: bumper locked between the floor and ceiling");
        vertical_collision_detected = false;
      }
    }
  }

  // if potential collision was detected and we should start the repulsing_
  if (horizontal_collision_detected || vertical_collision_detected) {

    ROS_INFO("[ControlManager]: repulsion was initiated");

    mrs_msgs::BumperStatus bumper_status;
    bumper_status.repulsing = true;
    try {
      publisher_bumper_status_.publish(bumper_status);
    }
    catch (...) {
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: Exception caught during publishing topic %s.", publisher_bumper_status_.getTopic().c_str());
    }

    repulsing_ = true;

    callbacks_enabled_ = false;

    mrs_msgs::ReferenceSrvResponse::ConstPtr tracker_response;

    std_srvs::SetBoolRequest req_enable_callbacks;

    // create the reference in the fcu_untilted frame
    mrs_msgs::ReferenceStamped reference_fcu_untilted;

    reference_fcu_untilted.header.frame_id = "fcu_untilted";

    if (horizontal_collision_detected) {
      reference_fcu_untilted.reference.position.x = cos(direction) * repulsion_distance;
      reference_fcu_untilted.reference.position.y = sin(direction) * repulsion_distance;
    } else {
      reference_fcu_untilted.reference.position.x = 0;
      reference_fcu_untilted.reference.position.y = 0;
    }

    reference_fcu_untilted.reference.yaw = 0;

    if (vertical_collision_detected) {
      reference_fcu_untilted.reference.position.z = vertical_repulsion_distance;
    } else {
      reference_fcu_untilted.reference.position.z = 0;
    }

    {
      std::scoped_lock lock(mutex_tracker_list_);

      // transform the reference into the currently used frame
      // this is under the mutex_tracker_list_ since we don't wont the odometry switch to happen
      // to the tracker before we actually call the goto service

      auto ret = transformer_->transformSingle(uav_state.header.frame_id, reference_fcu_untilted);

      if (!ret) {

        ROS_WARN("[ControlManager]: the reference could not be transformed.");
        return false;
      }

      reference_fcu_untilted = ret.value();

      // copy the reference into the service type message
      mrs_msgs::ReferenceSrvRequest req_goto_out;
      req_goto_out.reference = reference_fcu_untilted.reference;

      // disable callbacks of all trackers
      req_enable_callbacks.data = false;
      for (unsigned int i = 0; i < tracker_list_.size(); i++) {
        tracker_list_[i]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
      }

      // enable the callbacks for the active tracker
      req_enable_callbacks.data = true;
      tracker_list_[active_tracker_idx_]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));

      // call the goto
      tracker_response =
          tracker_list_[active_tracker_idx_]->goTo(mrs_msgs::ReferenceSrvRequest::ConstPtr(std::make_unique<mrs_msgs::ReferenceSrvRequest>(req_goto_out)));

      // disable the callbacks back again
      req_enable_callbacks.data = false;
      tracker_list_[active_tracker_idx_]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));

      // TODO check responses?
    }
  }

  // if repulsing_ and the distance is safe once again
  if ((repulsing_ && !horizontal_collision_detected && !vertical_collision_detected)) {

    ROS_INFO("[ControlManager]: repulsion was stopped");

    std_srvs::SetBoolRequest req_enable_callbacks;

    {
      std::scoped_lock lock(mutex_tracker_list_);

      // enable callbacks of all trackers
      req_enable_callbacks.data = true;
      for (unsigned int i = 0; i < tracker_list_.size(); i++) {
        tracker_list_[i]->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
      }

      // TODO check responses?
    }

    callbacks_enabled_ = true;

    repulsing_ = false;
  }

  return false;
}

//}

/* bumperGetSectorId() //{ */

int ControlManager::bumperGetSectorId(const double x, const double y, [[maybe_unused]] const double z) {

  // copy member variables
  auto bumper_data = mrs_lib::get_mutexed(mutex_bumper_data_, bumper_data_);

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

  // copy member variables
  auto last_attitude_cmd = mrs_lib::get_mutexed(mutex_last_attitude_cmd_, last_attitude_cmd_);

  {
    std::scoped_lock lock(mutex_landing_state_machine_);

    previous_state_landing_ = current_state_landing_;
    current_state_landing_  = new_state;
  }

  switch (current_state_landing_) {

    case IDLE_STATE:
      break;
    case LANDING_STATE: {

      elanding_timer_.start();
      eland_triggered_ = true;

      if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {
        landing_uav_mass_ = _uav_mass_;
      } else {
        landing_uav_mass_ = _uav_mass_ + last_attitude_cmd->mass_difference;
      }
    }

    break;
  }

  ROS_INFO("[ControlManager]: Switching emergency landing state %s -> %s", state_names[previous_state_landing_], state_names[current_state_landing_]);
}

//}

/* //{ changePartialLandingState() */

void ControlManager::changePartialLandingState(LandingStates_t new_state) {

  // copy member variables
  auto last_attitude_cmd = mrs_lib::get_mutexed(mutex_last_attitude_cmd_, last_attitude_cmd_);

  {
    std::scoped_lock lock(mutex_partial_landing_state_machine_);

    previous_state_partial_landing_ = current_state_partial_landing_;
    current_state_partial_landing_  = new_state;
  }

  switch (current_state_partial_landing_) {

    case IDLE_STATE:
      break;

    case LANDING_STATE: {

      partial_landing_timer_.start();
      partial_landing_triggered_ = true;

      if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {
        landing_uav_mass_ = _uav_mass_;
      } else {
        landing_uav_mass_ = _uav_mass_ + last_attitude_cmd->mass_difference;
      }
    }

    break;
  }

  ROS_INFO("[ControlManager]: Triggering partial landing state %s -> %s", state_names[previous_state_partial_landing_],
           state_names[current_state_partial_landing_]);
}

//}

/* hover() //{ */

bool ControlManager::hover(std::string& message_out) {

  if (!is_initialized_) {

    message_out = std::string("ControlManager is not initialized");
    return false;
  }

  {
    std::scoped_lock lock(mutex_tracker_list_);

    std_srvs::TriggerResponse::ConstPtr tracker_response;
    char                                message[400];

    std_srvs::TriggerRequest hover_out;

    tracker_response = tracker_list_[active_tracker_idx_]->hover(std_srvs::TriggerRequest::ConstPtr(std::make_unique<std_srvs::TriggerRequest>(hover_out)));

    if (tracker_response != std_srvs::TriggerResponse::Ptr()) {

      message_out = tracker_response->message;
      return tracker_response->success;

    } else {

      sprintf((char*)&message, "The tracker '%s' does not implement 'goto' service!", _tracker_names_[active_tracker_idx_].c_str());
      message_out = std::string(message);
      return false;
    }
  }
}

//}

/* //{ ehover() */

bool ControlManager::ehover(std::string& message_out) {

  if (!is_initialized_)
    return false;

  // copy the member variables
  auto last_attitude_cmd = mrs_lib::get_mutexed(mutex_last_attitude_cmd_, last_attitude_cmd_);
  auto last_position_cmd = mrs_lib::get_mutexed(mutex_last_position_cmd_, last_position_cmd_);

  char message[400];
  bool success = false;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    try {

      // check if the tracker is not active
      if (_ehover_tracker_idx_ == active_tracker_idx_) {

        sprintf((char*)&message, "Not switching, the tracker %s is already active!", _ehover_tracker_name_.c_str());
        ROS_INFO("[ControlManager]: %s", message);

      } else {

        ROS_INFO("[ControlManager]: Activating tracker %s", _tracker_names_[_ehover_tracker_idx_].c_str());
        tracker_list_[_ehover_tracker_idx_]->activate(last_position_cmd);
        sprintf((char*)&message, "Tracker %s has been activated", _ehover_tracker_name_.c_str());
        ROS_INFO("[ControlManager]: %s", message);

        {
          std::scoped_lock lock(mutex_controller_tracker_switch_time_);

          // update the time (used in failsafe)
          controller_tracker_switch_time_ = ros::Time::now();
        }

        // super important, switch the active tracker idx
        try {

          tracker_list_[active_tracker_idx_]->deactivate();
          active_tracker_idx_ = _ehover_tracker_idx_;

          success = true;
        }
        catch (std::runtime_error& exrun) {

          sprintf((char*)&message, "[ControlManager]: Could not deactivate tracker %s", _tracker_names_[active_tracker_idx_].c_str());
          ROS_ERROR("[ControlManager]: %s", message);

          message_out = std::string(message);
          success     = false;
        }
      }
    }
    catch (std::runtime_error& exrun) {

      sprintf((char*)&message, "[ControlManager]: Error during activation of tracker %s", _ehover_tracker_name_.c_str());
      ROS_ERROR("[ControlManager]: %s", message);
      ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());

      message_out = std::string(message);
      success     = false;
    }
  }

  {
    std::scoped_lock lock(mutex_controller_list_);

    try {

      ROS_INFO("[ControlManager]: Activating controller %s", _controller_names_[_eland_controller_idx_].c_str());

      // check if the controller is not active
      if (_eland_controller_idx_ == active_controller_idx_) {

        sprintf((char*)&message, "Not switching, the controller %s is already active!", _eland_controller_name_.c_str());
        ROS_INFO("[ControlManager]: %s", message);

      } else {

        controller_list_[_eland_controller_idx_]->activate(last_attitude_cmd);
        sprintf((char*)&message, "Controller %s has been activated", _controller_names_[_eland_controller_idx_].c_str());
        ROS_INFO("[ControlManager]: %s", message);

        {
          std::scoped_lock lock(mutex_controller_tracker_switch_time_);

          // update the time (used in failsafe)
          controller_tracker_switch_time_ = ros::Time::now();
        }

        try {

          // deactivate the old controller
          controller_list_[active_controller_idx_]->deactivate();
          active_controller_idx_ = _eland_controller_idx_;  // super important

          success = true;
        }
        catch (std::runtime_error& exrun) {

          sprintf((char*)&message, "[ControlManager]: Could not deactivate controller %s", _tracker_names_[active_tracker_idx_].c_str());
          ROS_ERROR("[ControlManager]: %s", message);

          message_out = std::string(message);
          success     = false;
        }
      }
    }
    catch (std::runtime_error& exrun) {

      sprintf((char*)&message, "[ControlManager]: Error during activation of controller %s", _ehover_tracker_name_.c_str());
      ROS_ERROR("[ControlManager]: %s", message);
      ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());

      message_out = std::string(message);
      success     = false;
    }
  }

  if (success) {
    sprintf((char*)&message, "[ControlManager]: ehover activated.");
    message_out = std::string(message);
  }

  return success;
}

//}

/* eland() //{ */

bool ControlManager::eland(std::string& message_out) {

  if (!is_initialized_)
    return false;

  if (eland_triggered_ || failsafe_triggered_) {
    return false;
  }

  // copy member variables
  auto last_position_cmd = mrs_lib::get_mutexed(mutex_last_position_cmd_, last_position_cmd_);
  auto last_attitude_cmd = mrs_lib::get_mutexed(mutex_last_attitude_cmd_, last_attitude_cmd_);

  char message[400];
  bool success = false;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    try {

      // check if the tracker is not active
      if (_ehover_tracker_idx_ == active_tracker_idx_) {

        sprintf((char*)&message, "Not switching, the tracker %s is already active!", _ehover_tracker_name_.c_str());
        ROS_INFO("[ControlManager]: %s", message);

      } else {

        ROS_INFO("[ControlManager]: Activating tracker %s", _tracker_names_[_ehover_tracker_idx_].c_str());
        tracker_list_[_ehover_tracker_idx_]->activate(last_position_cmd);
        sprintf((char*)&message, "Tracker %s has been activated", _ehover_tracker_name_.c_str());
        ROS_INFO("[ControlManager]: %s", message);

        {
          std::scoped_lock lock(mutex_controller_tracker_switch_time_);

          // update the time (used in failsafe)
          controller_tracker_switch_time_ = ros::Time::now();
        }

        // super important, switch the active tracker idx
        try {

          tracker_list_[active_tracker_idx_]->deactivate();
          active_tracker_idx_ = _ehover_tracker_idx_;

          success = true;
        }
        catch (std::runtime_error& exrun) {

          sprintf((char*)&message, "[ControlManager]: Could not deactivate tracker %s", _tracker_names_[active_tracker_idx_].c_str());
          ROS_ERROR("[ControlManager]: %s", message);

          message_out = std::string(message);
          success     = false;
        }
      }
    }
    catch (std::runtime_error& exrun) {

      sprintf((char*)&message, "[ControlManager]: Error during activation of tracker %s", _ehover_tracker_name_.c_str());
      ROS_ERROR("[ControlManager]: %s", message);
      ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());

      message_out = std::string(message);
      success     = false;
    }
  }

  {
    std::scoped_lock lock(mutex_controller_list_);

    try {

      ROS_INFO("[ControlManager]: Activating controller %s", _controller_names_[_eland_controller_idx_].c_str());

      // check if the controller is not active
      if (_eland_controller_idx_ == active_controller_idx_) {

        sprintf((char*)&message, "Not switching, the controller %s is already active!", _eland_controller_name_.c_str());
        ROS_INFO("[ControlManager]: %s", message);

      } else {

        controller_list_[_eland_controller_idx_]->activate(last_attitude_cmd);
        sprintf((char*)&message, "Controller %s has been activated", _controller_names_[_eland_controller_idx_].c_str());
        ROS_INFO("[ControlManager]: %s", message);

        {
          std::scoped_lock lock(mutex_controller_tracker_switch_time_);

          // update the time (used in failsafe)
          controller_tracker_switch_time_ = ros::Time::now();
        }

        try {

          controller_list_[active_controller_idx_]->deactivate();
          active_controller_idx_ = _eland_controller_idx_;  // super important

          success = true;
        }
        catch (std::runtime_error& exrun) {

          sprintf((char*)&message, "[ControlManager]: Could not deactivate controller %s", _controller_names_[active_controller_idx_].c_str());
          ROS_ERROR("[ControlManager]: %s", message);

          message_out = std::string(message);
          success     = false;
        }
      }
    }
    catch (std::runtime_error& exrun) {

      sprintf((char*)&message, "[ControlManager]: Error during activation of controller %s", _ehover_tracker_name_.c_str());
      ROS_ERROR("[ControlManager]: %s", message);
      ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());

      message_out = std::string(message);
      success     = false;
    }
  }

  std_srvs::Trigger eland_out;
  service_client_eland_.call(eland_out);

  if (eland_out.response.success) {

    changeLandingState(LANDING_STATE);
    changePartialLandingState(IDLE_STATE);
    partial_landing_timer_.stop();

    setOdometryCallbacks(false);

    sprintf((char*)&message, "[ControlManager]: eland activated.");
    message_out = std::string(message);

  } else {

    sprintf((char*)&message, "[ControlManager]: Error during activation of eland: %s", eland_out.response.message.c_str());
    ROS_ERROR("[ControlManager]: %s", message);

    message_out = std::string(message);
    success     = false;
  }

  return success;
}

//}

/* partialLanding() //{ */

bool ControlManager::partialLanding(std::string& message_out) {

  // copy member variables
  auto last_attitude_cmd  = mrs_lib::get_mutexed(mutex_last_attitude_cmd_, last_attitude_cmd_);
  auto last_position_cmd  = mrs_lib::get_mutexed(mutex_last_position_cmd_, last_position_cmd_);
  auto active_tracker_idx = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);

  if (!is_initialized_)
    return false;

  if (eland_triggered_ || failsafe_triggered_ || partial_landing_triggered_) {
    message_out = "Cannot trigger partial land: eland, failsafe or partial landing is already triggered.";
    return false;
  }

  if (!_partial_landing_enabled_) {
    message_out = "Partial landing not enabled.";
    return false;
  }

  char message[400];
  bool success = false;

  partial_landing_previous_tracker_idx_ = active_tracker_idx;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    try {

      // check if the tracker is not active
      if (_ehover_tracker_idx_ == active_tracker_idx_) {

        sprintf((char*)&message, "Not switching, the tracker %s is already active!", _ehover_tracker_name_.c_str());
        ROS_INFO("[ControlManager]: %s", message);

      } else {

        ROS_INFO("[ControlManager]: Activating tracker %s", _tracker_names_[_ehover_tracker_idx_].c_str());
        tracker_list_[_ehover_tracker_idx_]->activate(last_position_cmd);
        sprintf((char*)&message, "Tracker %s has been activated", _ehover_tracker_name_.c_str());
        ROS_INFO("[ControlManager]: %s", message);

        {
          std::scoped_lock lock(mutex_controller_tracker_switch_time_);

          // update the time (used in failsafe)
          controller_tracker_switch_time_ = ros::Time::now();
        }

        // super important, switch the active tracker idx
        try {

          tracker_list_[active_tracker_idx_]->deactivate();
          active_tracker_idx_ = _ehover_tracker_idx_;

          success = true;
        }
        catch (std::runtime_error& exrun) {

          sprintf((char*)&message, "[ControlManager]: Could not deactivate tracker %s", _tracker_names_[active_tracker_idx_].c_str());
          ROS_ERROR("[ControlManager]: %s", message);

          message_out = std::string(message);
          success     = false;
        }
      }
    }
    catch (std::runtime_error& exrun) {

      sprintf((char*)&message, "[ControlManager]: Error during activation of tracker %s", _ehover_tracker_name_.c_str());
      ROS_ERROR("[ControlManager]: %s", message);
      ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());

      message_out = std::string(message);
      success     = false;
    }
  }

  std_srvs::Trigger land_out;
  service_client_land_.call(land_out);

  if (land_out.response.success) {

    changePartialLandingState(LANDING_STATE);
    sprintf((char*)&message, "[ControlManager]: partial landing activated.");
    message_out = std::string(message);

  } else {

    sprintf((char*)&message, "[ControlManager]: Error during activation of land: %s", land_out.response.message.c_str());
    ROS_ERROR("[ControlManager]: %s", message);

    message_out = std::string(message);
    success     = false;
  }

  return success;
}

//}

/* failsafe() //{ */

bool ControlManager::failsafe() {

  // copy member variables
  auto last_attitude_cmd     = mrs_lib::get_mutexed(mutex_last_attitude_cmd_, last_attitude_cmd_);
  auto last_position_cmd     = mrs_lib::get_mutexed(mutex_last_position_cmd_, last_position_cmd_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);

  if (!is_initialized_)
    return false;

  if (failsafe_triggered_) {
    return false;
  }

  if (_failsafe_controller_idx_ != active_controller_idx) {

    {
      // TODO: I dont like this locking here, push it downstream pls
      std::scoped_lock lock(mutex_controller_list_);

      try {

        ROS_INFO("[ControlManager]: Activating controller %s", _failsafe_controller_name_.c_str());
        controller_list_[_failsafe_controller_idx_]->activate(last_attitude_cmd);

        {
          std::scoped_lock lock(mutex_controller_tracker_switch_time_);

          // update the time (used in failsafe)
          controller_tracker_switch_time_ = ros::Time::now();
        }

        failsafe_triggered_ = true;
        elanding_timer_.stop();

        if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {
          landing_uav_mass_ = _uav_mass_;
        } else {
          landing_uav_mass_ = _uav_mass_ + last_attitude_cmd->mass_difference;
        }

        eland_triggered_ = false;
        failsafe_timer_.start();

        setOdometryCallbacks(false);

        ROS_INFO("[ControlManager]: Controller %s has been activated", _failsafe_controller_name_.c_str());

        // super important, switch the active controller idx
        try {

          controller_list_[active_controller_idx_]->deactivate();
          active_controller_idx_ = _failsafe_controller_idx_;
        }
        catch (std::runtime_error& exrun) {
          ROS_ERROR("[ControlManager]: Could not deactivate controller %s", _controller_names_[active_controller_idx_].c_str());
        }
      }
      catch (std::runtime_error& exrun) {
        ROS_ERROR("[ControlManager]: Error during activation of controller %s", _failsafe_controller_name_.c_str());
        ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
      }
    }
  }

  return true;
}

//}

/* escalatingFailsafe() //{ */

bool ControlManager::escalatingFailsafe(std::string& message_out) {

  if ((ros::Time::now() - escalating_failsafe_time_).toSec() < _escalating_failsafe_timeout_) {

    message_out = "too soon for escalating failsafe";
    ROS_WARN_THROTTLE(0.1, "[ControlManager]: %s", message_out.c_str());
    return false;
  }

  ROS_WARN("[ControlManager]: escalating failsafe triggered");

  if (!eland_triggered_ && !failsafe_triggered_ && motors_) {

    escalating_failsafe_time_ = ros::Time::now();
    ROS_WARN_THROTTLE(0.1, "[ControlManager]: escalating failsafe calls for eland");
    return eland(message_out);

  } else if (eland_triggered_) {

    message_out = "escalating failsafe escalates to failsafe";
    ROS_WARN_THROTTLE(0.1, "[ControlManager]: %s", message_out.c_str());
    escalating_failsafe_time_ = ros::Time::now();
    return failsafe();

  } else if (failsafe_triggered_) {

    escalating_failsafe_time_ = ros::Time::now();
    message_out               = "escalating failsafe escalates to disarm";
    ROS_WARN_THROTTLE(0.1, "[ControlManager]: %s", message_out.c_str());
    return arming(false);
  }

  return false;
}

//}

// | ------------------------ routines ------------------------ |

/* arming() //{ */

bool ControlManager::arming(bool input) {

  mavros_msgs::CommandBool srv_out;
  srv_out.request.value = input;

  // if disarming, switch motors_ off
  if (!input) {
    switchMotors(false);
  }

  failsafe_timer_.stop();
  elanding_timer_.stop();

  // we cannot disarm if the drone is not in offboard mode
  if (isOffboard()) {

    ROS_INFO("[ControlManager]: calling for disarming");

    if (service_client_arm_.call(srv_out)) {

      if (srv_out.response.success) {
        ROS_INFO("[ControlManager]: service call for disarm was successful");
      } else {
        ROS_ERROR("[ControlManager]: service call for disarm was unsuccessful");
      }

    } else {
      ROS_ERROR("[ControlManager]: calling for disarm resulted in failure: %d", srv_out.response.result);
    }

    shutdown();

    return srv_out.response.success;

  } else {

    ROS_WARN("[ControlManager]: cannot disarm, not in OFFBOARD mode.");
    return false;
  }
}

//}

/* switchMotors() //{ */

void ControlManager::switchMotors(bool input) {

  ROS_INFO("[ControlManager]: switching motors %s", input ? "ON" : "OFF");

  motors_ = input;

  // if switching motors off, switch to NullTracker
  if (!motors_) {

    ROS_INFO("[ControlManager]: switching to NullTracker after switching motors off");

    switchTracker(_null_tracker_name_);

    ROS_INFO_STREAM("[ControlManager]: switching to " << _eland_controller_name_ << " after switching motors off");

    switchController(_eland_controller_name_);
  }
}

//}

/* setOdometryCallbacks() //{ */

void ControlManager::setOdometryCallbacks(const bool input) {

  ROS_INFO("[ControlManager]: switching odometry callabcks to %s", input ? "ON" : "OFF");

  std_srvs::SetBool srv;

  srv.request.data = input;

  bool res = service_client_set_odometry_callbacks_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN("[ControlManager]: service call for toggle odometry callbacks returned: %s.", srv.response.message.c_str());
    }

  } else {
    ROS_ERROR("[ControlManager]: service call for toggle odometry callbacks failed!");
  }
}

//}

/* switchTracker() //{ */

std::tuple<bool, std::string> ControlManager::switchTracker(const std::string tracker_name) {

  // copy member variables
  auto last_attitude_cmd  = mrs_lib::get_mutexed(mutex_last_attitude_cmd_, last_attitude_cmd_);
  auto last_position_cmd  = mrs_lib::get_mutexed(mutex_last_position_cmd_, last_position_cmd_);
  auto active_tracker_idx = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);

  std::stringstream message;

  if (!got_uav_state_) {

    message << "Can't switch tracker, missing odometry!";
    ROS_ERROR_STREAM("[ControlManager]: " << message.str());
    return std::tuple(false, message.str());
  }

  if (!got_odometry_innovation_) {

    message << "Can't switch tracker, missing odometry innovation!";
    ROS_ERROR_STREAM("[ControlManager]: " << message.str());
    return std::tuple(false, message.str());
  }

  if (!got_pixhawk_odometry_) {

    message << "Can't switch tracker, missing PixHawk odometry!";
    ROS_ERROR_STREAM("[ControlManager]: " << message.str());
    return std::tuple(false, message.str());
  }

  int new_tracker_idx = -1;

  for (unsigned int i = 0; i < _tracker_names_.size(); i++) {
    if (tracker_name == _tracker_names_[i]) {
      new_tracker_idx = i;
    }
  }

  // check if the tracker exists
  if (new_tracker_idx < 0) {

    message << "The tracker " << tracker_name << " does not exist!";
    ROS_ERROR_STREAM("[ControlManager]: " << message.str());
    return std::tuple(false, message.str());
  }

  // check if the tracker is already active
  if (new_tracker_idx == active_tracker_idx) {

    message << "Not switching, the tracker " << tracker_name << " is already active!";
    ROS_INFO_STREAM("[ControlManager]: " << message.str());
    return std::tuple(true, message.str());
  }

  {
    std::scoped_lock lock(mutex_tracker_list_);

    try {

      ROS_INFO("[ControlManager]: Activating tracker %s", _tracker_names_[new_tracker_idx].c_str());

      if (!tracker_list_[new_tracker_idx]->activate(last_position_cmd)) {

        message << "Tracker " << tracker_name << " was not activated";
        ROS_ERROR_STREAM("[ControlManager]: " << message.str());
        return std::tuple(false, message.str());

      } else {

        message << "Tracker " << tracker_name << " was activated";
        ROS_INFO_STREAM("[ControlManager]: " << message.str());

        {
          std::scoped_lock lock(mutex_controller_tracker_switch_time_);

          // update the time (used in failsafe)
          controller_tracker_switch_time_ = ros::Time::now();
        }

        // super important, switch the active tracker idx
        try {

          ROS_INFO("[ControlManager]: deactivating %s", _tracker_names_[active_tracker_idx_].c_str());
          tracker_list_[active_tracker_idx_]->deactivate();

          // if switching from null tracker, activate the active the controller
          if (_tracker_names_[active_tracker_idx_].compare(_null_tracker_name_) == 0) {

            ROS_INFO("[ControlManager]: reactivating %s due to switching from NullTracker", _controller_names_[active_controller_idx_].c_str());
            {
              std::scoped_lock lock(mutex_controller_list_);

              mrs_msgs::AttitudeCommand::Ptr output_command(std::make_unique<mrs_msgs::AttitudeCommand>());

              output_command->total_mass       = _uav_mass_;
              output_command->mass_difference  = 0.0;
              output_command->disturbance_bx_b = _initial_body_disturbance_x_;
              output_command->disturbance_by_b = _initial_body_disturbance_y_;
              output_command->disturbance_wx_w = 0.0;
              output_command->disturbance_wy_w = 0.0;
              output_command->disturbance_bx_w = 0.0;
              output_command->disturbance_by_w = 0.0;
              output_command->thrust           = _min_thrust_null_tracker_;
              output_command->controller       = "none";

              {
                std::scoped_lock lock(mutex_last_attitude_cmd_);

                last_attitude_cmd_ = output_command;
                last_attitude_cmd  = last_attitude_cmd_;
              }

              controller_list_[active_controller_idx_]->activate(last_attitude_cmd);

              {
                std::scoped_lock lock(mutex_controller_tracker_switch_time_);

                // update the time (used in failsafe)
                controller_tracker_switch_time_ = ros::Time::now();
              }
            }

            // if switching to null tracker, deactivate the active controller
          } else if (_tracker_names_[new_tracker_idx].compare(_null_tracker_name_) == 0) {

            ROS_INFO("[ControlManager]: deactivating %s due to switching to NullTracker", _controller_names_[active_controller_idx_].c_str());
            {
              std::scoped_lock lock(mutex_controller_list_);

              controller_list_[active_controller_idx_]->deactivate();
            }
          }

          active_tracker_idx_ = new_tracker_idx;
        }
        catch (std::runtime_error& exrun) {
          ROS_ERROR("[ControlManager]: Could not deactivate tracker %s", _tracker_names_[active_tracker_idx_].c_str());
        }
      }
    }
    catch (std::runtime_error& exrun) {
      ROS_ERROR("[ControlManager]: Error during activation of tracker %s", tracker_name.c_str());
      ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
    }
  }

  return std::tuple(true, message.str());
}

//}

/* switchController() //{ */

std::tuple<bool, std::string> ControlManager::switchController(const std::string controller_name) {

  // copy member variables
  auto last_attitude_cmd     = mrs_lib::get_mutexed(mutex_last_attitude_cmd_, last_attitude_cmd_);
  auto last_position_cmd     = mrs_lib::get_mutexed(mutex_last_position_cmd_, last_position_cmd_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);

  std::stringstream message;

  if (!got_uav_state_) {

    message << "Can't switch controller, missing odometry!";
    ROS_ERROR_STREAM("[ControlManager]: " << message.str());
    return std::tuple(false, message.str());
  }

  if (!got_odometry_innovation_) {

    message << "Can't switch controller, missing odometry innovation!";
    ROS_ERROR_STREAM("[ControlManager]: " << message.str());
    return std::tuple(false, message.str());
  }

  if (!got_pixhawk_odometry_) {

    message << "Can't switch controller, missing PixHawk odometry!";
    ROS_ERROR_STREAM("[ControlManager]: " << message.str());
    return std::tuple(false, message.str());
  }

  int new_controller_idx = -1;

  for (unsigned int i = 0; i < _controller_names_.size(); i++) {
    if (controller_name == _controller_names_[i]) {
      new_controller_idx = i;
    }
  }

  // check if the controller exists
  if (new_controller_idx < 0) {

    message << "The controller " << controller_name << " does not exist!";
    ROS_ERROR_STREAM("[ControlManager]: " << message.str());
    return std::tuple(false, message.str());
  }

  // check if the controller is not active
  if (new_controller_idx == active_controller_idx) {

    message << "Not switching, the controller " << controller_name << " is already active!";
    ROS_INFO_STREAM("[ControlManager]: " << message.str());
    return std::tuple(true, message.str());
  }

  {
    std::scoped_lock lock(mutex_controller_list_);

    try {

      ROS_INFO("[ControlManager]: Activating controller %s", _controller_names_[new_controller_idx].c_str());
      if (!controller_list_[new_controller_idx]->activate(last_attitude_cmd)) {

        message << "Controller " << controller_name << " was not activated";
        ROS_ERROR_STREAM("[ControlManager]: " << message.str());
        return std::tuple(false, message.str());

      } else {

        message << "Controller " << controller_name << " has been activated";
        ROS_INFO_STREAM("[ControlManager]: " << message.str());

        ROS_INFO("[ControlManager]: triggering hover after switching to %s, re-activating %s.", _controller_names_[new_controller_idx].c_str(),
                 _tracker_names_[active_tracker_idx_].c_str());

        // reactivate the current tracker
        // TODO this is not the most elegant way to handle the tracker after a controller switch
        // but it serves the purpose
        {
          std::scoped_lock lock(mutex_tracker_list_);

          tracker_list_[active_tracker_idx_]->deactivate();
          tracker_list_[active_tracker_idx_]->activate(mrs_msgs::PositionCommand::Ptr());
        }

        {
          std::scoped_lock lock(mutex_controller_tracker_switch_time_);

          // update the time (used in failsafe)
          controller_tracker_switch_time_ = ros::Time::now();
        }

        // super important, switch which the active controller idx
        try {

          controller_list_[active_controller_idx_]->deactivate();
          active_controller_idx_ = new_controller_idx;
        }
        catch (std::runtime_error& exrun) {
          ROS_ERROR("[ControlManager]: Could not deactivate controller %s", _controller_names_[active_controller_idx_].c_str());
        }
      }
    }
    catch (std::runtime_error& exrun) {
      ROS_ERROR("[ControlManager]: Error during activation of controller %s", controller_name.c_str());
      ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
    }
  }

  mrs_msgs::TrackerConstraintsSrvRequest sanitized_constraints;
  {
    std::scoped_lock lock(mutex_constraints_);

    sanitized_constraints_ = current_constraints_;
    sanitized_constraints  = sanitized_constraints_;
  }

  setConstraints(sanitized_constraints);

  return std::tuple(true, message.str());
}

//}

/* updateTrackers() //{ */

void ControlManager::updateTrackers(void) {

  // copy member variables
  auto uav_state          = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_attitude_cmd  = mrs_lib::get_mutexed(mutex_last_attitude_cmd_, last_attitude_cmd_);
  auto active_tracker_idx = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);

  // --------------------------------------------------------------
  // |                     Update the trackers                    |
  // --------------------------------------------------------------

  mrs_msgs::PositionCommand::ConstPtr tracker_output_cmd;
  mrs_msgs::UavState::ConstPtr        uav_state_const_ptr(std::make_unique<mrs_msgs::UavState>(uav_state));

  try {

    // for each tracker
    for (unsigned int i = 0; i < tracker_list_.size(); i++) {

      if ((int)i == active_tracker_idx) {

        std::scoped_lock lock(mutex_tracker_list_);

        // if it is the active one, update and retrieve the command
        tracker_output_cmd = tracker_list_[i]->update(uav_state_const_ptr, last_attitude_cmd);

      } else {

        std::scoped_lock lock(mutex_tracker_list_);

        // if it is not the active one, just update without retrieving the command
        tracker_list_[i]->update(uav_state_const_ptr, last_attitude_cmd);
      }
    }

    if (tracker_output_cmd != mrs_msgs::PositionCommand::Ptr() && validatePositionCommand(tracker_output_cmd)) {

      std::scoped_lock lock(mutex_last_position_cmd_);

      last_position_cmd_ = tracker_output_cmd;

    } else {

      if (active_tracker_idx != _null_tracker_idx_) {

        if (active_tracker_idx == _ehover_tracker_idx_) {

          ROS_ERROR_THROTTLE(1.0, "[ControlManager]: The ehover tracker (%s) returned empty or invalid command!", _tracker_names_[active_tracker_idx].c_str());

          failsafe();

        } else {

          ROS_ERROR_THROTTLE(1.0, "[ControlManager]: The tracker %s returned empty or invalid command!", _tracker_names_[active_tracker_idx].c_str());

          std::string ehover_message;

          [[maybe_unused]] bool ehover_res = ehover(ehover_message);
        }

      } else {

        std::scoped_lock lock(mutex_last_position_cmd_);

        last_position_cmd_ = tracker_output_cmd;
      }
    }
  }
  catch (std::runtime_error& exrun) {
    ROS_INFO("[ControlManager]: Exception while updateing trackers.");
    ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());
  }
}

//}

/* updateControllers() //{ */

void ControlManager::updateControllers(mrs_msgs::UavState uav_state_for_control) {

  // copy member variables
  auto last_position_cmd     = mrs_lib::get_mutexed(mutex_last_position_cmd_, last_position_cmd_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);

  // --------------------------------------------------------------
  // |                   Update the controller                    |
  // --------------------------------------------------------------

  mrs_msgs::UavState::ConstPtr uav_state_const_ptr(std::make_unique<mrs_msgs::UavState>(uav_state_for_control));

  mrs_msgs::AttitudeCommand::ConstPtr controller_output_cmd;

  if (last_position_cmd == mrs_msgs::PositionCommand::Ptr()) {

    mrs_msgs::AttitudeCommand::Ptr output_command(std::make_unique<mrs_msgs::AttitudeCommand>());

    output_command->total_mass      = _uav_mass_;
    output_command->mass_difference = 0.0;

    output_command->disturbance_bx_b = _initial_body_disturbance_x_;
    output_command->disturbance_by_b = _initial_body_disturbance_y_;
    output_command->disturbance_wx_w = 0.0;
    output_command->disturbance_wy_w = 0.0;
    output_command->disturbance_bx_w = 0.0;
    output_command->disturbance_by_w = 0.0;

    output_command->thrust = _min_thrust_null_tracker_;

    output_command->controller = "none";

    {
      std::scoped_lock lock(mutex_last_attitude_cmd_);

      last_attitude_cmd_ = output_command;
    }

  } else {

    try {

      // for each controller
      for (unsigned int i = 0; i < controller_list_.size(); i++) {

        if ((int)i == active_controller_idx) {

          std::scoped_lock lock(mutex_controller_list_);

          // if it is the active one, update and retrieve the command
          controller_output_cmd = controller_list_[active_controller_idx]->update(uav_state_const_ptr, last_position_cmd);

        } else {

          std::scoped_lock lock(mutex_controller_list_);

          // if it is not the active one, just update without retrieving the command
          controller_list_[i]->update(uav_state_const_ptr, last_position_cmd);
        }
      }

      // normally the active controller returns a valid command
      if (controller_output_cmd != mrs_msgs::AttitudeCommand::Ptr() && validateAttitudeCommand(controller_output_cmd)) {

        std::scoped_lock lock(mutex_last_attitude_cmd_);

        last_attitude_cmd_ = controller_output_cmd;

        // but it can return an empty command, due to some critical internal error
        // which means we should trigger the failsafe landing
      } else {

        // only if the controller is still active, trigger failsafe
        // if not active, we don't care, we should not ask the controller for
        // the result anyway -> this could mean a race condition occured
        // like it once happend during landing
        bool controller_status = false;

        {
          std::scoped_lock lock(mutex_controller_list_);

          controller_status = controller_list_[active_controller_idx]->getStatus().active;
        }

        if (controller_status) {

          ROS_ERROR("[ControlManager]: triggering failsafe, the controller returned empty or invalid command");

          failsafe();
        }
      }
    }
    catch (std::runtime_error& exrun) {

      ROS_INFO("[ControlManager]: Exception while updating the active controller.");
      ROS_ERROR("[ControlManager]: Exception: %s", exrun.what());

      ROS_WARN("[ControlManager]: triggering failsafe due to an exception in the controller");

      failsafe();
    }
  }
}

//}

/* publish() //{ */

void ControlManager::publish(void) {

  // copy member variables
  auto last_attitude_cmd     = mrs_lib::get_mutexed(mutex_last_attitude_cmd_, last_attitude_cmd_);
  auto last_position_cmd     = mrs_lib::get_mutexed(mutex_last_position_cmd_, last_position_cmd_);
  auto active_tracker_idx    = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);
  auto uav_state             = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  tf::Quaternion desired_orientation;

  // --------------------------------------------------------------
  // |                  publish the position cmd                  |
  // --------------------------------------------------------------

  if (last_position_cmd != mrs_msgs::PositionCommand::Ptr()) {

    // publish the odom topic (position command for debugging, e.g. rviz)
    nav_msgs::Odometry cmd_odom;

    cmd_odom.header               = last_position_cmd->header;
    cmd_odom.pose.pose.position   = last_position_cmd->position;
    cmd_odom.twist.twist.linear.x = last_position_cmd->velocity.x;
    cmd_odom.twist.twist.linear.y = last_position_cmd->velocity.y;
    cmd_odom.twist.twist.linear.z = last_position_cmd->velocity.z;

    // | -------------- prepared desired orientation -------------- |

    if (last_attitude_cmd != mrs_msgs::AttitudeCommand::Ptr()) {

      // when controlling with quaternion or attitude rates, the quaternion should be filled in
      if (last_attitude_cmd->mode_mask == last_attitude_cmd->MODE_QUATER_ATTITUDE || last_attitude_cmd->mode_mask == last_attitude_cmd->MODE_ATTITUDE_RATE) {

        desired_orientation.setX(last_attitude_cmd->quater_attitude.x);
        desired_orientation.setY(last_attitude_cmd->quater_attitude.y);
        desired_orientation.setZ(last_attitude_cmd->quater_attitude.z);
        desired_orientation.setW(last_attitude_cmd->quater_attitude.w);

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
      publisher_cmd_odom_.publish(cmd_odom);
    }
    catch (...) {
      ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_cmd_odom_.getTopic().c_str());
    }

    // publish the full command structure
    try {
      publisher_position_cmd_.publish(last_position_cmd);  // the last_position_cmd is already a ConstPtr
    }
    catch (...) {
      ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_position_cmd_.getTopic().c_str());
    }
  }

  // --------------------------------------------------------------
  // |                 Publish the control command                |
  // --------------------------------------------------------------

  mavros_msgs::AttitudeTarget attitude_target;
  attitude_target.header.stamp    = ros::Time::now();
  attitude_target.header.frame_id = "base_link";

  bool should_publish = false;

  if (!motors_) {

    ROS_WARN_THROTTLE(1.0, "[ControlManager]: motors are off");

    should_publish = false;

  } else if (active_tracker_idx == _null_tracker_idx_) {

    ROS_WARN_THROTTLE(1.0, "[ControlManager]: NullTracker is active, publishing zeros...");

    // set the quaternion to the current odometry.. better than setting it to something unrelated
    attitude_target.orientation = uav_state.pose.orientation;

    attitude_target.body_rate.x = 0.0;
    attitude_target.body_rate.y = 0.0;
    attitude_target.body_rate.z = 0.0;

    attitude_target.type_mask = attitude_target.IGNORE_ATTITUDE;

    attitude_target.thrust = _min_thrust_null_tracker_;

    should_publish = true;

  } else if (active_tracker_idx != _null_tracker_idx_ && last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN_THROTTLE(1.0, "[ControlManager]: the controller (%s) returned nil command! Not publishing anything...",
                      _controller_names_[active_controller_idx].c_str());

    // set the quaternion to the current odometry.. better than setting it to something unrelated
    attitude_target.orientation = uav_state.pose.orientation;

    attitude_target.body_rate.x = 0.0;
    attitude_target.body_rate.y = 0.0;
    attitude_target.body_rate.z = 0.0;

    attitude_target.type_mask = attitude_target.IGNORE_ATTITUDE;

    attitude_target.thrust = _min_thrust_null_tracker_;

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

      attitude_target.orientation = last_attitude_cmd->quater_attitude;

      attitude_target.body_rate.x = 0.0;
      attitude_target.body_rate.y = 0.0;
      attitude_target.body_rate.z = 0.0;

      attitude_target.type_mask = attitude_target.IGNORE_YAW_RATE | attitude_target.IGNORE_ROLL_RATE | attitude_target.IGNORE_PITCH_RATE;

    } else if (last_attitude_cmd->mode_mask == last_attitude_cmd->MODE_ATTITUDE_RATE) {

      attitude_target.body_rate.x = last_attitude_cmd->attitude_rate.x;
      attitude_target.body_rate.y = last_attitude_cmd->attitude_rate.y;
      attitude_target.body_rate.z = last_attitude_cmd->attitude_rate.z;

      attitude_target.orientation = last_attitude_cmd->quater_attitude;

      attitude_target.type_mask = attitude_target.IGNORE_ATTITUDE;
    }

    should_publish = true;
  } else {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: not publishing a control command");
  }

  if (should_publish) {

    // test the output
    if (!std::isfinite(attitude_target.orientation.x)) {
      ROS_ERROR("[ControlManager]: NaN detected in variable \"attitude_target.orientation.x\"!!!");
      return;
    }
    if (!std::isfinite(attitude_target.orientation.y)) {
      ROS_ERROR("[ControlManager]: NaN detected in variable \"attitude_target.orientation.y\"!!!");
      return;
    }
    if (!std::isfinite(attitude_target.orientation.z)) {
      ROS_ERROR("[ControlManager]: NaN detected in variable \"attitude_target.orientation.z\"!!!");
      return;
    }
    if (!std::isfinite(attitude_target.orientation.w)) {
      ROS_ERROR("[ControlManager]: NaN detected in variable \"attitude_target.orientation.w\"!!!");
      return;
    }
    if (!std::isfinite(attitude_target.thrust)) {
      ROS_ERROR("[ControlManager]: NaN detected in variable \"attitude_target.thrust\"!!!");
      return;
    }

    try {

      publisher_control_output_.publish(attitude_target);
    }
    catch (...) {
      ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_control_output_.getTopic().c_str());
    }
  }

  // | --------- publish the attitude_cmd for debugging --------- |

  if (last_attitude_cmd != mrs_msgs::AttitudeCommand::Ptr()) {
    try {
      publisher_attitude_cmd_.publish(last_attitude_cmd);  // the control command is already a ConstPtr
    }
    catch (...) {
      ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_attitude_cmd_.getTopic().c_str());
    }
  }

  // | ------------ publish the desired thrust force ------------ |

  if (last_attitude_cmd != mrs_msgs::AttitudeCommand::Ptr()) {

    mrs_msgs::Float64Stamped thrust_out;
    thrust_out.header.stamp = ros::Time::now();
    thrust_out.value        = (pow((last_attitude_cmd->thrust - _motor_params_.hover_thrust_b) / _motor_params_.hover_thrust_a, 2) / _g_) * 10.0;

    try {
      publisher_thrust_force_.publish(thrust_out);
    }
    catch (...) {
      ROS_ERROR("[ControlManager]: Exception caught during publishing topic %s.", publisher_thrust_force_.getTopic().c_str());
    }
  }
}

//}

/* sanitizeYaw() //{ */

double ControlManager::sanitizeYaw(const double yaw_in) {

  double yaw_out = yaw_in;

  // if desired yaw_out is grater then 2*M_PI mod it
  if (fabs(yaw_out) > 2 * M_PI) {
    yaw_out = fmod(yaw_out, 2 * M_PI);
  }

  // move it to its place
  if (yaw_out > M_PI) {
    yaw_out -= 2 * M_PI;
  } else if (yaw_out < -M_PI) {
    yaw_out += 2 * M_PI;
  }

  return yaw_out;
}

//}

/* angleDist() //{ */

double ControlManager::angleDist(const double in1, const double in2) {

  double sanitized_difference = fabs(sanitizeYaw(in1) - sanitizeYaw(in2));

  if (sanitized_difference > M_PI) {
    sanitized_difference = 2 * M_PI - sanitized_difference;
  }

  return fabs(sanitized_difference);
}

//}

/* resolveFrameName() //{ */

std::string ControlManager::resolveFrameName(const std::string in) {

  // copy member variables
  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  if (in.compare("") == STRING_EQUAL) {

    return uav_state.header.frame_id;
  }

  size_t found = in.find("/");
  if (found == std::string::npos) {

    return _uav_name_ + "/" + in;
  }

  return in;
}

//}

/* validateTrackerCommand() //{ */

bool ControlManager::validatePositionCommand(const mrs_msgs::PositionCommand::ConstPtr position_command) {

  // check attitude

  if (!std::isfinite(position_command->attitude.x)) {
    ROS_ERROR("NaN detected in variable \"position_command->attitude.x\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->attitude.y)) {
    ROS_ERROR("NaN detected in variable \"position_command->attitude.y\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->attitude.z)) {
    ROS_ERROR("NaN detected in variable \"position_command->attitude.z\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->attitude.w)) {
    ROS_ERROR("NaN detected in variable \"position_command->attitude.w\"!!!");
    return false;
  }

  // check positions

  if (!std::isfinite(position_command->position.x)) {
    ROS_ERROR("NaN detected in variable \"position_command->position.x\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->position.y)) {
    ROS_ERROR("NaN detected in variable \"position_command->position.y\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->position.z)) {
    ROS_ERROR("NaN detected in variable \"position_command->position.z\"!!!");
    return false;
  }

  // check velocities

  if (!std::isfinite(position_command->velocity.x)) {
    ROS_ERROR("NaN detected in variable \"position_command->velocity.x\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->velocity.y)) {
    ROS_ERROR("NaN detected in variable \"position_command->velocity.y\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->velocity.z)) {
    ROS_ERROR("NaN detected in variable \"position_command->velocity.z\"!!!");
    return false;
  }

  // check accelerations

  if (!std::isfinite(position_command->acceleration.x)) {
    ROS_ERROR("NaN detected in variable \"position_command->acceleration.x\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->acceleration.y)) {
    ROS_ERROR("NaN detected in variable \"position_command->acceleration.y\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->acceleration.z)) {
    ROS_ERROR("NaN detected in variable \"position_command->acceleration.z\"!!!");
    return false;
  }

  // check jerk

  if (!std::isfinite(position_command->jerk.x)) {
    ROS_ERROR("NaN detected in variable \"position_command->jerk.x\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->jerk.y)) {
    ROS_ERROR("NaN detected in variable \"position_command->jerk.y\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->jerk.z)) {
    ROS_ERROR("NaN detected in variable \"position_command->jerk.z\"!!!");
    return false;
  }

  // check snap

  if (!std::isfinite(position_command->snap.x)) {
    ROS_ERROR("NaN detected in variable \"position_command->snap.x\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->snap.y)) {
    ROS_ERROR("NaN detected in variable \"position_command->snap.y\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->snap.z)) {
    ROS_ERROR("NaN detected in variable \"position_command->snap.z\"!!!");
    return false;
  }

  // check attitude rate

  if (!std::isfinite(position_command->attitude_rate.x)) {
    ROS_ERROR("NaN detected in variable \"position_command->attitude_rate.x\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->attitude_rate.y)) {
    ROS_ERROR("NaN detected in variable \"position_command->attitude_rate.y\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->attitude_rate.z)) {
    ROS_ERROR("NaN detected in variable \"position_command->attitude_rate.z\"!!!");
    return false;
  }

  // check yaws

  if (!std::isfinite(position_command->yaw)) {
    ROS_ERROR("NaN detected in variable \"position_command->yaw\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->yaw_dot)) {
    ROS_ERROR("NaN detected in variable \"position_command->yaw_dot\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->yaw_ddot)) {
    ROS_ERROR("NaN detected in variable \"position_command->yaw_ddot\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->yaw_dddot)) {
    ROS_ERROR("NaN detected in variable \"position_command->yaw_dddot\"!!!");
    return false;
  }

  if (!std::isfinite(position_command->yaw_ddddot)) {
    ROS_ERROR("NaN detected in variable \"position_command->yaw_ddddot\"!!!");
    return false;
  }

  return true;
}

//}

/* validateAttitudeCommand() //{ */

bool ControlManager::validateAttitudeCommand(const mrs_msgs::AttitudeCommand::ConstPtr attitude_command) {

  // check euler attitude

  if (!std::isfinite(attitude_command->euler_attitude.x)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->euler_attitude.x\"!!!");
    return false;
  }

  if (!std::isfinite(attitude_command->euler_attitude.y)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->euler_attitude.y\"!!!");
    return false;
  }

  if (!std::isfinite(attitude_command->euler_attitude.z)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->euler_attitude.z\"!!!");
    return false;
  }

  // check quater attitude

  if (!std::isfinite(attitude_command->quater_attitude.x)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->quater_attitude.x\"!!!");
    return false;
  }

  if (!std::isfinite(attitude_command->quater_attitude.y)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->quater_attitude.y\"!!!");
    return false;
  }

  if (!std::isfinite(attitude_command->quater_attitude.z)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->quater_attitude.z\"!!!");
    return false;
  }

  if (!std::isfinite(attitude_command->quater_attitude.w)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->quater_attitude.w\"!!!");
    return false;
  }

  // check attitude rate

  if (!std::isfinite(attitude_command->attitude_rate.x)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->attitude_rate.x\"!!!");
    return false;
  }

  if (!std::isfinite(attitude_command->attitude_rate.y)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->attitude_rate.y\"!!!");
    return false;
  }

  if (!std::isfinite(attitude_command->attitude_rate.z)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->attitude_rate.z\"!!!");
    return false;
  }

  // check desired_acceleration

  if (!std::isfinite(attitude_command->desired_acceleration.x)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->desired_acceleration.x\"!!!");
    return false;
  }

  if (!std::isfinite(attitude_command->desired_acceleration.y)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->desired_acceleration.y\"!!!");
    return false;
  }

  if (!std::isfinite(attitude_command->desired_acceleration.z)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->desired_acceleration.z\"!!!");
    return false;
  }

  // check the constraints

  if (!std::isfinite(attitude_command->horizontal_speed_constraint)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->horizontal_speed_constraint\"!!!");
    return false;
  }

  if (!std::isfinite(attitude_command->horizontal_acc_constraint)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->horizontal_acc_constraint\"!!!");
    return false;
  }

  if (!std::isfinite(attitude_command->vertical_asc_speed_constraint)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->vertical_asc_speed_constraint\"!!!");
    return false;
  }

  if (!std::isfinite(attitude_command->vertical_asc_acc_constraint)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->vertical_asc_acc_constraint\"!!!");
    return false;
  }

  if (!std::isfinite(attitude_command->vertical_desc_speed_constraint)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->vertical_desc_speed_constraint\"!!!");
    return false;
  }

  if (!std::isfinite(attitude_command->vertical_desc_acc_constraint)) {
    ROS_ERROR("NaN detected in variable \"attitude_command->vertical_desc_acc_constraint\"!!!");
    return false;
  }

  return true;
}

//}

/* validateOdometry() //{ */

bool ControlManager::validateOdometry(const nav_msgs::OdometryConstPtr odometry) {

  // check position

  if (!std::isfinite(odometry->pose.pose.position.x)) {
    ROS_ERROR("NaN detected in variable \"odometry->pose.pose.position.x\"!!!");
    return false;
  }

  if (!std::isfinite(odometry->pose.pose.position.y)) {
    ROS_ERROR("NaN detected in variable \"odometry->pose.pose.position.y\"!!!");
    return false;
  }

  if (!std::isfinite(odometry->pose.pose.position.z)) {
    ROS_ERROR("NaN detected in variable \"odometry->pose.pose.position.z\"!!!");
    return false;
  }

  // check orientation

  if (!std::isfinite(odometry->pose.pose.orientation.x)) {
    ROS_ERROR("NaN detected in variable \"odometry->pose.pose.orientation.x\"!!!");
    return false;
  }

  if (!std::isfinite(odometry->pose.pose.orientation.y)) {
    ROS_ERROR("NaN detected in variable \"odometry->pose.pose.orientation.y\"!!!");
    return false;
  }

  if (!std::isfinite(odometry->pose.pose.orientation.z)) {
    ROS_ERROR("NaN detected in variable \"odometry->pose.pose.orientation.z\"!!!");
    return false;
  }

  if (!std::isfinite(odometry->pose.pose.orientation.w)) {
    ROS_ERROR("NaN detected in variable \"odometry->pose.pose.orientation.w\"!!!");
    return false;
  }

  // check velocity

  if (!std::isfinite(odometry->twist.twist.linear.x)) {
    ROS_ERROR("NaN detected in variable \"odometry->twist.twist.linear.x\"!!!");
    return false;
  }

  if (!std::isfinite(odometry->twist.twist.linear.y)) {
    ROS_ERROR("NaN detected in variable \"odometry->twist.twist.linear.y\"!!!");
    return false;
  }

  if (!std::isfinite(odometry->twist.twist.linear.z)) {
    ROS_ERROR("NaN detected in variable \"odometry->twist.twist.linear.z\"!!!");
    return false;
  }

  return true;
}

//}

/* validateUavState() //{ */

bool ControlManager::validateUavState(const mrs_msgs::UavStateConstPtr uav_state) {

  // check position

  if (!std::isfinite(uav_state->pose.position.x)) {
    ROS_ERROR("NaN detected in variable \"uav_state->pose.position.x\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->pose.position.y)) {
    ROS_ERROR("NaN detected in variable \"uav_state->pose.position.y\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->pose.position.z)) {
    ROS_ERROR("NaN detected in variable \"uav_state->pose.position.z\"!!!");
    return false;
  }

  // check orientation

  if (!std::isfinite(uav_state->pose.orientation.x)) {
    ROS_ERROR("NaN detected in variable \"uav_state->pose.orientation.x\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->pose.orientation.y)) {
    ROS_ERROR("NaN detected in variable \"uav_state->pose.orientation.y\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->pose.orientation.z)) {
    ROS_ERROR("NaN detected in variable \"uav_state->pose.orientation.z\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->pose.orientation.w)) {
    ROS_ERROR("NaN detected in variable \"uav_state->pose.orientation.w\"!!!");
    return false;
  }

  // check linear velocity

  if (!std::isfinite(uav_state->velocity.linear.x)) {
    ROS_ERROR("NaN detected in variable \"uav_state->velocity.linear.x\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->velocity.linear.y)) {
    ROS_ERROR("NaN detected in variable \"uav_state->velocity.linear.y\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->velocity.linear.z)) {
    ROS_ERROR("NaN detected in variable \"uav_state->velocity.linear.z\"!!!");
    return false;
  }

  // check angular velocity

  if (!std::isfinite(uav_state->velocity.angular.x)) {
    ROS_ERROR("NaN detected in variable \"uav_state->velocity.angular.x\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->velocity.angular.y)) {
    ROS_ERROR("NaN detected in variable \"uav_state->velocity.angular.y\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->velocity.angular.z)) {
    ROS_ERROR("NaN detected in variable \"uav_state->velocity.angular.z\"!!!");
    return false;
  }

  // check linear acceleration

  if (!std::isfinite(uav_state->acceleration.linear.x)) {
    ROS_ERROR("NaN detected in variable \"uav_state->acceleration.linear.x\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->acceleration.linear.y)) {
    ROS_ERROR("NaN detected in variable \"uav_state->acceleration.linear.y\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->acceleration.linear.z)) {
    ROS_ERROR("NaN detected in variable \"uav_state->acceleration.linear.z\"!!!");
    return false;
  }

  // check angular acceleration

  if (!std::isfinite(uav_state->acceleration.angular.x)) {
    ROS_ERROR("NaN detected in variable \"uav_state->acceleration.angular.x\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->acceleration.angular.y)) {
    ROS_ERROR("NaN detected in variable \"uav_state->acceleration.angular.y\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->acceleration.angular.z)) {
    ROS_ERROR("NaN detected in variable \"uav_state->acceleration.angular.z\"!!!");
    return false;
  }

  // check acceleration angular disturbance

  if (!std::isfinite(uav_state->acceleration_disturbance.angular.x)) {
    ROS_ERROR("NaN detected in variable \"uav_state->acceleration_disturbance.angular.x\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->acceleration_disturbance.angular.y)) {
    ROS_ERROR("NaN detected in variable \"uav_state->acceleration_disturbance.angular.y\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->acceleration_disturbance.angular.z)) {
    ROS_ERROR("NaN detected in variable \"uav_state->acceleration_disturbance.angular.z\"!!!");
    return false;
  }

  // check acceleration linear disturbance

  if (!std::isfinite(uav_state->acceleration_disturbance.linear.x)) {
    ROS_ERROR("NaN detected in variable \"uav_state->acceleration_disturbance.linear.x\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->acceleration_disturbance.linear.y)) {
    ROS_ERROR("NaN detected in variable \"uav_state->acceleration_disturbance.linear.y\"!!!");
    return false;
  }

  if (!std::isfinite(uav_state->acceleration_disturbance.linear.z)) {
    ROS_ERROR("NaN detected in variable \"uav_state->acceleration_disturbance.linear.z\"!!!");
    return false;
  }

  return true;
}

//}

/* RCChannelToRange() //{ */

double ControlManager::RCChannelToRange(double rc_value, double range, double deadband) {

  double tmp_0_to_1    = (rc_value - PWM_MIN) / (PWM_RANGE);
  double tmp_neg1_to_1 = (tmp_0_to_1 - 0.5) * 2.0;

  if (tmp_neg1_to_1 > 1.0) {
    tmp_neg1_to_1 = 1.0;
  } else if (tmp_neg1_to_1 < -1.0) {
    tmp_neg1_to_1 = -1.0;
  }

  // check the deadband
  if (tmp_neg1_to_1 < deadband && tmp_neg1_to_1 > -deadband) {
    return 0.0;
  }

  if (tmp_neg1_to_1 > 0) {

    double tmp = (tmp_neg1_to_1 - deadband) / (1.0 - deadband);

    return range * tmp;

  } else {

    double tmp = (-tmp_neg1_to_1 - deadband) / (1.0 - deadband);

    return -range * tmp;
  }

  return 0.0;
}

//}

}  // namespace control_manager

}  // namespace mrs_uav_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_manager::control_manager::ControlManager, nodelet::Nodelet)
