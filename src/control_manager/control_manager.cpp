/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_uav_managers/control_manager/common.h>
#include <control_manager/output_publisher.h>

#include <mrs_uav_managers/controller.h>
#include <mrs_uav_managers/tracker.h>

#include <mrs_msgs/String.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/Float64StampedSrv.h>
#include <mrs_msgs/ObstacleSectors.h>
#include <mrs_msgs/BoolStamped.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/DynamicsConstraints.h>
#include <mrs_msgs/ControlError.h>
#include <mrs_msgs/GetFloat64.h>
#include <mrs_msgs/ValidateReference.h>
#include <mrs_msgs/ValidateReferenceList.h>
#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/EstimatorInput.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>

#include <mrs_lib/safety_zone/safety_zone.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/quadratic_throttle_model.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>

#include <mrs_msgs/HwApiCapabilities.h>
#include <mrs_msgs/HwApiStatus.h>
#include <mrs_msgs/HwApiRcChannels.h>

#include <mrs_msgs/HwApiActuatorCmd.h>
#include <mrs_msgs/HwApiControlGroupCmd.h>
#include <mrs_msgs/HwApiAttitudeRateCmd.h>
#include <mrs_msgs/HwApiAttitudeCmd.h>
#include <mrs_msgs/HwApiAccelerationHdgRateCmd.h>
#include <mrs_msgs/HwApiAccelerationHdgCmd.h>
#include <mrs_msgs/HwApiVelocityHdgRateCmd.h>
#include <mrs_msgs/HwApiVelocityHdgCmd.h>
#include <mrs_msgs/HwApiPositionCmd.h>

#include <std_msgs/Float64.h>

#include <future>

#include <pluginlib/class_loader.h>

#include <nodelet/loader.h>

#include <eigen3/Eigen/Eigen>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <mrs_msgs/Reference.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/ReferenceList.h>
#include <mrs_msgs/TrajectoryReference.h>

#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/ReferenceStampedSrvRequest.h>
#include <mrs_msgs/ReferenceStampedSrvResponse.h>

#include <mrs_msgs/VelocityReferenceStampedSrv.h>
#include <mrs_msgs/VelocityReferenceStampedSrvRequest.h>
#include <mrs_msgs/VelocityReferenceStampedSrvResponse.h>

#include <mrs_msgs/TransformReferenceSrv.h>
#include <mrs_msgs/TransformReferenceSrvRequest.h>
#include <mrs_msgs/TransformReferenceSrvResponse.h>

#include <mrs_msgs/TransformReferenceListSrv.h>
#include <mrs_msgs/TransformReferenceListSrvRequest.h>
#include <mrs_msgs/TransformReferenceListSrvResponse.h>

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

//}

/* defines //{ */

#define TAU 2 * M_PI
#define REF_X 0
#define REF_Y 1
#define REF_Z 2
#define REF_HEADING 3
#define ELAND_STR "eland"
#define EHOVER_STR "ehover"
#define ESCALATING_FAILSAFE_STR "escalating_failsafe"
#define FAILSAFE_STR "failsafe"
#define INPUT_UAV_STATE 0
#define INPUT_ODOMETRY 1
#define RC_DEADBAND 0.2

//}

/* using //{ */

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

//}

namespace mrs_uav_managers
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

const char* state_names[2] = {"IDLING", "LANDING"};

// state machine
typedef enum
{

  FCU_FRAME,
  RELATIVE_FRAME,
  ABSOLUTE_FRAME

} ReferenceFrameType_t;

// state machine
typedef enum
{

  ESC_NONE_STATE     = 0,
  ESC_EHOVER_STATE   = 1,
  ESC_ELAND_STATE    = 2,
  ESC_FAILSAFE_STATE = 3,
  ESC_FINISHED_STATE = 4,

} EscalatingFailsafeStates_t;

/* class ControllerParams() //{ */

class ControllerParams {

public:
  ControllerParams(std::string address, std::string name_space, double eland_threshold, double failsafe_threshold, double odometry_innovation_threshold,
                   bool human_switchable);

public:
  double      failsafe_threshold;
  double      eland_threshold;
  double      odometry_innovation_threshold;
  std::string address;
  std::string name_space;
  bool        human_switchable;
};

ControllerParams::ControllerParams(std::string address, std::string name_space, double eland_threshold, double failsafe_threshold,
                                   double odometry_innovation_threshold, bool human_switchable) {

  this->eland_threshold               = eland_threshold;
  this->odometry_innovation_threshold = odometry_innovation_threshold;
  this->failsafe_threshold            = failsafe_threshold;
  this->address                       = address;
  this->name_space                    = name_space;
  this->human_switchable              = human_switchable;
}

//}

/* class TrackerParams() //{ */

class TrackerParams {

public:
  TrackerParams(std::string address, std::string name_space, bool human_switchable);

public:
  std::string address;
  std::string name_space;
  bool        human_switchable;
};

TrackerParams::TrackerParams(std::string address, std::string name_space, bool human_switchable) {

  this->address          = address;
  this->name_space       = name_space;
  this->human_switchable = human_switchable;
}

//}

class ControlManager : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_ = false;
  std::string       _uav_name_;
  std::string       _body_frame_;

  std::string _custom_config_;
  std::string _platform_config_;
  std::string _world_config_;
  std::string _network_config_;

  // | --------------- dynamic loading of trackers -------------- |

  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_managers::Tracker>> tracker_loader_;  // pluginlib loader of dynamically loaded trackers
  std::vector<std::string>                                           _tracker_names_;  // list of tracker names
  std::map<std::string, TrackerParams>                               trackers_;        // map between tracker names and tracker param
  std::vector<boost::shared_ptr<mrs_uav_managers::Tracker>>          tracker_list_;    // list of trackers, routines are callable from this
  std::mutex                                                         mutex_tracker_list_;

  // | ------------- dynamic loading of controllers ------------- |

  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_managers::Controller>> controller_loader_;  // pluginlib loader of dynamically loaded controllers
  std::vector<std::string>                                              _controller_names_;  // list of controller names
  std::map<std::string, ControllerParams>                               controllers_;        // map between controller names and controller params
  std::vector<boost::shared_ptr<mrs_uav_managers::Controller>>          controller_list_;    // list of controllers, routines are callable from this
  std::mutex                                                            mutex_controller_list_;

  // | ------------------------- HW API ------------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiCapabilities> sh_hw_api_capabilities_;

  OutputPublisher control_output_publisher_;

  ControlOutputModalities_t _hw_api_inputs_;

  double desired_uav_state_rate_ = 100.0;

  // this timer will check till we already got the hardware api diagnostics
  // then it will trigger the initialization of the controllers and finish
  // the initialization of the ControlManager
  ros::Timer timer_hw_api_capabilities_;
  void       timerHwApiCapabilities(const ros::TimerEvent& event);

  void preinitialize(void);
  void initialize(void);

  // | ------------ tracker and controller switching ------------ |

  std::tuple<bool, std::string> switchController(const std::string& controller_name);
  std::tuple<bool, std::string> switchTracker(const std::string& tracker_name);

  // the time of last switching of a tracker or a controller
  ros::Time  controller_tracker_switch_time_;
  std::mutex mutex_controller_tracker_switch_time_;

  // | -------------------- the transformer  -------------------- |

  std::shared_ptr<mrs_lib::Transformer> transformer_;

  // | ------------------- scope timer logger ------------------- |

  bool                                       scope_timer_enabled_ = false;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;

  // | --------------------- general params --------------------- |

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

  // enabling the emergency handoff -> will disable eland and failsafe
  bool _rc_emergency_handoff_ = false;

  // what throttle should be output when null tracker is active?
  double _min_throttle_null_tracker_ = 0.0;

  // rates of all the timers
  double _status_timer_rate_   = 0;
  double _safety_timer_rate_   = 0;
  double _elanding_timer_rate_ = 0;
  double _failsafe_timer_rate_ = 0;
  double _bumper_timer_rate_   = 0;

  bool _snap_trajectory_to_safety_area_ = false;

  // | -------------- uav_state/odometry subscriber ------------- |

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_odometry_;
  mrs_lib::SubscribeHandler<mrs_msgs::UavState> sh_uav_state_;

  mrs_msgs::UavState uav_state_;
  mrs_msgs::UavState previous_uav_state_;
  bool               got_uav_state_               = false;
  double             _uav_state_max_missing_time_ = 0;  // how long should we tolerate missing state estimate?
  double             uav_roll_                    = 0;
  double             uav_pitch_                   = 0;
  double             uav_yaw_                     = 0;
  double             uav_heading_                 = 0;
  std::mutex         mutex_uav_state_;

  // odometry hiccup detection
  double uav_state_avg_dt_        = 1;
  double uav_state_hiccup_factor_ = 1;
  int    uav_state_count_         = 0;

  mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> sh_gnss_;

  // | -------------- safety area max z subscriber -------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped> sh_max_z_;

  // | ------------- odometry innovation subscriber ------------- |

  // odometry innovation is published by the odometry node
  // it is used to issue eland if the estimator's input is too wonky
  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_odometry_innovation_;

  // | --------------------- common handlers -------------------- |

  // contains handlers that are shared with trackers and controllers
  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers_;

  // | --------------- tracker and controller IDs --------------- |

  // keeping track of currently active controllers and trackers
  unsigned int active_tracker_idx_    = 0;
  unsigned int active_controller_idx_ = 0;

  // indeces of some notable trackers
  unsigned int _ehover_tracker_idx_               = 0;
  unsigned int _landoff_tracker_idx_              = 0;
  unsigned int _joystick_tracker_idx_             = 0;
  unsigned int _joystick_controller_idx_          = 0;
  unsigned int _failsafe_controller_idx_          = 0;
  unsigned int _joystick_fallback_controller_idx_ = 0;
  unsigned int _joystick_fallback_tracker_idx_    = 0;
  unsigned int _null_tracker_idx_                 = 0;
  unsigned int _eland_controller_idx_             = 0;

  // | -------------- enabling the output publisher ------------- |

  void              toggleOutput(const bool& input);
  std::atomic<bool> output_enabled_ = false;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mrs_msgs::ControllerDiagnostics>     ph_controller_diagnostics_;
  mrs_lib::PublisherHandler<mrs_msgs::TrackerCommand>            ph_tracker_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::EstimatorInput>            ph_mrs_odom_input_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry>                  ph_control_reference_odom_;
  mrs_lib::PublisherHandler<mrs_msgs::ControlManagerDiagnostics> ph_diagnostics_;
  mrs_lib::PublisherHandler<std_msgs::Empty>                     ph_offboard_on_;
  mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>            ph_tilt_error_;
  mrs_lib::PublisherHandler<std_msgs::Float64>                   ph_mass_estimate_;
  mrs_lib::PublisherHandler<std_msgs::Float64>                   ph_mass_nominal_;
  mrs_lib::PublisherHandler<std_msgs::Float64>                   ph_throttle_;
  mrs_lib::PublisherHandler<std_msgs::Float64>                   ph_thrust_;
  mrs_lib::PublisherHandler<mrs_msgs::ControlError>              ph_control_error_;
  mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>     ph_safety_area_markers_;
  mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>     ph_safety_area_coordinates_markers_;
  mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>     ph_disturbances_markers_;
  mrs_lib::PublisherHandler<mrs_msgs::DynamicsConstraints>       ph_current_constraints_;
  mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>            ph_heading_;
  mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>            ph_speed_;

  // | --------------------- service servers -------------------- |

  ros::ServiceServer service_server_switch_tracker_;
  ros::ServiceServer service_server_switch_controller_;
  ros::ServiceServer service_server_reset_tracker_;
  ros::ServiceServer service_server_hover_;
  ros::ServiceServer service_server_ehover_;
  ros::ServiceServer service_server_failsafe_;
  ros::ServiceServer service_server_failsafe_escalating_;
  ros::ServiceServer service_server_toggle_output_;
  ros::ServiceServer service_server_arm_;
  ros::ServiceServer service_server_enable_callbacks_;
  ros::ServiceServer service_server_set_constraints_;
  ros::ServiceServer service_server_use_joystick_;
  ros::ServiceServer service_server_use_safety_area_;
  ros::ServiceServer service_server_emergency_reference_;
  ros::ServiceServer service_server_pirouette_;
  ros::ServiceServer service_server_eland_;
  ros::ServiceServer service_server_parachute_;
  ros::ServiceServer service_server_set_min_z_;

  // human callbable services for references
  ros::ServiceServer service_server_goto_;
  ros::ServiceServer service_server_goto_fcu_;
  ros::ServiceServer service_server_goto_relative_;
  ros::ServiceServer service_server_goto_altitude_;
  ros::ServiceServer service_server_set_heading_;
  ros::ServiceServer service_server_set_heading_relative_;

  // the reference service and subscriber
  ros::ServiceServer                                    service_server_reference_;
  mrs_lib::SubscribeHandler<mrs_msgs::ReferenceStamped> sh_reference_;

  // the velocity reference service and subscriber
  ros::ServiceServer                                            service_server_velocity_reference_;
  mrs_lib::SubscribeHandler<mrs_msgs::VelocityReferenceStamped> sh_velocity_reference_;

  // trajectory tracking
  ros::ServiceServer                                       service_server_trajectory_reference_;
  mrs_lib::SubscribeHandler<mrs_msgs::TrajectoryReference> sh_trajectory_reference_;
  ros::ServiceServer                                       service_server_start_trajectory_tracking_;
  ros::ServiceServer                                       service_server_stop_trajectory_tracking_;
  ros::ServiceServer                                       service_server_resume_trajectory_tracking_;
  ros::ServiceServer                                       service_server_goto_trajectory_start_;

  // transform service servers
  ros::ServiceServer service_server_transform_reference_;
  ros::ServiceServer service_server_transform_reference_list_;
  ros::ServiceServer service_server_transform_pose_;
  ros::ServiceServer service_server_transform_vector3_;

  // safety area services
  ros::ServiceServer service_server_validate_reference_;
  ros::ServiceServer service_server_validate_reference_2d_;
  ros::ServiceServer service_server_validate_reference_list_;

  // bumper service servers
  ros::ServiceServer service_server_bumper_enabler_;
  ros::ServiceServer service_server_bumper_repulsion_enabler_;

  // service clients
  mrs_lib::ServiceClientHandler<std_srvs::SetBool> sch_arming_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_eland_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_shutdown_;
  mrs_lib::ServiceClientHandler<std_srvs::SetBool> sch_set_odometry_callbacks_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_parachute_;

  // safety area min z servers
  ros::ServiceServer service_server_get_min_z_;

  // | --------- trackers' and controllers' last results -------- |

  // the last result of an active tracker
  std::optional<mrs_msgs::TrackerCommand> last_tracker_cmd_;
  std::mutex                              mutex_last_tracker_cmd_;

  // the last result of an active controller
  Controller::ControlOutput last_control_output_;
  std::mutex                mutex_last_control_output_;

  // | -------------- HW API diagnostics subscriber ------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus> sh_hw_api_status_;

  bool offboard_mode_          = false;
  bool offboard_mode_was_true_ = false;  // if it was ever true
  bool armed_                  = false;

  // | -------------------- throttle and mass ------------------- |

  // throttle mass estimation during eland
  double    throttle_mass_estimate_   = 0;
  bool      throttle_under_threshold_ = false;
  ros::Time throttle_mass_estimate_first_time_;

  // | ---------------------- safety params --------------------- |

  // failsafe when tilt error is too large
  bool   _tilt_error_disarm_enabled_;
  double _tilt_error_disarm_timeout_;
  double _tilt_error_disarm_threshold_;

  ros::Time tilt_error_disarm_time_;
  bool      tilt_error_disarm_over_thr_ = false;

  // elanding when tilt error is too large
  bool   _tilt_limit_eland_enabled_;
  double _tilt_limit_eland_ = 0;  // [rad]

  // disarming when tilt error is too large
  bool   _tilt_limit_disarm_enabled_;
  double _tilt_limit_disarm_ = 0;  // [rad]

  // elanding when yaw error is too large
  bool   _yaw_error_eland_enabled_;
  double _yaw_error_eland_ = 0;  // [rad]

  // keeping track of control errors
  std::optional<double> tilt_error_ = 0;
  std::optional<double> yaw_error_  = 0;
  std::mutex            mutex_attitude_error_;

  std::optional<Eigen::Vector3d> position_error_;
  std::mutex                     mutex_position_error_;

  // control error for triggering failsafe, eland, etc.
  // this filled with the current controllers failsafe threshold
  double _failsafe_threshold_                = 0;  // control error for triggering failsafe
  double _eland_threshold_                   = 0;  // control error for triggering eland
  bool   _odometry_innovation_check_enabled_ = false;
  double _odometry_innovation_threshold_     = 0;  // innovation size for triggering eland

  bool callbacks_enabled_ = true;

  // | ------------------------ parachute ----------------------- |

  bool _parachute_enabled_ = false;

  std::tuple<bool, std::string> deployParachute(void);
  bool                          parachuteSrv(void);

  // | ----------------------- safety area ---------------------- |

  // safety area
  std::unique_ptr<mrs_lib::SafetyZone> safety_zone_;

  std::atomic<bool> use_safety_area_ = false;

  std::string _safety_area_horizontal_frame_;
  std::string _safety_area_vertical_frame_;

  std::atomic<double> _safety_area_min_z_ = 0;

  double _safety_area_max_z_ = 0;

  // safety area routines
  // those are passed to trackers using the common_handlers object
  bool   isPointInSafetyArea2d(const mrs_msgs::ReferenceStamped& point);
  bool   isPointInSafetyArea3d(const mrs_msgs::ReferenceStamped& point);
  bool   isPathToPointInSafetyArea2d(const mrs_msgs::ReferenceStamped& from, const mrs_msgs::ReferenceStamped& to);
  bool   isPathToPointInSafetyArea3d(const mrs_msgs::ReferenceStamped& from, const mrs_msgs::ReferenceStamped& to);
  double getMinZ(const std::string& frame_id);
  double getMaxZ(const std::string& frame_id);

  // | ------------------------ callbacks ----------------------- |

  // topic callbacks
  void callbackOdometry(const nav_msgs::Odometry::ConstPtr msg);
  void callbackUavState(const mrs_msgs::UavState::ConstPtr msg);
  void callbackHwApiStatus(const mrs_msgs::HwApiStatus::ConstPtr msg);
  void callbackGNSS(const sensor_msgs::NavSatFix::ConstPtr msg);
  void callbackRC(const mrs_msgs::HwApiRcChannels::ConstPtr msg);

  // topic timeouts
  void timeoutUavState(const double& missing_for);

  // switching controller and tracker services
  bool callbackSwitchTracker(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);
  bool callbackSwitchController(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);
  bool callbackTrackerResetStatic(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // reference callbacks
  void callbackReferenceTopic(const mrs_msgs::ReferenceStamped::ConstPtr msg);
  void callbackVelocityReferenceTopic(const mrs_msgs::VelocityReferenceStamped::ConstPtr msg);
  void callbackTrajectoryReferenceTopic(const mrs_msgs::TrajectoryReference::ConstPtr msg);
  bool callbackGoto(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res);
  bool callbackGotoFcu(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res);
  bool callbackGotoRelative(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res);
  bool callbackGotoAltitude(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res);
  bool callbackSetHeading(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res);
  bool callbackSetHeadingRelative(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res);
  bool callbackReferenceService(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);
  bool callbackVelocityReferenceService(mrs_msgs::VelocityReferenceStampedSrv::Request& req, mrs_msgs::VelocityReferenceStampedSrv::Response& res);
  bool callbackTrajectoryReferenceService(mrs_msgs::TrajectoryReferenceSrv::Request& req, mrs_msgs::TrajectoryReferenceSrv::Response& res);
  bool callbackEmergencyReference(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);

  // safety callbacks
  bool callbackHover(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackStartTrajectoryTracking([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackStopTrajectoryTracking([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackResumeTrajectoryTracking([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackGotoTrajectoryStart([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackEHover(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackFailsafe(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackFailsafeEscalating(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackEland(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackParachute([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackSetMinZ(mrs_msgs::Float64StampedSrv::Request& req, mrs_msgs::Float64StampedSrv::Response& res);
  bool callbackToggleOutput(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool callbackArm(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool callbackEnableCallbacks(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool callbackEnableBumper(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool callbackUseSafetyArea(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  bool callbackGetMinZ(mrs_msgs::GetFloat64::Request& req, mrs_msgs::GetFloat64::Response& res);

  bool callbackValidateReference(mrs_msgs::ValidateReference::Request& req, mrs_msgs::ValidateReference::Response& res);
  bool callbackValidateReference2d(mrs_msgs::ValidateReference::Request& req, mrs_msgs::ValidateReference::Response& res);
  bool callbackValidateReferenceList(mrs_msgs::ValidateReferenceList::Request& req, mrs_msgs::ValidateReferenceList::Response& res);

  // transformation callbacks
  bool callbackTransformReference(mrs_msgs::TransformReferenceSrv::Request& req, mrs_msgs::TransformReferenceSrv::Response& res);
  bool callbackTransformReferenceList(mrs_msgs::TransformReferenceListSrv::Request& req, mrs_msgs::TransformReferenceListSrv::Response& res);
  bool callbackTransformPose(mrs_msgs::TransformPoseSrv::Request& req, mrs_msgs::TransformPoseSrv::Response& res);
  bool callbackTransformVector3(mrs_msgs::TransformVector3Srv::Request& req, mrs_msgs::TransformVector3Srv::Response& res);

  // | ----------------------- constraints ---------------------- |

  // sets constraints to all trackers
  bool callbackSetConstraints(mrs_msgs::DynamicsConstraintsSrv::Request& req, mrs_msgs::DynamicsConstraintsSrv::Response& res);

  // constraints management
  bool              got_constraints_ = false;
  std::mutex        mutex_constraints_;
  void              setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest& constraints);
  void              setConstraintsToTrackers(const mrs_msgs::DynamicsConstraintsSrvRequest& constraints);
  void              setConstraintsToControllers(const mrs_msgs::DynamicsConstraintsSrvRequest& constraints);
  std::atomic<bool> constraints_being_enforced_ = false;

  std::optional<mrs_msgs::DynamicsConstraintsSrvRequest> enforceControllersConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest& constraints);

  mrs_msgs::DynamicsConstraintsSrvRequest current_constraints_;
  mrs_msgs::DynamicsConstraintsSrvRequest sanitized_constraints_;

  // | ------------------ emergency triggered? ------------------ |

  std::atomic<bool> failsafe_triggered_ = false;
  std::atomic<bool> eland_triggered_    = false;

  // | ------------------------- timers ------------------------- |

  // timer for regular status publishing
  ros::Timer timer_status_;
  void       timerStatus(const ros::TimerEvent& event);

  // timer for issuing the failsafe landing
  ros::Timer timer_failsafe_;
  void       timerFailsafe(const ros::TimerEvent& event);

  // oneshot timer for running controllers and trackers
  void              asyncControl(void);
  std::atomic<bool> running_async_control_ = false;
  std::future<void> async_control_result_;

  // timer for issuing emergancy landing
  ros::Timer timer_eland_;
  void       timerEland(const ros::TimerEvent& event);

  // timer for regular checking of controller errors
  ros::Timer        timer_safety_;
  void              timerSafety(const ros::TimerEvent& event);
  std::atomic<bool> running_safety_timer_        = false;
  std::atomic<bool> odometry_switch_in_progress_ = false;

  // timer for issuing the pirouette
  ros::Timer timer_pirouette_;
  void       timerPirouette(const ros::TimerEvent& event);

  // | --------------------- obstacle bumper -------------------- |

  // bumper timer
  ros::Timer timer_bumper_;
  void       timerBumper(const ros::TimerEvent& event);

  // bumper subscriber
  mrs_lib::SubscribeHandler<mrs_msgs::ObstacleSectors> sh_bumper_;

  bool        _bumper_switch_tracker_    = false;
  bool        _bumper_switch_controller_ = false;
  std::string _bumper_tracker_name_;
  std::string _bumper_controller_name_;
  std::string bumper_previous_tracker_;
  std::string bumper_previous_controller_;

  std::atomic<bool> bumper_enabled_   = false;
  std::atomic<bool> bumper_repulsing_ = false;

  bool _bumper_horizontal_derive_from_dynamics_;
  bool _bumper_vertical_derive_from_dynamics_;

  double _bumper_horizontal_distance_ = 0;
  double _bumper_vertical_distance_   = 0;

  double _bumper_horizontal_overshoot_ = 0;
  double _bumper_vertical_overshoot_   = 0;

  int  bumperGetSectorId(const double& x, const double& y, const double& z);
  void bumperPushFromObstacle(void);

  // | --------------- safety checks and failsafes -------------- |

  // escalating failsafe (eland -> failsafe -> disarm)
  bool                       _service_escalating_failsafe_enabled_ = false;
  bool                       _rc_escalating_failsafe_enabled_      = false;
  double                     _escalating_failsafe_timeout_         = 0;
  ros::Time                  escalating_failsafe_time_;
  bool                       _escalating_failsafe_ehover_   = false;
  bool                       _escalating_failsafe_eland_    = false;
  bool                       _escalating_failsafe_failsafe_ = false;
  double                     _rc_escalating_failsafe_threshold_;
  int                        _rc_escalating_failsafe_channel_  = 0;
  bool                       rc_escalating_failsafe_triggered_ = false;
  EscalatingFailsafeStates_t state_escalating_failsafe_        = ESC_NONE_STATE;

  std::string _tracker_error_action_;

  // emergancy landing state machine
  LandingStates_t current_state_landing_  = IDLE_STATE;
  LandingStates_t previous_state_landing_ = IDLE_STATE;
  std::mutex      mutex_landing_state_machine_;
  void            changeLandingState(LandingStates_t new_state);
  double          _uav_mass_ = 0;
  double          _elanding_cutoff_mass_factor_;
  double          _elanding_cutoff_timeout_;
  double          landing_uav_mass_ = 0;

  // initial body disturbance loaded from params
  double _initial_body_disturbance_x_ = 0;
  double _initial_body_disturbance_y_ = 0;

  // profiling
  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // diagnostics publishing
  void       publishDiagnostics(void);
  std::mutex mutex_diagnostics_;

  void                                             ungripSrv(void);
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_ungrip_;

  bool isFlyingNormally(void);

  // | ------------------------ pirouette ----------------------- |

  bool       _pirouette_enabled_ = false;
  double     _pirouette_speed_;
  double     _pirouette_timer_rate_;
  std::mutex mutex_pirouette_;
  double     pirouette_initial_heading_;
  double     pirouette_iterator_;
  bool       callbackPirouette(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // | -------------------- joystick control -------------------- |

  mrs_lib::SubscribeHandler<sensor_msgs::Joy> sh_joystick_;

  void callbackJoystick(const sensor_msgs::Joy::ConstPtr msg);
  bool callbackUseJoystick([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // joystick buttons mappings
  int _channel_A_, _channel_B_, _channel_X_, _channel_Y_, _channel_start_, _channel_back_, _channel_LT_, _channel_RT_, _channel_L_joy_, _channel_R_joy_;

  // channel numbers and channel multipliers
  int    _channel_pitch_, _channel_roll_, _channel_heading_, _channel_throttle_;
  double _channel_mult_pitch_, _channel_mult_roll_, _channel_mult_heading_, _channel_mult_throttle_;

  ros::Timer timer_joystick_;
  void       timerJoystick(const ros::TimerEvent& event);
  double     _joystick_timer_rate_ = 0;

  double _joystick_carrot_distance_ = 0;

  ros::Time joystick_start_press_time_;
  bool      joystick_start_pressed_ = false;

  ros::Time joystick_back_press_time_;
  bool      joystick_back_pressed_ = false;
  bool      joystick_goto_enabled_ = false;

  bool      joystick_failsafe_pressed_ = false;
  ros::Time joystick_failsafe_press_time_;

  bool      joystick_eland_pressed_ = false;
  ros::Time joystick_eland_press_time_;

  // | ------------------- RC joystick control ------------------ |

  // listening to the RC channels as told by pixhawk
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiRcChannels> sh_hw_api_rc_;

  // the RC channel mapping of the main 4 control signals
  double _rc_channel_pitch_, _rc_channel_roll_, _rc_channel_heading_, _rc_channel_throttle_;

  bool              _rc_goto_enabled_               = false;
  std::atomic<bool> rc_goto_active_                 = false;
  double            rc_joystick_channel_last_value_ = 0.5;
  bool              rc_joystick_channel_was_low_    = false;
  int               _rc_joystick_channel_           = 0;

  double _rc_horizontal_speed_ = 0;
  double _rc_vertical_speed_   = 0;
  double _rc_heading_rate_     = 0;

  // | ------------------- trajectory loading ------------------- |

  mrs_lib::PublisherHandler<geometry_msgs::PoseArray>        pub_debug_original_trajectory_poses_;
  mrs_lib::PublisherHandler<visualization_msgs::MarkerArray> pub_debug_original_trajectory_markers_;

  // | --------------------- other routines --------------------- |

  // this is called to update the trackers and to receive position control command from the active one
  void updateTrackers(void);

  // this is called to update the controllers and to receive attitude control command from the active one
  void updateControllers(const mrs_msgs::UavState& uav_state);

  // sets the reference to the active tracker
  std::tuple<bool, std::string> setReference(const mrs_msgs::ReferenceStamped reference_in);

  // sets the velocity reference to the active tracker
  std::tuple<bool, std::string> setVelocityReference(const mrs_msgs::VelocityReferenceStamped& reference_in);

  // sets the reference trajectory to the active tracker
  std::tuple<bool, std::string, bool, std::vector<std::string>, std::vector<bool>, std::vector<std::string>> setTrajectoryReference(
      const mrs_msgs::TrajectoryReference trajectory_in);

  // publishes
  void publish(void);

  bool loadConfigFile(const std::string& file_path, const std::string ns);

  double getMass(void);

  // publishes rviz-visualizable control reference
  void publishControlReferenceOdom(const std::optional<mrs_msgs::TrackerCommand>& tracker_command, const Controller::ControlOutput& control_output);

  void initializeControlOutput(void);

  // tell the mrs_odometry to disable its callbacks
  void odometryCallbacksSrv(const bool input);

  mrs_msgs::ReferenceStamped velocityReferenceToReference(const mrs_msgs::VelocityReferenceStamped& vel_reference);

  void                          setCallbacks(bool in);
  bool                          isOffboard(void);
  bool                          elandSrv(void);
  std::tuple<bool, std::string> arming(const bool input);

  // safety functions impl
  std::tuple<bool, std::string> ehover(void);
  std::tuple<bool, std::string> hover(void);
  std::tuple<bool, std::string> startTrajectoryTracking(void);
  std::tuple<bool, std::string> stopTrajectoryTracking(void);
  std::tuple<bool, std::string> resumeTrajectoryTracking(void);
  std::tuple<bool, std::string> gotoTrajectoryStart(void);
  std::tuple<bool, std::string> eland(void);
  std::tuple<bool, std::string> failsafe(void);
  std::tuple<bool, std::string> escalatingFailsafe(void);

  EscalatingFailsafeStates_t getNextEscFailsafeState(void);
};

//}

/* //{ onInit() */

void ControlManager::onInit() {
  preinitialize();
}

//}

/* preinitialize() //{ */

void ControlManager::preinitialize(void) {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "ControlManager";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_hw_api_capabilities_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiCapabilities>(shopts, "hw_api_capabilities_in");

  timer_hw_api_capabilities_ = nh_.createTimer(ros::Rate(1.0), &ControlManager::timerHwApiCapabilities, this);
}

//}

/* initialize() //{ */

void ControlManager::initialize(void) {

  joystick_start_press_time_      = ros::Time(0);
  joystick_failsafe_press_time_   = ros::Time(0);
  joystick_eland_press_time_      = ros::Time(0);
  escalating_failsafe_time_       = ros::Time(0);
  controller_tracker_switch_time_ = ros::Time(0);

  ROS_INFO("[ControlManager]: initializing");

  // --------------------------------------------------------------
  // |         common handler for trackers and controllers        |
  // --------------------------------------------------------------

  common_handlers_ = std::make_shared<mrs_uav_managers::control_manager::CommonHandlers_t>();

  // --------------------------------------------------------------
  // |                           params                           |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "ControlManager");

  param_loader.loadParam("custom_config", _custom_config_);
  param_loader.loadParam("platform_config", _platform_config_);
  param_loader.loadParam("world_config", _world_config_);
  param_loader.loadParam("network_config", _network_config_);

  if (_custom_config_ != "") {
    param_loader.addYamlFile(_custom_config_);
  }

  if (_platform_config_ != "") {
    param_loader.addYamlFile(_platform_config_);
  }

  if (_world_config_ != "") {
    param_loader.addYamlFile(_world_config_);
  }

  if (_network_config_ != "") {
    param_loader.addYamlFile(_network_config_);
  }

  param_loader.addYamlFileFromParam("private_config");
  param_loader.addYamlFileFromParam("public_config");
  param_loader.addYamlFileFromParam("private_trackers");
  param_loader.addYamlFileFromParam("private_controllers");
  param_loader.addYamlFileFromParam("public_controllers");

  // params passed from the launch file are not prefixed
  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("body_frame", _body_frame_);
  param_loader.loadParam("enable_profiler", _profiler_enabled_);
  param_loader.loadParam("uav_mass", _uav_mass_);
  param_loader.loadParam("body_disturbance_x", _initial_body_disturbance_x_);
  param_loader.loadParam("body_disturbance_y", _initial_body_disturbance_y_);
  param_loader.loadParam("g", common_handlers_->g);

  // motor params are also not prefixed, since they are common to more nodes
  param_loader.loadParam("motor_params/a", common_handlers_->throttle_model.A);
  param_loader.loadParam("motor_params/b", common_handlers_->throttle_model.B);
  param_loader.loadParam("motor_params/n_motors", common_handlers_->throttle_model.n_motors);

  // | ----------------------- safety area ---------------------- |

  bool use_safety_area;
  param_loader.loadParam("safety_area/enabled", use_safety_area);
  use_safety_area_ = use_safety_area;

  param_loader.loadParam("safety_area/horizontal/frame_name", _safety_area_horizontal_frame_);

  param_loader.loadParam("safety_area/vertical/frame_name", _safety_area_vertical_frame_);
  param_loader.loadParam("safety_area/vertical/max_z", _safety_area_max_z_);

  {
    double temp;
    param_loader.loadParam("safety_area/vertical/min_z", temp);

    _safety_area_min_z_ = temp;
  }

  if (use_safety_area_) {

    Eigen::MatrixXd border_points = param_loader.loadMatrixDynamic2("safety_area/horizontal/points", -1, 2);

    try {

      std::vector<Eigen::MatrixXd> polygon_obstacle_points;
      std::vector<Eigen::MatrixXd> point_obstacle_points;

      safety_zone_ = std::make_unique<mrs_lib::SafetyZone>(border_points);
    }

    catch (mrs_lib::SafetyZone::BorderError& e) {
      ROS_ERROR("[ControlManager]: SafetyArea: wrong configruation for the safety zone border polygon");
      ros::shutdown();
    }
    catch (...) {
      ROS_ERROR("[ControlManager]: SafetyArea: unhandled exception!");
      ros::shutdown();
    }

    ROS_INFO("[ControlManager]: safety area initialized");
  }

  param_loader.setPrefix("mrs_uav_managers/control_manager/");

  param_loader.loadParam("state_input", _state_input_);

  if (!(_state_input_ == INPUT_UAV_STATE || _state_input_ == INPUT_ODOMETRY)) {
    ROS_ERROR("[ControlManager]: the state_input parameter has to be in {0, 1}");
    ros::shutdown();
  }

  param_loader.loadParam("safety/min_throttle_null_tracker", _min_throttle_null_tracker_);
  param_loader.loadParam("safety/ehover_tracker", _ehover_tracker_name_);
  param_loader.loadParam("safety/failsafe_controller", _failsafe_controller_name_);

  param_loader.loadParam("safety/eland/controller", _eland_controller_name_);
  param_loader.loadParam("safety/eland/cutoff_mass_factor", _elanding_cutoff_mass_factor_);
  param_loader.loadParam("safety/eland/cutoff_timeout", _elanding_cutoff_timeout_);
  param_loader.loadParam("safety/eland/timer_rate", _elanding_timer_rate_);
  param_loader.loadParam("safety/eland/disarm", _eland_disarm_enabled_);

  param_loader.loadParam("safety/escalating_failsafe/service/enabled", _service_escalating_failsafe_enabled_);
  param_loader.loadParam("safety/escalating_failsafe/rc/enabled", _rc_escalating_failsafe_enabled_);
  param_loader.loadParam("safety/escalating_failsafe/rc/channel_number", _rc_escalating_failsafe_channel_);
  param_loader.loadParam("safety/escalating_failsafe/rc/threshold", _rc_escalating_failsafe_threshold_);
  param_loader.loadParam("safety/escalating_failsafe/timeout", _escalating_failsafe_timeout_);
  param_loader.loadParam("safety/escalating_failsafe/ehover", _escalating_failsafe_ehover_);
  param_loader.loadParam("safety/escalating_failsafe/eland", _escalating_failsafe_eland_);
  param_loader.loadParam("safety/escalating_failsafe/failsafe", _escalating_failsafe_failsafe_);

  param_loader.loadParam("safety/tilt_limit/eland/enabled", _tilt_limit_eland_enabled_);
  param_loader.loadParam("safety/tilt_limit/eland/limit", _tilt_limit_eland_);

  _tilt_limit_eland_ = M_PI * (_tilt_limit_eland_ / 180.0);

  if (_tilt_limit_eland_enabled_ && fabs(_tilt_limit_eland_) < 1e-3) {
    ROS_ERROR("[ControlManager]: safety/tilt_limit/eland/enabled = 'TRUE' but the limit is too low");
    ros::shutdown();
  }

  param_loader.loadParam("safety/tilt_limit/disarm/enabled", _tilt_limit_disarm_enabled_);
  param_loader.loadParam("safety/tilt_limit/disarm/limit", _tilt_limit_disarm_);

  _tilt_limit_disarm_ = M_PI * (_tilt_limit_disarm_ / 180.0);

  if (_tilt_limit_disarm_enabled_ && fabs(_tilt_limit_disarm_) < 1e-3) {
    ROS_ERROR("[ControlManager]: safety/tilt_limit/disarm/enabled = 'TRUE' but the limit is too low");
    ros::shutdown();
  }

  param_loader.loadParam("safety/yaw_error_eland/enabled", _yaw_error_eland_enabled_);
  param_loader.loadParam("safety/yaw_error_eland/limit", _yaw_error_eland_);

  _yaw_error_eland_ = M_PI * (_yaw_error_eland_ / 180.0);

  if (_yaw_error_eland_enabled_ && fabs(_yaw_error_eland_) < 1e-3) {
    ROS_ERROR("[ControlManager]: safety/yaw_error_eland/enabled = 'TRUE' but the limit is too low");
    ros::shutdown();
  }

  param_loader.loadParam("status_timer_rate", _status_timer_rate_);
  param_loader.loadParam("safety/safety_timer_rate", _safety_timer_rate_);
  param_loader.loadParam("safety/failsafe_timer_rate", _failsafe_timer_rate_);
  param_loader.loadParam("safety/rc_emergency_handoff/enabled", _rc_emergency_handoff_);

  param_loader.loadParam("safety/odometry_max_missing_time", _uav_state_max_missing_time_);
  param_loader.loadParam("safety/odometry_innovation_eland/enabled", _odometry_innovation_check_enabled_);

  param_loader.loadParam("safety/tilt_error_disarm/enabled", _tilt_error_disarm_enabled_);
  param_loader.loadParam("safety/tilt_error_disarm/timeout", _tilt_error_disarm_timeout_);
  param_loader.loadParam("safety/tilt_error_disarm/error_threshold", _tilt_error_disarm_threshold_);

  _tilt_error_disarm_threshold_ = M_PI * (_tilt_error_disarm_threshold_ / 180.0);

  if (_tilt_error_disarm_enabled_ && fabs(_tilt_error_disarm_threshold_) < 1e-3) {
    ROS_ERROR("[ControlManager]: safety/tilt_error_disarm/enabled = 'TRUE' but the limit is too low");
    ros::shutdown();
  }

  // default constraints

  param_loader.loadParam("default_constraints/horizontal/speed", current_constraints_.constraints.horizontal_speed);
  param_loader.loadParam("default_constraints/horizontal/acceleration", current_constraints_.constraints.horizontal_acceleration);
  param_loader.loadParam("default_constraints/horizontal/jerk", current_constraints_.constraints.horizontal_jerk);
  param_loader.loadParam("default_constraints/horizontal/snap", current_constraints_.constraints.horizontal_snap);

  param_loader.loadParam("default_constraints/vertical/ascending/speed", current_constraints_.constraints.vertical_ascending_speed);
  param_loader.loadParam("default_constraints/vertical/ascending/acceleration", current_constraints_.constraints.vertical_ascending_acceleration);
  param_loader.loadParam("default_constraints/vertical/ascending/jerk", current_constraints_.constraints.vertical_ascending_jerk);
  param_loader.loadParam("default_constraints/vertical/ascending/snap", current_constraints_.constraints.vertical_ascending_snap);

  param_loader.loadParam("default_constraints/vertical/descending/speed", current_constraints_.constraints.vertical_descending_speed);
  param_loader.loadParam("default_constraints/vertical/descending/acceleration", current_constraints_.constraints.vertical_descending_acceleration);
  param_loader.loadParam("default_constraints/vertical/descending/jerk", current_constraints_.constraints.vertical_descending_jerk);
  param_loader.loadParam("default_constraints/vertical/descending/snap", current_constraints_.constraints.vertical_descending_snap);

  param_loader.loadParam("default_constraints/heading/speed", current_constraints_.constraints.heading_speed);
  param_loader.loadParam("default_constraints/heading/acceleration", current_constraints_.constraints.heading_acceleration);
  param_loader.loadParam("default_constraints/heading/jerk", current_constraints_.constraints.heading_jerk);
  param_loader.loadParam("default_constraints/heading/snap", current_constraints_.constraints.heading_snap);

  param_loader.loadParam("default_constraints/angular_speed/roll", current_constraints_.constraints.roll_rate);
  param_loader.loadParam("default_constraints/angular_speed/pitch", current_constraints_.constraints.pitch_rate);
  param_loader.loadParam("default_constraints/angular_speed/yaw", current_constraints_.constraints.yaw_rate);

  param_loader.loadParam("default_constraints/tilt", current_constraints_.constraints.tilt);

  current_constraints_.constraints.tilt = M_PI * (current_constraints_.constraints.tilt / 180.0);

  // joystick

  param_loader.loadParam("joystick/enabled", _joystick_enabled_);
  param_loader.loadParam("joystick/mode", _joystick_mode_);
  param_loader.loadParam("joystick/carrot_distance", _joystick_carrot_distance_);
  param_loader.loadParam("joystick/joystick_timer_rate", _joystick_timer_rate_);
  param_loader.loadParam("joystick/attitude_control/tracker", _joystick_tracker_name_);
  param_loader.loadParam("joystick/attitude_control/controller", _joystick_controller_name_);
  param_loader.loadParam("joystick/attitude_control/fallback/tracker", _joystick_fallback_tracker_name_);
  param_loader.loadParam("joystick/attitude_control/fallback/controller", _joystick_fallback_controller_name_);

  param_loader.loadParam("joystick/channels/A", _channel_A_);
  param_loader.loadParam("joystick/channels/B", _channel_B_);
  param_loader.loadParam("joystick/channels/X", _channel_X_);
  param_loader.loadParam("joystick/channels/Y", _channel_Y_);
  param_loader.loadParam("joystick/channels/start", _channel_start_);
  param_loader.loadParam("joystick/channels/back", _channel_back_);
  param_loader.loadParam("joystick/channels/LT", _channel_LT_);
  param_loader.loadParam("joystick/channels/RT", _channel_RT_);
  param_loader.loadParam("joystick/channels/L_joy", _channel_L_joy_);
  param_loader.loadParam("joystick/channels/R_joy", _channel_R_joy_);

  // load channels
  param_loader.loadParam("joystick/channels/pitch", _channel_pitch_);
  param_loader.loadParam("joystick/channels/roll", _channel_roll_);
  param_loader.loadParam("joystick/channels/heading", _channel_heading_);
  param_loader.loadParam("joystick/channels/throttle", _channel_throttle_);

  // load channel multipliers
  param_loader.loadParam("joystick/channel_multipliers/pitch", _channel_mult_pitch_);
  param_loader.loadParam("joystick/channel_multipliers/roll", _channel_mult_roll_);
  param_loader.loadParam("joystick/channel_multipliers/heading", _channel_mult_heading_);
  param_loader.loadParam("joystick/channel_multipliers/throttle", _channel_mult_throttle_);

  bool bumper_enabled;
  param_loader.loadParam("obstacle_bumper/enabled", bumper_enabled);
  bumper_enabled_ = bumper_enabled;

  param_loader.loadParam("obstacle_bumper/switch_tracker", _bumper_switch_tracker_);
  param_loader.loadParam("obstacle_bumper/switch_controller", _bumper_switch_controller_);
  param_loader.loadParam("obstacle_bumper/tracker", _bumper_tracker_name_);
  param_loader.loadParam("obstacle_bumper/controller", _bumper_controller_name_);
  param_loader.loadParam("obstacle_bumper/timer_rate", _bumper_timer_rate_);

  param_loader.loadParam("obstacle_bumper/horizontal/min_distance_to_obstacle", _bumper_horizontal_distance_);
  param_loader.loadParam("obstacle_bumper/horizontal/derived_from_dynamics", _bumper_horizontal_derive_from_dynamics_);

  param_loader.loadParam("obstacle_bumper/vertical/min_distance_to_obstacle", _bumper_vertical_distance_);
  param_loader.loadParam("obstacle_bumper/vertical/derived_from_dynamics", _bumper_vertical_derive_from_dynamics_);

  param_loader.loadParam("obstacle_bumper/horizontal/overshoot", _bumper_horizontal_overshoot_);
  param_loader.loadParam("obstacle_bumper/vertical/overshoot", _bumper_vertical_overshoot_);

  param_loader.loadParam("safety/tracker_error_action", _tracker_error_action_);

  param_loader.loadParam("trajectory_tracking/snap_to_safety_area", _snap_trajectory_to_safety_area_);

  // check the values of tracker error action
  if (_tracker_error_action_ != ELAND_STR && _tracker_error_action_ != EHOVER_STR) {
    ROS_ERROR("[ControlManager]: the tracker_error_action parameter (%s) is not correct, requires {%s, %s}", _tracker_error_action_.c_str(), ELAND_STR,
              EHOVER_STR);
    ros::shutdown();
  }

  param_loader.loadParam("rc_joystick/enabled", _rc_goto_enabled_);
  param_loader.loadParam("rc_joystick/channel_number", _rc_joystick_channel_);
  param_loader.loadParam("rc_joystick/horizontal_speed", _rc_horizontal_speed_);
  param_loader.loadParam("rc_joystick/vertical_speed", _rc_vertical_speed_);
  param_loader.loadParam("rc_joystick/heading_rate", _rc_heading_rate_);

  param_loader.loadParam("rc_joystick/channels/pitch", _rc_channel_pitch_);
  param_loader.loadParam("rc_joystick/channels/roll", _rc_channel_roll_);
  param_loader.loadParam("rc_joystick/channels/heading", _rc_channel_heading_);
  param_loader.loadParam("rc_joystick/channels/throttle", _rc_channel_throttle_);

  param_loader.loadParam("pirouette/speed", _pirouette_speed_);
  param_loader.loadParam("pirouette/timer_rate", _pirouette_timer_rate_);

  param_loader.loadParam("safety/parachute/enabled", _parachute_enabled_);

  // --------------------------------------------------------------
  // |             initialize the last control output             |
  // --------------------------------------------------------------

  initializeControlOutput();

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(nh_, "ControlManager");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | ------------------- scope timer logger ------------------- |

  param_loader.loadParam("scope_timer/enabled", scope_timer_enabled_);
  const std::string scope_timer_log_filename = param_loader.loadParam2("scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<mrs_lib::ScopeTimerLogger>(scope_timer_log_filename, scope_timer_enabled_);

  // bind transformer to trackers and controllers for use
  common_handlers_->transformer = transformer_;

  // bind scope timer to trackers and controllers for use
  common_handlers_->scope_timer.enabled = scope_timer_enabled_;
  common_handlers_->scope_timer.logger  = scope_timer_logger_;

  common_handlers_->safety_area.use_safety_area       = use_safety_area_;
  common_handlers_->safety_area.isPointInSafetyArea2d = boost::bind(&ControlManager::isPointInSafetyArea2d, this, _1);
  common_handlers_->safety_area.isPointInSafetyArea3d = boost::bind(&ControlManager::isPointInSafetyArea3d, this, _1);
  common_handlers_->safety_area.getMinZ               = boost::bind(&ControlManager::getMinZ, this, _1);
  common_handlers_->safety_area.getMaxZ               = boost::bind(&ControlManager::getMaxZ, this, _1);

  common_handlers_->getMass = boost::bind(&ControlManager::getMass, this);

  common_handlers_->detailed_model_params = loadDetailedUavModelParams(nh_, "ControlManager", _platform_config_, _custom_config_);

  common_handlers_->control_output_modalities = _hw_api_inputs_;

  common_handlers_->uav_name = _uav_name_;

  common_handlers_->parent_nh = nh_;

  // --------------------------------------------------------------
  // |                        load trackers                       |
  // --------------------------------------------------------------

  std::vector<std::string> custom_trackers;

  param_loader.loadParam("mrs_trackers", _tracker_names_);
  param_loader.loadParam("trackers", custom_trackers);

  if (!custom_trackers.empty()) {
    _tracker_names_.insert(_tracker_names_.end(), custom_trackers.begin(), custom_trackers.end());
  }

  param_loader.loadParam("null_tracker", _null_tracker_name_);
  param_loader.loadParam("landing_takeoff_tracker", _landoff_tracker_name_);

  tracker_loader_ = std::make_unique<pluginlib::ClassLoader<mrs_uav_managers::Tracker>>("mrs_uav_managers", "mrs_uav_managers::Tracker");

  for (int i = 0; i < int(_tracker_names_.size()); i++) {

    std::string tracker_name = _tracker_names_.at(i);

    // load the controller parameters
    std::string address;
    std::string name_space;
    bool        human_switchable;

    param_loader.loadParam(tracker_name + "/address", address);
    param_loader.loadParam(tracker_name + "/namespace", name_space);
    param_loader.loadParam(tracker_name + "/human_switchable", human_switchable, false);

    TrackerParams new_tracker(address, name_space, human_switchable);
    trackers_.insert(std::pair<std::string, TrackerParams>(tracker_name, new_tracker));

    try {
      ROS_INFO("[ControlManager]: loading the tracker '%s'", new_tracker.address.c_str());
      tracker_list_.push_back(tracker_loader_->createInstance(new_tracker.address.c_str()));
    }
    catch (pluginlib::CreateClassException& ex1) {
      ROS_ERROR("[ControlManager]: CreateClassException for the tracker '%s'", new_tracker.address.c_str());
      ROS_ERROR("[ControlManager]: Error: %s", ex1.what());
      ros::shutdown();
    }
    catch (pluginlib::PluginlibException& ex) {
      ROS_ERROR("[ControlManager]: PluginlibException for the tracker '%s'", new_tracker.address.c_str());
      ROS_ERROR("[ControlManager]: Error: %s", ex.what());
      ros::shutdown();
    }
  }

  ROS_INFO("[ControlManager]: trackers were loaded");

  for (int i = 0; i < int(tracker_list_.size()); i++) {

    std::map<std::string, TrackerParams>::iterator it;
    it = trackers_.find(_tracker_names_.at(i));

    // create private handlers
    std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers =
        std::make_shared<mrs_uav_managers::control_manager::PrivateHandlers_t>();

    private_handlers->loadConfigFile = boost::bind(&ControlManager::loadConfigFile, this, _1, it->second.name_space);
    private_handlers->name_space     = it->second.name_space;
    private_handlers->runtime_name   = _tracker_names_.at(i);
    private_handlers->param_loader   = std::make_unique<mrs_lib::ParamLoader>(ros::NodeHandle(nh_, it->second.name_space), _tracker_names_.at(i));

    if (_custom_config_ != "") {
      private_handlers->param_loader->addYamlFile(_custom_config_);
    }

    if (_platform_config_ != "") {
      private_handlers->param_loader->addYamlFile(_platform_config_);
    }

    if (_world_config_ != "") {
      private_handlers->param_loader->addYamlFile(_world_config_);
    }

    if (_network_config_ != "") {
      private_handlers->param_loader->addYamlFile(_network_config_);
    }

    bool success = false;

    try {
      ROS_INFO("[ControlManager]: initializing the tracker '%s'", it->second.address.c_str());
      success = tracker_list_.at(i)->initialize(ros::NodeHandle(nh_, it->second.name_space), common_handlers_, private_handlers);
    }
    catch (std::runtime_error& ex) {
      ROS_ERROR("[ControlManager]: exception caught during tracker initialization: '%s'", ex.what());
    }

    if (!success) {
      ROS_ERROR("[ControlManager]: failed to initialize the tracker '%s'", it->second.address.c_str());
      ros::shutdown();
    }
  }

  ROS_INFO("[ControlManager]: trackers were initialized");

  // --------------------------------------------------------------
  // |           check the existance of selected trackers         |
  // --------------------------------------------------------------

  // | ------ check for the existance of the hover tracker ------ |

  // check if the hover_tracker is within the loaded trackers
  {
    auto idx = idxInVector(_ehover_tracker_name_, _tracker_names_);

    if (idx) {
      _ehover_tracker_idx_ = idx.value();
    } else {
      ROS_ERROR("[ControlManager]: the safety/hover_tracker (%s) is not within the loaded trackers", _ehover_tracker_name_.c_str());
      ros::shutdown();
    }
  }

  // | ----- check for the existence of the landoff tracker ----- |

  {
    auto idx = idxInVector(_landoff_tracker_name_, _tracker_names_);

    if (idx) {
      _landoff_tracker_idx_ = idx.value();
    } else {
      ROS_ERROR("[ControlManager]: the landoff tracker (%s) is not within the loaded trackers", _landoff_tracker_name_.c_str());
      ros::shutdown();
    }
  }

  // | ------- check for the existence of the null tracker ------ |

  {
    auto idx = idxInVector(_null_tracker_name_, _tracker_names_);

    if (idx) {
      _null_tracker_idx_ = idx.value();
    } else {
      ROS_ERROR("[ControlManager]: the null tracker (%s) is not within the loaded trackers", _null_tracker_name_.c_str());
      ros::shutdown();
    }
  }

  // --------------------------------------------------------------
  // |         check existance of trackers for joystick           |
  // --------------------------------------------------------------

  if (_joystick_enabled_) {

    auto idx = idxInVector(_joystick_tracker_name_, _tracker_names_);

    if (idx) {
      _joystick_tracker_idx_ = idx.value();
    } else {
      ROS_ERROR("[ControlManager]: the joystick tracker (%s) is not within the loaded trackers", _joystick_tracker_name_.c_str());
      ros::shutdown();
    }
  }

  if (_bumper_switch_tracker_) {

    auto idx = idxInVector(_bumper_tracker_name_, _tracker_names_);

    if (!idx) {
      ROS_ERROR("[ControlManager]: the bumper tracker (%s) is not within the loaded trackers", _bumper_tracker_name_.c_str());
      ros::shutdown();
    }
  }

  {
    auto idx = idxInVector(_joystick_fallback_tracker_name_, _tracker_names_);

    if (idx) {
      _joystick_fallback_tracker_idx_ = idx.value();
    } else {
      ROS_ERROR("[ControlManager]: the joystick fallback tracker (%s) is not within the loaded trackers", _joystick_fallback_tracker_name_.c_str());
      ros::shutdown();
    }
  }

  // --------------------------------------------------------------
  // |                    load the controllers                    |
  // --------------------------------------------------------------

  std::vector<std::string> custom_controllers;

  param_loader.loadParam("mrs_controllers", _controller_names_);
  param_loader.loadParam("controllers", custom_controllers);

  if (!custom_controllers.empty()) {
    _controller_names_.insert(_controller_names_.end(), custom_controllers.begin(), custom_controllers.end());
  }

  controller_loader_ = std::make_unique<pluginlib::ClassLoader<mrs_uav_managers::Controller>>("mrs_uav_managers", "mrs_uav_managers::Controller");

  // for each controller in the list
  for (int i = 0; i < int(_controller_names_.size()); i++) {

    std::string controller_name = _controller_names_.at(i);

    // load the controller parameters
    std::string address;
    std::string name_space;
    double      eland_threshold, failsafe_threshold, odometry_innovation_threshold;
    bool        human_switchable;
    param_loader.loadParam(controller_name + "/address", address);
    param_loader.loadParam(controller_name + "/namespace", name_space);
    param_loader.loadParam(controller_name + "/eland_threshold", eland_threshold);
    param_loader.loadParam(controller_name + "/failsafe_threshold", failsafe_threshold);
    param_loader.loadParam(controller_name + "/odometry_innovation_threshold", odometry_innovation_threshold);
    param_loader.loadParam(controller_name + "/human_switchable", human_switchable, false);

    // check if the controller can output some of the required outputs
    {

      ControlOutputModalities_t outputs;
      param_loader.loadParam(controller_name + "/outputs/actuators", outputs.actuators, false);
      param_loader.loadParam(controller_name + "/outputs/control_group", outputs.control_group, false);
      param_loader.loadParam(controller_name + "/outputs/attitude_rate", outputs.attitude_rate, false);
      param_loader.loadParam(controller_name + "/outputs/attitude", outputs.attitude, false);
      param_loader.loadParam(controller_name + "/outputs/acceleration_hdg_rate", outputs.acceleration_hdg_rate, false);
      param_loader.loadParam(controller_name + "/outputs/acceleration_hdg", outputs.acceleration_hdg, false);
      param_loader.loadParam(controller_name + "/outputs/velocity_hdg_rate", outputs.velocity_hdg_rate, false);
      param_loader.loadParam(controller_name + "/outputs/velocity_hdg", outputs.velocity_hdg, false);
      param_loader.loadParam(controller_name + "/outputs/position", outputs.position, false);

      bool meets_actuators             = (_hw_api_inputs_.actuators && outputs.actuators);
      bool meets_control_group         = (_hw_api_inputs_.control_group && outputs.control_group);
      bool meets_attitude_rate         = (_hw_api_inputs_.attitude_rate && outputs.attitude_rate);
      bool meets_attitude              = (_hw_api_inputs_.attitude && outputs.attitude);
      bool meets_acceleration_hdg_rate = (_hw_api_inputs_.acceleration_hdg_rate && outputs.acceleration_hdg_rate);
      bool meets_acceleration_hdg      = (_hw_api_inputs_.acceleration_hdg && outputs.acceleration_hdg);
      bool meets_velocity_hdg_rate     = (_hw_api_inputs_.velocity_hdg_rate && outputs.velocity_hdg_rate);
      bool meets_velocity_hdg          = (_hw_api_inputs_.velocity_hdg && outputs.velocity_hdg);
      bool meets_position              = (_hw_api_inputs_.position && outputs.position);

      bool meets_requirements = meets_actuators || meets_control_group || meets_attitude_rate || meets_attitude || meets_acceleration_hdg_rate ||
                                meets_acceleration_hdg || meets_velocity_hdg_rate || meets_velocity_hdg || meets_position;

      if (!meets_requirements) {

        ROS_ERROR("[ControlManager]: the controller '%s' does not meet the control output requirements, which are some of the following",
                  controller_name.c_str());

        if (_hw_api_inputs_.actuators) {
          ROS_ERROR("[ControlManager]: - actuators");
        }

        if (_hw_api_inputs_.control_group) {
          ROS_ERROR("[ControlManager]: - control group");
        }

        if (_hw_api_inputs_.attitude_rate) {
          ROS_ERROR("[ControlManager]: - attitude rate");
        }

        if (_hw_api_inputs_.attitude) {
          ROS_ERROR("[ControlManager]: - attitude");
        }

        if (_hw_api_inputs_.acceleration_hdg_rate) {
          ROS_ERROR("[ControlManager]: - acceleration+hdg rate");
        }

        if (_hw_api_inputs_.acceleration_hdg) {
          ROS_ERROR("[ControlManager]: - acceleration+hdg");
        }

        if (_hw_api_inputs_.velocity_hdg_rate) {
          ROS_ERROR("[ControlManager]: - velocity+hdg rate");
        }

        if (_hw_api_inputs_.velocity_hdg) {
          ROS_ERROR("[ControlManager]: - velocity+hdg");
        }

        if (_hw_api_inputs_.position) {
          ROS_ERROR("[ControlManager]: - position");
        }

        ros::shutdown();
      }

      if ((_hw_api_inputs_.actuators || _hw_api_inputs_.control_group) && !common_handlers_->detailed_model_params) {
        ROS_ERROR(
            "[ControlManager]: the HW API supports 'actuators' or 'control_group' input, but the 'detailed uav model params' were not loaded sucessfully");
        ros::shutdown();
      }
    }

    // | --- alter the timer rates based on the hw capabilities --- |

    CONTROL_OUTPUT lowest_output = getLowestOuput(_hw_api_inputs_);

    if (lowest_output == ACTUATORS_CMD || lowest_output == CONTROL_GROUP) {
      _safety_timer_rate_     = 200.0;
      desired_uav_state_rate_ = 250.0;
    } else if (lowest_output == ATTITUDE_RATE || lowest_output == ATTITUDE) {
      _safety_timer_rate_     = 100.0;
      desired_uav_state_rate_ = 100.0;
    } else if (lowest_output == ACCELERATION_HDG_RATE || lowest_output == ACCELERATION_HDG) {
      _safety_timer_rate_     = 30.0;
      _status_timer_rate_     = 1.0;
      desired_uav_state_rate_ = 40.0;

      if (_uav_state_max_missing_time_ < 0.2) {
        _uav_state_max_missing_time_ = 0.2;
      }
    } else if (lowest_output >= VELOCITY_HDG_RATE) {
      _safety_timer_rate_     = 20.0;
      _status_timer_rate_     = 1.0;
      desired_uav_state_rate_ = 20.0;

      if (_uav_state_max_missing_time_ < 1.0) {
        _uav_state_max_missing_time_ = 1.0;
      }
    }

    if (eland_threshold == 0) {
      eland_threshold = 1e6;
    }

    if (failsafe_threshold == 0) {
      failsafe_threshold = 1e6;
    }

    if (odometry_innovation_threshold == 0) {
      odometry_innovation_threshold = 1e6;
    }

    ControllerParams new_controller(address, name_space, eland_threshold, failsafe_threshold, odometry_innovation_threshold, human_switchable);
    controllers_.insert(std::pair<std::string, ControllerParams>(controller_name, new_controller));

    try {
      ROS_INFO("[ControlManager]: loading the controller '%s'", new_controller.address.c_str());
      controller_list_.push_back(controller_loader_->createInstance(new_controller.address.c_str()));
    }
    catch (pluginlib::CreateClassException& ex1) {
      ROS_ERROR("[ControlManager]: CreateClassException for the controller '%s'", new_controller.address.c_str());
      ROS_ERROR("[ControlManager]: Error: %s", ex1.what());
      ros::shutdown();
    }
    catch (pluginlib::PluginlibException& ex) {
      ROS_ERROR("[ControlManager]: PluginlibException for the controller '%s'", new_controller.address.c_str());
      ROS_ERROR("[ControlManager]: Error: %s", ex.what());
      ros::shutdown();
    }
  }

  ROS_INFO("[ControlManager]: controllers were loaded");

  for (int i = 0; i < int(controller_list_.size()); i++) {

    std::map<std::string, ControllerParams>::iterator it;
    it = controllers_.find(_controller_names_.at(i));

    // create private handlers
    std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers =
        std::make_shared<mrs_uav_managers::control_manager::PrivateHandlers_t>();

    private_handlers->loadConfigFile = boost::bind(&ControlManager::loadConfigFile, this, _1, it->second.name_space);
    private_handlers->name_space     = it->second.name_space;
    private_handlers->runtime_name   = _controller_names_.at(i);
    private_handlers->param_loader   = std::make_unique<mrs_lib::ParamLoader>(ros::NodeHandle(nh_, it->second.name_space), _controller_names_.at(i));

    if (_custom_config_ != "") {
      private_handlers->param_loader->addYamlFile(_custom_config_);
    }

    if (_platform_config_ != "") {
      private_handlers->param_loader->addYamlFile(_platform_config_);
    }

    if (_world_config_ != "") {
      private_handlers->param_loader->addYamlFile(_world_config_);
    }

    if (_network_config_ != "") {
      private_handlers->param_loader->addYamlFile(_network_config_);
    }

    bool success = false;

    try {

      ROS_INFO("[ControlManager]: initializing the controller '%s'", it->second.address.c_str());
      success = controller_list_.at(i)->initialize(ros::NodeHandle(nh_, it->second.name_space), common_handlers_, private_handlers);
    }
    catch (std::runtime_error& ex) {
      ROS_ERROR("[ControlManager]: exception caught during controller initialization: '%s'", ex.what());
    }

    if (!success) {
      ROS_ERROR("[ControlManager]: failed to initialize the controller '%s'", it->second.address.c_str());
      ros::shutdown();
    }
  }

  ROS_INFO("[ControlManager]: controllers were initialized");

  {
    auto idx = idxInVector(_failsafe_controller_name_, _controller_names_);

    if (idx) {
      _failsafe_controller_idx_ = idx.value();
    } else {
      ROS_ERROR("[ControlManager]: the failsafe controller (%s) is not within the loaded controllers", _failsafe_controller_name_.c_str());
      ros::shutdown();
    }
  }

  {
    auto idx = idxInVector(_eland_controller_name_, _controller_names_);

    if (idx) {
      _eland_controller_idx_ = idx.value();
    } else {
      ROS_ERROR("[ControlManager]: the eland controller (%s) is not within the loaded controllers", _eland_controller_name_.c_str());
      ros::shutdown();
    }
  }

  {
    auto idx = idxInVector(_joystick_controller_name_, _controller_names_);

    if (idx) {
      _joystick_controller_idx_ = idx.value();
    } else {
      ROS_ERROR("[ControlManager]: the joystick controller (%s) is not within the loaded controllers", _joystick_controller_name_.c_str());
      ros::shutdown();
    }
  }

  if (_bumper_switch_controller_) {

    auto idx = idxInVector(_bumper_controller_name_, _controller_names_);

    if (!idx) {
      ROS_ERROR("[ControlManager]: the bumper controller (%s) is not within the loaded controllers", _bumper_controller_name_.c_str());
      ros::shutdown();
    }
  }

  {
    auto idx = idxInVector(_joystick_fallback_controller_name_, _controller_names_);

    if (idx) {
      _joystick_fallback_controller_idx_ = idx.value();
    } else {
      ROS_ERROR("[ControlManager]: the joystick fallback controller (%s) is not within the loaded controllers", _joystick_fallback_controller_name_.c_str());
      ros::shutdown();
    }
  }

  // --------------------------------------------------------------
  // |                  activate the NullTracker                  |
  // --------------------------------------------------------------

  ROS_INFO("[ControlManager]: activating the null tracker");

  tracker_list_.at(_null_tracker_idx_)->activate(last_tracker_cmd_);
  active_tracker_idx_ = _null_tracker_idx_;

  // --------------------------------------------------------------
  // |    activate the eland controller as the first controller   |
  // --------------------------------------------------------------

  ROS_INFO("[ControlManager]: activating the the eland controller (%s) as the first controller", _controller_names_.at(_eland_controller_idx_).c_str());

  controller_list_.at(_eland_controller_idx_)->activate(last_control_output_);
  active_controller_idx_ = _eland_controller_idx_;

  // update the time
  {
    std::scoped_lock lock(mutex_controller_tracker_switch_time_);

    controller_tracker_switch_time_ = ros::Time::now();
  }

  output_enabled_ = false;

  // | --------------- set the default constraints -------------- |

  sanitized_constraints_ = current_constraints_;
  setConstraints(current_constraints_);

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "ControlManager", _profiler_enabled_);

  // | ----------------------- publishers ----------------------- |

  control_output_publisher_ = OutputPublisher(nh_);

  ph_controller_diagnostics_             = mrs_lib::PublisherHandler<mrs_msgs::ControllerDiagnostics>(nh_, "controller_diagnostics_out", 1);
  ph_tracker_cmd_                        = mrs_lib::PublisherHandler<mrs_msgs::TrackerCommand>(nh_, "tracker_cmd_out", 1);
  ph_mrs_odom_input_                     = mrs_lib::PublisherHandler<mrs_msgs::EstimatorInput>(nh_, "estimator_input_out", 1);
  ph_control_reference_odom_             = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, "control_reference_out", 1);
  ph_diagnostics_                        = mrs_lib::PublisherHandler<mrs_msgs::ControlManagerDiagnostics>(nh_, "diagnostics_out", 1);
  ph_offboard_on_                        = mrs_lib::PublisherHandler<std_msgs::Empty>(nh_, "offboard_on_out", 1);
  ph_tilt_error_                         = mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>(nh_, "tilt_error_out", 1);
  ph_mass_estimate_                      = mrs_lib::PublisherHandler<std_msgs::Float64>(nh_, "mass_estimate_out", 1, false, 10.0);
  ph_mass_nominal_                       = mrs_lib::PublisherHandler<std_msgs::Float64>(nh_, "mass_nominal_out", 1, true);
  ph_throttle_                           = mrs_lib::PublisherHandler<std_msgs::Float64>(nh_, "throttle_out", 1, false, 10.0);
  ph_thrust_                             = mrs_lib::PublisherHandler<std_msgs::Float64>(nh_, "thrust_out", 1, false, 100.0);
  ph_control_error_                      = mrs_lib::PublisherHandler<mrs_msgs::ControlError>(nh_, "control_error_out", 1);
  ph_safety_area_markers_                = mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>(nh_, "safety_area_markers_out", 1, true, 1.0);
  ph_safety_area_coordinates_markers_    = mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>(nh_, "safety_area_coordinates_markers_out", 1, true, 1.0);
  ph_disturbances_markers_               = mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>(nh_, "disturbances_markers_out", 1, false, 10.0);
  ph_current_constraints_                = mrs_lib::PublisherHandler<mrs_msgs::DynamicsConstraints>(nh_, "current_constraints_out", 1);
  ph_heading_                            = mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>(nh_, "heading_out", 1);
  ph_speed_                              = mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>(nh_, "speed_out", 1, false, 10.0);
  pub_debug_original_trajectory_poses_   = mrs_lib::PublisherHandler<geometry_msgs::PoseArray>(nh_, "trajectory_original/poses_out", 1, true);
  pub_debug_original_trajectory_markers_ = mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>(nh_, "trajectory_original/markers_out", 1, true);

  // | ------------------ publish nominal mass ------------------ |

  {
    std_msgs::Float64 nominal_mass;

    nominal_mass.data = _uav_mass_;

    ph_mass_nominal_.publish(nominal_mass);
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "ControlManager";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  if (_state_input_ == INPUT_UAV_STATE) {
    sh_uav_state_ = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in", &ControlManager::callbackUavState, this);
  } else if (_state_input_ == INPUT_ODOMETRY) {
    sh_odometry_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odometry_in", &ControlManager::callbackOdometry, this);
  }

  if (_odometry_innovation_check_enabled_) {
    sh_odometry_innovation_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odometry_innovation_in");
  }

  sh_bumper_    = mrs_lib::SubscribeHandler<mrs_msgs::ObstacleSectors>(shopts, "bumper_sectors_in");
  sh_max_z_     = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(shopts, "max_z_in");
  sh_joystick_  = mrs_lib::SubscribeHandler<sensor_msgs::Joy>(shopts, "joystick_in", &ControlManager::callbackJoystick, this);
  sh_gnss_      = mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>(shopts, "gnss_in", &ControlManager::callbackGNSS, this);
  sh_hw_api_rc_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiRcChannels>(shopts, "hw_api_rc_in", &ControlManager::callbackRC, this);

  sh_hw_api_status_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus>(shopts, "hw_api_status_in", &ControlManager::callbackHwApiStatus, this);

  // | -------------------- general services -------------------- |

  service_server_switch_tracker_             = nh_.advertiseService("switch_tracker_in", &ControlManager::callbackSwitchTracker, this);
  service_server_switch_controller_          = nh_.advertiseService("switch_controller_in", &ControlManager::callbackSwitchController, this);
  service_server_reset_tracker_              = nh_.advertiseService("tracker_reset_static_in", &ControlManager::callbackTrackerResetStatic, this);
  service_server_hover_                      = nh_.advertiseService("hover_in", &ControlManager::callbackHover, this);
  service_server_ehover_                     = nh_.advertiseService("ehover_in", &ControlManager::callbackEHover, this);
  service_server_failsafe_                   = nh_.advertiseService("failsafe_in", &ControlManager::callbackFailsafe, this);
  service_server_failsafe_escalating_        = nh_.advertiseService("failsafe_escalating_in", &ControlManager::callbackFailsafeEscalating, this);
  service_server_toggle_output_              = nh_.advertiseService("toggle_output_in", &ControlManager::callbackToggleOutput, this);
  service_server_arm_                        = nh_.advertiseService("arm_in", &ControlManager::callbackArm, this);
  service_server_enable_callbacks_           = nh_.advertiseService("enable_callbacks_in", &ControlManager::callbackEnableCallbacks, this);
  service_server_set_constraints_            = nh_.advertiseService("set_constraints_in", &ControlManager::callbackSetConstraints, this);
  service_server_use_joystick_               = nh_.advertiseService("use_joystick_in", &ControlManager::callbackUseJoystick, this);
  service_server_use_safety_area_            = nh_.advertiseService("use_safety_area_in", &ControlManager::callbackUseSafetyArea, this);
  service_server_eland_                      = nh_.advertiseService("eland_in", &ControlManager::callbackEland, this);
  service_server_parachute_                  = nh_.advertiseService("parachute_in", &ControlManager::callbackParachute, this);
  service_server_set_min_z_                  = nh_.advertiseService("set_min_z_in", &ControlManager::callbackSetMinZ, this);
  service_server_transform_reference_        = nh_.advertiseService("transform_reference_in", &ControlManager::callbackTransformReference, this);
  service_server_transform_reference_list_   = nh_.advertiseService("transform_reference_list_in", &ControlManager::callbackTransformReferenceList, this);
  service_server_transform_pose_             = nh_.advertiseService("transform_pose_in", &ControlManager::callbackTransformPose, this);
  service_server_transform_vector3_          = nh_.advertiseService("transform_vector3_in", &ControlManager::callbackTransformVector3, this);
  service_server_bumper_enabler_             = nh_.advertiseService("bumper_in", &ControlManager::callbackEnableBumper, this);
  service_server_get_min_z_                  = nh_.advertiseService("get_min_z_in", &ControlManager::callbackGetMinZ, this);
  service_server_validate_reference_         = nh_.advertiseService("validate_reference_in", &ControlManager::callbackValidateReference, this);
  service_server_validate_reference_2d_      = nh_.advertiseService("validate_reference_2d_in", &ControlManager::callbackValidateReference2d, this);
  service_server_validate_reference_list_    = nh_.advertiseService("validate_reference_list_in", &ControlManager::callbackValidateReferenceList, this);
  service_server_start_trajectory_tracking_  = nh_.advertiseService("start_trajectory_tracking_in", &ControlManager::callbackStartTrajectoryTracking, this);
  service_server_stop_trajectory_tracking_   = nh_.advertiseService("stop_trajectory_tracking_in", &ControlManager::callbackStopTrajectoryTracking, this);
  service_server_resume_trajectory_tracking_ = nh_.advertiseService("resume_trajectory_tracking_in", &ControlManager::callbackResumeTrajectoryTracking, this);
  service_server_goto_trajectory_start_      = nh_.advertiseService("goto_trajectory_start_in", &ControlManager::callbackGotoTrajectoryStart, this);

  sch_arming_                 = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "hw_api_arming_out");
  sch_eland_                  = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "eland_out");
  sch_shutdown_               = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "shutdown_out");
  sch_set_odometry_callbacks_ = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "set_odometry_callbacks_out");
  sch_ungrip_                 = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "ungrip_out");
  sch_parachute_              = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "parachute_out");

  // | ---------------- setpoint command services --------------- |

  // human callable
  service_server_goto_                 = nh_.advertiseService("goto_in", &ControlManager::callbackGoto, this);
  service_server_goto_fcu_             = nh_.advertiseService("goto_fcu_in", &ControlManager::callbackGotoFcu, this);
  service_server_goto_relative_        = nh_.advertiseService("goto_relative_in", &ControlManager::callbackGotoRelative, this);
  service_server_goto_altitude_        = nh_.advertiseService("goto_altitude_in", &ControlManager::callbackGotoAltitude, this);
  service_server_set_heading_          = nh_.advertiseService("set_heading_in", &ControlManager::callbackSetHeading, this);
  service_server_set_heading_relative_ = nh_.advertiseService("set_heading_relative_in", &ControlManager::callbackSetHeadingRelative, this);

  service_server_reference_ = nh_.advertiseService("reference_in", &ControlManager::callbackReferenceService, this);
  sh_reference_             = mrs_lib::SubscribeHandler<mrs_msgs::ReferenceStamped>(shopts, "reference_in", &ControlManager::callbackReferenceTopic, this);

  service_server_velocity_reference_ = nh_.advertiseService("velocity_reference_in", &ControlManager::callbackVelocityReferenceService, this);
  sh_velocity_reference_ =
      mrs_lib::SubscribeHandler<mrs_msgs::VelocityReferenceStamped>(shopts, "velocity_reference_in", &ControlManager::callbackVelocityReferenceTopic, this);

  service_server_trajectory_reference_ = nh_.advertiseService("trajectory_reference_in", &ControlManager::callbackTrajectoryReferenceService, this);
  sh_trajectory_reference_ =
      mrs_lib::SubscribeHandler<mrs_msgs::TrajectoryReference>(shopts, "trajectory_reference_in", &ControlManager::callbackTrajectoryReferenceTopic, this);

  // | --------------------- other services --------------------- |

  service_server_emergency_reference_ = nh_.advertiseService("emergency_reference_in", &ControlManager::callbackEmergencyReference, this);
  service_server_pirouette_           = nh_.advertiseService("pirouette_in", &ControlManager::callbackPirouette, this);

  // | ------------------------- timers ------------------------- |

  timer_status_    = nh_.createTimer(ros::Rate(_status_timer_rate_), &ControlManager::timerStatus, this);
  timer_safety_    = nh_.createTimer(ros::Rate(_safety_timer_rate_), &ControlManager::timerSafety, this);
  timer_bumper_    = nh_.createTimer(ros::Rate(1.0), &ControlManager::timerBumper, this);
  timer_eland_     = nh_.createTimer(ros::Rate(_elanding_timer_rate_), &ControlManager::timerEland, this, false, false);
  timer_failsafe_  = nh_.createTimer(ros::Rate(_failsafe_timer_rate_), &ControlManager::timerFailsafe, this, false, false);
  timer_pirouette_ = nh_.createTimer(ros::Rate(_pirouette_timer_rate_), &ControlManager::timerPirouette, this, false, false);
  timer_joystick_  = nh_.createTimer(ros::Rate(_joystick_timer_rate_), &ControlManager::timerJoystick, this);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ControlManager]: could not load all parameters!");
    ros::shutdown();
  }

  is_initialized_ = true;

  ROS_INFO("[ControlManager]: initialized");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerHwApiCapabilities() //{ */

void ControlManager::timerHwApiCapabilities(const ros::TimerEvent& event) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerHwApiCapabilities", _status_timer_rate_, 1.0, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::timerHwApiCapabilities", scope_timer_logger_, scope_timer_enabled_);

  if (!sh_hw_api_capabilities_.hasMsg()) {
    ROS_INFO_THROTTLE(1.0, "[ControlManager]: waiting for HW API capabilities");
    return;
  }

  auto hw_ap_capabilities = sh_hw_api_capabilities_.getMsg();

  ROS_INFO("[ControlManager]: got HW API capabilities, the possible control modes are:");

  if (hw_ap_capabilities->accepts_actuator_cmd) {
    ROS_INFO("[ControlManager]: - actuator command");
    _hw_api_inputs_.actuators = true;
  }

  if (hw_ap_capabilities->accepts_control_group_cmd) {
    ROS_INFO("[ControlManager]: - control group command");
    _hw_api_inputs_.control_group = true;
  }

  if (hw_ap_capabilities->accepts_attitude_rate_cmd) {
    ROS_INFO("[ControlManager]: - attitude rate command");
    _hw_api_inputs_.attitude_rate = true;
  }

  if (hw_ap_capabilities->accepts_attitude_cmd) {
    ROS_INFO("[ControlManager]: - attitude command");
    _hw_api_inputs_.attitude = true;
  }

  if (hw_ap_capabilities->accepts_acceleration_hdg_rate_cmd) {
    ROS_INFO("[ControlManager]: - acceleration+hdg rate command");
    _hw_api_inputs_.acceleration_hdg_rate = true;
  }

  if (hw_ap_capabilities->accepts_acceleration_hdg_cmd) {
    ROS_INFO("[ControlManager]: - acceleration+hdg command");
    _hw_api_inputs_.acceleration_hdg = true;
  }

  if (hw_ap_capabilities->accepts_velocity_hdg_rate_cmd) {
    ROS_INFO("[ControlManager]: - velocityhdg rate command");
    _hw_api_inputs_.velocity_hdg_rate = true;
  }

  if (hw_ap_capabilities->accepts_velocity_hdg_cmd) {
    ROS_INFO("[ControlManager]: - velocityhdg command");
    _hw_api_inputs_.velocity_hdg = true;
  }

  if (hw_ap_capabilities->accepts_position_cmd) {
    ROS_INFO("[ControlManager]: - position command");
    _hw_api_inputs_.position = true;
  }

  initialize();

  timer_hw_api_capabilities_.stop();
}

//}

/* //{ timerStatus() */

void ControlManager::timerStatus(const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerStatus", _status_timer_rate_, 0.1, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::timerStatus", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto uav_state             = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_control_output   = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);
  auto yaw_error             = mrs_lib::get_mutexed(mutex_attitude_error_, yaw_error_);
  auto position_error        = mrs_lib::get_mutexed(mutex_position_error_, position_error_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);
  auto active_tracker_idx    = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);

  double uav_x, uav_y, uav_z;
  uav_x = uav_state.pose.position.x;
  uav_y = uav_state.pose.position.y;
  uav_z = uav_state.pose.position.z;

  // --------------------------------------------------------------
  // |                      print the status                      |
  // --------------------------------------------------------------

  {
    std::string controller = _controller_names_.at(active_controller_idx);
    std::string tracker    = _tracker_names_.at(active_tracker_idx);
    double      mass       = last_control_output.diagnostics.total_mass;
    double      bx_b       = last_control_output.diagnostics.disturbance_bx_b;
    double      by_b       = last_control_output.diagnostics.disturbance_by_b;
    double      wx_w       = last_control_output.diagnostics.disturbance_wx_w;
    double      wy_w       = last_control_output.diagnostics.disturbance_wy_w;

    ROS_INFO_THROTTLE(5.0, "[ControlManager]: tracker: '%s', controller: '%s', mass: '%.2f kg', disturbances: body [%.2f, %.2f] N, world [%.2f, %.2f] N",
                      tracker.c_str(), controller.c_str(), mass, bx_b, by_b, wx_w, wy_w);
  }

  // --------------------------------------------------------------
  // |                   publish the diagnostics                  |
  // --------------------------------------------------------------

  publishDiagnostics();

  // --------------------------------------------------------------
  // |                publish if the offboard is on               |
  // --------------------------------------------------------------

  if (offboard_mode_) {

    std_msgs::Empty offboard_on_out;

    ph_offboard_on_.publish(offboard_on_out);
  }

  // --------------------------------------------------------------
  // |                   publish the tilt error                   |
  // --------------------------------------------------------------
  {
    std::scoped_lock lock(mutex_attitude_error_);

    if (tilt_error_) {

      mrs_msgs::Float64Stamped tilt_error_out;
      tilt_error_out.header.stamp    = ros::Time::now();
      tilt_error_out.header.frame_id = uav_state.header.frame_id;
      tilt_error_out.value           = (180.0 / M_PI) * tilt_error_.value();

      ph_tilt_error_.publish(tilt_error_out);
    }
  }

  // --------------------------------------------------------------
  // |                  publish the control error                 |
  // --------------------------------------------------------------

  if (position_error) {

    Eigen::Vector3d pos_error_value = position_error.value();

    mrs_msgs::ControlError msg_out;

    msg_out.header.stamp    = ros::Time::now();
    msg_out.header.frame_id = uav_state.header.frame_id;

    msg_out.position_errors.x    = pos_error_value(0);
    msg_out.position_errors.y    = pos_error_value(1);
    msg_out.position_errors.z    = pos_error_value(2);
    msg_out.total_position_error = pos_error_value.norm();

    if (yaw_error_) {
      msg_out.yaw_error = yaw_error.value();
    }

    std::map<std::string, ControllerParams>::iterator it;

    it = controllers_.find(_controller_names_.at(active_controller_idx));

    msg_out.position_eland_threshold    = it->second.eland_threshold;
    msg_out.position_failsafe_threshold = it->second.failsafe_threshold;

    ph_control_error_.publish(msg_out);
  }

  // --------------------------------------------------------------
  // |                  publish the mass estimate                 |
  // --------------------------------------------------------------

  if (last_control_output.diagnostics.mass_estimator) {

    std_msgs::Float64 mass_estimate_out;
    mass_estimate_out.data = _uav_mass_ + last_control_output.diagnostics.mass_difference;

    ph_mass_estimate_.publish(mass_estimate_out);
  }

  // --------------------------------------------------------------
  // |                 publish the current heading                |
  // --------------------------------------------------------------

  if (_state_input_ == INPUT_UAV_STATE && sh_uav_state_.hasMsg()) {

    try {

      double heading;

      heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();

      mrs_msgs::Float64Stamped heading_out;
      heading_out.header = uav_state.header;
      heading_out.value  = heading;

      ph_heading_.publish(heading_out);
    }
    catch (...) {
      ROS_ERROR_THROTTLE(1.0, "exception caught, could not transform heading");
    }
  }

  // --------------------------------------------------------------
  // |                  publish the current speed                 |
  // --------------------------------------------------------------

  if (_state_input_ == INPUT_UAV_STATE && sh_uav_state_.hasMsg()) {

    double speed = sqrt(pow(uav_state.velocity.linear.x, 2) + pow(uav_state.velocity.linear.y, 2) + pow(uav_state.velocity.linear.z, 2));

    mrs_msgs::Float64Stamped speed_out;
    speed_out.header = uav_state.header;
    speed_out.value  = speed;

    ph_speed_.publish(speed_out);
  }

  // --------------------------------------------------------------
  // |               publish the safety area markers              |
  // --------------------------------------------------------------

  if (use_safety_area_) {

    mrs_msgs::ReferenceStamped temp_ref;
    temp_ref.header.frame_id = _safety_area_horizontal_frame_;

    geometry_msgs::TransformStamped tf;

    auto ret = transformer_->getTransform(_safety_area_horizontal_frame_, "local_origin", ros::Time(0));

    if (ret) {

      ROS_INFO_ONCE("[ControlManager]: got TFs, publishing safety area markers");

      visualization_msgs::MarkerArray safety_area_marker_array;
      visualization_msgs::MarkerArray safety_area_coordinates_marker_array;

      mrs_lib::Polygon border = safety_zone_->getBorder();

      std::vector<geometry_msgs::Point> border_points_bot_original = border.getPointMessageVector(getMinZ(_safety_area_horizontal_frame_));
      std::vector<geometry_msgs::Point> border_points_top_original = border.getPointMessageVector(getMaxZ(_safety_area_horizontal_frame_));

      std::vector<geometry_msgs::Point> border_points_bot_transformed = border_points_bot_original;
      std::vector<geometry_msgs::Point> border_points_top_transformed = border_points_bot_original;

      // if we fail in transforming the area at some point
      // do not publish it at all
      bool tf_success = true;

      geometry_msgs::TransformStamped tf = ret.value();

      /* transform area points to local origin //{ */

      // transform border bottom points to local origin
      for (size_t i = 0; i < border_points_bot_original.size(); i++) {

        temp_ref.header.frame_id      = _safety_area_horizontal_frame_;
        temp_ref.header.stamp         = ros::Time(0);
        temp_ref.reference.position.x = border_points_bot_original.at(i).x;
        temp_ref.reference.position.y = border_points_bot_original.at(i).y;
        temp_ref.reference.position.z = border_points_bot_original.at(i).z;

        if (auto ret = transformer_->transform(temp_ref, tf)) {

          temp_ref = ret.value();

          border_points_bot_transformed.at(i).x = temp_ref.reference.position.x;
          border_points_bot_transformed.at(i).y = temp_ref.reference.position.y;
          border_points_bot_transformed.at(i).z = temp_ref.reference.position.z;

        } else {
          tf_success = false;
        }
      }

      // transform border top points to local origin
      for (size_t i = 0; i < border_points_top_original.size(); i++) {

        temp_ref.header.frame_id      = _safety_area_horizontal_frame_;
        temp_ref.header.stamp         = ros::Time(0);
        temp_ref.reference.position.x = border_points_top_original.at(i).x;
        temp_ref.reference.position.y = border_points_top_original.at(i).y;
        temp_ref.reference.position.z = border_points_top_original.at(i).z;

        if (auto ret = transformer_->transform(temp_ref, tf)) {

          temp_ref = ret.value();

          border_points_top_transformed.at(i).x = temp_ref.reference.position.x;
          border_points_top_transformed.at(i).y = temp_ref.reference.position.y;
          border_points_top_transformed.at(i).z = temp_ref.reference.position.z;

        } else {
          tf_success = false;
        }
      }

      //}

      visualization_msgs::Marker safety_area_marker;

      safety_area_marker.header.frame_id = _uav_name_ + "/local_origin";
      safety_area_marker.type            = visualization_msgs::Marker::LINE_LIST;
      safety_area_marker.color.a         = 0.15;
      safety_area_marker.scale.x         = 0.2;
      safety_area_marker.color.r         = 1;
      safety_area_marker.color.g         = 0;
      safety_area_marker.color.b         = 0;

      safety_area_marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

      visualization_msgs::Marker safety_area_coordinates_marker;

      safety_area_coordinates_marker.header.frame_id = _uav_name_ + "/local_origin";
      safety_area_coordinates_marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
      safety_area_coordinates_marker.color.a         = 1;
      safety_area_coordinates_marker.scale.z         = 1.0;
      safety_area_coordinates_marker.color.r         = 0;
      safety_area_coordinates_marker.color.g         = 0;
      safety_area_coordinates_marker.color.b         = 0;

      safety_area_coordinates_marker.id = 0;

      safety_area_coordinates_marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

      /* adding safety area points //{ */

      // bottom border
      for (size_t i = 0; i < border_points_bot_transformed.size(); i++) {

        safety_area_marker.points.push_back(border_points_bot_transformed.at(i));
        safety_area_marker.points.push_back(border_points_bot_transformed.at((i + 1) % border_points_bot_transformed.size()));

        std::stringstream ss;

        if (_safety_area_horizontal_frame_ == "latlon_origin") {
          ss << "idx: " << i << std::endl
             << std::setprecision(6) << std::fixed << "lat: " << border_points_bot_original.at(i).x << std::endl
             << "lon: " << border_points_bot_original.at(i).y;
        } else {
          ss << "idx: " << i << std::endl
             << std::setprecision(1) << std::fixed << "x: " << border_points_bot_original.at(i).x << std::endl
             << "y: " << border_points_bot_original.at(i).y;
        }

        safety_area_coordinates_marker.color.r = 0;
        safety_area_coordinates_marker.color.g = 0;
        safety_area_coordinates_marker.color.b = 0;

        safety_area_coordinates_marker.pose.position = border_points_bot_transformed.at(i);
        safety_area_coordinates_marker.text          = ss.str();
        safety_area_coordinates_marker.id++;

        safety_area_coordinates_marker_array.markers.push_back(safety_area_coordinates_marker);
      }

      // top border + top/bot edges
      for (size_t i = 0; i < border_points_top_transformed.size(); i++) {

        safety_area_marker.points.push_back(border_points_top_transformed.at(i));
        safety_area_marker.points.push_back(border_points_top_transformed.at((i + 1) % border_points_top_transformed.size()));

        safety_area_marker.points.push_back(border_points_bot_transformed.at(i));
        safety_area_marker.points.push_back(border_points_top_transformed.at(i));

        std::stringstream ss;

        if (_safety_area_horizontal_frame_ == "latlon_origin") {
          ss << "idx: " << i << std::endl
             << std::setprecision(6) << std::fixed << "lat: " << border_points_bot_original.at(i).x << std::endl
             << "lon: " << border_points_bot_original.at(i).y;
        } else {
          ss << "idx: " << i << std::endl
             << std::setprecision(1) << std::fixed << "x: " << border_points_bot_original.at(i).x << std::endl
             << "y: " << border_points_bot_original.at(i).y;
        }

        safety_area_coordinates_marker.color.r = 1;
        safety_area_coordinates_marker.color.g = 1;
        safety_area_coordinates_marker.color.b = 1;

        safety_area_coordinates_marker.pose.position = border_points_top_transformed.at(i);
        safety_area_coordinates_marker.text          = ss.str();
        safety_area_coordinates_marker.id++;

        safety_area_coordinates_marker_array.markers.push_back(safety_area_coordinates_marker);
      }

      //}

      if (tf_success) {

        safety_area_marker_array.markers.push_back(safety_area_marker);

        ph_safety_area_markers_.publish(safety_area_marker_array);

        ph_safety_area_coordinates_markers_.publish(safety_area_coordinates_marker_array);
      }

    } else {
      ROS_WARN_ONCE("[ControlManager]: missing TFs, can not publish safety area markers");
    }
  }

  // --------------------------------------------------------------
  // |              publish the disturbances markers              |
  // --------------------------------------------------------------

  if (last_control_output.diagnostics.disturbance_estimator && got_uav_state_) {

    visualization_msgs::MarkerArray msg_out;

    double id = 0;

    double multiplier = 1.0;

    Eigen::Quaterniond quat_eigen = mrs_lib::AttitudeConverter(uav_state.pose.orientation);

    Eigen::Vector3d      vec3d;
    geometry_msgs::Point point;

    /* world disturbance //{ */
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

      marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

      //}

      /* origin //{ */
      point.x = uav_x;
      point.y = uav_y;
      point.z = uav_z;

      marker.points.push_back(point);

      //}

      /* tip //{ */

      point.x = uav_x + multiplier * last_control_output.diagnostics.disturbance_wx_w;
      point.y = uav_y + multiplier * last_control_output.diagnostics.disturbance_wy_w;
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

    /* body disturbance //{ */
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

      marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

      //}

      /* origin //{ */

      point.x = uav_x;
      point.y = uav_y;
      point.z = uav_z;

      marker.points.push_back(point);

      //}

      /* tip //{ */

      vec3d << multiplier * last_control_output.diagnostics.disturbance_bx_b, multiplier * last_control_output.diagnostics.disturbance_by_b, 0;
      vec3d = quat_eigen * vec3d;

      point.x = uav_x + vec3d(0);
      point.y = uav_y + vec3d(1);
      point.z = uav_z + vec3d(2);

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

    ph_disturbances_markers_.publish(msg_out);
  }

  // --------------------------------------------------------------
  // |               publish the current constraints              |
  // --------------------------------------------------------------

  if (got_constraints_) {

    auto sanitized_constraints = mrs_lib::get_mutexed(mutex_constraints_, sanitized_constraints_);

    mrs_msgs::DynamicsConstraints constraints = sanitized_constraints.constraints;

    ph_current_constraints_.publish(constraints);
  }
}

//}

/* //{ timerSafety() */

void ControlManager::timerSafety(const ros::TimerEvent& event) {

  mrs_lib::AtomicScopeFlag unset_running(running_safety_timer_);

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerSafety", _safety_timer_rate_, 0.05, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::timerSafety", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto last_control_output   = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);
  auto last_tracker_cmd      = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);
  auto [uav_state, uav_yaw]  = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_, uav_yaw_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);
  auto active_tracker_idx    = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);

  if (!got_uav_state_ || (_state_input_ == INPUT_UAV_STATE && _odometry_innovation_check_enabled_ && !sh_odometry_innovation_.hasMsg()) ||
      active_tracker_idx == _null_tracker_idx_) {
    return;
  }

  if (odometry_switch_in_progress_) {
    ROS_WARN("[ControlManager]: timerSafety tried to run while odometry switch in progress");
    return;
  }

  // | ------------------------ timeouts ------------------------ |

  if (_state_input_ == INPUT_UAV_STATE && sh_uav_state_.hasMsg()) {
    double missing_for = (ros::Time::now() - sh_uav_state_.lastMsgTime()).toSec();

    if (missing_for > _uav_state_max_missing_time_) {
      timeoutUavState(missing_for);
    }
  }

  if (_state_input_ == INPUT_ODOMETRY && sh_odometry_.hasMsg()) {
    double missing_for = (ros::Time::now() - sh_odometry_.lastMsgTime()).toSec();

    if (missing_for > _uav_state_max_missing_time_) {
      timeoutUavState(missing_for);
    }
  }

  // | -------------- eland and failsafe thresholds ------------- |

  std::map<std::string, ControllerParams>::iterator it;
  it = controllers_.find(_controller_names_.at(active_controller_idx));

  _eland_threshold_               = it->second.eland_threshold;
  _failsafe_threshold_            = it->second.failsafe_threshold;
  _odometry_innovation_threshold_ = it->second.odometry_innovation_threshold;

  // | --------- calculate control errors and tilt angle -------- |

  // This means that the timerFailsafe only does its work when Controllers and Trackers produce valid output.
  // Cases when the commands are not valid should be handle in updateControllers() and updateTrackers() methods.
  if (!last_tracker_cmd || !last_control_output.control_output) {
    return;
  }

  {
    std::scoped_lock lock(mutex_attitude_error_);

    tilt_error_ = 0;
    yaw_error_  = 0;
  }

  {
    Eigen::Vector3d position_error = Eigen::Vector3d::Zero();

    bool position_error_set = false;

    if (last_tracker_cmd->use_position_horizontal && !std::holds_alternative<mrs_msgs::HwApiPositionCmd>(last_control_output.control_output.value())) {

      position_error(0) = last_tracker_cmd->position.x - uav_state.pose.position.x;
      position_error(1) = last_tracker_cmd->position.y - uav_state.pose.position.y;

      position_error_set = true;
    }

    if (last_tracker_cmd->use_position_vertical && !std::holds_alternative<mrs_msgs::HwApiPositionCmd>(last_control_output.control_output.value())) {

      position_error(2) = last_tracker_cmd->position.z - uav_state.pose.position.z;

      position_error_set = true;
    }

    if (position_error_set) {

      mrs_lib::set_mutexed(mutex_position_error_, {position_error}, position_error_);
    }
  }

  // rotate the drone's z axis
  tf2::Transform uav_state_transform = mrs_lib::AttitudeConverter(uav_state.pose.orientation);
  tf2::Vector3   uav_z_in_world      = uav_state_transform * tf2::Vector3(0, 0, 1);

  // calculate the angle between the drone's z axis and the world's z axis
  double tilt_angle = acos(uav_z_in_world.dot(tf2::Vector3(0, 0, 1)));

  // | ------------ calculate the tilt and yaw error ------------ |

  // | --------------------- the tilt error --------------------- |

  if (last_control_output.desired_orientation) {

    // calculate the desired drone's z axis in the world frame
    tf2::Transform attitude_cmd_transform = mrs_lib::AttitudeConverter(last_control_output.desired_orientation.value());
    tf2::Vector3   uav_z_in_world_desired = attitude_cmd_transform * tf2::Vector3(0, 0, 1);

    {
      std::scoped_lock lock(mutex_attitude_error_);

      // calculate the angle between the drone's z axis and the world's z axis
      tilt_error_ = acos(uav_z_in_world.dot(uav_z_in_world_desired));

      // calculate the yaw error
      double cmd_yaw = mrs_lib::AttitudeConverter(last_control_output.desired_orientation.value()).getYaw();
      yaw_error_     = fabs(radians::diff(cmd_yaw, uav_yaw));
    }
  }

  auto position_error          = mrs_lib::get_mutexed(mutex_position_error_, position_error_);
  auto [tilt_error, yaw_error] = mrs_lib::get_mutexed(mutex_attitude_error_, tilt_error_, yaw_error_);

  // --------------------------------------------------------------
  // |   activate the failsafe controller in case of large error  |
  // --------------------------------------------------------------

  if (position_error) {

    if (position_error->norm() > _failsafe_threshold_ && !failsafe_triggered_) {

      auto controller_tracker_switch_time = mrs_lib::get_mutexed(mutex_controller_tracker_switch_time_, controller_tracker_switch_time_);

      if ((ros::Time::now() - controller_tracker_switch_time).toSec() > 1.0) {

        if (!failsafe_triggered_) {

          ROS_ERROR("[ControlManager]: activating failsafe land: control_error=%.2f/%.2f m (x: %.2f, y: %.2f, z: %.2f)", position_error->norm(),
                    _failsafe_threshold_, position_error.value()(0), position_error.value()(1), position_error.value()(2));

          failsafe();
        }
      }
    }
  }

  // --------------------------------------------------------------
  // |     activate emergency land in case of large innovation    |
  // --------------------------------------------------------------

  if (_odometry_innovation_check_enabled_) {

    std::optional<nav_msgs::Odometry::ConstPtr> innovation;

    if (sh_odometry_innovation_.hasMsg()) {
      innovation = {sh_odometry_innovation_.getMsg()};
    } else {
      ROS_WARN_THROTTLE(1.0, "[ControlManager]: missing estimator innnovation but the innovation check is enabled!");
    }

    if (innovation) {

      auto [x, y, z] = mrs_lib::getPosition(innovation.value());

      double heading = 0;
      try {
        heading = mrs_lib::getHeading(innovation.value());
      }
      catch (mrs_lib::AttitudeConverter::GetHeadingException& e) {
        ROS_ERROR_THROTTLE(1.0, "[ControlManager]: exception caught: '%s'", e.what());
      }

      double last_innovation = mrs_lib::geometry::dist(vec3_t(x, y, z), vec3_t(0, 0, 0));

      if (last_innovation > _odometry_innovation_threshold_ || radians::diff(heading, 0) > M_PI_2) {

        auto controller_tracker_switch_time = mrs_lib::get_mutexed(mutex_controller_tracker_switch_time_, controller_tracker_switch_time_);

        if ((ros::Time::now() - controller_tracker_switch_time).toSec() > 1.0) {

          if (!failsafe_triggered_ && !eland_triggered_) {

            ROS_ERROR("[ControlManager]: activating emergency land: odometry innovation too large: %.2f/%.2f (x: %.2f, y: %.2f, z: %.2f, heading: %.2f)",
                      last_innovation, _odometry_innovation_threshold_, x, y, z, heading);

            eland();
          }
        }
      }
    }
  }

  // --------------------------------------------------------------
  // |   activate emergency land in case of medium control error  |
  // --------------------------------------------------------------

  // | ------------------- tilt control error ------------------- |

  if (_tilt_limit_eland_enabled_ && tilt_angle > _tilt_limit_eland_) {

    auto controller_tracker_switch_time = mrs_lib::get_mutexed(mutex_controller_tracker_switch_time_, controller_tracker_switch_time_);

    if ((ros::Time::now() - controller_tracker_switch_time).toSec() > 1.0) {

      if (!failsafe_triggered_ && !eland_triggered_) {

        ROS_ERROR("[ControlManager]: activating emergency land: tilt angle too large (%.2f/%.2f deg)", (180.0 / M_PI) * tilt_angle,
                  (180.0 / M_PI) * _tilt_limit_eland_);

        eland();
      }
    }
  }

  // | ----------------- position control error ----------------- |

  if (position_error) {

    double error_size = position_error->norm();

    if (error_size > _eland_threshold_ / 2.0) {

      auto controller_tracker_switch_time = mrs_lib::get_mutexed(mutex_controller_tracker_switch_time_, controller_tracker_switch_time_);

      if ((ros::Time::now() - controller_tracker_switch_time).toSec() > 1.0) {

        if (!failsafe_triggered_ && !eland_triggered_) {

          ROS_DEBUG_THROTTLE(1.0, "[ControlManager]: releasing payload due to large position error");

          ungripSrv();
        }
      }
    }

    if (error_size > _eland_threshold_) {

      auto controller_tracker_switch_time = mrs_lib::get_mutexed(mutex_controller_tracker_switch_time_, controller_tracker_switch_time_);

      if ((ros::Time::now() - controller_tracker_switch_time).toSec() > 1.0) {

        if (!failsafe_triggered_ && !eland_triggered_) {

          ROS_ERROR("[ControlManager]: activating emergency land: position error %.2f/%.2f m (error x: %.2f, y: %.2f, z: %.2f)", error_size, _eland_threshold_,
                    position_error.value()(0), position_error.value()(1), position_error.value()(2));

          eland();
        }
      }
    }
  }

  // | -------------------- yaw control error ------------------- |
  // do not have to mutex the yaw_error_ here since I am filling it in this function

  if (_yaw_error_eland_enabled_ && yaw_error) {

    if (yaw_error.value() > (_yaw_error_eland_ / 2.0)) {

      auto controller_tracker_switch_time = mrs_lib::get_mutexed(mutex_controller_tracker_switch_time_, controller_tracker_switch_time_);

      if ((ros::Time::now() - controller_tracker_switch_time).toSec() > 1.0) {

        if (!failsafe_triggered_ && !eland_triggered_) {

          ROS_DEBUG_THROTTLE(1.0, "[ControlManager]: releasing payload: yaw error %.2f/%.2f deg", (180.0 / M_PI) * yaw_error.value(),
                             (180.0 / M_PI) * _yaw_error_eland_ / 2.0);

          ungripSrv();
        }
      }
    }

    if (yaw_error.value() > _yaw_error_eland_) {

      auto controller_tracker_switch_time = mrs_lib::get_mutexed(mutex_controller_tracker_switch_time_, controller_tracker_switch_time_);

      if ((ros::Time::now() - controller_tracker_switch_time).toSec() > 1.0) {

        if (!failsafe_triggered_ && !eland_triggered_) {

          ROS_ERROR("[ControlManager]: activating emergency land: yaw error %.2f/%.2f deg", (180.0 / M_PI) * yaw_error.value(),
                    (180.0 / M_PI) * _yaw_error_eland_);

          eland();
        }
      }
    }
  }

  // --------------------------------------------------------------
  // |      disarm the drone when the tilt exceeds the limit      |
  // --------------------------------------------------------------
  if (_tilt_limit_disarm_enabled_ && tilt_angle > _tilt_limit_disarm_) {

    ROS_ERROR("[ControlManager]: tilt angle too large, disarming: tilt angle=%.2f/%.2f deg", (180.0 / M_PI) * tilt_angle, (180.0 / M_PI) * _tilt_limit_disarm_);

    arming(false);
  }

  // --------------------------------------------------------------
  // |     disarm the drone when tilt error exceeds the limit     |
  // --------------------------------------------------------------

  if (_tilt_error_disarm_enabled_ && tilt_error) {

    auto controller_tracker_switch_time = mrs_lib::get_mutexed(mutex_controller_tracker_switch_time_, controller_tracker_switch_time_);

    // the time from the last controller/tracker switch
    // fyi: we should not
    double time_from_ctrl_tracker_switch = (ros::Time::now() - controller_tracker_switch_time).toSec();

    // if the tile error is over the threshold
    // && we are not ramping up during takeoff
    if (fabs(tilt_error.value()) > _tilt_error_disarm_threshold_ && !last_control_output.diagnostics.ramping_up) {

      // only account for the error if some time passed from the last tracker/controller switch
      if (time_from_ctrl_tracker_switch > 1.0) {

        // if the threshold was not exceeded before
        if (!tilt_error_disarm_over_thr_) {

          tilt_error_disarm_over_thr_ = true;
          tilt_error_disarm_time_     = ros::Time::now();

          ROS_WARN("[ControlManager]: tilt error exceeded threshold (%.2f/%.2f deg)", (180.0 / M_PI) * tilt_error.value(),
                   (180.0 / M_PI) * _tilt_error_disarm_threshold_);

          // if it was exceeded before, just keep it
        } else {

          ROS_WARN_THROTTLE(0.1, "[ControlManager]: tilt error (%.2f deg) over threshold for %.2f s", (180.0 / M_PI) * tilt_error.value(),
                            (ros::Time::now() - tilt_error_disarm_time_).toSec());
        }

        // if the tile error is bad, but the controller just switched,
        // don't think its bad anymore
      } else {

        tilt_error_disarm_over_thr_ = false;
        tilt_error_disarm_time_     = ros::Time::now();
      }

      // if the tilt error is fine
    } else {

      // make it fine
      tilt_error_disarm_over_thr_ = false;
      tilt_error_disarm_time_     = ros::Time::now();
    }

    // calculate the time over the threshold
    double tot = (ros::Time::now() - tilt_error_disarm_time_).toSec();

    // if the tot exceeds the limit (and if we are actually over the threshold)
    if (tilt_error_disarm_over_thr_ && (tot > _tilt_error_disarm_timeout_)) {

      bool is_flying = offboard_mode_ && active_tracker_idx != _null_tracker_idx_;

      // only when flying and not in failsafe
      if (is_flying) {

        ROS_ERROR("[ControlManager]: tilt error too large for %.2f s, disarming", tot);

        toggleOutput(false);
        arming(false);
      }
    }
  }

  // | --------- dropping out of OFFBOARD in mid flight --------- |

  // if we are not in offboard and the drone is in mid air (NullTracker is not active)
  if (offboard_mode_was_true_ && !offboard_mode_ && active_tracker_idx != _null_tracker_idx_) {

    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: we fell out of OFFBOARD in mid air, disabling output");

    toggleOutput(false);
  }
}  // namespace control_manager

//}

/* //{ timerEland() */

void ControlManager::timerEland(const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerEland", _elanding_timer_rate_, 0.01, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::timerEland", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto last_control_output = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);

  if (!last_control_output.control_output) {
    return;
  }

  auto throttle = extractThrottle(last_control_output);

  if (!throttle) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: TODO: implement landing detection mechanism for the current control modality");
    return;
  }

  if (current_state_landing_ == IDLE_STATE) {

    return;

  } else if (current_state_landing_ == LANDING_STATE) {

    // --------------------------------------------------------------
    // |                            TODO                            |
    // This section needs work. The throttle landing detection      |
    // mechanism should be extracted and other mechanisms, such     |
    // as velocity-based detection should be used for high          |
    // modalities                                                   |
    // --------------------------------------------------------------

    if (!last_control_output.control_output) {
      ROS_WARN_THROTTLE(1.0, "[ControlManager]: timerEland: last_control_output has not been initialized, returning");
      ROS_WARN_THROTTLE(1.0, "[ControlManager]: tip: the RC eland is probably triggered");
      return;
    }

    // recalculate the mass based on the throttle
    throttle_mass_estimate_ = mrs_lib::quadratic_throttle_model::throttleToForce(common_handlers_->throttle_model, throttle.value()) / common_handlers_->g;
    ROS_INFO_THROTTLE(1.0, "[ControlManager]: landing: initial mass: %.2f throttle_mass_estimate: %.2f", landing_uav_mass_, throttle_mass_estimate_);

    // condition for automatic motor turn off
    if (((throttle_mass_estimate_ < _elanding_cutoff_mass_factor_ * landing_uav_mass_) || throttle < 0.01)) {
      if (!throttle_under_threshold_) {

        throttle_mass_estimate_first_time_ = ros::Time::now();
        throttle_under_threshold_          = true;
      }

      ROS_INFO_THROTTLE(0.1, "[ControlManager]: throttle is under cutoff factor for %.2f s", (ros::Time::now() - throttle_mass_estimate_first_time_).toSec());

    } else {
      throttle_mass_estimate_first_time_ = ros::Time::now();
      throttle_under_threshold_          = false;
    }

    if (throttle_under_threshold_ && ((ros::Time::now() - throttle_mass_estimate_first_time_).toSec() > _elanding_cutoff_timeout_)) {
      // enable callbacks? ... NO

      ROS_INFO("[ControlManager]: reached cutoff throttle, disabling output");
      toggleOutput(false);

      // disarm the drone
      if (_eland_disarm_enabled_) {

        ROS_INFO("[ControlManager]: calling for disarm");
        arming(false);
      }

      changeLandingState(IDLE_STATE);

      ROS_WARN("[ControlManager]: emergency landing finished");

      ROS_DEBUG("[ControlManager]: stopping eland timer");
      timer_eland_.stop();
      ROS_DEBUG("[ControlManager]: eland timer stopped");

      // we should NOT set eland_triggered_=true
    }
  }
}

//}

/* //{ timerFailsafe() */

void ControlManager::timerFailsafe(const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[ControlManager]: timerFailsafe() spinning");

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerFailsafe", _failsafe_timer_rate_, 0.01, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::timerFailsafe", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  updateControllers(uav_state);

  publish();

  auto last_control_output = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);

  if (!last_control_output.control_output) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: timerFailsafe: the control output produced by the failsafe controller is empty!");
    return;
  }

  auto throttle = extractThrottle(last_control_output);

  if (!throttle) {
    ROS_DEBUG_THROTTLE(1.0, "[ControlManager]: FailsafeTimer: could not extract throttle out of the last control output");
    return;
  }

  // --------------------------------------------------------------
  // |                            TODO                            |
  // This section needs work. The throttle landing detection      |
  // mechanism should be extracted and other mechanisms, such     |
  // as velocity-based detection should be used for high          |
  // modalities                                                   |
  // --------------------------------------------------------------

  double throttle_mass_estimate_ = mrs_lib::quadratic_throttle_model::throttleToForce(common_handlers_->throttle_model, throttle.value()) / common_handlers_->g;
  ROS_INFO_THROTTLE(1.0, "[ControlManager]: failsafe: initial mass: %.2f throttle_mass_estimate: %.2f", landing_uav_mass_, throttle_mass_estimate_);

  // condition for automatic motor turn off
  if (((throttle_mass_estimate_ < _elanding_cutoff_mass_factor_ * landing_uav_mass_))) {

    if (!throttle_under_threshold_) {

      throttle_mass_estimate_first_time_ = ros::Time::now();
      throttle_under_threshold_          = true;
    }

    ROS_INFO_THROTTLE(0.1, "[ControlManager]: throttle is under cutoff factor for %.2f s", (ros::Time::now() - throttle_mass_estimate_first_time_).toSec());

  } else {

    throttle_mass_estimate_first_time_ = ros::Time::now();
    throttle_under_threshold_          = false;
  }

  // condition for automatic motor turn off
  if (throttle_under_threshold_ && ((ros::Time::now() - throttle_mass_estimate_first_time_).toSec() > _elanding_cutoff_timeout_)) {

    ROS_INFO_THROTTLE(1.0, "[ControlManager]: detecting zero throttle, disarming");

    arming(false);
  }
}

//}

/* //{ timerJoystick() */

void ControlManager::timerJoystick(const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerJoystick", _status_timer_rate_, 0.05, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::timerJoystick", scope_timer_logger_, scope_timer_enabled_);

  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  // if start was pressed and held for > 3.0 s
  if (joystick_start_pressed_ && joystick_start_press_time_ != ros::Time(0) && (ros::Time::now() - joystick_start_press_time_).toSec() > 3.0) {

    joystick_start_press_time_ = ros::Time(0);

    ROS_INFO("[ControlManager]: transitioning to joystick control: activating '%s' and '%s'", _joystick_tracker_name_.c_str(),
             _joystick_controller_name_.c_str());

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

    eland();
  }

  // if back was pressed and held for > 0.1 s
  if (joystick_back_pressed_ && joystick_back_press_time_ != ros::Time(0) && (ros::Time::now() - joystick_back_press_time_).toSec() > 0.1) {

    joystick_back_press_time_ = ros::Time(0);

    // activate/deactivate the joystick goto functionality
    joystick_goto_enabled_ = !joystick_goto_enabled_;

    ROS_INFO("[ControlManager]: joystick control %s", joystick_goto_enabled_ ? "activated" : "deactivated");
  }

  // if the GOTO functionality is enabled...
  if (joystick_goto_enabled_ && sh_joystick_.hasMsg()) {

    auto joystick_data = sh_joystick_.getMsg();

    // create the reference

    mrs_msgs::Vec4::Request request;

    if (fabs(joystick_data->axes.at(_channel_pitch_)) >= 0.05 || fabs(joystick_data->axes.at(_channel_roll_)) >= 0.05 ||
        fabs(joystick_data->axes.at(_channel_heading_)) >= 0.05 || fabs(joystick_data->axes.at(_channel_throttle_)) >= 0.05) {

      if (_joystick_mode_ == 0) {

        request.goal.at(REF_X)       = _channel_mult_pitch_ * joystick_data->axes.at(_channel_pitch_) * _joystick_carrot_distance_;
        request.goal.at(REF_Y)       = _channel_mult_roll_ * joystick_data->axes.at(_channel_roll_) * _joystick_carrot_distance_;
        request.goal.at(REF_Z)       = _channel_mult_throttle_ * joystick_data->axes.at(_channel_throttle_);
        request.goal.at(REF_HEADING) = _channel_mult_heading_ * joystick_data->axes.at(_channel_heading_);

        mrs_msgs::Vec4::Response response;

        callbackGotoFcu(request, response);

      } else if (_joystick_mode_ == 1) {

        mrs_msgs::TrajectoryReference trajectory;

        double dt = 0.2;

        trajectory.fly_now         = true;
        trajectory.header.frame_id = "fcu_untilted";
        trajectory.use_heading     = true;
        trajectory.dt              = dt;

        mrs_msgs::Reference point;
        point.position.x = 0;
        point.position.y = 0;
        point.position.z = 0;
        point.heading    = 0;

        trajectory.points.push_back(point);

        double speed = 1.0;

        for (int i = 0; i < 50; i++) {

          point.position.x += _channel_mult_pitch_ * joystick_data->axes.at(_channel_pitch_) * (speed * dt);
          point.position.y += _channel_mult_roll_ * joystick_data->axes.at(_channel_roll_) * (speed * dt);
          point.position.z += _channel_mult_throttle_ * joystick_data->axes.at(_channel_throttle_) * (speed * dt);
          point.heading = _channel_mult_heading_ * joystick_data->axes.at(_channel_heading_);

          trajectory.points.push_back(point);
        }

        setTrajectoryReference(trajectory);
      }
    }
  }

  if (rc_goto_active_ && last_tracker_cmd && sh_hw_api_rc_.hasMsg()) {

    // create the reference
    mrs_msgs::VelocityReferenceStampedSrv::Request request;

    double des_x       = 0;
    double des_y       = 0;
    double des_z       = 0;
    double des_heading = 0;

    bool nothing_to_do = true;

    // copy member variables
    mrs_msgs::HwApiRcChannelsConstPtr rc_channels = sh_hw_api_rc_.getMsg();

    if (rc_channels->channels.size() < 4) {

      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: RC control channel numbers are out of range (the # of channels in rc/in topic is %d)",
                         int(rc_channels->channels.size()));
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: tip: this could be caused by the RC failsafe not being configured!");

    } else {

      double tmp_x       = RCChannelToRange(rc_channels->channels.at(_rc_channel_pitch_), _rc_horizontal_speed_, 0.1);
      double tmp_y       = -RCChannelToRange(rc_channels->channels.at(_rc_channel_roll_), _rc_horizontal_speed_, 0.1);
      double tmp_z       = RCChannelToRange(rc_channels->channels.at(_rc_channel_throttle_), _rc_vertical_speed_, 0.3);
      double tmp_heading = -RCChannelToRange(rc_channels->channels.at(_rc_channel_heading_), _rc_heading_rate_, 0.1);

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

      if (abs(tmp_heading) > 1e-3) {
        des_heading   = tmp_heading;
        nothing_to_do = false;
      }
    }

    if (!nothing_to_do) {

      request.reference.header.frame_id = "fcu_untilted";

      request.reference.reference.use_heading_rate = true;

      request.reference.reference.velocity.x   = des_x;
      request.reference.reference.velocity.y   = des_y;
      request.reference.reference.velocity.z   = des_z;
      request.reference.reference.heading_rate = des_heading;

      mrs_msgs::VelocityReferenceStampedSrv::Response response;

      // disable callbacks of all trackers
      std_srvs::SetBoolRequest req_enable_callbacks;

      // enable the callbacks for the active tracker
      req_enable_callbacks.data = true;
      {
        std::scoped_lock lock(mutex_tracker_list_);

        tracker_list_.at(active_tracker_idx_)
            ->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
      }

      callbacks_enabled_ = true;

      callbackVelocityReferenceService(request, response);

      callbacks_enabled_ = false;

      ROS_INFO_THROTTLE(1.0, "[ControlManager]: goto by RC with speed x=%.2f, y=%.2f, z=%.2f, heading_rate=%.2f", des_x, des_y, des_z, des_heading);

      // disable the callbacks back again
      req_enable_callbacks.data = false;
      {
        std::scoped_lock lock(mutex_tracker_list_);

        tracker_list_.at(active_tracker_idx_)
            ->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
      }
    }
  }
}

//}

/* //{ timerBumper() */

void ControlManager::timerBumper(const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerBumper", _bumper_timer_rate_, 0.05, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::timerBumper", scope_timer_logger_, scope_timer_enabled_);

  // | ---------------------- preconditions --------------------- |

  if (!sh_bumper_.hasMsg()) {
    return;
  }

  if (!bumper_enabled_) {
    return;
  }

  if (!isFlyingNormally() && !bumper_repulsing_) {
    return;
  }

  if (!got_uav_state_) {
    return;
  }

  if (sh_bumper_.hasMsg() && (ros::Time::now() - sh_bumper_.lastMsgTime()).toSec() > 1.0) {

    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: obstacle bumper is missing messages although we got them before");

    return;
  }

  timer_bumper_.setPeriod(ros::Duration(1.0 / _bumper_timer_rate_));

  // | ------------------ call the bumper logic ----------------- |

  bumperPushFromObstacle();
}

//}

/* //{ timerPirouette() */

void ControlManager::timerPirouette(const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerPirouette", _pirouette_timer_rate_, 0.01, event);
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::timerPirouette", scope_timer_logger_, scope_timer_enabled_);

  pirouette_iterator_++;

  double pirouette_duration  = (2 * M_PI) / _pirouette_speed_;
  double pirouette_n_steps   = pirouette_duration * _pirouette_timer_rate_;
  double pirouette_step_size = (2 * M_PI) / pirouette_n_steps;

  if (rc_escalating_failsafe_triggered_ || failsafe_triggered_ || eland_triggered_ || (pirouette_iterator_ > pirouette_duration * _pirouette_timer_rate_)) {

    _pirouette_enabled_ = false;
    timer_pirouette_.stop();

    setCallbacks(true);

    return;
  }

  // set the reference
  mrs_msgs::ReferenceStamped reference_request;

  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  reference_request.header.frame_id      = "";
  reference_request.header.stamp         = ros::Time(0);
  reference_request.reference.position.x = last_tracker_cmd->position.x;
  reference_request.reference.position.y = last_tracker_cmd->position.y;
  reference_request.reference.position.z = last_tracker_cmd->position.z;
  reference_request.reference.heading    = pirouette_initial_heading_ + pirouette_iterator_ * pirouette_step_size;

  // enable the callbacks for the active tracker
  {
    std::scoped_lock lock(mutex_tracker_list_);

    std_srvs::SetBoolRequest req_enable_callbacks;
    req_enable_callbacks.data = true;

    tracker_list_.at(active_tracker_idx_)
        ->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));

    callbacks_enabled_ = true;
  }

  setReference(reference_request);

  {
    std::scoped_lock lock(mutex_tracker_list_);

    // disable the callbacks for the active tracker
    std_srvs::SetBoolRequest req_enable_callbacks;
    req_enable_callbacks.data = false;

    tracker_list_.at(active_tracker_idx_)
        ->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));

    callbacks_enabled_ = false;
  }
}

//}

// --------------------------------------------------------------
// |                           asyncs                           |
// --------------------------------------------------------------

/* asyncControl() //{ */

void ControlManager::asyncControl(void) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::AtomicScopeFlag unset_running(running_async_control_);

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("asyncControl");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::asyncControl", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto uav_state           = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto current_constraints = mrs_lib::get_mutexed(mutex_constraints_, current_constraints_);

  if (!failsafe_triggered_) {  // when failsafe is triggered, updateControllers() and publish() is called in timerFailsafe()

    // run the safety timer
    // in the case of large control errors, the safety mechanisms will be triggered before the controllers and trackers are updated...
    while (running_safety_timer_) {

      ROS_DEBUG("[ControlManager]: waiting for safety timer to finish");
      ros::Duration wait(0.001);
      wait.sleep();

      if (!running_safety_timer_) {
        ROS_DEBUG("[ControlManager]: safety timer finished");
        break;
      }
    }

    ros::TimerEvent safety_timer_event;
    timerSafety(safety_timer_event);

    updateTrackers();

    updateControllers(uav_state);

    if (got_constraints_) {

      // update the constraints to trackers, if need to
      auto enforce = enforceControllersConstraints(current_constraints);

      if (enforce && !constraints_being_enforced_) {

        setConstraintsToTrackers(enforce.value());
        mrs_lib::set_mutexed(mutex_constraints_, enforce.value(), sanitized_constraints_);

        constraints_being_enforced_ = true;

        auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);

        ROS_WARN_THROTTLE(1.0, "[ControlManager]: the controller '%s' is enforcing constraints over the ConstraintManager",
                          _controller_names_.at(active_controller_idx).c_str());

      } else if (!enforce && constraints_being_enforced_) {

        ROS_INFO_THROTTLE(1.0, "[ControlManager]: constraint values returned to original values");

        constraints_being_enforced_ = false;

        mrs_lib::set_mutexed(mutex_constraints_, current_constraints, sanitized_constraints_);

        setConstraintsToTrackers(current_constraints);
      }
    }

    publish();
  }

  // if odometry switch happened, we finish it here and turn the safety timer back on
  if (odometry_switch_in_progress_) {

    ROS_DEBUG("[ControlManager]: starting safety timer");
    timer_safety_.start();
    ROS_DEBUG("[ControlManager]: safety timer started");
    odometry_switch_in_progress_ = false;

    {
      std::scoped_lock lock(mutex_uav_state_);

      ROS_INFO("[ControlManager]: odometry after switch: x=%.2f, y=%.2f, z=%.2f, heading=%.2f", uav_state.pose.position.x, uav_state.pose.position.y,
               uav_state.pose.position.z, uav_heading_);
    }
  }
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | --------------------- topic callbacks -------------------- |

/* //{ callbackOdometry() */

void ControlManager::callbackOdometry(const nav_msgs::Odometry::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackOdometry");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackOdometry", scope_timer_logger_, scope_timer_enabled_);

  nav_msgs::OdometryConstPtr odom = msg;

  // | ------------------ check for time stamp ------------------ |

  {
    std::scoped_lock lock(mutex_uav_state_);

    if (uav_state_.header.stamp == odom->header.stamp) {
      return;
    }
  }

  // | --------------------- check for nans --------------------- |

  if (!validateOdometry(*odom, "ControlManager", "odometry")) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: incoming 'odometry' contains invalid values, throwing it away");
    return;
  }

  // | ---------------------- frame switch ---------------------- |

  /* Odometry frame switch //{ */

  // | -- prepare an OdometryConstPtr for trackers & controllers -- |

  mrs_msgs::UavState uav_state_odom;

  uav_state_odom.header   = odom->header;
  uav_state_odom.pose     = odom->pose.pose;
  uav_state_odom.velocity = odom->twist.twist;

  // | ----- check for change in odometry frame of reference ---- |

  if (got_uav_state_) {

    if (odom->header.frame_id != uav_state_.header.frame_id) {

      ROS_INFO("[ControlManager]: detecting switch of odometry frame");
      {
        std::scoped_lock lock(mutex_uav_state_);

        ROS_INFO("[ControlManager]: odometry before switch: x=%.2f, y=%.2f, z=%.2f, heading=%.2f", uav_state_.pose.position.x, uav_state_.pose.position.y,
                 uav_state_.pose.position.z, uav_heading_);
      }

      odometry_switch_in_progress_ = true;

      // we have to stop safety timer, otherwise it will interfere
      ROS_DEBUG("[ControlManager]: stopping the safety timer");
      timer_safety_.stop();
      ROS_DEBUG("[ControlManager]: safety timer stopped");

      // wait for the safety timer to stop if its running
      while (running_safety_timer_) {

        ROS_DEBUG("[ControlManager]: waiting for safety timer to finish");
        ros::Duration wait(0.001);
        wait.sleep();

        if (!running_safety_timer_) {
          ROS_DEBUG("[ControlManager]: safety timer finished");
          break;
        }
      }

      // we have to also for the oneshot control timer to finish
      while (running_async_control_) {

        ROS_DEBUG("[ControlManager]: waiting for control timer to finish");
        ros::Duration wait(0.001);
        wait.sleep();

        if (!running_async_control_) {
          ROS_DEBUG("[ControlManager]: control timer finished");
          break;
        }
      }

      {
        std::scoped_lock lock(mutex_controller_list_, mutex_tracker_list_);

        tracker_list_.at(active_tracker_idx_)->switchOdometrySource(uav_state_odom);
        controller_list_.at(active_controller_idx_)->switchOdometrySource(uav_state_odom);
      }
    }
  }

  //}

  // | ----------- copy the odometry to the uav_state ----------- |

  {
    std::scoped_lock lock(mutex_uav_state_);

    previous_uav_state_ = uav_state_;

    uav_state_ = mrs_msgs::UavState();

    uav_state_.header           = odom->header;
    uav_state_.pose             = odom->pose.pose;
    uav_state_.velocity.angular = odom->twist.twist.angular;

    // transform the twist into the header's frame
    {
      // the velocity from the odometry
      geometry_msgs::Vector3Stamped speed_child_frame;
      speed_child_frame.header.frame_id = odom->child_frame_id;
      speed_child_frame.header.stamp    = odom->header.stamp;
      speed_child_frame.vector.x        = odom->twist.twist.linear.x;
      speed_child_frame.vector.y        = odom->twist.twist.linear.y;
      speed_child_frame.vector.z        = odom->twist.twist.linear.z;

      auto res = transformer_->transformSingle(speed_child_frame, odom->header.frame_id);

      if (res) {
        uav_state_.velocity.linear.x = res.value().vector.x;
        uav_state_.velocity.linear.y = res.value().vector.y;
        uav_state_.velocity.linear.z = res.value().vector.z;
      } else {
        ROS_ERROR_THROTTLE(1.0, "[ControlManager]: could not transform the odometry speed from '%s' to '%s'", odom->child_frame_id.c_str(),
                           odom->header.frame_id.c_str());
        return;
      }
    }

    // calculate the euler angles
    std::tie(uav_roll_, uav_pitch_, uav_yaw_) = mrs_lib::AttitudeConverter(odom->pose.pose.orientation);

    try {
      uav_heading_ = mrs_lib::AttitudeConverter(odom->pose.pose.orientation).getHeading();
    }
    catch (...) {
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: could not calculate UAV heading");
    }

    transformer_->setDefaultFrame(odom->header.frame_id);

    got_uav_state_ = true;
  }

  // run the control loop asynchronously in an OneShotTimer
  // but only if its not already running
  if (!running_async_control_) {

    running_async_control_ = true;

    async_control_result_ = std::async(std::launch::async, &ControlManager::asyncControl, this);
  }
}

//}

/* //{ callbackUavState() */

void ControlManager::callbackUavState(const mrs_msgs::UavState::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackUavState");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackUavState", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::UavStateConstPtr uav_state = msg;

  // | ------------------ check for time stamp ------------------ |

  {
    std::scoped_lock lock(mutex_uav_state_);

    if (uav_state_.header.stamp == uav_state->header.stamp) {
      return;
    }
  }

  // | --------------------- check for nans --------------------- |

  if (!validateUavState(*uav_state, "ControlManager", "uav_state")) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: incoming 'uav_state' contains invalid values, throwing it away");
    return;
  }

  // | -------------------- check for hiccups ------------------- |

  /* hickup detection //{ */

  double alpha               = 0.99;
  double alpha2              = 0.666;
  double uav_state_count_lim = 1000;

  double uav_state_dt = (ros::Time::now() - previous_uav_state_.header.stamp).toSec();

  // belive only reasonable numbers
  if (uav_state_dt <= 1.0) {

    uav_state_avg_dt_ = alpha * uav_state_avg_dt_ + (1 - alpha) * uav_state_dt;

    if (uav_state_count_ < uav_state_count_lim) {
      uav_state_count_++;
    }
  }

  if (uav_state_count_ == uav_state_count_lim) {

    /* ROS_INFO_STREAM("[ControlManager]: uav_state_dt = " << uav_state_dt); */

    if (uav_state_dt < uav_state_avg_dt_ && uav_state_dt > 0.0001) {

      uav_state_hiccup_factor_ = alpha2 * uav_state_hiccup_factor_ + (1 - alpha2) * (uav_state_avg_dt_ / uav_state_dt);

    } else if (uav_state_avg_dt_ > 0.0001) {

      uav_state_hiccup_factor_ = alpha2 * uav_state_hiccup_factor_ + (1 - alpha2) * (uav_state_dt / uav_state_avg_dt_);
    }

    if (uav_state_hiccup_factor_ > 3.141592653) {

      /* ROS_ERROR_STREAM_THROTTLE(0.1, "[ControlManager]: hiccup factor = " << uav_state_hiccup_factor_); */

      ROS_WARN_THROTTLE(2.0, "[ControlManager]: ");
      ROS_WARN_THROTTLE(2.0, "[ControlManager]: // | ------------------------- WARNING ------------------------ |");
      ROS_WARN_THROTTLE(2.0, "[ControlManager]: // |                                                            |");
      ROS_WARN_THROTTLE(2.0, "[ControlManager]: // |            UAV_STATE has a large hiccup factor!            |");
      ROS_WARN_THROTTLE(2.0, "[ControlManager]: // |           hint, hint: you are probably rosbagging          |");
      ROS_WARN_THROTTLE(2.0, "[ControlManager]: // |           lot of data or publishing lot of large           |");
      ROS_WARN_THROTTLE(2.0, "[ControlManager]: // |          messages without mutual nodelet managers.         |");
      ROS_WARN_THROTTLE(2.0, "[ControlManager]: // |                                                            |");
      ROS_WARN_THROTTLE(2.0, "[ControlManager]: // | ------------------------- WARNING ------------------------ |");
      ROS_WARN_THROTTLE(2.0, "[ControlManager]: ");
    }
  }

  //}

  // | ---------------------- frame switch ---------------------- |

  /* frame switch //{ */

  // | ----- check for change in odometry frame of reference ---- |

  if (got_uav_state_) {

    if (uav_state->estimator_iteration != uav_state_.estimator_iteration) {

      ROS_INFO("[ControlManager]: detecting switch of odometry frame");
      {
        std::scoped_lock lock(mutex_uav_state_);

        ROS_INFO("[ControlManager]: odometry before switch: x=%.2f, y=%.2f, z=%.2f, heading=%.2f", uav_state_.pose.position.x, uav_state_.pose.position.y,
                 uav_state_.pose.position.z, uav_heading_);
      }

      odometry_switch_in_progress_ = true;

      // we have to stop safety timer, otherwise it will interfere
      ROS_DEBUG("[ControlManager]: stopping the safety timer");
      timer_safety_.stop();
      ROS_DEBUG("[ControlManager]: safety timer stopped");

      // wait for the safety timer to stop if its running
      while (running_safety_timer_) {

        ROS_DEBUG("[ControlManager]: waiting for safety timer to finish");
        ros::Duration wait(0.001);
        wait.sleep();

        if (!running_safety_timer_) {
          ROS_DEBUG("[ControlManager]: safety timer finished");
          break;
        }
      }

      // we have to also for the oneshot control timer to finish
      while (running_async_control_) {

        ROS_DEBUG("[ControlManager]: waiting for control timer to finish");
        ros::Duration wait(0.001);
        wait.sleep();

        if (!running_async_control_) {
          ROS_DEBUG("[ControlManager]: control timer finished");
          break;
        }
      }

      {
        std::scoped_lock lock(mutex_controller_list_, mutex_tracker_list_);

        tracker_list_.at(active_tracker_idx_)->switchOdometrySource(*uav_state);
        controller_list_.at(active_controller_idx_)->switchOdometrySource(*uav_state);
      }
    }
  }

  //}

  // --------------------------------------------------------------
  // |           copy the UavState message for later use          |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_uav_state_);

    previous_uav_state_ = uav_state_;

    uav_state_ = *uav_state;

    std::tie(uav_roll_, uav_pitch_, uav_yaw_) = mrs_lib::AttitudeConverter(uav_state_.pose.orientation);

    try {
      uav_heading_ = mrs_lib::AttitudeConverter(uav_state_.pose.orientation).getHeading();
    }
    catch (...) {
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: could not calculate UAV heading, not updating it");
    }

    transformer_->setDefaultFrame(uav_state->header.frame_id);

    got_uav_state_ = true;
  }

  // run the control loop asynchronously in an OneShotTimer
  // but only if its not already running
  if (!running_async_control_) {

    running_async_control_ = true;

    async_control_result_ = std::async(std::launch::async, &ControlManager::asyncControl, this);
  }
}

//}

/* //{ callbackGNSS() */

void ControlManager::callbackGNSS(const sensor_msgs::NavSatFix::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackGNSS");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackGNSS", scope_timer_logger_, scope_timer_enabled_);

  transformer_->setLatLon(msg->latitude, msg->longitude);
}

//}

/* callbackJoystick() //{ */

void ControlManager::callbackJoystick(const sensor_msgs::Joy::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackJoystick");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackJoystick", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto active_tracker_idx    = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);

  sensor_msgs::JoyConstPtr joystick_data = msg;

  // TODO check if the array is smaller than the largest idx
  if (joystick_data->buttons.size() == 0 || joystick_data->axes.size() == 0) {
    return;
  }

  // | ---- switching back to fallback tracker and controller --- |

  // if any of the A, B, X, Y buttons are pressed when flying with joystick, switch back to fallback controller and tracker
  if ((joystick_data->buttons.at(_channel_A_) == 1 || joystick_data->buttons.at(_channel_B_) == 1 || joystick_data->buttons.at(_channel_X_) == 1 ||
       joystick_data->buttons.at(_channel_Y_) == 1) &&
      active_tracker_idx == _joystick_tracker_idx_ && active_controller_idx == _joystick_controller_idx_) {

    ROS_INFO("[ControlManager]: switching from joystick to normal control");

    switchTracker(_joystick_fallback_tracker_name_);
    switchController(_joystick_fallback_controller_name_);

    joystick_goto_enabled_ = false;
  }

  // | ------- joystick control activation ------- |

  // if start button was pressed
  if (joystick_data->buttons.at(_channel_start_) == 1) {

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
  if (joystick_data->buttons.at(_channel_back_) == 1) {

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
  if (joystick_data->axes.at(_channel_LT_) < -0.99 && joystick_data->axes.at(_channel_RT_) < -0.99) {

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
  if (joystick_data->buttons.at(_channel_L_joy_) == 1 && joystick_data->buttons.at(_channel_R_joy_) == 1) {

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

/* //{ callbackHwApiStatus() */

void ControlManager::callbackHwApiStatus(const mrs_msgs::HwApiStatus::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackHwApiStatus");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackHwApiStatus", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::HwApiStatusConstPtr state = msg;

  // | ------ detect and print the changes in offboard mode ----- |
  if (state->offboard) {

    if (!offboard_mode_) {
      offboard_mode_          = true;
      offboard_mode_was_true_ = true;
      ROS_INFO("[ControlManager]: detected: OFFBOARD mode ON");
    }

  } else {

    if (offboard_mode_) {
      offboard_mode_ = false;
      ROS_INFO("[ControlManager]: detected: OFFBOARD mode OFF");
    }
  }

  // | --------- detect and print the changes in arming --------- |
  if (state->armed == true) {

    if (!armed_) {
      armed_ = true;
      ROS_INFO("[ControlManager]: detected: vehicle ARMED");
    }

  } else {

    if (armed_) {
      armed_ = false;
      ROS_INFO("[ControlManager]: detected: vehicle DISARMED");
    }
  }
}

//}

/* //{ callbackRC() */

void ControlManager::callbackRC(const mrs_msgs::HwApiRcChannels::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackRC");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackRC", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::HwApiRcChannelsConstPtr rc = msg;

  ROS_INFO_ONCE("[ControlManager]: getting RC channels");

  // | ------------------- rc joystic control ------------------- |

  // when the switch change its position
  if (_rc_goto_enabled_) {

    if (_rc_joystick_channel_ >= int(rc->channels.size())) {

      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: RC joystick activation channel number (%d) is out of range [0-%d]", _rc_joystick_channel_,
                         int(rc->channels.size()));

    } else {

      bool channel_low  = rc->channels.at(_rc_joystick_channel_) < (0.5 - RC_DEADBAND) ? true : false;
      bool channel_high = rc->channels.at(_rc_joystick_channel_) > (0.5 + RC_DEADBAND) ? true : false;

      if (channel_low) {
        rc_joystick_channel_was_low_ = true;
      }

      // rc control activation
      if (!rc_goto_active_) {

        if (rc_joystick_channel_last_value_ < (0.5 - RC_DEADBAND) && channel_high) {

          if (isFlyingNormally()) {

            ROS_INFO_THROTTLE(1.0, "[ControlManager]: activating RC joystick");

            callbacks_enabled_ = false;

            std_srvs::SetBoolRequest req_goto_out;
            req_goto_out.data = false;

            std_srvs::SetBoolRequest req_enable_callbacks;
            req_enable_callbacks.data = callbacks_enabled_;

            {
              std::scoped_lock lock(mutex_tracker_list_);

              // disable callbacks of all trackers
              for (int i = 0; i < int(tracker_list_.size()); i++) {
                tracker_list_.at(i)->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
              }
            }

            rc_goto_active_ = true;

          } else {

            ROS_WARN_THROTTLE(1.0, "[ControlManager]: can not activate RC joystick, not flying normally");
          }

        } else if (channel_high && !rc_joystick_channel_was_low_) {

          ROS_WARN_THROTTLE(1.0, "[ControlManager]: can not activate RC joystick, the switch is ON from the beginning");
        }
      }

      // rc control deactivation
      if (rc_goto_active_ && channel_low) {

        ROS_INFO("[ControlManager]: deactivating RC joystick");

        callbacks_enabled_ = true;

        std_srvs::SetBoolRequest req_goto_out;
        req_goto_out.data = true;

        std_srvs::SetBoolRequest req_enable_callbacks;
        req_enable_callbacks.data = callbacks_enabled_;

        {
          std::scoped_lock lock(mutex_tracker_list_);

          // enable callbacks of all trackers
          for (int i = 0; i < int(tracker_list_.size()); i++) {
            tracker_list_.at(i)->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
          }
        }

        rc_goto_active_ = false;
      }

      // do not forget to update the last... variable
      // only do that if its out of the deadband
      if (channel_high || channel_low) {
        rc_joystick_channel_last_value_ = rc->channels.at(_rc_joystick_channel_);
      }
    }
  }

  // | ----------------- RC escalating failsafe ----------------- |

  if (_rc_escalating_failsafe_enabled_) {

    if (_rc_escalating_failsafe_channel_ >= int(rc->channels.size())) {

      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: RC eland channel number (%d) is out of range [0-%d]", _rc_escalating_failsafe_channel_,
                         int(rc->channels.size()));

    } else {

      if (rc->channels.at(_rc_escalating_failsafe_channel_) >= _rc_escalating_failsafe_threshold_) {

        ROS_WARN_THROTTLE(1.0, "[ControlManager]: triggering escalating failsafe by RC");

        auto [success, message] = escalatingFailsafe();

        if (success) {
          rc_escalating_failsafe_triggered_ = true;
        }
      }
    }
  }
}

//}

// | --------------------- topic timeouts --------------------- |

/* timeoutUavState() //{ */

void ControlManager::timeoutUavState(const double& missing_for) {

  auto last_control_output = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);

  if (output_enabled_ && last_control_output.control_output && !failsafe_triggered_) {

    // We need to fire up timerFailsafe, which will regularly trigger the controllers
    // in place of the callbackUavState/callbackOdometry().

    ROS_ERROR_THROTTLE(0.1, "[ControlManager]: not receiving uav_state/odometry for %.3f s, initiating failsafe land", missing_for);

    failsafe();
  }
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackSwitchTracker() */

bool ControlManager::callbackSwitchTracker(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  if (failsafe_triggered_ || eland_triggered_) {

    std::stringstream ss;
    ss << "can not switch tracker, eland or failsafe active";

    res.message = ss.str();
    res.success = false;

    ROS_WARN_STREAM("[ControlManager]: " << ss.str());

    return true;
  }

  if (rc_goto_active_) {

    std::stringstream ss;
    ss << "can not switch tracker, RC joystick is active";

    res.message = ss.str();
    res.success = false;

    ROS_WARN_STREAM("[ControlManager]: " << ss.str());

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

  if (!is_initialized_) {
    return false;
  }

  if (failsafe_triggered_ || eland_triggered_) {

    std::stringstream ss;
    ss << "can not switch controller, eland or failsafe active";

    res.message = ss.str();
    res.success = false;

    ROS_WARN_STREAM("[ControlManager]: " << ss.str());

    return true;
  }

  if (rc_goto_active_) {

    std::stringstream ss;
    ss << "can not switch controller, RC joystick is active";

    res.message = ss.str();
    res.success = false;

    ROS_WARN_STREAM("[ControlManager]: " << ss.str());

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

  if (!is_initialized_) {
    return false;
  }

  std::stringstream message;

  if (failsafe_triggered_ || eland_triggered_) {

    message << "can not reset tracker, eland or failsafe active";

    res.message = message.str();
    res.success = false;

    ROS_WARN_STREAM("[ControlManager]: " << message.str());

    return true;
  }

  // reactivate the current tracker
  {
    std::scoped_lock lock(mutex_tracker_list_);

    std::string tracker_name = _tracker_names_.at(active_tracker_idx_);

    bool succ = tracker_list_.at(active_tracker_idx_)->resetStatic();

    if (succ) {
      message << "the tracker '" << tracker_name << "' was reset";
      ROS_INFO_STREAM("[ControlManager]: " << message.str());
    } else {
      message << "the tracker '" << tracker_name << "' reset failed!";
      ROS_ERROR_STREAM("[ControlManager]: " << message.str());
    }
  }

  res.message = message.str();
  res.success = true;

  return true;
}

//}

/* //{ callbackEHover() */

bool ControlManager::callbackEHover([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  if (failsafe_triggered_ || eland_triggered_) {

    std::stringstream ss;
    ss << "can not switch controller, eland or failsafe active";

    res.message = ss.str();
    res.success = false;

    ROS_WARN_STREAM("[ControlManager]: " << ss.str());

    return true;
  }

  ROS_WARN_THROTTLE(1.0, "[ControlManager]: ehover trigger by callback");

  auto [success, message] = ehover();

  res.success = success;
  res.message = message;

  return true;
}

//}

/* callbackFailsafe() //{ */

bool ControlManager::callbackFailsafe([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  if (failsafe_triggered_) {

    std::stringstream ss;
    ss << "can not activate failsafe, it is already active";

    res.message = ss.str();
    res.success = false;

    ROS_INFO_STREAM("[ControlManager]: " << ss.str());

    return true;
  }

  ROS_WARN_THROTTLE(1.0, "[ControlManager]: failsafe triggered by callback");

  auto [success, message] = failsafe();

  res.success = success;
  res.message = message;

  return true;
}

//}

/* callbackFailsafeEscalating() //{ */

bool ControlManager::callbackFailsafeEscalating([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  if (_service_escalating_failsafe_enabled_) {

    ROS_WARN_THROTTLE(1.0, "[ControlManager]: escalating failsafe triggered by callback");

    auto [success, message] = escalatingFailsafe();

    res.success = success;
    res.message = message;

  } else {

    std::stringstream ss;
    ss << "escalating failsafe is disabled";

    res.success = false;
    res.message = ss.str();

    ROS_WARN_THROTTLE(1.0, "[ControlManager]: %s", ss.str().c_str());
  }

  return true;
}

//}

/* //{ callbackELand() */

bool ControlManager::callbackEland([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  ROS_WARN_THROTTLE(1.0, "[ControlManager]: eland triggered by callback");

  auto [success, message] = eland();

  res.success = success;
  res.message = message;

  return true;
}

//}

/* //{ callbackParachute() */

bool ControlManager::callbackParachute([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  if (!_parachute_enabled_) {

    std::stringstream ss;
    ss << "parachute disabled";
    ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    res.message = ss.str();
    res.success = false;
  }

  ROS_WARN_THROTTLE(1.0, "[ControlManager]: parachute triggered by callback");

  auto [success, message] = deployParachute();

  res.success = success;
  res.message = message;

  return true;
}

//}

/* //{ callbackSetMinZ() */

bool ControlManager::callbackSetMinZ([[maybe_unused]] mrs_msgs::Float64StampedSrv::Request& req, mrs_msgs::Float64StampedSrv::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  if (!use_safety_area_) {
    res.success = true;
    res.message = "safety area is disabled";
    return true;
  }

  // | -------- transform min_z to the safety area frame -------- |

  mrs_msgs::ReferenceStamped point;
  point.header               = req.header;
  point.reference.position.z = req.value;

  auto result = transformer_->transformSingle(point, _safety_area_vertical_frame_);

  if (result) {

    _safety_area_min_z_ = result.value().reference.position.z;

    res.success = true;
    res.message = "safety area's min z updated";

  } else {

    res.success = false;
    res.message = "could not transform the value to safety area's vertical frame";
  }

  return true;
}

//}

/* //{ callbackToggleOutput() */

bool ControlManager::callbackToggleOutput(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  ROS_INFO("[ControlManager]: toggling output by service");

  // copy member variables
  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  std::stringstream ss;

  bool prereq_check = true;

  {
    mrs_msgs::ReferenceStamped current_coord;
    current_coord.header.frame_id      = uav_state.header.frame_id;
    current_coord.reference.position.x = uav_state.pose.position.x;
    current_coord.reference.position.y = uav_state.pose.position.y;

    if (!isPointInSafetyArea2d(current_coord)) {
      ss << "cannot toggle output, the UAV is outside of the safety area!";
      prereq_check = false;
    }
  }

  if (req.data && (failsafe_triggered_ || eland_triggered_ || rc_escalating_failsafe_triggered_)) {
    ss << "cannot toggle output ON, we landed in emergency";
    prereq_check = false;
  }

  if (!sh_hw_api_status_.hasMsg() || (ros::Time::now() - sh_hw_api_status_.lastMsgTime()).toSec() > 1.0) {
    ss << "cannot toggle output ON, missing HW API status!";
    prereq_check = false;
  }

  if (!prereq_check) {

    res.message = ss.str();
    res.success = false;

    ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());

    return false;

  } else {

    toggleOutput(req.data);

    ss << "Output: " << (output_enabled_ ? "ON" : "OFF");
    res.message = ss.str();
    res.success = true;

    ROS_INFO_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());

    publishDiagnostics();

    return true;
  }
}

//}

/* callbackArm() //{ */

bool ControlManager::callbackArm(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  ROS_INFO("[ControlManager]: arming by service");

  std::stringstream ss;

  if (failsafe_triggered_ || eland_triggered_) {

    ss << "can not " << (req.data ? "arm" : "disarm") << ", eland or failsafe active";

    res.message = ss.str();
    res.success = false;

    ROS_ERROR_STREAM("[ControlManager]: " << ss.str());

    return true;
  }

  if (req.data) {

    ss << "this service is not allowed to arm the UAV";
    res.success = false;
    ROS_ERROR_STREAM("[ControlManager]: " << ss.str());

  } else {

    auto [success, message] = arming(false);

    if (success) {

      ss << "disarmed";
      res.success = true;
      ROS_INFO_STREAM("[ControlManager]: " << ss.str());

    } else {

      ss << "could not disarm: " << message;
      res.success = false;
      ROS_ERROR_STREAM("[ControlManager]: " << ss.str());
    }
  }

  res.message = ss.str();

  return true;
}

//}

/* //{ callbackEnableCallbacks() */

bool ControlManager::callbackEnableCallbacks(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  setCallbacks(req.data);

  std::stringstream ss;

  ss << "callbacks " << (callbacks_enabled_ ? "enabled" : "disabled");

  res.message = ss.str();
  res.success = true;

  ROS_INFO_STREAM("[ControlManager]: " << ss.str());

  return true;
}

//}

/* callbackSetConstraints() //{ */

bool ControlManager::callbackSetConstraints(mrs_msgs::DynamicsConstraintsSrv::Request& req, mrs_msgs::DynamicsConstraintsSrv::Response& res) {

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  {
    std::scoped_lock lock(mutex_constraints_);

    current_constraints_ = req;

    auto enforced = enforceControllersConstraints(current_constraints_);

    if (enforced) {
      sanitized_constraints_      = enforced.value();
      constraints_being_enforced_ = true;
    } else {
      sanitized_constraints_      = req;
      constraints_being_enforced_ = false;
    }

    got_constraints_ = true;

    setConstraintsToControllers(current_constraints_);
    setConstraintsToTrackers(sanitized_constraints_);
  }

  res.message = "setting constraints";
  res.success = true;

  return true;
}

//}

/* //{ callbackEmergencyReference() */

bool ControlManager::callbackEmergencyReference(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  callbacks_enabled_ = false;

  mrs_msgs::ReferenceSrvResponse::ConstPtr tracker_response;

  std::stringstream ss;

  // transform the reference to the current frame
  mrs_msgs::ReferenceStamped original_reference;
  original_reference.header    = req.header;
  original_reference.reference = req.reference;

  auto ret = transformer_->transformSingle(original_reference, uav_state.header.frame_id);

  if (!ret) {

    ss << "the emergency reference could not be transformed";

    ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    res.message = ss.str();
    res.success = false;
    return true;
  }

  mrs_msgs::ReferenceStamped transformed_reference = ret.value();

  std_srvs::SetBoolRequest req_enable_callbacks;

  mrs_msgs::ReferenceSrvRequest req_goto_out;
  req_goto_out.reference = transformed_reference.reference;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    // disable callbacks of all trackers
    req_enable_callbacks.data = false;
    for (int i = 0; i < int(tracker_list_.size()); i++) {
      tracker_list_.at(i)->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
    }

    // enable the callbacks for the active tracker
    req_enable_callbacks.data = true;
    tracker_list_.at(active_tracker_idx_)
        ->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));

    // call the setReference()
    tracker_response = tracker_list_.at(active_tracker_idx_)
                           ->setReference(mrs_msgs::ReferenceSrvRequest::ConstPtr(std::make_unique<mrs_msgs::ReferenceSrvRequest>(req_goto_out)));

    // disable the callbacks back again
    req_enable_callbacks.data = false;
    tracker_list_.at(active_tracker_idx_)
        ->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));

    if (tracker_response != mrs_msgs::ReferenceSrvResponse::Ptr()) {
      res.message = tracker_response->message;
      res.success = tracker_response->success;
    } else {
      ss << "the tracker '" << _tracker_names_.at(active_tracker_idx_) << "' does not implement the 'setReference()' function!";
      res.message = ss.str();
      res.success = false;
    }
  }

  return true;
}

//}

/* callbackPirouette() //{ */

bool ControlManager::callbackPirouette([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  // copy member variables
  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  double uav_heading;
  try {
    uav_heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (...) {
    std::stringstream ss;
    ss << "could not calculate the UAV heading to initialize the pirouette";

    ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());

    res.message = ss.str();
    res.success = false;

    return false;
  }

  if (_pirouette_enabled_) {
    res.success = false;
    res.message = "already active";
    return true;
  }

  if (failsafe_triggered_ || eland_triggered_ || rc_escalating_failsafe_triggered_) {

    std::stringstream ss;
    ss << "can not activate the pirouette, eland or failsafe active";

    res.message = ss.str();
    res.success = false;

    ROS_ERROR_STREAM("[ControlManager]: " << ss.str());

    return true;
  }

  _pirouette_enabled_ = true;

  setCallbacks(false);

  pirouette_initial_heading_ = uav_heading;
  pirouette_iterator_        = 0;
  timer_pirouette_.start();

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

  std::stringstream ss;

  {
    auto [success, response] = switchTracker(_joystick_tracker_name_);

    if (!success) {

      ss << "switching to '" << _joystick_tracker_name_ << "' was unsuccessfull: '" << response << "'";
      ROS_ERROR_STREAM("[ControlManager]: " << ss.str());

      res.success = false;
      res.message = ss.str();

      return true;
    }
  }

  auto [success, response] = switchController(_joystick_controller_name_);

  if (!success) {

    ss << "switching to '" << _joystick_controller_name_ << "' was unsuccessfull: '" << response << "'";
    ROS_ERROR_STREAM("[ControlManager]: " << ss.str());

    res.success = false;
    res.message = ss.str();

    // switch back to hover tracker
    switchTracker(_ehover_tracker_name_);

    // switch back to safety controller
    switchController(_eland_controller_name_);

    ROS_ERROR_STREAM("[ControlManager]: " << ss.str());

    return true;
  }

  ss << "switched to joystick control";

  res.success = true;
  res.message = ss.str();

  ROS_INFO_STREAM("[ControlManager]: " << ss.str());

  return true;
}

//}

/* //{ callbackHover() */

bool ControlManager::callbackHover([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  auto [success, message] = hover();

  res.success = success;
  res.message = message;

  return true;
}

//}

/* //{ callbackStartTrajectoryTracking() */

bool ControlManager::callbackStartTrajectoryTracking([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  auto [success, message] = startTrajectoryTracking();

  res.success = success;
  res.message = message;

  return true;
}

//}

/* //{ callbackStopTrajectoryTracking() */

bool ControlManager::callbackStopTrajectoryTracking([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  auto [success, message] = stopTrajectoryTracking();

  res.success = success;
  res.message = message;

  return true;
}

//}

/* //{ callbackResumeTrajectoryTracking() */

bool ControlManager::callbackResumeTrajectoryTracking([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  auto [success, message] = resumeTrajectoryTracking();

  res.success = success;
  res.message = message;

  return true;
}

//}

/* //{ callbackGotoTrajectoryStart() */

bool ControlManager::callbackGotoTrajectoryStart([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  auto [success, message] = gotoTrajectoryStart();

  res.success = success;
  res.message = message;

  return true;
}

//}

/* //{ callbackTransformReference() */

bool ControlManager::callbackTransformReference(mrs_msgs::TransformReferenceSrv::Request& req, mrs_msgs::TransformReferenceSrv::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  // transform the reference to the current frame
  mrs_msgs::ReferenceStamped transformed_reference = req.reference;

  if (auto ret = transformer_->transformSingle(transformed_reference, req.frame_id)) {

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

/* //{ callbackTransformReferenceList() */

bool ControlManager::callbackTransformReferenceList(mrs_msgs::TransformReferenceListSrv::Request& req, mrs_msgs::TransformReferenceListSrv::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  // transform the reference list to the current frame
  mrs_msgs::ReferenceStamped transformed_reference; 
  mrs_msgs::ReferenceList original_reference_list = req.list; 
  mrs_msgs::ReferenceList transformed_reference_list;

  for (size_t i=0; i < original_reference_list.list.size(); i++) {

    transformed_reference.header = original_reference_list.header;
    transformed_reference.reference = original_reference_list.list.at(i);

    if (auto ret = transformer_->transformSingle(transformed_reference, req.frame_id)) {

      transformed_reference_list.list.push_back(ret.value().reference);
      transformed_reference_list.header = ret.value().header;

    } else {

      res.message = "The reference list could not be transformed";
      res.success = false;
      return true;
    }

  }

  res.list = transformed_reference_list;
  res.message   = "transformation successful";
  res.success   = true;
  return true;
}

//}

/* //{ callbackTransformPose() */

bool ControlManager::callbackTransformPose(mrs_msgs::TransformPoseSrv::Request& req, mrs_msgs::TransformPoseSrv::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  // transform the reference to the current frame
  geometry_msgs::PoseStamped transformed_pose = req.pose;

  if (auto ret = transformer_->transformSingle(transformed_pose, req.frame_id)) {

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

  if (!is_initialized_) {
    return false;
  }

  // transform the reference to the current frame
  geometry_msgs::Vector3Stamped transformed_vector3 = req.vector;

  if (auto ret = transformer_->transformSingle(transformed_vector3, req.frame_id)) {

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

/* //{ callbackEnableBumper() */

bool ControlManager::callbackEnableBumper(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  bumper_enabled_ = req.data;

  std::stringstream ss;

  ss << "bumper " << (bumper_enabled_ ? "enalbed" : "disabled");

  ROS_INFO_STREAM("[ControlManager]: " << ss.str());

  res.success = true;
  res.message = ss.str();

  return true;
}

//}

/* //{ callbackUseSafetyArea() */

bool ControlManager::callbackUseSafetyArea(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  use_safety_area_ = req.data;

  std::stringstream ss;

  ss << "safety area " << (use_safety_area_ ? "enabled" : "disabled");

  ROS_INFO_STREAM("[ControlManager]: " << ss.str());

  res.success = true;
  res.message = ss.str();

  return true;
}

//}

/* //{ callbackGetMinZ() */

bool ControlManager::callbackGetMinZ([[maybe_unused]] mrs_msgs::GetFloat64::Request& req, mrs_msgs::GetFloat64::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  res.success = true;
  res.value   = getMinZ(uav_state.header.frame_id);

  return true;
}

//}

/* //{ callbackValidateReference() */

bool ControlManager::callbackValidateReference(mrs_msgs::ValidateReference::Request& req, mrs_msgs::ValidateReference::Response& res) {

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  if (!validateReference(req.reference.reference, "ControlManager", "reference_for_validation")) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: NaN detected in variable 'req.reference'!!!");
    res.message = "NaNs/infs in input!";
    res.success = false;
    return true;
  }

  // copy member variables
  auto uav_state        = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  // transform the reference to the current frame
  mrs_msgs::ReferenceStamped original_reference;
  original_reference.header    = req.reference.header;
  original_reference.reference = req.reference.reference;

  auto ret = transformer_->transformSingle(original_reference, uav_state.header.frame_id);

  if (!ret) {

    ROS_WARN_THROTTLE(1.0, "[ControlManager]: the reference could not be transformed");
    res.message = "the reference could not be transformed";
    res.success = false;
    return true;
  }

  mrs_msgs::ReferenceStamped transformed_reference = ret.value();

  if (!isPointInSafetyArea3d(transformed_reference)) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: reference validation failed, the point is outside of the safety area!");
    res.message = "the point is outside of the safety area";
    res.success = false;
    return true;
  }

  if (last_tracker_cmd) {

    mrs_msgs::ReferenceStamped from_point;
    from_point.header.frame_id      = uav_state.header.frame_id;
    from_point.reference.position.x = last_tracker_cmd->position.x;
    from_point.reference.position.y = last_tracker_cmd->position.y;
    from_point.reference.position.z = last_tracker_cmd->position.z;

    if (!isPathToPointInSafetyArea3d(from_point, transformed_reference)) {
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: reference validation failed, the path is going outside the safety area!");
      res.message = "the path is going outside the safety area";
      res.success = false;
      return true;
    }
  }

  res.message = "the reference is ok";
  res.success = true;
  return true;
}

//}

/* //{ callbackValidateReference2d() */

bool ControlManager::callbackValidateReference2d(mrs_msgs::ValidateReference::Request& req, mrs_msgs::ValidateReference::Response& res) {

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  if (!validateReference(req.reference.reference, "ControlManager", "reference_for_validation")) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: NaN detected in variable 'req.reference'!!!");
    res.message = "NaNs/infs in input!";
    res.success = false;
    return true;
  }

  // copy member variables
  auto uav_state        = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  // transform the reference to the current frame
  mrs_msgs::ReferenceStamped original_reference;
  original_reference.header    = req.reference.header;
  original_reference.reference = req.reference.reference;

  auto ret = transformer_->transformSingle(original_reference, uav_state.header.frame_id);

  if (!ret) {

    ROS_WARN_THROTTLE(1.0, "[ControlManager]: the reference could not be transformed");
    res.message = "the reference could not be transformed";
    res.success = false;
    return true;
  }

  mrs_msgs::ReferenceStamped transformed_reference = ret.value();

  if (!isPointInSafetyArea2d(transformed_reference)) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: reference validation failed, the point is outside of the safety area!");
    res.message = "the point is outside of the safety area";
    res.success = false;
    return true;
  }

  if (last_tracker_cmd) {

    mrs_msgs::ReferenceStamped from_point;
    from_point.header.frame_id      = uav_state.header.frame_id;
    from_point.reference.position.x = last_tracker_cmd->position.x;
    from_point.reference.position.y = last_tracker_cmd->position.y;
    from_point.reference.position.z = last_tracker_cmd->position.z;

    if (!isPathToPointInSafetyArea2d(from_point, transformed_reference)) {
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: reference validation failed, the path is going outside the safety area!");
      res.message = "the path is going outside the safety area";
      res.success = false;
      return true;
    }
  }

  res.message = "the reference is ok";
  res.success = true;
  return true;
}

//}

/* //{ callbackValidateReferenceList() */

bool ControlManager::callbackValidateReferenceList(mrs_msgs::ValidateReferenceList::Request& req, mrs_msgs::ValidateReferenceList::Response& res) {

  if (!is_initialized_) {
    res.message = "not initialized";
    return false;
  }

  // copy member variables
  auto uav_state        = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  // get the transformer
  auto ret = transformer_->getTransform(uav_state.header.frame_id, req.list.header.frame_id, req.list.header.stamp);

  if (!ret) {

    ROS_DEBUG("[ControlManager]: could not find transform for the reference");
    res.message = "could not find transform";
    return false;
  }

  geometry_msgs::TransformStamped tf = ret.value();

  for (int i = 0; i < int(req.list.list.size()); i++) {

    res.success.push_back(true);

    mrs_msgs::ReferenceStamped original_reference;
    original_reference.header    = req.list.header;
    original_reference.reference = req.list.list.at(i);

    res.success.at(i) = validateReference(original_reference.reference, "ControlManager", "reference_list");

    auto ret = transformer_->transformSingle(original_reference, uav_state.header.frame_id);

    if (!ret) {

      ROS_DEBUG("[ControlManager]: the reference could not be transformed");
      res.success.at(i) = false;
    }

    mrs_msgs::ReferenceStamped transformed_reference = ret.value();

    if (!isPointInSafetyArea3d(transformed_reference)) {
      res.success.at(i) = false;
    }

    if (last_tracker_cmd) {

      mrs_msgs::ReferenceStamped from_point;
      from_point.header.frame_id      = uav_state.header.frame_id;
      from_point.reference.position.x = last_tracker_cmd->position.x;
      from_point.reference.position.y = last_tracker_cmd->position.y;
      from_point.reference.position.z = last_tracker_cmd->position.z;

      if (!isPathToPointInSafetyArea3d(from_point, transformed_reference)) {
        res.success.at(i) = false;
      }
    }
  }

  res.message = "references were checked";
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

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackReferenceService");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackReferenceService", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::ReferenceStamped des_reference;
  des_reference.header    = req.header;
  des_reference.reference = req.reference;

  auto [success, message] = setReference(des_reference);

  res.success = success;
  res.message = message;

  return true;
}

//}

/* //{ callbackReferenceTopic() */

void ControlManager::callbackReferenceTopic(const mrs_msgs::ReferenceStamped::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackReferenceTopic");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackReferenceTopic", scope_timer_logger_, scope_timer_enabled_);

  setReference(*msg);
}

//}

/* //{ callbackVelocityReferenceService() */

bool ControlManager::callbackVelocityReferenceService(mrs_msgs::VelocityReferenceStampedSrv::Request&  req,
                                                      mrs_msgs::VelocityReferenceStampedSrv::Response& res) {

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackVelocityReferenceService");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackVelocityReferenceService", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::VelocityReferenceStamped des_reference;
  des_reference = req.reference;

  auto [success, message] = setVelocityReference(des_reference);

  res.success = success;
  res.message = message;

  return true;
}

//}

/* //{ callbackVelocityReferenceTopic() */

void ControlManager::callbackVelocityReferenceTopic(const mrs_msgs::VelocityReferenceStamped::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackVelocityReferenceTopic");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackVelocityReferenceTopic", scope_timer_logger_, scope_timer_enabled_);

  setVelocityReference(*msg);
}

//}

/* //{ callbackTrajectoryReferenceService() */

bool ControlManager::callbackTrajectoryReferenceService(mrs_msgs::TrajectoryReferenceSrv::Request& req, mrs_msgs::TrajectoryReferenceSrv::Response& res) {

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackTrajectoryReferenceService");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackTrajectoryReferenceService", scope_timer_logger_, scope_timer_enabled_);

  auto [success, message, modified, tracker_names, tracker_successes, tracker_messages] = setTrajectoryReference(req.trajectory);

  res.success          = success;
  res.message          = message;
  res.modified         = modified;
  res.tracker_names    = tracker_names;
  res.tracker_messages = tracker_messages;

  for (size_t i = 0; i < tracker_successes.size(); i++) {
    res.tracker_successes.push_back(tracker_successes.at(i));
  }

  return true;
}

//}

/* //{ callbackTrajectoryReferenceTopic() */

void ControlManager::callbackTrajectoryReferenceTopic(const mrs_msgs::TrajectoryReference::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackTrajectoryReferenceTopic");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackTrajectoryReferenceTopic", scope_timer_logger_, scope_timer_enabled_);

  setTrajectoryReference(*msg);
}

//}

// | ------------- human-callable "goto" services ------------- |

/* //{ callbackGoto() */

bool ControlManager::callbackGoto(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackGoto");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackGoto", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::ReferenceStamped des_reference;
  des_reference.header.frame_id      = "";
  des_reference.header.stamp         = ros::Time(0);
  des_reference.reference.position.x = req.goal.at(REF_X);
  des_reference.reference.position.y = req.goal.at(REF_Y);
  des_reference.reference.position.z = req.goal.at(REF_Z);
  des_reference.reference.heading    = req.goal.at(REF_HEADING);

  auto [success, message] = setReference(des_reference);

  res.success = success;
  res.message = message;

  return true;
}

//}

/* //{ callbackGotoFcu() */

bool ControlManager::callbackGotoFcu(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackGotoFcu");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackGotoFcu", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::ReferenceStamped des_reference;
  des_reference.header.frame_id      = "fcu_untilted";
  des_reference.header.stamp         = ros::Time(0);
  des_reference.reference.position.x = req.goal.at(REF_X);
  des_reference.reference.position.y = req.goal.at(REF_Y);
  des_reference.reference.position.z = req.goal.at(REF_Z);
  des_reference.reference.heading    = req.goal.at(REF_HEADING);

  auto [success, message] = setReference(des_reference);

  res.success = success;
  res.message = message;

  return true;
}

//}

/* //{ callbackGotoRelative() */

bool ControlManager::callbackGotoRelative(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackGotoRelative");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackGotoRelative", scope_timer_logger_, scope_timer_enabled_);

  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  if (!last_tracker_cmd) {
    res.message = "not flying";
    res.success = false;
    return true;
  }

  mrs_msgs::ReferenceStamped des_reference;
  des_reference.header.frame_id      = "";
  des_reference.header.stamp         = ros::Time(0);
  des_reference.reference.position.x = last_tracker_cmd->position.x + req.goal.at(REF_X);
  des_reference.reference.position.y = last_tracker_cmd->position.y + req.goal.at(REF_Y);
  des_reference.reference.position.z = last_tracker_cmd->position.z + req.goal.at(REF_Z);
  des_reference.reference.heading    = last_tracker_cmd->heading + req.goal.at(REF_HEADING);

  auto [success, message] = setReference(des_reference);

  res.success = success;
  res.message = message;

  return true;
}

//}

/* //{ callbackGotoAltitude() */

bool ControlManager::callbackGotoAltitude(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res) {

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackGotoAltitude");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackGotoAltitude", scope_timer_logger_, scope_timer_enabled_);

  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  if (!last_tracker_cmd) {
    res.message = "not flying";
    res.success = false;
    return true;
  }

  mrs_msgs::ReferenceStamped des_reference;
  des_reference.header.frame_id      = "";
  des_reference.header.stamp         = ros::Time(0);
  des_reference.reference.position.x = last_tracker_cmd->position.x;
  des_reference.reference.position.y = last_tracker_cmd->position.y;
  des_reference.reference.position.z = req.goal;
  des_reference.reference.heading    = last_tracker_cmd->heading;

  auto [success, message] = setReference(des_reference);

  res.success = success;
  res.message = message;

  return true;
}

//}

/* //{ callbackSetHeading() */

bool ControlManager::callbackSetHeading(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res) {

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackSetHeading");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackSetHeading", scope_timer_logger_, scope_timer_enabled_);

  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  if (!last_tracker_cmd) {
    res.message = "not flying";
    res.success = false;
    return true;
  }

  mrs_msgs::ReferenceStamped des_reference;
  des_reference.header.frame_id      = "";
  des_reference.header.stamp         = ros::Time(0);
  des_reference.reference.position.x = last_tracker_cmd->position.x;
  des_reference.reference.position.y = last_tracker_cmd->position.y;
  des_reference.reference.position.z = last_tracker_cmd->position.z;
  des_reference.reference.heading    = req.goal;

  auto [success, message] = setReference(des_reference);

  res.success = success;
  res.message = message;

  return true;
}

//}

/* //{ callbackSetHeadingRelative() */

bool ControlManager::callbackSetHeadingRelative(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res) {

  if (!is_initialized_) {
    res.message = "not initialized";
    res.success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackSetHeadingRelative");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::callbackSetHeadingRelative", scope_timer_logger_, scope_timer_enabled_);

  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  if (!last_tracker_cmd) {
    res.message = "not flying";
    res.success = false;
    return true;
  }

  mrs_msgs::ReferenceStamped des_reference;
  des_reference.header.frame_id      = "";
  des_reference.header.stamp         = ros::Time(0);
  des_reference.reference.position.x = last_tracker_cmd->position.x;
  des_reference.reference.position.y = last_tracker_cmd->position.y;
  des_reference.reference.position.z = last_tracker_cmd->position.z;
  des_reference.reference.heading    = last_tracker_cmd->heading + req.goal;

  auto [success, message] = setReference(des_reference);

  res.success = success;
  res.message = message;

  return true;
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* setReference() //{ */

std::tuple<bool, std::string> ControlManager::setReference(const mrs_msgs::ReferenceStamped reference_in) {

  std::stringstream ss;

  if (!callbacks_enabled_) {
    ss << "can not set the reference, the callbacks are disabled";
    ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  if (!validateReference(reference_in.reference, "ControlManager", "reference")) {
    ss << "incoming reference is not finite!!!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  // copy member variables
  auto uav_state        = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  // transform the reference to the current frame
  auto ret = transformer_->transformSingle(reference_in, uav_state.header.frame_id);

  if (!ret) {

    ss << "the reference could not be transformed";
    ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  mrs_msgs::ReferenceStamped transformed_reference = ret.value();

  // safety area check
  if (!isPointInSafetyArea3d(transformed_reference)) {
    ss << "failed to set the reference, the point is outside of the safety area!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  if (last_tracker_cmd) {

    mrs_msgs::ReferenceStamped from_point;
    from_point.header.frame_id      = uav_state.header.frame_id;
    from_point.reference.position.x = last_tracker_cmd->position.x;
    from_point.reference.position.y = last_tracker_cmd->position.y;
    from_point.reference.position.z = last_tracker_cmd->position.z;

    if (!isPathToPointInSafetyArea3d(from_point, transformed_reference)) {
      ss << "failed to set the reference, the path is going outside the safety area!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
      return std::tuple(false, ss.str());
    }
  }

  mrs_msgs::ReferenceSrvResponse::ConstPtr tracker_response;

  // prepare the message for current tracker
  mrs_msgs::ReferenceSrvRequest reference_request;
  reference_request.reference = transformed_reference.reference;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    ROS_INFO("[ControlManager]: setting reference to x=%.2f, y=%.2f, z=%.2f, hdg=%.2f (expressed in '%s')", reference_request.reference.position.x,
             reference_request.reference.position.y, reference_request.reference.position.z, reference_request.reference.heading,
             transformed_reference.header.frame_id.c_str());

    tracker_response = tracker_list_.at(active_tracker_idx_)
                           ->setReference(mrs_msgs::ReferenceSrvRequest::ConstPtr(std::make_unique<mrs_msgs::ReferenceSrvRequest>(reference_request)));

    if (tracker_response != mrs_msgs::ReferenceSrvResponse::Ptr()) {

      return std::tuple(tracker_response->success, tracker_response->message);

    } else {

      ss << "the tracker '" << _tracker_names_.at(active_tracker_idx_) << "' does not implement the 'setReference()' function!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: failed to set the reference: " << ss.str());
      return std::tuple(false, ss.str());
    }
  }
}

//}

/* setVelocityReference() //{ */

std::tuple<bool, std::string> ControlManager::setVelocityReference(const mrs_msgs::VelocityReferenceStamped& reference_in) {

  std::stringstream ss;

  if (!callbacks_enabled_) {
    ss << "can not set the reference, the callbacks are disabled";
    ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  if (!validateVelocityReference(reference_in.reference, "ControlManager", "velocity_reference")) {
    ss << "velocity command is not valid!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  {
    std::scoped_lock lock(mutex_last_tracker_cmd_);

    if (!last_tracker_cmd_) {
      ss << "could not set velocity command, not flying!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
      return std::tuple(false, ss.str());
    }
  }

  // copy member variables
  auto uav_state        = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  // | -- transform the velocity reference to the current frame - |

  mrs_msgs::VelocityReferenceStamped transformed_reference = reference_in;

  auto ret = transformer_->getTransform(reference_in.header.frame_id, uav_state.header.frame_id, reference_in.header.stamp);

  geometry_msgs::TransformStamped tf;

  if (!ret) {
    ss << "could not find tf from " << reference_in.header.frame_id << " to " << uav_state.header.frame_id;
    ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  } else {
    tf = ret.value();
  }

  // transform the velocity
  {
    geometry_msgs::Vector3Stamped velocity;
    velocity.header   = reference_in.header;
    velocity.vector.x = reference_in.reference.velocity.x;
    velocity.vector.y = reference_in.reference.velocity.y;
    velocity.vector.z = reference_in.reference.velocity.z;

    auto ret = transformer_->transform(velocity, tf);

    if (!ret) {

      ss << "the velocity reference could not be transformed";
      ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
      return std::tuple(false, ss.str());

    } else {
      transformed_reference.reference.velocity.x = ret->vector.x;
      transformed_reference.reference.velocity.y = ret->vector.y;
      transformed_reference.reference.velocity.z = ret->vector.z;
    }
  }

  // transform the z and the heading
  {
    geometry_msgs::PoseStamped pose;
    pose.header           = reference_in.header;
    pose.pose.position.x  = 0;
    pose.pose.position.y  = 0;
    pose.pose.position.z  = reference_in.reference.altitude;
    pose.pose.orientation = mrs_lib::AttitudeConverter(0, 0, reference_in.reference.heading);

    auto ret = transformer_->transform(pose, tf);

    if (!ret) {

      ss << "the velocity reference could not be transformed";
      ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
      return std::tuple(false, ss.str());

    } else {
      transformed_reference.reference.altitude = ret->pose.position.z;
      transformed_reference.reference.heading  = mrs_lib::AttitudeConverter(ret->pose.orientation).getHeading();
    }
  }

  // the heading rate doees not need to be transformed
  transformed_reference.reference.heading_rate = reference_in.reference.heading_rate;

  transformed_reference.header.stamp    = tf.header.stamp;
  transformed_reference.header.frame_id = transformer_->frame_to(tf);

  mrs_msgs::ReferenceStamped eqivalent_reference = velocityReferenceToReference(transformed_reference);

  ROS_DEBUG("[ControlManager]: equivalent reference: %.2f, %.2f, %.2f, %.2f", eqivalent_reference.reference.position.x,
            eqivalent_reference.reference.position.y, eqivalent_reference.reference.position.z, eqivalent_reference.reference.heading);

  // safety area check
  if (!isPointInSafetyArea3d(eqivalent_reference)) {
    ss << "failed to set the reference, the point is outside of the safety area!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  if (last_tracker_cmd) {

    mrs_msgs::ReferenceStamped from_point;
    from_point.header.frame_id      = uav_state.header.frame_id;
    from_point.reference.position.x = last_tracker_cmd->position.x;
    from_point.reference.position.y = last_tracker_cmd->position.y;
    from_point.reference.position.z = last_tracker_cmd->position.z;

    if (!isPathToPointInSafetyArea3d(from_point, eqivalent_reference)) {
      ss << "failed to set the reference, the path is going outside the safety area!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
      return std::tuple(false, ss.str());
    }
  }

  mrs_msgs::VelocityReferenceSrvResponse::ConstPtr tracker_response;

  // prepare the message for current tracker
  mrs_msgs::VelocityReferenceSrvRequest reference_request;
  reference_request.reference = transformed_reference.reference;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    tracker_response =
        tracker_list_.at(active_tracker_idx_)
            ->setVelocityReference(mrs_msgs::VelocityReferenceSrvRequest::ConstPtr(std::make_unique<mrs_msgs::VelocityReferenceSrvRequest>(reference_request)));

    if (tracker_response != mrs_msgs::VelocityReferenceSrvResponse::Ptr()) {

      return std::tuple(tracker_response->success, tracker_response->message);

    } else {

      ss << "the tracker '" << _tracker_names_.at(active_tracker_idx_) << "' does not implement the 'setVelocityReference()' function!";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: failed to set the velocity reference: " << ss.str());
      return std::tuple(false, ss.str());
    }
  }
}

//}

/* setTrajectoryReference() //{ */

std::tuple<bool, std::string, bool, std::vector<std::string>, std::vector<bool>, std::vector<std::string>> ControlManager::setTrajectoryReference(
    const mrs_msgs::TrajectoryReference trajectory_in) {

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  std::stringstream ss;

  if (!callbacks_enabled_) {
    ss << "can not set the reference, the callbacks are disabled";
    ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());
  }

  /* validate the size and check for NaNs //{ */

  // check for the size 0, which is invalid
  if (trajectory_in.points.size() == 0) {

    ss << "can not load trajectory with size 0";
    ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());
  }

  for (int i = 0; i < int(trajectory_in.points.size()); i++) {

    // check the point for NaN/inf
    bool valid = validateReference(trajectory_in.points.at(i), "ControlManager", "trajectory_in.points");

    if (!valid) {

      ss << "trajectory contains NaNs/infs.";
      ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
      return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());
    }
  }

  //}

  /* publish the debugging topics of the original trajectory //{ */

  {

    geometry_msgs::PoseArray debug_trajectory_out;
    debug_trajectory_out.header = trajectory_in.header;

    debug_trajectory_out.header.frame_id = transformer_->resolveFrame(debug_trajectory_out.header.frame_id);

    if (debug_trajectory_out.header.stamp == ros::Time(0)) {
      debug_trajectory_out.header.stamp = ros::Time::now();
    }

    for (int i = 0; i < int(trajectory_in.points.size()) - 1; i++) {

      geometry_msgs::Pose new_pose;

      new_pose.position.x = trajectory_in.points.at(i).position.x;
      new_pose.position.y = trajectory_in.points.at(i).position.y;
      new_pose.position.z = trajectory_in.points.at(i).position.z;

      new_pose.orientation = mrs_lib::AttitudeConverter(0, 0, trajectory_in.points.at(i).heading);

      debug_trajectory_out.poses.push_back(new_pose);
    }

    pub_debug_original_trajectory_poses_.publish(debug_trajectory_out);

    visualization_msgs::MarkerArray msg_out;

    visualization_msgs::Marker marker;

    marker.header = trajectory_in.header;

    marker.header.frame_id = transformer_->resolveFrame(marker.header.frame_id);

    if (marker.header.frame_id == "") {
      marker.header.frame_id = uav_state.header.frame_id;
    }

    if (marker.header.stamp == ros::Time(0)) {
      marker.header.stamp = ros::Time::now();
    }

    marker.type             = visualization_msgs::Marker::LINE_LIST;
    marker.color.a          = 1;
    marker.scale.x          = 0.05;
    marker.color.r          = 0;
    marker.color.g          = 1;
    marker.color.b          = 0;
    marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

    for (int i = 0; i < int(trajectory_in.points.size()) - 1; i++) {

      geometry_msgs::Point point1;

      point1.x = trajectory_in.points.at(i).position.x;
      point1.y = trajectory_in.points.at(i).position.y;
      point1.z = trajectory_in.points.at(i).position.z;

      marker.points.push_back(point1);

      geometry_msgs::Point point2;

      point2.x = trajectory_in.points.at(i + 1).position.x;
      point2.y = trajectory_in.points.at(i + 1).position.y;
      point2.z = trajectory_in.points.at(i + 1).position.z;

      marker.points.push_back(point2);
    }

    msg_out.markers.push_back(marker);

    pub_debug_original_trajectory_markers_.publish(msg_out);
  }

  //}

  mrs_msgs::TrajectoryReference processed_trajectory = trajectory_in;

  int trajectory_size = int(processed_trajectory.points.size());

  bool trajectory_modified = false;

  /* safety area check //{ */

  if (use_safety_area_) {

    int last_valid_idx    = 0;
    int first_invalid_idx = -1;

    double min_z = getMinZ(processed_trajectory.header.frame_id);
    double max_z = getMaxZ(processed_trajectory.header.frame_id);

    for (int i = 0; i < trajectory_size; i++) {

      if (_snap_trajectory_to_safety_area_) {

        // saturate the trajectory to min and max Z
        if (processed_trajectory.points.at(i).position.z < min_z) {

          processed_trajectory.points.at(i).position.z = min_z;
          ROS_WARN_THROTTLE(1.0, "[ControlManager]: the trajectory violates the minimum Z!");
          trajectory_modified = true;
        }

        if (processed_trajectory.points.at(i).position.z > max_z) {

          processed_trajectory.points.at(i).position.z = max_z;
          ROS_WARN_THROTTLE(1.0, "[ControlManager]: the trajectory violates the maximum Z!");
          trajectory_modified = true;
        }
      }

      // check the point against the safety area
      mrs_msgs::ReferenceStamped des_reference;
      des_reference.header    = processed_trajectory.header;
      des_reference.reference = processed_trajectory.points.at(i);

      if (!isPointInSafetyArea3d(des_reference)) {

        ROS_WARN_THROTTLE(1.0, "[ControlManager]: the trajectory contains points outside of the safety area!");
        trajectory_modified = true;

        // the first invalid point
        if (first_invalid_idx == -1) {

          first_invalid_idx = i;

          last_valid_idx = i - 1;
        }

        // the point is ok
      } else {

        // we found a point, which is ok, after finding a point which was not ok
        if (first_invalid_idx != -1) {

          // special case, we had no valid point so far
          if (last_valid_idx == -1) {

            ss << "the trajectory starts outside of the safety area!";
            ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
            return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());

            // we have a valid point in the past
          } else {

            if (!_snap_trajectory_to_safety_area_) {
              break;
            }

            bool interpolation_success = true;

            // iterpolate between the last valid point and this new valid point
            double angle = atan2((processed_trajectory.points.at(i).position.y - processed_trajectory.points.at(last_valid_idx).position.y),
                                 (processed_trajectory.points.at(i).position.x - processed_trajectory.points.at(last_valid_idx).position.x));

            double dist_two_points = mrs_lib::geometry::dist(
                vec2_t(processed_trajectory.points.at(i).position.x, processed_trajectory.points.at(i).position.y),
                vec2_t(processed_trajectory.points.at(last_valid_idx).position.x, processed_trajectory.points.at(last_valid_idx).position.y));
            double step = dist_two_points / (i - last_valid_idx);

            for (int j = last_valid_idx; j < i; j++) {

              mrs_msgs::ReferenceStamped temp_point;
              temp_point.header.frame_id      = processed_trajectory.header.frame_id;
              temp_point.reference.position.x = processed_trajectory.points.at(last_valid_idx).position.x + (j - last_valid_idx) * cos(angle) * step;
              temp_point.reference.position.y = processed_trajectory.points.at(last_valid_idx).position.y + (j - last_valid_idx) * sin(angle) * step;

              if (!isPointInSafetyArea2d(temp_point)) {

                interpolation_success = false;
                break;

              } else {

                processed_trajectory.points.at(j).position.x = temp_point.reference.position.x;
                processed_trajectory.points.at(j).position.y = temp_point.reference.position.y;
              }
            }

            if (!interpolation_success) {
              break;
            }
          }

          first_invalid_idx = -1;
        }
      }
    }

    // special case, the trajectory does not end with a valid point
    if (first_invalid_idx != -1) {

      // super special case, the whole trajectory is invalid
      if (first_invalid_idx == 0) {

        ss << "the whole trajectory is outside of the safety area!";
        ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
        return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());

        // there is a good portion of the trajectory in the beginning
      } else {

        trajectory_size = last_valid_idx + 1;
        processed_trajectory.points.resize(trajectory_size);
        trajectory_modified = true;
      }
    }
  }

  if (trajectory_size == 0) {

    ss << "the trajectory happened to be empty after all the checks! This message should not appear!";
    ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());
  }

  //}

  /* transform the trajectory to the current control frame //{ */

  std::optional<geometry_msgs::TransformStamped> tf_traj_state;

  if (processed_trajectory.header.stamp > ros::Time::now()) {
    tf_traj_state = transformer_->getTransform(processed_trajectory.header.frame_id, "", processed_trajectory.header.stamp);
  } else {
    tf_traj_state = transformer_->getTransform(processed_trajectory.header.frame_id, "", uav_state_.header.stamp);
  }

  if (!tf_traj_state) {
    ss << "could not create TF transformer for the trajectory";
    ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());
  }

  processed_trajectory.header.frame_id = transformer_->frame_to(*tf_traj_state);

  for (int i = 0; i < trajectory_size; i++) {

    mrs_msgs::ReferenceStamped trajectory_point;
    trajectory_point.header    = processed_trajectory.header;
    trajectory_point.reference = processed_trajectory.points.at(i);

    auto ret = transformer_->transform(trajectory_point, *tf_traj_state);

    if (!ret) {

      ss << "trajectory cannnot be transformed";
      ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
      return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());

    } else {

      // transform the points in the trajectory to the current frame
      processed_trajectory.points.at(i) = ret.value().reference;
    }
  }

  //}

  mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr response;
  mrs_msgs::TrajectoryReferenceSrvRequest            request;

  // check for empty trajectory
  if (processed_trajectory.points.size() == 0) {
    ss << "reference trajectory was processing and it is now empty, this should not happen!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());
  }

  // prepare the message for current tracker
  request.trajectory = processed_trajectory;

  bool                     success;
  std::string              message;
  bool                     modified;
  std::vector<std::string> tracker_names;
  std::vector<bool>        tracker_successes;
  std::vector<std::string> tracker_messages;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    // set the trajectory to the currently active tracker
    response =
        tracker_list_.at(active_tracker_idx_)
            ->setTrajectoryReference(mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr(std::make_unique<mrs_msgs::TrajectoryReferenceSrvRequest>(request)));

    tracker_names.push_back(_tracker_names_.at(active_tracker_idx_));

    if (response != mrs_msgs::TrajectoryReferenceSrvResponse::Ptr()) {

      success  = response->success;
      message  = response->message;
      modified = response->modified || trajectory_modified;
      tracker_successes.push_back(response->success);
      tracker_messages.push_back(response->message);

    } else {

      ss << "the active tracker '" << _tracker_names_.at(active_tracker_idx_) << "' does not implement the 'setTrajectoryReference()' function!";
      ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());

      success  = true;
      message  = ss.str();
      modified = false;
      tracker_successes.push_back(false);
      tracker_messages.push_back(ss.str());
    }

    // set the trajectory to the non-active trackers
    for (size_t i = 0; i < tracker_list_.size(); i++) {

      if (i != active_tracker_idx_) {

        tracker_names.push_back(_tracker_names_.at(i));

        response = tracker_list_.at(i)->setTrajectoryReference(
            mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr(std::make_unique<mrs_msgs::TrajectoryReferenceSrvRequest>(request)));

        if (response != mrs_msgs::TrajectoryReferenceSrvResponse::Ptr()) {

          tracker_successes.push_back(response->success);
          tracker_messages.push_back(response->message);

          if (response->success) {
            std::stringstream ss;
            ss << "trajectory loaded to non-active tracker '" << _tracker_names_.at(i);
            ROS_INFO_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
          }

        } else {

          std::stringstream ss;
          ss << "the tracker \"" << _tracker_names_.at(i) << "\" does not implement setTrajectoryReference()";
          tracker_successes.push_back(false);
          tracker_messages.push_back(ss.str());
        }
      }
    }
  }

  return std::tuple(success, message, modified, tracker_names, tracker_successes, tracker_messages);
}

//}

/* isOffboard() //{ */

bool ControlManager::isOffboard(void) {

  if (!sh_hw_api_status_.hasMsg()) {
    return false;
  }

  mrs_msgs::HwApiStatusConstPtr hw_api_status = sh_hw_api_status_.getMsg();

  return hw_api_status->connected && hw_api_status->offboard;
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
    for (int i = 0; i < int(tracker_list_.size()); i++) {
      tracker_list_.at(i)->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
    }
  }
}

//}

/* publishDiagnostics() //{ */

void ControlManager::publishDiagnostics(void) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("publishDiagnostics");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::publishDiagnostics", scope_timer_logger_, scope_timer_enabled_);

  std::scoped_lock lock(mutex_diagnostics_);

  mrs_msgs::ControlManagerDiagnostics diagnostics_msg;

  diagnostics_msg.stamp    = ros::Time::now();
  diagnostics_msg.uav_name = _uav_name_;

  diagnostics_msg.desired_uav_state_rate = desired_uav_state_rate_;

  diagnostics_msg.output_enabled = output_enabled_;

  diagnostics_msg.joystick_active = rc_goto_active_;

  {
    std::scoped_lock lock(mutex_tracker_list_, mutex_controller_list_);

    diagnostics_msg.flying_normally = isFlyingNormally();
  }

  diagnostics_msg.bumper_active = bumper_repulsing_;

  // | ----------------- fill the tracker status ---------------- |

  {
    std::scoped_lock lock(mutex_tracker_list_);

    mrs_msgs::TrackerStatus tracker_status;

    diagnostics_msg.active_tracker = _tracker_names_.at(active_tracker_idx_);
    diagnostics_msg.tracker_status = tracker_list_.at(active_tracker_idx_)->getStatus();
  }

  // | --------------- fill the controller status --------------- |

  {
    std::scoped_lock lock(mutex_controller_list_);

    mrs_msgs::ControllerStatus controller_status;

    diagnostics_msg.active_controller = _controller_names_.at(active_controller_idx_);
    diagnostics_msg.controller_status = controller_list_.at(active_controller_idx_)->getStatus();
  }

  // | ------------ fill in the available controllers ----------- |

  for (int i = 0; i < int(_controller_names_.size()); i++) {
    if ((_controller_names_.at(i) != _failsafe_controller_name_) && (_controller_names_.at(i) != _eland_controller_name_)) {
      diagnostics_msg.available_controllers.push_back(_controller_names_.at(i));
      diagnostics_msg.human_switchable_controllers.push_back(controllers_.at(_controller_names_.at(i)).human_switchable);
    }
  }

  // | ------------- fill in the available trackers ------------- |

  for (int i = 0; i < int(_tracker_names_.size()); i++) {
    if (_tracker_names_.at(i) != _null_tracker_name_) {
      diagnostics_msg.available_trackers.push_back(_tracker_names_.at(i));
      diagnostics_msg.human_switchable_trackers.push_back(trackers_.at(_tracker_names_.at(i)).human_switchable);
    }
  }

  // | ------------------------- publish ------------------------ |

  ph_diagnostics_.publish(diagnostics_msg);
}

//}

/* setConstraintsToTrackers() //{ */

void ControlManager::setConstraintsToTrackers(const mrs_msgs::DynamicsConstraintsSrvRequest& constraints) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("setConstraintsToTrackers");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::setConstraintsToTrackers", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr response;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    // for each tracker
    for (int i = 0; i < int(tracker_list_.size()); i++) {

      // if it is the active one, update and retrieve the command
      response = tracker_list_.at(i)->setConstraints(
          mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr(std::make_unique<mrs_msgs::DynamicsConstraintsSrvRequest>(constraints)));
    }
  }
}

//}

/* setConstraintsToControllers() //{ */

void ControlManager::setConstraintsToControllers(const mrs_msgs::DynamicsConstraintsSrvRequest& constraints) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("setConstraintsToControllers");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::setConstraintsToControllers", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr response;

  {
    std::scoped_lock lock(mutex_controller_list_);

    // for each controller
    for (int i = 0; i < int(controller_list_.size()); i++) {

      // if it is the active one, update and retrieve the command
      response = controller_list_.at(i)->setConstraints(
          mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr(std::make_unique<mrs_msgs::DynamicsConstraintsSrvRequest>(constraints)));
    }
  }
}

//}

/* setConstraints() //{ */

void ControlManager::setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest& constraints) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("setConstraints");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::setConstraints", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr response;

  setConstraintsToTrackers(constraints);

  setConstraintsToControllers(constraints);
}

//}

/* enforceControllerConstraints() //{ */

std::optional<mrs_msgs::DynamicsConstraintsSrvRequest> ControlManager::enforceControllersConstraints(
    const mrs_msgs::DynamicsConstraintsSrvRequest& constraints) {

  // copy member variables
  auto last_control_output = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);

  if (!last_control_output.control_output || !last_control_output.diagnostics.controller_enforcing_constraints) {
    return {};
  }

  bool enforcing = false;

  auto constraints_out = constraints;

  std::scoped_lock lock(mutex_tracker_list_);

  // enforce horizontal speed
  if (last_control_output.diagnostics.horizontal_speed_constraint < constraints.constraints.horizontal_speed) {
    constraints_out.constraints.horizontal_speed = last_control_output.diagnostics.horizontal_speed_constraint;

    enforcing = true;
  }

  // enforce horizontal acceleration
  if (last_control_output.diagnostics.horizontal_acc_constraint < constraints.constraints.horizontal_acceleration) {
    constraints_out.constraints.horizontal_acceleration = last_control_output.diagnostics.horizontal_acc_constraint;

    enforcing = true;
  }

  // enforce vertical ascending speed
  if (last_control_output.diagnostics.vertical_asc_speed_constraint < constraints.constraints.vertical_ascending_speed) {
    constraints_out.constraints.vertical_ascending_speed = last_control_output.diagnostics.vertical_asc_speed_constraint;

    enforcing = true;
  }

  // enforce vertical ascending acceleration
  if (last_control_output.diagnostics.vertical_asc_acc_constraint < constraints.constraints.vertical_ascending_acceleration) {
    constraints_out.constraints.vertical_ascending_acceleration = last_control_output.diagnostics.vertical_asc_acc_constraint;

    enforcing = true;
  }

  // enforce vertical descending speed
  if (last_control_output.diagnostics.vertical_desc_speed_constraint < constraints.constraints.vertical_descending_speed) {
    constraints_out.constraints.vertical_descending_speed = last_control_output.diagnostics.vertical_desc_speed_constraint;

    enforcing = true;
  }

  // enforce vertical descending acceleration
  if (last_control_output.diagnostics.vertical_desc_acc_constraint < constraints.constraints.vertical_descending_acceleration) {
    constraints_out.constraints.vertical_descending_acceleration = last_control_output.diagnostics.vertical_desc_acc_constraint;

    enforcing = true;
  }

  if (enforcing) {
    return {constraints_out};
  } else {
    return {};
  }
}

//}

/* isFlyingNormally() //{ */

bool ControlManager::isFlyingNormally(void) {

  return callbacks_enabled_ && (output_enabled_) && (offboard_mode_) && (armed_) &&
         (((active_tracker_idx_ != _ehover_tracker_idx_) && (active_controller_idx_ != _eland_controller_idx_) &&
           (active_controller_idx_ != _failsafe_controller_idx_)) ||
          _controller_names_.size() == 1) &&
         (((active_tracker_idx_ != _null_tracker_idx_) && (active_tracker_idx_ != _landoff_tracker_idx_)) || _tracker_names_.size() == 1);
}

//}

/* //{ getMass() */

double ControlManager::getMass(void) {

  auto last_control_output = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);

  if (last_control_output.diagnostics.mass_estimator) {
    return _uav_mass_ + last_control_output.diagnostics.mass_difference;
  } else {
    return _uav_mass_;
  }
}

//}

/* loadConfigFile() //{ */

bool ControlManager::loadConfigFile(const std::string& file_path, const std::string ns) {

  const std::string name_space = nh_.getNamespace() + "/" + ns;

  ROS_INFO("[ControlManager]: loading '%s' under the namespace '%s'", file_path.c_str(), name_space.c_str());

  // load the user-requested file
  {
    std::string command = "rosparam load " + file_path + " " + name_space;
    int         result  = std::system(command.c_str());

    if (result != 0) {
      ROS_ERROR("[ControlManager]: failed to load '%s'", file_path.c_str());
      return false;
    }
  }

  // load the platform config
  if (_platform_config_ != "") {
    std::string command = "rosparam load " + _platform_config_ + " " + name_space;
    int         result  = std::system(command.c_str());

    if (result != 0) {
      ROS_ERROR("[ControlManager]: failed to load the platform config file '%s'", _platform_config_.c_str());
      return false;
    }
  }

  // load the custom config
  if (_custom_config_ != "") {
    std::string command = "rosparam load " + _custom_config_ + " " + name_space;
    int         result  = std::system(command.c_str());

    if (result != 0) {
      ROS_ERROR("[ControlManager]: failed to load the custom config file '%s'", _custom_config_.c_str());
      return false;
    }
  }

  return true;
}

//}

// | ----------------------- safety area ---------------------- |

/* //{ isPointInSafetyArea3d() */

bool ControlManager::isPointInSafetyArea3d(const mrs_msgs::ReferenceStamped& point) {

  if (!use_safety_area_) {
    return true;
  }

  auto tfed_horizontal = transformer_->transformSingle(point, _safety_area_horizontal_frame_);

  if (!tfed_horizontal) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: SafetyArea: Could not transform the point to the safety area horizontal frame");
    return false;
  }

  if (!safety_zone_->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y)) {
    return false;
  }

  if (point.reference.position.z < getMinZ(point.header.frame_id) || point.reference.position.z > getMaxZ(point.header.frame_id)) {
    return false;
  }

  return true;
}

//}

/* //{ isPointInSafetyArea2d() */

bool ControlManager::isPointInSafetyArea2d(const mrs_msgs::ReferenceStamped& point) {

  if (!use_safety_area_) {
    return true;
  }

  auto tfed_horizontal = transformer_->transformSingle(point, _safety_area_horizontal_frame_);

  if (!tfed_horizontal) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: SafetyArea: Could not transform the point to the safety area horizontal frame");
    return false;
  }

  if (!safety_zone_->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y)) {
    return false;
  }

  return true;
}

//}

/* //{ isPathToPointInSafetyArea3d() */

bool ControlManager::isPathToPointInSafetyArea3d(const mrs_msgs::ReferenceStamped& start, const mrs_msgs::ReferenceStamped& end) {

  if (!use_safety_area_) {
    return true;
  }

  if (!isPointInSafetyArea3d(start) || !isPointInSafetyArea3d(end)) {
    return false;
  }

  mrs_msgs::ReferenceStamped start_transformed, end_transformed;

  {
    auto ret = transformer_->transformSingle(start, _safety_area_horizontal_frame_);

    if (!ret) {

      ROS_ERROR("[ControlManager]: SafetyArea: Could not transform the first point in the path");

      return false;
    }

    start_transformed = ret.value();
  }

  {
    auto ret = transformer_->transformSingle(end, _safety_area_horizontal_frame_);

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

/* //{ isPathToPointInSafetyArea2d() */

bool ControlManager::isPathToPointInSafetyArea2d(const mrs_msgs::ReferenceStamped& start, const mrs_msgs::ReferenceStamped& end) {

  if (!use_safety_area_) {
    return true;
  }

  mrs_msgs::ReferenceStamped start_transformed, end_transformed;

  if (!isPointInSafetyArea2d(start) || !isPointInSafetyArea2d(end)) {
    return false;
  }

  {
    auto ret = transformer_->transformSingle(start, _safety_area_horizontal_frame_);

    if (!ret) {

      ROS_ERROR("[ControlManager]: SafetyArea: Could not transform the first point in the path");

      return false;
    }

    start_transformed = ret.value();
  }

  {
    auto ret = transformer_->transformSingle(end, _safety_area_horizontal_frame_);

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

/* //{ getMaxZ() */

double ControlManager::getMaxZ(const std::string& frame_id) {

  // | ------- first, calculate max_z from the safety area ------ |

  double safety_area_max_z = std::numeric_limits<float>::max();

  {

    geometry_msgs::PointStamped point;

    point.header.frame_id = _safety_area_vertical_frame_;
    point.point.x         = 0;
    point.point.y         = 0;
    point.point.z         = _safety_area_max_z_;

    auto ret = transformer_->transformSingle(point, frame_id);

    if (!ret) {
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: SafetyArea: Could not transform safety area's max_z to '%s'", frame_id.c_str());
    }

    safety_area_max_z = ret->point.z;
  }

  // | ------------ overwrite from estimation manager ----------- |

  double estimation_manager_max_z = std::numeric_limits<float>::max();

  {
    // if possible, override it with max z from the estimation manager
    if (sh_max_z_.hasMsg()) {

      auto msg = sh_max_z_.getMsg();

      // transform it into the safety area frame
      geometry_msgs::PointStamped point;
      point.header  = msg->header;
      point.point.x = 0;
      point.point.y = 0;
      point.point.z = msg->value;

      auto ret = transformer_->transformSingle(point, frame_id);

      if (!ret) {
        ROS_ERROR_THROTTLE(1.0, "[ControlManager]: SafetyArea: Could not transform estimation manager's max_z to the current control frame");
      }

      estimation_manager_max_z = ret->point.z;
    }
  }

  if (estimation_manager_max_z < safety_area_max_z) {
    return estimation_manager_max_z;
  } else {
    return safety_area_max_z;
  }
}

//}

/* //{ getMinZ() */

double ControlManager::getMinZ(const std::string& frame_id) {

  if (!use_safety_area_) {
    return std::numeric_limits<double>::lowest();
  }

  geometry_msgs::PointStamped point;

  point.header.frame_id = _safety_area_vertical_frame_;
  point.point.x         = 0;
  point.point.y         = 0;
  point.point.z         = _safety_area_min_z_;

  auto ret = transformer_->transformSingle(point, frame_id);

  if (!ret) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: SafetyArea: Could not transform safety area's min_z to '%s'", frame_id.c_str());
    return std::numeric_limits<double>::lowest();
  }

  return ret->point.z;
}

//}

// | --------------------- obstacle bumper -------------------- |

/* bumperPushFromObstacle() //{ */

void ControlManager::bumperPushFromObstacle(void) {

  // | --------------- fabricate the min distances -------------- |

  double min_distance_horizontal = _bumper_horizontal_distance_;
  double min_distance_vertical   = _bumper_vertical_distance_;

  if (_bumper_horizontal_derive_from_dynamics_ || _bumper_vertical_derive_from_dynamics_) {

    auto constraints = mrs_lib::get_mutexed(mutex_constraints_, current_constraints_);

    if (_bumper_horizontal_derive_from_dynamics_) {

      const double horizontal_t_stop    = constraints.constraints.horizontal_speed / constraints.constraints.horizontal_acceleration;
      const double horizontal_stop_dist = (horizontal_t_stop * constraints.constraints.horizontal_speed) / 2.0;

      min_distance_horizontal += 1.5 * horizontal_stop_dist;
    }

    if (_bumper_vertical_derive_from_dynamics_) {


      // larger from the two accelerations
      const double vert_acc = constraints.constraints.vertical_ascending_acceleration > constraints.constraints.vertical_descending_acceleration
                                  ? constraints.constraints.vertical_ascending_acceleration
                                  : constraints.constraints.vertical_descending_acceleration;

      // larger from the two speeds
      const double vert_speed = constraints.constraints.vertical_ascending_speed > constraints.constraints.vertical_descending_speed
                                    ? constraints.constraints.vertical_ascending_speed
                                    : constraints.constraints.vertical_descending_speed;

      const double vertical_t_stop    = vert_speed / vert_acc;
      const double vertical_stop_dist = (vertical_t_stop * vert_speed) / 2.0;

      min_distance_vertical += 1.5 * vertical_stop_dist;
    }
  }

  // | ----------------------------  ---------------------------- |

  // copy member variables
  mrs_msgs::ObstacleSectorsConstPtr bumper_data = sh_bumper_.getMsg();
  auto                              uav_state   = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  double sector_size = TAU / double(bumper_data->n_horizontal_sectors);

  double direction          = 0;
  double repulsion_distance = std::numeric_limits<double>::max();

  bool horizontal_collision_detected = false;
  bool vertical_collision_detected   = false;

  double min_horizontal_sector_distance = std::numeric_limits<double>::max();
  size_t min_sector_id                  = 0;

  for (unsigned long i = 0; i < bumper_data->n_horizontal_sectors; i++) {

    if (bumper_data->sectors.at(i) < 0) {
      continue;
    }

    if (bumper_data->sectors.at(i) < min_horizontal_sector_distance) {
      min_horizontal_sector_distance = bumper_data->sectors.at(i);
      min_sector_id                  = i;
    }
  }

  // if the sector is under the threshold distance
  if (min_horizontal_sector_distance < min_distance_horizontal) {

    // get the desired direction of motion
    double oposite_direction  = double(min_sector_id) * sector_size + M_PI;
    int    oposite_sector_idx = bumperGetSectorId(cos(oposite_direction), sin(oposite_direction), 0);

    // get the id of the oposite sector
    direction = oposite_direction;

    ROS_WARN_THROTTLE(1.0, "[ControlManager]: Bumper: found potential collision (sector %lu vs. %d), obstacle distance: %.2f, repulsing", min_sector_id,
                      oposite_sector_idx, bumper_data->sectors.at(min_sector_id));

    repulsion_distance = min_distance_horizontal + _bumper_horizontal_overshoot_ - bumper_data->sectors.at(min_sector_id);

    horizontal_collision_detected = true;
  }

  double vertical_repulsion_distance = 0;

  // check for vertical collision down
  if (bumper_data->sectors.at(bumper_data->n_horizontal_sectors) > 0 && bumper_data->sectors.at(bumper_data->n_horizontal_sectors) <= min_distance_vertical) {

    ROS_INFO_THROTTLE(1.0, "[ControlManager]: Bumper: potential collision below");
    vertical_collision_detected = true;
    vertical_repulsion_distance = min_distance_vertical - bumper_data->sectors.at(bumper_data->n_horizontal_sectors);
  }

  // check for vertical collision up
  if (bumper_data->sectors.at(bumper_data->n_horizontal_sectors + 1) > 0 &&
      bumper_data->sectors.at(bumper_data->n_horizontal_sectors + 1) <= min_distance_vertical) {

    ROS_INFO_THROTTLE(1.0, "[ControlManager]: Bumper: potential collision above");
    vertical_collision_detected = true;
    vertical_repulsion_distance = -(min_distance_vertical - bumper_data->sectors.at(bumper_data->n_horizontal_sectors + 1));
  }

  // if potential collision was detected and we should start the repulsing_
  if (horizontal_collision_detected || vertical_collision_detected) {

    if (!bumper_repulsing_) {

      if (_bumper_switch_tracker_) {

        auto        active_tracker_idx  = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);
        std::string active_tracker_name = _tracker_names_.at(active_tracker_idx);

        // remember the previously active tracker
        bumper_previous_tracker_ = active_tracker_name;

        if (active_tracker_name != _bumper_tracker_name_) {

          switchTracker(_bumper_tracker_name_);
        }
      }

      if (_bumper_switch_controller_) {

        auto        active_controller_idx  = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);
        std::string active_controller_name = _controller_names_.at(active_controller_idx);

        // remember the previously active controller
        bumper_previous_controller_ = active_controller_name;

        if (active_controller_name != _bumper_controller_name_) {

          switchController(_bumper_controller_name_);
        }
      }
    }

    bumper_repulsing_ = true;

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

    reference_fcu_untilted.reference.heading = 0;

    if (vertical_collision_detected) {
      reference_fcu_untilted.reference.position.z = vertical_repulsion_distance;
    } else {
      reference_fcu_untilted.reference.position.z = 0;
    }

    {
      std::scoped_lock lock(mutex_tracker_list_);

      // transform the reference into the currently used frame
      // this is under the mutex_tracker_list since we don't won't the odometry switch to happen
      // to the tracker before we actually call the goto service

      auto ret = transformer_->transformSingle(reference_fcu_untilted, uav_state.header.frame_id);

      if (!ret) {
        ROS_WARN_THROTTLE(1.0, "[ControlManager]: Bumper: bumper reference could not be transformed");
        return;
      }

      reference_fcu_untilted = ret.value();

      // copy the reference into the service type message
      mrs_msgs::ReferenceSrvRequest req_goto_out;
      req_goto_out.reference = reference_fcu_untilted.reference;

      // disable callbacks of all trackers
      req_enable_callbacks.data = false;
      for (size_t i = 0; i < tracker_list_.size(); i++) {
        tracker_list_.at(i)->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
      }

      // enable the callbacks for the active tracker
      req_enable_callbacks.data = true;
      tracker_list_.at(active_tracker_idx_)
          ->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));

      // call the goto
      tracker_response = tracker_list_.at(active_tracker_idx_)
                             ->setReference(mrs_msgs::ReferenceSrvRequest::ConstPtr(std::make_unique<mrs_msgs::ReferenceSrvRequest>(req_goto_out)));

      // disable the callbacks back again
      req_enable_callbacks.data = false;
      tracker_list_.at(active_tracker_idx_)
          ->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
    }
  }

  // if repulsing_ and the distance is safe once again
  if (bumper_repulsing_ && !horizontal_collision_detected && !vertical_collision_detected) {

    ROS_INFO_THROTTLE(1.0, "[ControlManager]: Bumper: no more collision, stopping repulsion");

    if (_bumper_switch_tracker_) {

      auto        active_tracker_idx  = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);
      std::string active_tracker_name = _tracker_names_.at(active_tracker_idx);

      if (active_tracker_name != bumper_previous_tracker_) {

        switchTracker(bumper_previous_tracker_);
      }
    }

    if (_bumper_switch_controller_) {

      auto        active_controller_idx  = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);
      std::string active_controller_name = _controller_names_.at(active_controller_idx);

      if (active_controller_name != bumper_previous_controller_) {

        switchController(bumper_previous_controller_);
      }
    }

    std_srvs::SetBoolRequest req_enable_callbacks;

    {
      std::scoped_lock lock(mutex_tracker_list_);

      // enable callbacks of all trackers
      req_enable_callbacks.data = true;
      for (size_t i = 0; i < tracker_list_.size(); i++) {
        tracker_list_.at(i)->enableCallbacks(std_srvs::SetBoolRequest::ConstPtr(std::make_unique<std_srvs::SetBoolRequest>(req_enable_callbacks)));
      }
    }

    callbacks_enabled_ = true;

    bumper_repulsing_ = false;
  }
}

//}

/* bumperGetSectorId() //{ */

int ControlManager::bumperGetSectorId(const double& x, const double& y, [[maybe_unused]] const double& z) {

  // copy member variables
  mrs_msgs::ObstacleSectorsConstPtr bumper_data = sh_bumper_.getMsg();

  // heading of the point in drone frame
  double point_heading_horizontal = atan2(y, x);

  point_heading_horizontal += TAU;

  // if point_heading_horizontal is greater then 2*M_PI mod it
  if (fabs(point_heading_horizontal) >= TAU) {
    point_heading_horizontal = fmod(point_heading_horizontal, TAU);
  }

  // heading of the right edge of the first sector
  double sector_size = TAU / double(bumper_data->n_horizontal_sectors);

  // calculate the idx
  int idx = floor((point_heading_horizontal + (sector_size / 2.0)) / sector_size);

  if (idx > int(bumper_data->n_horizontal_sectors) - 1) {
    idx -= bumper_data->n_horizontal_sectors;
  }

  return idx;
}

//}

// | ------------------------- safety ------------------------- |

/* //{ changeLandingState() */

void ControlManager::changeLandingState(LandingStates_t new_state) {

  // copy member variables
  auto last_control_output = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);

  {
    std::scoped_lock lock(mutex_landing_state_machine_);

    previous_state_landing_ = current_state_landing_;
    current_state_landing_  = new_state;
  }

  switch (current_state_landing_) {

    case IDLE_STATE:
      break;
    case LANDING_STATE: {

      ROS_DEBUG("[ControlManager]: starting eland timer");
      timer_eland_.start();
      ROS_DEBUG("[ControlManager]: eland timer started");
      eland_triggered_ = true;
      bumper_enabled_  = false;

      landing_uav_mass_ = getMass();
    }

    break;
  }

  ROS_INFO("[ControlManager]: switching emergency landing state %s -> %s", state_names[previous_state_landing_], state_names[current_state_landing_]);
}

//}

/* hover() //{ */

std::tuple<bool, std::string> ControlManager::hover(void) {

  if (!is_initialized_) {
    return std::tuple(false, "the ControlManager is not initialized");
  }

  if (eland_triggered_) {
    return std::tuple(false, "cannot hover, eland already triggered");
  }

  if (failsafe_triggered_) {
    return std::tuple(false, "cannot hover, failsafe already triggered");
  }

  {
    std::scoped_lock lock(mutex_tracker_list_);

    std_srvs::TriggerResponse::ConstPtr response;
    std_srvs::TriggerRequest            request;

    response = tracker_list_.at(active_tracker_idx_)->hover(std_srvs::TriggerRequest::ConstPtr(std::make_unique<std_srvs::TriggerRequest>(request)));

    if (response != std_srvs::TriggerResponse::Ptr()) {

      return std::tuple(response->success, response->message);

    } else {

      std::stringstream ss;
      ss << "the tracker '" << _tracker_names_.at(active_tracker_idx_) << "' does not implement the 'hover()' function!";

      return std::tuple(false, ss.str());
    }
  }
}

//}

/* //{ ehover() */

std::tuple<bool, std::string> ControlManager::ehover(void) {

  if (!is_initialized_) {
    return std::tuple(false, "the ControlManager is not initialized");
  }

  if (eland_triggered_) {
    return std::tuple(false, "cannot ehover, eland already triggered");
  }

  if (failsafe_triggered_) {
    return std::tuple(false, "cannot ehover, failsafe already triggered");
  }

  // copy the member variables
  auto last_control_output = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);
  auto active_tracker_idx  = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);

  if (active_tracker_idx == _null_tracker_idx_) {

    std::stringstream ss;
    ss << "can not trigger ehover while not flying";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());

    return std::tuple(false, ss.str());
  }

  ungripSrv();

  {

    auto [success, message] = switchTracker(_ehover_tracker_name_);

    // check if the tracker was successfully switched
    // this is vital, that is the core of the hover
    if (!success) {

      std::stringstream ss;
      ss << "error during switching to ehover tracker: '" << message << "'";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());

      return std::tuple(success, ss.str());
    }
  }

  {
    auto [success, message] = switchController(_eland_controller_name_);

    // check if the controller was successfully switched
    // this is not vital, we can continue without that
    if (!success) {

      std::stringstream ss;
      ss << "error during switching to ehover controller: '" << message << "'";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    }
  }

  std::stringstream ss;
  ss << "ehover activated";
  ROS_INFO_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());

  callbacks_enabled_ = false;

  return std::tuple(true, ss.str());
}

//}

/* eland() //{ */

std::tuple<bool, std::string> ControlManager::eland(void) {

  if (!is_initialized_) {
    return std::tuple(false, "the ControlManager is not initialized");
  }

  if (eland_triggered_) {
    return std::tuple(false, "cannot eland, eland already triggered");
  }

  if (failsafe_triggered_) {
    return std::tuple(false, "cannot eland, failsafe already triggered");
  }

  // copy member variables
  auto last_control_output = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);
  auto active_tracker_idx  = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);

  if (active_tracker_idx == _null_tracker_idx_) {

    std::stringstream ss;
    ss << "can not trigger eland while not flying";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());

    return std::tuple(false, ss.str());
  }

  if (_rc_emergency_handoff_) {

    toggleOutput(false);

    return std::tuple(true, "RC emergency handoff is ON, disabling output");
  }

  {
    auto [success, message] = switchTracker(_ehover_tracker_name_);

    // check if the tracker was successfully switched
    // this is vital
    if (!success) {

      std::stringstream ss;
      ss << "error during switching to eland tracker: '" << message << "'";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());

      return std::tuple(success, ss.str());
    }
  }

  {
    auto [success, message] = switchController(_eland_controller_name_);

    // check if the controller was successfully switched
    // this is not vital, we can continue without it
    if (!success) {

      std::stringstream ss;
      ss << "error during switching to eland controller: '" << message << "'";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    }
  }

  // | ----------------- call the eland service ----------------- |

  std::stringstream ss;
  bool              success;

  if (elandSrv()) {

    changeLandingState(LANDING_STATE);

    odometryCallbacksSrv(false);

    ss << "eland activated";
    ROS_INFO_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());

    success = true;

    callbacks_enabled_ = false;

  } else {

    ss << "error during activation of eland";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());

    success = false;
  }

  return std::tuple(success, ss.str());
}

//}

/* failsafe() //{ */

std::tuple<bool, std::string> ControlManager::failsafe(void) {

  // copy member variables
  auto last_control_output   = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);
  auto active_tracker_idx    = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);

  if (!is_initialized_) {
    return std::tuple(false, "the ControlManager is not initialized");
  }

  if (failsafe_triggered_) {
    return std::tuple(false, "cannot, failsafe already triggered");
  }

  if (active_tracker_idx == _null_tracker_idx_) {

    std::stringstream ss;
    ss << "can not trigger failsafe while not flying";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  if (_rc_emergency_handoff_) {

    toggleOutput(false);

    return std::tuple(true, "RC emergency handoff is ON, disabling output");
  }

  if (getLowestOuput(_hw_api_inputs_) == POSITION) {
    return eland();
  }

  if (_parachute_enabled_) {

    auto [success, message] = deployParachute();

    if (success) {

      std::stringstream ss;
      ss << "failsafe activated (parachute): '" << message << "'";
      ROS_INFO_STREAM("[ControlManager]: " << ss.str());

      return std::tuple(true, ss.str());

    } else {

      std::stringstream ss;
      ss << "could not deploy parachute: '" << message << "', continuing with normal failsafe";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    }
  }

  if (_failsafe_controller_idx_ != active_controller_idx) {

    try {

      std::scoped_lock lock(mutex_controller_list_);

      ROS_INFO("[ControlManager]: activating the controller '%s'", _failsafe_controller_name_.c_str());
      controller_list_.at(_failsafe_controller_idx_)->activate(last_control_output);

      {
        std::scoped_lock lock(mutex_controller_tracker_switch_time_);

        // update the time (used in failsafe)
        controller_tracker_switch_time_ = ros::Time::now();
      }

      failsafe_triggered_ = true;
      ROS_DEBUG("[ControlManager]: stopping eland timer");
      timer_eland_.stop();
      ROS_DEBUG("[ControlManager]: eland timer stopped");

      landing_uav_mass_ = getMass();

      eland_triggered_ = false;
      ROS_DEBUG("[ControlManager]: starting failsafe timer");
      timer_failsafe_.start();
      ROS_DEBUG("[ControlManager]: failsafe timer started");

      bumper_enabled_ = false;

      odometryCallbacksSrv(false);

      callbacks_enabled_ = false;

      ROS_INFO_THROTTLE(1.0, "[ControlManager]: the controller '%s' was activated", _failsafe_controller_name_.c_str());

      // super important, switch the active controller idx
      try {
        controller_list_.at(active_controller_idx_)->deactivate();
        active_controller_idx_ = _failsafe_controller_idx_;
      }
      catch (std::runtime_error& exrun) {
        ROS_ERROR_THROTTLE(1.0, "[ControlManager]: could not deactivate the controller '%s'", _controller_names_.at(active_controller_idx_).c_str());
      }
    }
    catch (std::runtime_error& exrun) {
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: error during activation of the controller '%s'", _failsafe_controller_name_.c_str());
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: exception: '%s'", exrun.what());
    }
  }

  return std::tuple(true, "failsafe activated");
}

//}

/* escalatingFailsafe() //{ */

std::tuple<bool, std::string> ControlManager::escalatingFailsafe(void) {

  std::stringstream ss;

  if ((ros::Time::now() - escalating_failsafe_time_).toSec() < _escalating_failsafe_timeout_) {

    ss << "too soon for escalating failsafe";
    ROS_WARN_STREAM_THROTTLE(0.1, "[ControlManager]: " << ss.str());

    return std::tuple(false, ss.str());
  }

  if (!output_enabled_) {

    ss << "not escalating failsafe, output is disabled";
    ROS_WARN_STREAM_THROTTLE(0.1, "[ControlManager]: " << ss.str());

    return std::tuple(false, ss.str());
  }

  ROS_WARN("[ControlManager]: escalating failsafe triggered");

  auto active_tracker_idx    = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);

  std::string active_tracker_name    = _tracker_names_.at(active_tracker_idx);
  std::string active_controller_name = _controller_names_.at(active_controller_idx);

  EscalatingFailsafeStates_t next_state = getNextEscFailsafeState();

  escalating_failsafe_time_ = ros::Time::now();

  switch (next_state) {

    case ESC_NONE_STATE: {

      ss << "escalating failsafe has run to impossible situation";
      ROS_ERROR_STREAM_THROTTLE(0.1, "[ControlManager]: " << ss.str());

      return std::tuple(false, "escalating failsafe has run to impossible situation");

      break;
    }

    case ESC_EHOVER_STATE: {

      ss << "escalating failsafe escalates to ehover";
      ROS_WARN_STREAM_THROTTLE(0.1, "[ControlManager]: " << ss.str());

      auto [success, message] = ehover();

      if (success) {
        state_escalating_failsafe_ = ESC_EHOVER_STATE;
      }

      return {success, message};

      break;
    }

    case ESC_ELAND_STATE: {

      ss << "escalating failsafe escalates to eland";
      ROS_WARN_STREAM_THROTTLE(0.1, "[ControlManager]: " << ss.str());

      auto [success, message] = eland();

      if (success) {
        state_escalating_failsafe_ = ESC_ELAND_STATE;
      }

      return {success, message};

      break;
    }

    case ESC_FAILSAFE_STATE: {

      escalating_failsafe_time_ = ros::Time::now();

      ss << "escalating failsafe escalates to failsafe";
      ROS_WARN_STREAM_THROTTLE(0.1, "[ControlManager]: " << ss.str());

      auto [success, message] = failsafe();

      if (success) {
        state_escalating_failsafe_ = ESC_FINISHED_STATE;
      }

      return {success, message};

      break;
    }

    case ESC_FINISHED_STATE: {

      escalating_failsafe_time_ = ros::Time::now();

      ss << "escalating failsafe has nothing more to do";
      ROS_WARN_STREAM_THROTTLE(0.1, "[ControlManager]: " << ss.str());

      return std::tuple(false, "escalating failsafe has nothing more to do");

      break;
    }

    default: {

      break;
    }
  }

  ROS_ERROR("[ControlManager]: escalatingFailsafe() reached the final return, this should not happen!");

  return std::tuple(false, "escalating failsafe exception");
}

//}

/* getNextEscFailsafeState() //{ */

EscalatingFailsafeStates_t ControlManager::getNextEscFailsafeState(void) {

  EscalatingFailsafeStates_t current_state = state_escalating_failsafe_;

  switch (current_state) {

    case ESC_FINISHED_STATE: {

      return ESC_FINISHED_STATE;

      break;
    }

    case ESC_NONE_STATE: {

      if (_escalating_failsafe_ehover_) {
        return ESC_EHOVER_STATE;
      } else if (_escalating_failsafe_eland_) {
        return ESC_ELAND_STATE;
      } else if (_escalating_failsafe_failsafe_) {
        return ESC_FAILSAFE_STATE;
      } else {
        return ESC_FINISHED_STATE;
      }

      break;
    }

    case ESC_EHOVER_STATE: {

      if (_escalating_failsafe_eland_) {
        return ESC_ELAND_STATE;
      } else if (_escalating_failsafe_failsafe_) {
        return ESC_FAILSAFE_STATE;
      } else {
        return ESC_FINISHED_STATE;
      }

      break;
    }

    case ESC_ELAND_STATE: {

      if (_escalating_failsafe_failsafe_) {
        return ESC_FAILSAFE_STATE;
      } else {
        return ESC_FINISHED_STATE;
      }

      break;
    }

    case ESC_FAILSAFE_STATE: {

      return ESC_FINISHED_STATE;

      break;
    }
  }

  ROS_ERROR("[ControlManager]: getNextEscFailsafeState() reached the final return, this should not happen!");

  return ESC_NONE_STATE;
}

//}

// | ------------------- trajectory tracking ------------------ |

/* startTrajectoryTracking() //{ */

std::tuple<bool, std::string> ControlManager::startTrajectoryTracking(void) {

  if (!is_initialized_) {
    return std::tuple(false, "the ControlManager is not initialized");
  }

  {
    std::scoped_lock lock(mutex_tracker_list_);

    std_srvs::TriggerResponse::ConstPtr response;
    std_srvs::TriggerRequest            request;

    response =
        tracker_list_.at(active_tracker_idx_)->startTrajectoryTracking(std_srvs::TriggerRequest::ConstPtr(std::make_unique<std_srvs::TriggerRequest>(request)));

    if (response != std_srvs::TriggerResponse::Ptr()) {

      return std::tuple(response->success, response->message);

    } else {

      std::stringstream ss;
      ss << "the tracker '" << _tracker_names_.at(active_tracker_idx_) << "' does not implement the 'startTrajectoryTracking()' function!";

      return std::tuple(false, ss.str());
    }
  }
}

//}

/* stopTrajectoryTracking() //{ */

std::tuple<bool, std::string> ControlManager::stopTrajectoryTracking(void) {

  if (!is_initialized_) {
    return std::tuple(false, "the ControlManager is not initialized");
  }

  {
    std::scoped_lock lock(mutex_tracker_list_);

    std_srvs::TriggerResponse::ConstPtr response;
    std_srvs::TriggerRequest            request;

    response =
        tracker_list_.at(active_tracker_idx_)->stopTrajectoryTracking(std_srvs::TriggerRequest::ConstPtr(std::make_unique<std_srvs::TriggerRequest>(request)));

    if (response != std_srvs::TriggerResponse::Ptr()) {

      return std::tuple(response->success, response->message);

    } else {

      std::stringstream ss;
      ss << "the tracker '" << _tracker_names_.at(active_tracker_idx_) << "' does not implement the 'stopTrajectoryTracking()' function!";

      return std::tuple(false, ss.str());
    }
  }
}

//}

/* resumeTrajectoryTracking() //{ */

std::tuple<bool, std::string> ControlManager::resumeTrajectoryTracking(void) {

  if (!is_initialized_) {
    return std::tuple(false, "the ControlManager is not initialized");
  }

  {
    std::scoped_lock lock(mutex_tracker_list_);

    std_srvs::TriggerResponse::ConstPtr response;
    std_srvs::TriggerRequest            request;

    response = tracker_list_.at(active_tracker_idx_)
                   ->resumeTrajectoryTracking(std_srvs::TriggerRequest::ConstPtr(std::make_unique<std_srvs::TriggerRequest>(request)));

    if (response != std_srvs::TriggerResponse::Ptr()) {

      return std::tuple(response->success, response->message);

    } else {

      std::stringstream ss;
      ss << "the tracker '" << _tracker_names_.at(active_tracker_idx_) << "' does not implement the 'resumeTrajectoryTracking()' function!";

      return std::tuple(false, ss.str());
    }
  }
}

//}

/* gotoTrajectoryStart() //{ */

std::tuple<bool, std::string> ControlManager::gotoTrajectoryStart(void) {

  if (!is_initialized_) {
    return std::tuple(false, "the ControlManager is not initialized");
  }

  {
    std::scoped_lock lock(mutex_tracker_list_);

    std_srvs::TriggerResponse::ConstPtr response;
    std_srvs::TriggerRequest            request;

    response =
        tracker_list_.at(active_tracker_idx_)->gotoTrajectoryStart(std_srvs::TriggerRequest::ConstPtr(std::make_unique<std_srvs::TriggerRequest>(request)));

    if (response != std_srvs::TriggerResponse::Ptr()) {

      return std::tuple(response->success, response->message);

    } else {

      std::stringstream ss;
      ss << "the tracker '" << _tracker_names_.at(active_tracker_idx_) << "' does not implement the 'gotoTrajectoryStart()' function!";

      return std::tuple(false, ss.str());
    }
  }
}

//}

// | ----------------- service client wrappers ---------------- |

/* arming() //{ */

std::tuple<bool, std::string> ControlManager::arming(const bool input) {

  std::stringstream ss;

  if (input) {

    ss << "not allowed to arm using the ControlManager, maybe later when we don't do bugs";
    ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  if (!input && !isOffboard()) {

    ss << "can not disarm, not in OFFBOARD mode";
    ROS_WARN_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  if (!input && _rc_emergency_handoff_) {

    toggleOutput(false);

    return std::tuple(true, "RC emergency handoff is ON, disabling output");
  }

  std_srvs::SetBool srv_out;

  srv_out.request.data = input ? 1 : 0;  // arm or disarm?

  ROS_INFO("[ControlManager]: calling for %s", input ? "arming" : "disarming");

  if (sch_arming_.call(srv_out)) {

    if (srv_out.response.success) {

      ss << "service call for " << (input ? "arming" : "disarming") << " was successful";
      ROS_INFO_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());

      if (!input) {

        toggleOutput(false);

        ROS_DEBUG("[ControlManager]: stopping failsafe timer");
        timer_failsafe_.stop();
        ROS_DEBUG("[ControlManager]: failsafe timer stopped");

        ROS_DEBUG("[ControlManager]: stopping the eland timer");
        timer_eland_.stop();
        ROS_DEBUG("[ControlManager]: eland timer stopped");
      }

    } else {
      ss << "service call for " << (input ? "arming" : "disarming") << " failed";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
    }

  } else {
    ss << "calling for " << (input ? "arming" : "disarming") << " resulted in failure: '" << srv_out.response.message << "'";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[ControlManager]: " << ss.str());
  }

  return std::tuple(srv_out.response.success, ss.str());
}

//}

/* odometryCallbacksSrv() //{ */

void ControlManager::odometryCallbacksSrv(const bool input) {

  ROS_INFO("[ControlManager]: switching odometry callbacks to %s", input ? "ON" : "OFF");

  std_srvs::SetBool srv;

  srv.request.data = input;

  bool res = sch_set_odometry_callbacks_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN("[ControlManager]: service call for toggle odometry callbacks returned: '%s'", srv.response.message.c_str());
    }

  } else {
    ROS_ERROR("[ControlManager]: service call for toggle odometry callbacks failed!");
  }
}

//}

/* elandSrv() //{ */

bool ControlManager::elandSrv(void) {

  ROS_INFO("[ControlManager]: calling for eland");

  std_srvs::Trigger srv;

  bool res = sch_eland_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN("[ControlManager]: service call for eland returned: '%s'", srv.response.message.c_str());
    }

    return srv.response.success;

  } else {

    ROS_ERROR("[ControlManager]: service call for eland failed!");

    return false;
  }
}

//}

/* parachuteSrv() //{ */

bool ControlManager::parachuteSrv(void) {

  ROS_INFO("[ControlManager]: calling for parachute deployment");

  std_srvs::Trigger srv;

  bool res = sch_parachute_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN("[ControlManager]: service call for parachute deployment returned: '%s'", srv.response.message.c_str());
    }

    return srv.response.success;

  } else {

    ROS_ERROR("[ControlManager]: service call for parachute deployment failed!");

    return false;
  }
}

//}

/* ungripSrv() //{ */

void ControlManager::ungripSrv(void) {

  std_srvs::Trigger srv;

  bool res = sch_ungrip_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_DEBUG_THROTTLE(1.0, "[ControlManager]: service call for ungripping payload returned: '%s'", srv.response.message.c_str());
    }

  } else {
    ROS_DEBUG_THROTTLE(1.0, "[ControlManager]: service call for ungripping payload failed!");
  }
}

//}

// | ------------------------ routines ------------------------ |

/* toggleOutput() //{ */

void ControlManager::toggleOutput(const bool& input) {

  if (input == output_enabled_) {
    ROS_WARN_THROTTLE(0.1, "[ControlManager]: output is already %s", input ? "ON" : "OFF");
    return;
  }

  ROS_INFO("[ControlManager]: switching output %s", input ? "ON" : "OFF");

  output_enabled_ = input;

  // if switching output off, switch to NullTracker
  if (!output_enabled_) {

    ROS_INFO("[ControlManager]: switching to 'NullTracker' after switching output off");

    switchTracker(_null_tracker_name_);

    ROS_INFO_STREAM("[ControlManager]: switching to the controller '" << _eland_controller_name_ << "' after switching output off");

    switchController(_eland_controller_name_);

    // | --------- deactivate all trackers and controllers -------- |

    for (int i = 0; i < int(tracker_list_.size()); i++) {

      std::map<std::string, TrackerParams>::iterator it;
      it = trackers_.find(_tracker_names_.at(i));

      try {
        ROS_INFO("[ControlManager]: deactivating the tracker '%s'", it->second.address.c_str());
        tracker_list_.at(i)->deactivate();
      }
      catch (std::runtime_error& ex) {
        ROS_ERROR("[ControlManager]: exception caught during tracker deactivation: '%s'", ex.what());
      }
    }

    for (int i = 0; i < int(controller_list_.size()); i++) {

      std::map<std::string, ControllerParams>::iterator it;
      it = controllers_.find(_controller_names_.at(i));

      try {
        ROS_INFO("[ControlManager]: deactivating the controller '%s'", it->second.address.c_str());
        controller_list_.at(i)->deactivate();
      }
      catch (std::runtime_error& ex) {
        ROS_ERROR("[ControlManager]: exception caught during controller deactivation: '%s'", ex.what());
      }
    }

    timer_failsafe_.stop();
    timer_eland_.stop();
    timer_pirouette_.stop();

    offboard_mode_was_true_ = false;
  }
}

//}

/* switchTracker() //{ */

std::tuple<bool, std::string> ControlManager::switchTracker(const std::string& tracker_name) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("switchTracker");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::switchTracker", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto last_tracker_cmd   = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);
  auto active_tracker_idx = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);

  std::stringstream ss;

  if (!got_uav_state_) {

    ss << "can not switch tracker, missing odometry!";
    ROS_ERROR_STREAM("[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  if (_state_input_ == INPUT_UAV_STATE && _odometry_innovation_check_enabled_ && !sh_odometry_innovation_.hasMsg()) {

    ss << "can not switch tracker, missing odometry innovation!";
    ROS_ERROR_STREAM("[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  auto new_tracker_idx = idxInVector(tracker_name, _tracker_names_);

  // check if the tracker exists
  if (!new_tracker_idx) {
    ss << "the tracker '" << tracker_name << "' does not exist!";
    ROS_ERROR_STREAM("[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  // check if the tracker is already active
  if (new_tracker_idx.value() == active_tracker_idx) {
    ss << "not switching, the tracker '" << tracker_name << "' is already active!";
    ROS_INFO_STREAM("[ControlManager]: " << ss.str());
    return std::tuple(true, ss.str());
  }

  {
    std::scoped_lock lock(mutex_tracker_list_);

    try {

      ROS_INFO("[ControlManager]: activating the tracker '%s'", _tracker_names_.at(new_tracker_idx.value()).c_str());

      auto [success, message] = tracker_list_.at(new_tracker_idx.value())->activate(last_tracker_cmd);

      if (!success) {

        ss << "the tracker '" << tracker_name << "' could not be activated: '" << message << "'";
        ROS_ERROR_STREAM("[ControlManager]: " << ss.str());
        return std::tuple(false, ss.str());

      } else {

        ss << "the tracker '" << tracker_name << "' was activated";
        ROS_INFO_STREAM("[ControlManager]: " << ss.str());

        {
          std::scoped_lock lock(mutex_controller_tracker_switch_time_);

          // update the time (used in failsafe)
          controller_tracker_switch_time_ = ros::Time::now();
        }

        // super important, switch the active tracker idx
        try {

          ROS_INFO("[ControlManager]: deactivating '%s'", _tracker_names_.at(active_tracker_idx_).c_str());
          tracker_list_.at(active_tracker_idx_)->deactivate();

          // if switching from null tracker, re-activate the already active the controller
          if (active_tracker_idx_ == _null_tracker_idx_) {

            ROS_INFO("[ControlManager]: reactivating '%s' due to switching from 'NullTracker'", _controller_names_.at(active_controller_idx_).c_str());
            {
              std::scoped_lock lock(mutex_controller_list_);

              initializeControlOutput();

              auto last_control_output = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);

              controller_list_.at(active_controller_idx_)->activate(last_control_output);

              {
                std::scoped_lock lock(mutex_controller_tracker_switch_time_);

                // update the time (used in failsafe)
                controller_tracker_switch_time_ = ros::Time::now();
              }
            }

            // if switching to null tracker, deactivate the active controller
          } else if (new_tracker_idx == _null_tracker_idx_) {

            ROS_INFO("[ControlManager]: deactivating '%s' due to switching to 'NullTracker'", _controller_names_.at(active_controller_idx_).c_str());
            {
              std::scoped_lock lock(mutex_controller_list_);

              controller_list_.at(active_controller_idx_)->deactivate();
            }

            {
              std::scoped_lock lock(mutex_last_tracker_cmd_);

              last_tracker_cmd_ = {};
            }

            initializeControlOutput();
          }

          active_tracker_idx_ = new_tracker_idx.value();
        }
        catch (std::runtime_error& exrun) {
          ROS_ERROR("[ControlManager]: could not deactivate the tracker '%s'", _tracker_names_.at(active_tracker_idx_).c_str());
        }
      }
    }
    catch (std::runtime_error& exrun) {
      ROS_ERROR("[ControlManager]: error during activation of the tracker '%s'", tracker_name.c_str());
      ROS_ERROR("[ControlManager]: exception: '%s'", exrun.what());
    }
  }

  return std::tuple(true, ss.str());
}

//}

/* switchController() //{ */

std::tuple<bool, std::string> ControlManager::switchController(const std::string& controller_name) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("switchController");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::switchController", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto last_control_output   = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);

  std::stringstream ss;

  if (!got_uav_state_) {

    ss << "can not switch controller, missing odometry!";
    ROS_ERROR_STREAM("[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  if (_state_input_ == INPUT_UAV_STATE && _odometry_innovation_check_enabled_ && !sh_odometry_innovation_.hasMsg()) {

    ss << "can not switch controller, missing odometry innovation!";
    ROS_ERROR_STREAM("[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  auto new_controller_idx = idxInVector(controller_name, _controller_names_);

  // check if the controller exists
  if (!new_controller_idx) {
    ss << "the controller '" << controller_name << "' does not exist!";
    ROS_ERROR_STREAM("[ControlManager]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  // check if the controller is not active
  if (new_controller_idx.value() == active_controller_idx) {

    ss << "not switching, the controller '" << controller_name << "' is already active!";
    ROS_INFO_STREAM("[ControlManager]: " << ss.str());
    return std::tuple(true, ss.str());
  }

  {
    std::scoped_lock lock(mutex_controller_list_);

    try {

      ROS_INFO("[ControlManager]: activating the controller '%s'", _controller_names_.at(new_controller_idx.value()).c_str());
      if (!controller_list_.at(new_controller_idx.value())->activate(last_control_output)) {

        ss << "the controller '" << controller_name << "' was not activated";
        ROS_ERROR_STREAM("[ControlManager]: " << ss.str());
        return std::tuple(false, ss.str());

      } else {

        ss << "the controller '" << controller_name << "' was activated";
        ROS_INFO_STREAM("[ControlManager]: " << ss.str());

        ROS_INFO("[ControlManager]: triggering hover after switching to '%s', re-activating '%s'", _controller_names_.at(new_controller_idx.value()).c_str(),
                 _tracker_names_.at(active_tracker_idx_).c_str());

        // reactivate the current tracker
        // TODO this is not the most elegant way to restart the tracker after a controller switch
        // but it serves the purpose
        {
          std::scoped_lock lock(mutex_tracker_list_);

          tracker_list_.at(active_tracker_idx_)->deactivate();
          tracker_list_.at(active_tracker_idx_)->activate({});
        }

        {
          std::scoped_lock lock(mutex_controller_tracker_switch_time_);

          // update the time (used in failsafe)
          controller_tracker_switch_time_ = ros::Time::now();
        }

        // super important, switch the active controller idx
        try {

          controller_list_.at(active_controller_idx_)->deactivate();
          active_controller_idx_ = new_controller_idx.value();
        }
        catch (std::runtime_error& exrun) {
          ROS_ERROR("[ControlManager]: could not deactivate controller '%s'", _controller_names_.at(active_controller_idx_).c_str());
        }
      }
    }
    catch (std::runtime_error& exrun) {
      ROS_ERROR("[ControlManager]: error during activation of controller '%s'", controller_name.c_str());
      ROS_ERROR("[ControlManager]: exception: '%s'", exrun.what());
    }
  }

  mrs_msgs::DynamicsConstraintsSrvRequest sanitized_constraints;

  {
    std::scoped_lock lock(mutex_constraints_);

    sanitized_constraints_ = current_constraints_;
    sanitized_constraints  = sanitized_constraints_;
  }

  setConstraintsToControllers(sanitized_constraints);

  return std::tuple(true, ss.str());
}

//}

/* updateTrackers() //{ */

void ControlManager::updateTrackers(void) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("updateTrackers");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::updateTrackers", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto uav_state           = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_control_output = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);

  // --------------------------------------------------------------
  // |                     Update the trackers                    |
  // --------------------------------------------------------------

  std::optional<mrs_msgs::TrackerCommand> tracker_command;

  unsigned int active_tracker_idx;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    active_tracker_idx = active_tracker_idx_;

    // for each tracker
    for (size_t i = 0; i < tracker_list_.size(); i++) {

      if (i == active_tracker_idx) {

        try {
          // active tracker => update and retrieve the command
          tracker_command = tracker_list_.at(i)->update(uav_state, last_control_output);
        }
        catch (std::runtime_error& exrun) {
          ROS_ERROR_THROTTLE(1.0, "[ControlManager]: caught an exception while updating the active tracker (%s)",
                             _tracker_names_.at(active_tracker_idx).c_str());
          ROS_ERROR_THROTTLE(1.0, "[ControlManager]: the exception: '%s'", exrun.what());
          tracker_command = {};
        }

      } else {

        try {
          // nonactive tracker => just update without retrieving the command
          tracker_list_.at(i)->update(uav_state, last_control_output);
        }
        catch (std::runtime_error& exrun) {
          ROS_ERROR_THROTTLE(1.0, "[ControlManager]: caught an exception while updating the tracker '%s'", _tracker_names_.at(i).c_str());
          ROS_ERROR_THROTTLE(1.0, "[ControlManager]: the exception: '%s'", exrun.what());
        }
      }
    }

    if (active_tracker_idx == _null_tracker_idx_) {
      return;
    }
  }

  if (validateTrackerCommand(tracker_command, "ControlManager", "tracker_command")) {

    std::scoped_lock lock(mutex_last_tracker_cmd_);

    last_tracker_cmd_ = tracker_command;

    // | --------- fill in the potentially missing header --------- |

    if (last_tracker_cmd_->header.frame_id == "") {
      last_tracker_cmd_->header.frame_id = uav_state.header.frame_id;
    }

    if (last_tracker_cmd_->header.stamp == ros::Time(0)) {
      last_tracker_cmd_->header.stamp = ros::Time::now();
    }

  } else {

    if (active_tracker_idx == _ehover_tracker_idx_) {

      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: the emergency tracker '%s' returned empty or invalid command!",
                         _tracker_names_.at(active_tracker_idx).c_str());
      failsafe();

    } else {

      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: the tracker '%s' returned empty or invalid command!", _tracker_names_.at(active_tracker_idx).c_str());

      if (_tracker_error_action_ == ELAND_STR) {
        eland();
      } else if (_tracker_error_action_ == EHOVER_STR) {
        ehover();
      } else {
        failsafe();
      }
    }
  }
}

//}

/* updateControllers() //{ */

void ControlManager::updateControllers(const mrs_msgs::UavState& uav_state) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("updateControllers");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::updateControllers", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  // | ----------------- update the controllers ----------------- |

  // the trackers are not running
  if (!last_tracker_cmd) {

    {
      std::scoped_lock lock(mutex_controller_list_);

      // nonactive controller => just update without retrieving the command
      for (int i = 0; i < int(controller_list_.size()); i++) {
        controller_list_.at(i)->updateInactive(uav_state, last_tracker_cmd);
      }
    }

    return;
  }

  Controller::ControlOutput control_output;

  unsigned int active_controller_idx;

  {
    std::scoped_lock lock(mutex_controller_list_);

    active_controller_idx = active_controller_idx_;

    // for each controller
    for (size_t i = 0; i < controller_list_.size(); i++) {

      if (i == active_controller_idx) {

        try {
          // active controller => update and retrieve the command
          control_output = controller_list_.at(active_controller_idx)->updateActive(uav_state, last_tracker_cmd.value());
        }
        catch (std::runtime_error& exrun) {

          ROS_ERROR_THROTTLE(1.0, "[ControlManager]: an exception while updating the active controller (%s)",
                             _controller_names_.at(active_controller_idx).c_str());
          ROS_ERROR_THROTTLE(1.0, "[ControlManager]: the exception: '%s'", exrun.what());
        }

      } else {

        try {
          // nonactive controller => just update without retrieving the command
          controller_list_.at(i)->updateInactive(uav_state, last_tracker_cmd);
        }
        catch (std::runtime_error& exrun) {

          ROS_ERROR_THROTTLE(1.0, "[ControlManager]: exception while updating the controller '%s'", _controller_names_.at(i).c_str());
          ROS_ERROR_THROTTLE(1.0, "[ControlManager]: exception: '%s'", exrun.what());
        }
      }
    }
  }

  // normally, the active controller returns a valid command
  if (validateControlOutput(control_output, _hw_api_inputs_, "ControlManager", "control_output")) {

    mrs_lib::set_mutexed(mutex_last_control_output_, control_output, last_control_output_);

    // but it can return an empty command, due to some critical internal error
    // which means we should trigger the failsafe landing
  } else {

    // only if the controller is still active, trigger escalating failsafe
    // if not active, we don't care, we should not ask the controller for
    // the result anyway -> this could mean a race condition occured
    // like it once happend during landing
    bool controller_status = false;

    {
      std::scoped_lock lock(mutex_controller_list_);

      controller_status = controller_list_.at(active_controller_idx)->getStatus().active;
    }

    if (controller_status) {

      if (failsafe_triggered_) {

        ROS_ERROR("[ControlManager]: disabling control, the active controller returned an empty command when failsafe was active");

        toggleOutput(false);

      } else if (eland_triggered_) {

        ROS_ERROR("[ControlManager]: triggering failsafe, the active controller returned an empty command when eland was active");

        failsafe();

      } else {

        ROS_ERROR("[ControlManager]: triggering eland, the active controller returned an empty command");

        eland();
      }
    }
  }
}

//}

/* publish() //{ */

void ControlManager::publish(void) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("publish");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ControlManager::publish", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto last_control_output   = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);
  auto last_tracker_cmd      = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);
  auto active_tracker_idx    = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);
  auto uav_state             = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  publishControlReferenceOdom(last_tracker_cmd, last_control_output);

  // --------------------------------------------------------------
  // |                 Publish the control command                |
  // --------------------------------------------------------------

  mrs_msgs::HwApiAttitudeCmd attitude_target;
  attitude_target.stamp = ros::Time::now();

  mrs_msgs::HwApiAttitudeRateCmd attitude_rate_target;
  attitude_rate_target.stamp = ros::Time::now();

  if (!output_enabled_) {

    ROS_WARN_THROTTLE(1.0, "[ControlManager]: output is disabled");

  } else if (active_tracker_idx == _null_tracker_idx_) {

    ROS_WARN_THROTTLE(5.0, "[ControlManager]: 'NullTracker' is active, not controlling");

    Controller::HwApiOutputVariant output =
        initializeDefaultOutput(_hw_api_inputs_, uav_state, _min_throttle_null_tracker_, common_handlers_->throttle_model.n_motors);

    {
      std::scoped_lock lock(mutex_last_control_output_);

      last_control_output_.control_output = output;
    }

    control_output_publisher_.publish(output);

  } else if (active_tracker_idx != _null_tracker_idx_ && !last_control_output.control_output) {

    ROS_WARN_THROTTLE(1.0, "[ControlManager]: the controller '%s' returned nil command, not publishing anything",
                      _controller_names_.at(active_controller_idx).c_str());

    Controller::HwApiOutputVariant output =
        initializeDefaultOutput(_hw_api_inputs_, uav_state, _min_throttle_null_tracker_, common_handlers_->throttle_model.n_motors);

    control_output_publisher_.publish(output);

  } else if (last_control_output.control_output) {

    if (validateHwApiAttitudeCmd(attitude_target, "ControlManager", "last_control_output.control_output")) {
      control_output_publisher_.publish(last_control_output.control_output.value());
    } else {
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: the attitude cmd is not valid just before publishing!");
      return;
    }

  } else {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: not publishing a control command");
  }

  // | ----------- publish the controller diagnostics ----------- |

  ph_controller_diagnostics_.publish(last_control_output.diagnostics);

  // | --------- publish the applied throttle and thrust -------- |

  auto throttle = extractThrottle(last_control_output);

  if (throttle) {

    {
      std_msgs::Float64 msg;
      msg.data = throttle.value();
      ph_throttle_.publish(msg);
    }

    double thrust = mrs_lib::quadratic_throttle_model::throttleToForce(common_handlers_->throttle_model, throttle.value());

    {
      std_msgs::Float64 msg;
      msg.data = thrust;
      ph_thrust_.publish(msg);
    }
  }

  // | ----------------- publish tracker command ---------------- |

  if (last_tracker_cmd) {
    ph_tracker_cmd_.publish(last_tracker_cmd.value());
  }

  // | --------------- publish the odometry input --------------- |

  if (last_control_output.control_output) {

    mrs_msgs::EstimatorInput msg;

    msg.header.frame_id = _uav_name_ + "/fcu";
    msg.header.stamp    = ros::Time::now();

    if (last_control_output.desired_unbiased_acceleration) {
      msg.control_acceleration.x = last_control_output.desired_unbiased_acceleration.value()(0);
      msg.control_acceleration.y = last_control_output.desired_unbiased_acceleration.value()(1);
      msg.control_acceleration.z = last_control_output.desired_unbiased_acceleration.value()(2);
    }

    if (last_control_output.desired_heading_rate) {
      msg.control_hdg_rate = last_control_output.desired_heading_rate.value();
    }

    if (last_control_output.desired_unbiased_acceleration) {
      ph_mrs_odom_input_.publish(msg);
    }
  }
}

//}

/* deployParachute() //{ */

std::tuple<bool, std::string> ControlManager::deployParachute(void) {

  // if not enabled, return false
  if (!_parachute_enabled_) {

    std::stringstream ss;
    ss << "can not deploy parachute, it is disabled";
    return std::tuple(false, ss.str());
  }

  // we can not disarm if the drone is not in offboard mode
  // this is super important!
  if (!isOffboard()) {

    std::stringstream ss;
    ss << "can not deploy parachute, not in offboard mode";
    return std::tuple(false, ss.str());
  }

  // call the parachute service
  bool succ = parachuteSrv();

  // if the deployment was successful,
  if (succ) {

    arming(false);

    std::stringstream ss;
    ss << "parachute deployed";

    return std::tuple(true, ss.str());

  } else {

    std::stringstream ss;
    ss << "error during deployment of parachute";

    return std::tuple(false, ss.str());
  }
}

//}

/* velocityReferenceToReference() //{ */

mrs_msgs::ReferenceStamped ControlManager::velocityReferenceToReference(const mrs_msgs::VelocityReferenceStamped& vel_reference) {

  auto last_tracker_cmd    = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);
  auto uav_state           = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto current_constraints = mrs_lib::get_mutexed(mutex_constraints_, current_constraints_);

  mrs_msgs::ReferenceStamped reference_out;

  reference_out.header = vel_reference.header;

  if (vel_reference.reference.use_heading) {
    reference_out.reference.heading = vel_reference.reference.heading;
  } else if (vel_reference.reference.use_heading_rate) {
    reference_out.reference.heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading() + vel_reference.reference.use_heading_rate;
  } else {
    reference_out.reference.heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }

  if (vel_reference.reference.use_altitude) {
    reference_out.reference.position.z = vel_reference.reference.altitude;
  } else {

    double stopping_time_z = 0;

    if (vel_reference.reference.velocity.x >= 0) {
      stopping_time_z = 1.5 * (fabs(vel_reference.reference.velocity.z) / current_constraints.constraints.vertical_ascending_acceleration) + 1.0;
    } else {
      stopping_time_z = 1.5 * (fabs(vel_reference.reference.velocity.z) / current_constraints.constraints.vertical_descending_acceleration) + 1.0;
    }

    reference_out.reference.position.z = last_tracker_cmd->position.z + vel_reference.reference.velocity.z * stopping_time_z;
  }

  {
    double stopping_time_x = 1.5 * (fabs(vel_reference.reference.velocity.x) / current_constraints.constraints.horizontal_acceleration) + 1.0;
    double stopping_time_y = 1.5 * (fabs(vel_reference.reference.velocity.y) / current_constraints.constraints.horizontal_acceleration) + 1.0;

    reference_out.reference.position.x = last_tracker_cmd->position.x + vel_reference.reference.velocity.x * stopping_time_x;
    reference_out.reference.position.y = last_tracker_cmd->position.y + vel_reference.reference.velocity.y * stopping_time_y;
  }

  return reference_out;
}

//}

/* publishControlReferenceOdom() //{ */

void ControlManager::publishControlReferenceOdom([[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand>& tracker_command,
                                                 [[maybe_unused]] const Controller::ControlOutput&               control_output) {

  if (!tracker_command || !control_output.control_output) {
    return;
  }

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  nav_msgs::Odometry msg;

  msg.header = tracker_command->header;

  if (tracker_command->use_position_horizontal) {
    msg.pose.pose.position.x = tracker_command->position.x;
    msg.pose.pose.position.y = tracker_command->position.y;
  } else {
    msg.pose.pose.position.x = uav_state.pose.position.x;
    msg.pose.pose.position.y = uav_state.pose.position.y;
  }

  if (tracker_command->use_position_vertical) {
    msg.pose.pose.position.z = tracker_command->position.z;
  } else {
    msg.pose.pose.position.z = uav_state.pose.position.z;
  }

  // transform the velocity in the reference to the child_frame
  if (tracker_command->use_velocity_horizontal || tracker_command->use_velocity_vertical) {

    msg.child_frame_id = _uav_name_ + "/" + _body_frame_;

    geometry_msgs::Vector3Stamped velocity;
    velocity.header = tracker_command->header;

    if (tracker_command->use_velocity_horizontal) {
      velocity.vector.x = tracker_command->velocity.x;
      velocity.vector.y = tracker_command->velocity.y;
    }

    if (tracker_command->use_velocity_vertical) {
      velocity.vector.z = tracker_command->velocity.z;
    }

    auto res = transformer_->transformSingle(velocity, msg.child_frame_id);

    if (res) {
      msg.twist.twist.linear.x = res.value().vector.x;
      msg.twist.twist.linear.y = res.value().vector.y;
      msg.twist.twist.linear.z = res.value().vector.z;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[ControlManager]: could not transform the reference speed from '%s' to '%s'", velocity.header.frame_id.c_str(),
                         msg.child_frame_id.c_str());
    }
  }

  // fill in the orientation or heading
  if (control_output.desired_orientation) {
    msg.pose.pose.orientation = mrs_lib::AttitudeConverter(control_output.desired_orientation.value());
  } else if (tracker_command->use_heading) {
    msg.pose.pose.orientation = mrs_lib::AttitudeConverter(0, 0, tracker_command->heading);
  }

  // fill in the attitude rate
  if (std::holds_alternative<mrs_msgs::HwApiAttitudeRateCmd>(control_output.control_output.value())) {

    auto attitude_cmd = std::get<mrs_msgs::HwApiAttitudeRateCmd>(control_output.control_output.value());

    msg.twist.twist.angular.x = attitude_cmd.body_rate.x;
    msg.twist.twist.angular.y = attitude_cmd.body_rate.y;
    msg.twist.twist.angular.z = attitude_cmd.body_rate.z;
  }

  ph_control_reference_odom_.publish(msg);
}

//}

/* initializeControlOutput() //{ */

void ControlManager::initializeControlOutput(void) {

  Controller::ControlOutput controller_output;

  controller_output.diagnostics.total_mass      = _uav_mass_;
  controller_output.diagnostics.mass_difference = 0.0;

  controller_output.diagnostics.disturbance_bx_b = _initial_body_disturbance_x_;
  controller_output.diagnostics.disturbance_by_b = _initial_body_disturbance_y_;

  if (std::abs(_initial_body_disturbance_x_) > 0.001 || std::abs(_initial_body_disturbance_y_) > 0.001) {
    controller_output.diagnostics.disturbance_estimator = true;
  }

  controller_output.diagnostics.disturbance_wx_w = 0.0;
  controller_output.diagnostics.disturbance_wy_w = 0.0;

  controller_output.diagnostics.disturbance_bx_w = 0.0;
  controller_output.diagnostics.disturbance_by_w = 0.0;

  controller_output.diagnostics.controller = "none";

  mrs_lib::set_mutexed(mutex_last_control_output_, controller_output, last_control_output_);
}

//}

}  // namespace control_manager

}  // namespace mrs_uav_managers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::control_manager::ControlManager, nodelet::Nodelet)
