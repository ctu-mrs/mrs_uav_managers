/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_managers/control_manager/common.h>
#include <control_manager/output_publisher.h>

#include <mrs_uav_managers/controller.h>
#include <mrs_uav_managers/tracker.h>

#include <mrs_msgs/msg/float64_stamped.hpp>
#include <mrs_msgs/msg/obstacle_sectors.hpp>
#include <mrs_msgs/msg/bool_stamped.hpp>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/safety_area_manager_diagnostics.hpp>
#include <mrs_msgs/msg/dynamics_constraints.hpp>
#include <mrs_msgs/msg/control_error.hpp>
#include <mrs_msgs/msg/tracker_command.hpp>
#include <mrs_msgs/msg/estimator_input.hpp>

#include <mrs_msgs/srv/validate_reference_array.hpp>
#include <mrs_msgs/srv/validate_reference.hpp>
#include <mrs_msgs/srv/get_float64.hpp>
#include <mrs_msgs/srv/string.hpp>
#include <mrs_msgs/srv/float64_stamped_srv.hpp>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <std_msgs/msg/empty.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <mrs_lib/node.h>
#include <mrs_lib/safety_zone.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/quadratic_throttle_model.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/service_server_handler.h>

#include <mrs_msgs/msg/hw_api_capabilities.hpp>
#include <mrs_msgs/msg/hw_api_status.hpp>
#include <mrs_msgs/msg/hw_api_rc_channels.hpp>

#include <mrs_msgs/msg/hw_api_actuator_cmd.hpp>
#include <mrs_msgs/msg/hw_api_control_group_cmd.hpp>
#include <mrs_msgs/msg/hw_api_attitude_rate_cmd.hpp>
#include <mrs_msgs/msg/hw_api_attitude_cmd.hpp>
#include <mrs_msgs/msg/hw_api_acceleration_hdg_rate_cmd.hpp>
#include <mrs_msgs/msg/hw_api_acceleration_hdg_cmd.hpp>
#include <mrs_msgs/msg/hw_api_velocity_hdg_rate_cmd.hpp>
#include <mrs_msgs/msg/hw_api_velocity_hdg_cmd.hpp>
#include <mrs_msgs/msg/hw_api_position_cmd.hpp>

#include <std_msgs/msg/float64.hpp>

#include <future>

#include <pluginlib/class_loader.hpp>

#include <eigen3/Eigen/Eigen>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mrs_msgs/msg/reference.hpp>
#include <mrs_msgs/msg/reference_stamped.hpp>
#include <mrs_msgs/msg/reference_array.hpp>
#include <mrs_msgs/msg/trajectory_reference.hpp>

#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/srv/velocity_reference_stamped_srv.hpp>
#include <mrs_msgs/srv/validate_path_to_point_srv.hpp>
#include <mrs_msgs/srv/get_reference_stamped_srv.hpp>
#include <mrs_msgs/srv/get_bool_srv.hpp>
#include <mrs_msgs/srv/transform_reference_srv.hpp>
#include <mrs_msgs/srv/transform_reference_array_srv.hpp>
#include <mrs_msgs/srv/transform_pose_srv.hpp>
#include <mrs_msgs/srv/transform_vector3_srv.hpp>
#include <mrs_msgs/srv/float64_stamped_srv.hpp>
#include <mrs_msgs/srv/vec4.hpp>
#include <mrs_msgs/srv/vec1.hpp>

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

using namespace std::chrono_literals;

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

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

const char *state_names[2] = {"IDLING", "LANDING"};

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

class ControlManager : public mrs_lib::Node {

public:
  ControlManager(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  trackers_subnode_;
  rclcpp::Node::SharedPtr  controllers_subnode_;
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  std::atomic<bool> is_initialized_ = false;
  std::string       _uav_name_;
  std::string       _body_frame_;

  std::string _uav_dynamics_config_;
  std::string _custom_config_;
  std::string _platform_config_;
  std::string _world_config_;
  std::string _network_config_;

  std::shared_ptr<mrs_lib::ParamLoader> param_loader_;

  // | --------------- dynamic loading of trackers -------------- |

  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_managers::Tracker>> tracker_loader_; // pluginlib loader of dynamically loaded trackers
  std::vector<std::string>                                           _tracker_names_; // list of tracker names
  std::map<std::string, TrackerParams>                               trackers_;       // map between tracker names and tracker param
  std::vector<std::shared_ptr<mrs_uav_managers::Tracker>>            tracker_list_;   // list of trackers, routines are callable from this
  std::mutex                                                         mutex_tracker_list_;

  // | ------------- dynamic loading of controllers ------------- |

  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_managers::Controller>> controller_loader_; // pluginlib loader of dynamically loaded controllers
  std::vector<std::string>                                              _controller_names_; // list of controller names
  std::map<std::string, ControllerParams>                               controllers_;       // map between controller names and controller params
  std::vector<std::shared_ptr<mrs_uav_managers::Controller>>            controller_list_;   // list of controllers, routines are callable from this
  std::mutex                                                            mutex_controller_list_;

  // | ------------------------- HW API ------------------------- |

  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiCapabilities> sh_hw_api_capabilities_;

  OutputPublisher control_output_publisher_;

  ControlOutputModalities_t _hw_api_inputs_;

  double desired_uav_state_rate_ = 100.0;

  // this timer will check till we already got the hardware api diagnostics
  // then it will trigger the initialization of the controllers and finish
  // the initialization of the ControlManager
  rclcpp::TimerBase::SharedPtr timer_hw_api_capabilities_;
  void                         timerHwApiCapabilities();

  void initialize(void);
  void shutdown();

  // | ------------ tracker and controller switching ------------ |

  std::tuple<bool, std::string> switchController(const std::string &controller_name);
  std::tuple<bool, std::string> switchTracker(const std::string &tracker_name);

  // the time of last switching of a tracker or a controller
  rclcpp::Time controller_tracker_switch_time_;
  std::mutex   mutex_controller_tracker_switch_time_;

  // | -------------------- the transformer  -------------------- |

  std::shared_ptr<mrs_lib::Transformer> transformer_;

  // | ------------------- scope timer logger ------------------- |

  bool                                       scope_timer_enabled_ = false;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;

  // | --------------------- general params --------------------- |

  // defines the type of state input: odometry or uav_state mesasge types
  int _state_input_;

  // names of important trackers
  std::string _null_tracker_name_;    // null tracker is active when UAV is not in the air
  std::string _ehover_tracker_name_;  // ehover tracker is used for emergency hovering
  std::string _landoff_tracker_name_; // landoff is used for landing and takeoff

  // names of important controllers
  std::string _failsafe_controller_name_; // controller used for feed-forward failsafe
  std::string _eland_controller_name_;    // controller used for emergancy landing

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

  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> sh_odometry_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::UavState> sh_uav_state_;

  mrs_msgs::msg::UavState uav_state_;
  mrs_msgs::msg::UavState previous_uav_state_;
  bool                    got_uav_state_               = false;
  double                  _uav_state_max_missing_time_ = 0; // how long should we tolerate missing state estimate?
  double                  uav_roll_                    = 0;
  double                  uav_pitch_                   = 0;
  double                  uav_yaw_                     = 0;
  double                  uav_heading_                 = 0;
  std::mutex              mutex_uav_state_;

  // odometry hiccup detection
  double uav_state_avg_dt_        = 1;
  double uav_state_hiccup_factor_ = 1;
  int    uav_state_count_         = 0;

  mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix> sh_gnss_;

  // | -------------- safety area max z subscriber -------------- |

  mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped> sh_max_z_;

  // | ------------- odometry innovation subscriber ------------- |

  // odometry innovation is published by the odometry node
  // it is used to issue eland if the estimator's input is too wonky
  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> sh_odometry_innovation_;

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

  void              toggleOutput(const bool &input);
  std::atomic<bool> output_enabled_ = false;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mrs_msgs::msg::ControllerDiagnostics>     ph_controller_diagnostics_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::TrackerCommand>            ph_tracker_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::EstimatorInput>            ph_mrs_odom_input_;
  mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>                  ph_control_reference_odom_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::ControlManagerDiagnostics> ph_diagnostics_;
  mrs_lib::PublisherHandler<std_msgs::msg::Empty>                     ph_offboard_on_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>            ph_tilt_error_;
  mrs_lib::PublisherHandler<std_msgs::msg::Float64>                   ph_mass_estimate_;
  mrs_lib::PublisherHandler<std_msgs::msg::Float64>                   ph_mass_nominal_;
  mrs_lib::PublisherHandler<std_msgs::msg::Float64>                   ph_throttle_;
  mrs_lib::PublisherHandler<std_msgs::msg::Float64>                   ph_thrust_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::ControlError>              ph_control_error_;
  mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>     ph_disturbances_markers_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::DynamicsConstraints>       ph_current_constraints_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>            ph_heading_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>            ph_speed_;

  // | --------------------- service servers -------------------- |

  mrs_lib::ServiceServerHandler<mrs_msgs::srv::String>  ss_switch_tracker_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::String>  ss_switch_controller_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> ss_reset_tracker_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> ss_hover_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> ss_eland_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> ss_ehover_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> ss_failsafe_;

  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>                ss_failsafe_escalating_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>                ss_toggle_output_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>                ss_arm_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>                ss_enable_callbacks_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::DynamicsConstraintsSrv> ss_set_constraints_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>                ss_use_joystick_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>                ss_use_safety_area_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv>    ss_emergency_reference_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>                ss_pirouette_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>                ss_parachute_;

  // human callbable services for references
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::Vec4> ss_goto_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::Vec4> ss_goto_fcu_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::Vec4> ss_goto_relative_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::Vec1> ss_goto_altitude_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::Vec1> ss_goto_heading_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::Vec1> ss_set_heading_relative_;

  // the reference service and subscriber
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv> ss_reference_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ReferenceStamped>       sh_reference_;

  // the velocity reference service and subscriber
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::VelocityReferenceStampedSrv> ss_velocity_reference_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::VelocityReferenceStamped>       sh_velocity_reference_;

  // trajectory tracking
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::TrajectoryReferenceSrv> ss_trajectory_reference_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::TrajectoryReference>       sh_trajectory_reference_;

  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> ss_start_trajectory_tracking_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> ss_stop_trajectory_tracking_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> ss_resume_trajectory_tracking_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> ss_goto_trajectory_start_;

  // transform service servers
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::TransformReferenceSrv>      ss_transform_reference_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::TransformReferenceArraySrv> ss_transform_reference_array_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::TransformPoseSrv>           ss_transform_pose_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::TransformVector3Srv>        ss_transform_vector3_;

  // safety area services
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::ValidateReference>      ss_validate_reference_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::ValidateReference>      ss_validate_reference_2d_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::ValidateReferenceArray> ss_validate_reference_array_;

  // bumper service servers
  mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool> ss_bumper_enabler_;

  // service clients
  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>                sch_arming_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                sch_eland_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                sch_shutdown_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>                sch_set_odometry_callbacks_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                sch_parachute_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>    sch_point_in_safety_area_2d_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>    sch_point_in_safety_area_3d_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ValidatePathToPointSrv> sch_path_to_point_in_safety_area_2d_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ValidatePathToPointSrv> sch_path_to_point_in_safety_area_3d_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::GetReferenceStampedSrv> sch_get_max_z_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::GetReferenceStampedSrv> sch_get_min_z_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::GetBoolSrv>             sch_is_safety_area_enabled_;

  // safety area min z servers
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::GetFloat64> ss_get_min_z_;

  // | --------- trackers' and controllers' last results -------- |

  // the last result of an active tracker
  std::optional<mrs_msgs::msg::TrackerCommand> last_tracker_cmd_;
  std::mutex                                   mutex_last_tracker_cmd_;

  // the last result of an active controller
  Controller::ControlOutput last_control_output_;
  std::mutex                mutex_last_control_output_;

  // | -------------- HW API diagnostics subscriber ------------- |

  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus> sh_hw_api_status_;

  std::atomic<bool> offboard_mode_          = false;
  std::atomic<bool> offboard_mode_was_true_ = false; // if it was ever true
  std::atomic<bool> armed_                  = false;

  // | -------------------- throttle and mass ------------------- |

  // throttle mass estimation during eland
  double       throttle_mass_estimate_   = 0;
  bool         throttle_under_threshold_ = false;
  rclcpp::Time throttle_mass_estimate_first_time_;

  // | ---------------------- safety params --------------------- |

  // failsafe when tilt error is too large
  bool   _tilt_error_disarm_enabled_;
  double _tilt_error_disarm_timeout_;
  double _tilt_error_disarm_threshold_;

  rclcpp::Time tilt_error_disarm_time_;
  bool         tilt_error_disarm_over_thr_ = false;

  // elanding when tilt error is too large
  bool   _tilt_limit_eland_enabled_;
  double _tilt_limit_eland_ = 0; // [rad]

  // disarming when tilt error is too large
  bool   _tilt_limit_disarm_enabled_;
  double _tilt_limit_disarm_ = 0; // [rad]

  // elanding when yaw error is too large
  bool   _yaw_error_eland_enabled_;
  double _yaw_error_eland_ = 0; // [rad]

  // keeping track of control errors
  std::optional<double> tilt_error_ = 0;
  std::optional<double> yaw_error_  = 0;
  std::mutex            mutex_attitude_error_;

  std::optional<Eigen::Vector3d> position_error_;
  std::mutex                     mutex_position_error_;

  // control error for triggering failsafe, eland, etc.
  // this filled with the current controllers failsafe threshold
  double _failsafe_threshold_                = 0; // control error for triggering failsafe
  double _eland_threshold_                   = 0; // control error for triggering eland
  bool   _odometry_innovation_check_enabled_ = false;
  double _odometry_innovation_threshold_     = 0; // innovation size for triggering eland

  std::atomic<bool> callbacks_enabled_ = true;

  // | ------------------------ parachute ----------------------- |

  bool _parachute_enabled_ = false;

  std::tuple<bool, std::string> deployParachute(void);
  bool                          parachuteSrv(void);

  // safety area routines
  mrs_lib::SubscriberHandler<mrs_msgs::msg::SafetyAreaManagerDiagnostics> sh_safety_area_diag_;

  // those are passed to trackers using the common_handlers object
  bool   isPointInSafetyArea2d(const mrs_msgs::msg::ReferenceStamped &point);
  bool   isPointInSafetyArea3d(const mrs_msgs::msg::ReferenceStamped &point);
  bool   isPathToPointInSafetyArea2d(const mrs_msgs::msg::ReferenceStamped &from, const mrs_msgs::msg::ReferenceStamped &to);
  bool   isPathToPointInSafetyArea3d(const mrs_msgs::msg::ReferenceStamped &from, const mrs_msgs::msg::ReferenceStamped &to);
  double getMinZ(const std::string &frame_id);
  double getMaxZ(const std::string &frame_id);

  // | ------------------------ callbacks ----------------------- |

  // topic callbacks
  void callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void callbackUavState(const mrs_msgs::msg::UavState::ConstSharedPtr msg);
  void callbackHwApiStatus(const mrs_msgs::msg::HwApiStatus::ConstSharedPtr msg);
  void callbackGNSS(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void callbackRC(const mrs_msgs::msg::HwApiRcChannels::ConstSharedPtr msg);

  // topic timeouts
  void timeoutUavState(const double &missing_for);

  // switching controller and tracker services
  bool callbackSwitchTracker(const std::shared_ptr<mrs_msgs::srv::String::Request> request, const std::shared_ptr<mrs_msgs::srv::String::Response> response);
  bool callbackSwitchController(const std::shared_ptr<mrs_msgs::srv::String::Request> request, const std::shared_ptr<mrs_msgs::srv::String::Response> response);
  bool callbackTrackerResetStatic(const std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
                                  const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // reference callbacks
  void callbackReferenceTopic(const mrs_msgs::msg::ReferenceStamped::ConstSharedPtr msg);
  void callbackVelocityReferenceTopic(const mrs_msgs::msg::VelocityReferenceStamped::ConstSharedPtr msg);
  void callbackTrajectoryReferenceTopic(const mrs_msgs::msg::TrajectoryReference::ConstSharedPtr msg);
  bool callbackGoto(const std::shared_ptr<mrs_msgs::srv::Vec4::Request> request, const std::shared_ptr<mrs_msgs::srv::Vec4::Response> response);
  bool callbackGotoFcu(const std::shared_ptr<mrs_msgs::srv::Vec4::Request> request, const std::shared_ptr<mrs_msgs::srv::Vec4::Response> response);
  bool callbackGotoRelative(const std::shared_ptr<mrs_msgs::srv::Vec4::Request> request, const std::shared_ptr<mrs_msgs::srv::Vec4::Response> response);
  bool callbackGotoAltitude(const std::shared_ptr<mrs_msgs::srv::Vec1::Request> request, const std::shared_ptr<mrs_msgs::srv::Vec1::Response> response);
  bool callbackSetHeading(const std::shared_ptr<mrs_msgs::srv::Vec1::Request> request, const std::shared_ptr<mrs_msgs::srv::Vec1::Response> response);
  bool callbackSetHeadingRelative(const std::shared_ptr<mrs_msgs::srv::Vec1::Request> request, const std::shared_ptr<mrs_msgs::srv::Vec1::Response> response);
  bool callbackReferenceService(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request>  request,
                                const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response);
  bool callbackVelocityReferenceService(const std::shared_ptr<mrs_msgs::srv::VelocityReferenceStampedSrv::Request>  request,
                                        const std::shared_ptr<mrs_msgs::srv::VelocityReferenceStampedSrv::Response> response);
  bool callbackTrajectoryReferenceService(const std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Request>  request,
                                          const std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Response> response);
  bool callbackEmergencyReference(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request>  request,
                                  const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response);

  // safety callbacks
  bool callbackHover(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool callbackStartTrajectoryTracking(const std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
                                       const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool callbackStopTrajectoryTracking(const std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
                                      const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool callbackResumeTrajectoryTracking(const std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
                                        const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool callbackGotoTrajectoryStart(const std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
                                   const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool callbackEHover(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool callbackFailsafe(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool callbackFailsafeEscalating(const std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
                                  const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool callbackEland(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool callbackParachute(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool callbackToggleOutput(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  bool callbackArm(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  bool callbackEnableCallbacks(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                               const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  bool callbackEnableBumper(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  bool callbackGetMinZ(const std::shared_ptr<mrs_msgs::srv::GetFloat64::Request> request, const std::shared_ptr<mrs_msgs::srv::GetFloat64::Response> response);

  bool callbackValidateReference(const std::shared_ptr<mrs_msgs::srv::ValidateReference::Request>  request,
                                 const std::shared_ptr<mrs_msgs::srv::ValidateReference::Response> response);
  bool callbackValidateReference2d(const std::shared_ptr<mrs_msgs::srv::ValidateReference::Request>  request,
                                   const std::shared_ptr<mrs_msgs::srv::ValidateReference::Response> response);
  bool callbackValidateReferenceArray(const std::shared_ptr<mrs_msgs::srv::ValidateReferenceArray::Request>  request,
                                      const std::shared_ptr<mrs_msgs::srv::ValidateReferenceArray::Response> response);

  // transformation callbacks
  bool callbackTransformReference(const std::shared_ptr<mrs_msgs::srv::TransformReferenceSrv::Request>  request,
                                  const std::shared_ptr<mrs_msgs::srv::TransformReferenceSrv::Response> response);
  bool callbackTransformReferenceArray(const std::shared_ptr<mrs_msgs::srv::TransformReferenceArraySrv::Request>  request,
                                       const std::shared_ptr<mrs_msgs::srv::TransformReferenceArraySrv::Response> response);
  bool callbackTransformPose(const std::shared_ptr<mrs_msgs::srv::TransformPoseSrv::Request>  request,
                             const std::shared_ptr<mrs_msgs::srv::TransformPoseSrv::Response> response);
  bool callbackTransformVector3(const std::shared_ptr<mrs_msgs::srv::TransformVector3Srv::Request>  request,
                                const std::shared_ptr<mrs_msgs::srv::TransformVector3Srv::Response> response);

  // | ----------------------- constraints ---------------------- |

  // sets constraints to all trackers
  bool callbackSetConstraints(const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request>  request,
                              const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response> response);

  // constraints management
  bool              got_constraints_ = false;
  std::mutex        mutex_constraints_;
  void              setConstraints(const mrs_msgs::srv::DynamicsConstraintsSrv::Request &constraints);
  void              setConstraintsToTrackers(const mrs_msgs::srv::DynamicsConstraintsSrv::Request &constraints);
  void              setConstraintsToControllers(const mrs_msgs::srv::DynamicsConstraintsSrv::Request &constraints);
  std::atomic<bool> constraints_being_enforced_ = false;

  std::optional<mrs_msgs::srv::DynamicsConstraintsSrv::Request>
  enforceControllersConstraints(const mrs_msgs::srv::DynamicsConstraintsSrv::Request &constraints);

  mrs_msgs::srv::DynamicsConstraintsSrv::Request current_constraints_;
  mrs_msgs::srv::DynamicsConstraintsSrv::Request sanitized_constraints_;

  // | ------------------ emergency triggered? ------------------ |

  std::atomic<bool> failsafe_triggered_ = false;
  std::atomic<bool> eland_triggered_    = false;

  // | ------------------------- timers ------------------------- |

  // timer for regular status publishing
  std::shared_ptr<TimerType> timer_status_;
  void                       timerStatus();

  // timer for issuing the failsafe landing
  std::shared_ptr<TimerType> timer_failsafe_;
  void                       timerFailsafe();

  // oneshot timer for running controllers and trackers
  void              asyncControl(void);
  std::atomic<bool> running_async_control_ = false;
  std::future<void> async_control_result_;

  // timer for issuing emergancy landing
  std::shared_ptr<TimerType> timer_eland_;
  void                       timerEland();

  // timer for regular checking of controller errors
  std::shared_ptr<TimerType> timer_safety_;
  void                       timerSafety();
  std::atomic<bool>          running_safety_timer_        = false;
  std::atomic<bool>          odometry_switch_in_progress_ = false;

  // timer for issuing the pirouette
  std::shared_ptr<TimerType> timer_pirouette_;
  void                       timerPirouette();

  // | --------------------- obstacle bumper -------------------- |

  // bumper timer
  std::shared_ptr<TimerType> timer_bumper_;
  void                       timerBumper();

  // bumper subscriber
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ObstacleSectors> sh_bumper_;

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

  int  bumperGetSectorId(const double &x, const double &y, const double &z);
  void bumperPushFromObstacle(void);

  // | --------------- safety checks and failsafes -------------- |

  // escalating failsafe (eland -> failsafe -> disarm)
  bool                       _service_escalating_failsafe_enabled_ = false;
  bool                       _rc_escalating_failsafe_enabled_      = false;
  double                     _escalating_failsafe_timeout_         = 0;
  rclcpp::Time               escalating_failsafe_time_;
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
  void publishDiagnostics(void);

  void                                                  ungripSrv(void);
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sch_ungrip_;

  bool isFlyingNormally(void);

  // | ------------------------ pirouette ----------------------- |

  bool       _pirouette_enabled_ = false;
  double     _pirouette_speed_;
  double     _pirouette_timer_rate_;
  std::mutex mutex_pirouette_;
  double     pirouette_initial_heading_;
  double     pirouette_iterator_;
  bool callbackPirouette(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // | -------------------- joystick control -------------------- |

  mrs_lib::SubscriberHandler<sensor_msgs::msg::Joy> sh_joystick_;

  void callbackJoystick(const sensor_msgs::msg::Joy::ConstSharedPtr msg);
  bool callbackUseJoystick(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // joystick buttons mappings
  int _channel_A_, _channel_B_, _channel_X_, _channel_Y_, _channel_start_, _channel_back_, _channel_LT_, _channel_RT_, _channel_L_joy_, _channel_R_joy_;

  // channel numbers and channel multipliers
  int    _channel_pitch_, _channel_roll_, _channel_heading_, _channel_throttle_;
  double _channel_mult_pitch_, _channel_mult_roll_, _channel_mult_heading_, _channel_mult_throttle_;

  std::shared_ptr<TimerType> timer_joystick_;
  void                       timerJoystick();
  double                     _joystick_timer_rate_ = 0;

  double _joystick_carrot_distance_ = 0;

  rclcpp::Time joystick_start_press_time_;
  bool         joystick_start_pressed_ = false;

  rclcpp::Time joystick_back_press_time_;
  bool         joystick_back_pressed_ = false;
  bool         joystick_goto_enabled_ = false;

  bool         joystick_failsafe_pressed_ = false;
  rclcpp::Time joystick_failsafe_press_time_;

  bool         joystick_eland_pressed_ = false;
  rclcpp::Time joystick_eland_press_time_;

  // | ------------------- RC joystick control ------------------ |

  // listening to the RC channels as told by pixhawk
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiRcChannels> sh_hw_api_rc_;

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

  mrs_lib::PublisherHandler<geometry_msgs::msg::PoseArray>        pub_debug_original_trajectory_poses_;
  mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray> pub_debug_original_trajectory_markers_;

  // | --------------------- other routines --------------------- |

  // this is called to update the trackers and to receive position control command from the active one
  void updateTrackers(void);

  // this is called to update the controllers and to receive attitude control command from the active one
  void updateControllers(const mrs_msgs::msg::UavState &uav_state);

  // sets the reference to the active tracker
  std::tuple<bool, std::string> setReference(const mrs_msgs::msg::ReferenceStamped reference_in);

  // sets the velocity reference to the active tracker
  std::tuple<bool, std::string> setVelocityReference(const mrs_msgs::msg::VelocityReferenceStamped &reference_in);

  // sets the reference trajectory to the active tracker
  std::tuple<bool, std::string, bool, std::vector<std::string>, std::vector<bool>, std::vector<std::string>>
  setTrajectoryReference(const mrs_msgs::msg::TrajectoryReference trajectory_in);

  // publishes
  void publish(void);

  double getMass(void);

  // publishes rviz-visualizable control reference
  void publishControlReferenceOdom(const std::optional<mrs_msgs::msg::TrackerCommand> &tracker_command, const Controller::ControlOutput &control_output);

  void initializeControlOutput(void);

  // tell the mrs_odometry to disable its callbacks
  void odometryCallbacksSrv(const bool input);

  mrs_msgs::msg::ReferenceStamped velocityReferenceToReference(const mrs_msgs::msg::VelocityReferenceStamped &vel_reference);

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

/* ControlManager() //{ */

ControlManager::ControlManager(rclcpp::NodeOptions options) : mrs_lib::Node("control_manager", options) {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_hw_api_capabilities_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiCapabilities>(shopts, "~/hw_api_capabilities_in");

  timer_hw_api_capabilities_ = node_->create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&ControlManager::timerHwApiCapabilities, this));
}

//}

/* initialize() //{ */

void ControlManager::initialize(void) {

  rclcpp::on_shutdown([this]() { this->shutdown(); });

  joystick_start_press_time_      = rclcpp::Time(0, 0, clock_->get_clock_type());
  joystick_failsafe_press_time_   = rclcpp::Time(0, 0, clock_->get_clock_type());
  joystick_eland_press_time_      = rclcpp::Time(0, 0, clock_->get_clock_type());
  escalating_failsafe_time_       = rclcpp::Time(0, 0, clock_->get_clock_type());
  controller_tracker_switch_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  RCLCPP_INFO(node_->get_logger(), "initializing");

  // --------------------------------------------------------------
  // |         common handler for trackers and controllers        |
  // --------------------------------------------------------------

  common_handlers_ = std::make_shared<mrs_uav_managers::control_manager::CommonHandlers_t>();

  // --------------------------------------------------------------
  // |                           params                           |
  // --------------------------------------------------------------

  param_loader_ = std::make_shared<mrs_lib::ParamLoader>(node_);

  param_loader_->loadParam("custom_config", _custom_config_);
  param_loader_->loadParam("platform_config", _platform_config_);
  param_loader_->loadParam("world_config", _world_config_);
  param_loader_->loadParam("network_config", _network_config_);

  if (_custom_config_ != "") {
    param_loader_->addYamlFile(_custom_config_);
  }

  if (_platform_config_ != "") {
    param_loader_->addYamlFile(_platform_config_);
  }

  if (_world_config_ != "") {
    param_loader_->addYamlFile(_world_config_);
  }

  if (_network_config_ != "") {
    param_loader_->addYamlFile(_network_config_);
  }

  param_loader_->addYamlFileFromParam("private_config");
  param_loader_->addYamlFileFromParam("public_config");
  param_loader_->addYamlFileFromParam("private_trackers");
  param_loader_->addYamlFileFromParam("private_controllers");
  param_loader_->addYamlFileFromParam("public_controllers");

  // params passed from the launch file are not prefixed
  param_loader_->loadParam("uav_name", _uav_name_);
  param_loader_->loadParam("body_frame", _body_frame_);
  param_loader_->loadParam("enable_profiler", _profiler_enabled_);
  param_loader_->loadParam("uav_mass", _uav_mass_);

  if (_uav_mass_ <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "nominal uav mass should be > 0.0.");
    rclcpp::shutdown();
    exit(1);
  }

  param_loader_->loadParam("body_disturbance_x", _initial_body_disturbance_x_);
  param_loader_->loadParam("body_disturbance_y", _initial_body_disturbance_y_);
  param_loader_->loadParam("g", common_handlers_->g);

  // motor params are also not prefixed, since they are common to more nodes
  param_loader_->loadParam("motor_params/a", common_handlers_->throttle_model.A);
  param_loader_->loadParam("motor_params/b", common_handlers_->throttle_model.B);
  param_loader_->loadParam("motor_params/n_motors", common_handlers_->throttle_model.n_motors);

  param_loader_->setPrefix("mrs_uav_managers/control_manager/");

  param_loader_->loadParam("state_input", _state_input_);

  if (!(_state_input_ == INPUT_UAV_STATE || _state_input_ == INPUT_ODOMETRY)) {
    RCLCPP_ERROR(node_->get_logger(), "the state_input parameter has to be in {0, 1}");
    rclcpp::shutdown();
    exit(1);
  }

  param_loader_->loadParam("safety/min_throttle_null_tracker", _min_throttle_null_tracker_);
  param_loader_->loadParam("safety/ehover_tracker", _ehover_tracker_name_);
  param_loader_->loadParam("safety/failsafe_controller", _failsafe_controller_name_);

  param_loader_->loadParam("safety/eland/controller", _eland_controller_name_);
  param_loader_->loadParam("safety/eland/cutoff_mass_factor", _elanding_cutoff_mass_factor_);
  param_loader_->loadParam("safety/eland/cutoff_timeout", _elanding_cutoff_timeout_);
  param_loader_->loadParam("safety/eland/timer_rate", _elanding_timer_rate_);
  param_loader_->loadParam("safety/eland/disarm", _eland_disarm_enabled_);

  param_loader_->loadParam("safety/escalating_failsafe/service/enabled", _service_escalating_failsafe_enabled_);
  param_loader_->loadParam("safety/escalating_failsafe/rc/enabled", _rc_escalating_failsafe_enabled_);
  param_loader_->loadParam("safety/escalating_failsafe/rc/channel_number", _rc_escalating_failsafe_channel_);
  param_loader_->loadParam("safety/escalating_failsafe/rc/threshold", _rc_escalating_failsafe_threshold_);
  param_loader_->loadParam("safety/escalating_failsafe/timeout", _escalating_failsafe_timeout_);
  param_loader_->loadParam("safety/escalating_failsafe/ehover", _escalating_failsafe_ehover_);
  param_loader_->loadParam("safety/escalating_failsafe/eland", _escalating_failsafe_eland_);
  param_loader_->loadParam("safety/escalating_failsafe/failsafe", _escalating_failsafe_failsafe_);

  param_loader_->loadParam("safety/tilt_limit/eland/enabled", _tilt_limit_eland_enabled_);
  param_loader_->loadParam("safety/tilt_limit/eland/limit", _tilt_limit_eland_);

  _tilt_limit_eland_ = M_PI * (_tilt_limit_eland_ / 180.0);

  if (_tilt_limit_eland_enabled_ && fabs(_tilt_limit_eland_) < 1e-3) {
    RCLCPP_ERROR(node_->get_logger(), "safety/tilt_limit/eland/enabled = 'TRUE' but the limit is too low");
    rclcpp::shutdown();
    exit(1);
  }

  param_loader_->loadParam("safety/tilt_limit/disarm/enabled", _tilt_limit_disarm_enabled_);
  param_loader_->loadParam("safety/tilt_limit/disarm/limit", _tilt_limit_disarm_);

  _tilt_limit_disarm_ = M_PI * (_tilt_limit_disarm_ / 180.0);

  if (_tilt_limit_disarm_enabled_ && fabs(_tilt_limit_disarm_) < 1e-3) {
    RCLCPP_ERROR(node_->get_logger(), "safety/tilt_limit/disarm/enabled = 'TRUE' but the limit is too low");
    rclcpp::shutdown();
    exit(1);
  }

  param_loader_->loadParam("safety/yaw_error_eland/enabled", _yaw_error_eland_enabled_);
  param_loader_->loadParam("safety/yaw_error_eland/limit", _yaw_error_eland_);

  _yaw_error_eland_ = M_PI * (_yaw_error_eland_ / 180.0);

  if (_yaw_error_eland_enabled_ && fabs(_yaw_error_eland_) < 1e-3) {
    RCLCPP_ERROR(node_->get_logger(), "safety/yaw_error_eland/enabled = 'TRUE' but the limit is too low");
    rclcpp::shutdown();
    exit(1);
  }

  param_loader_->loadParam("status_timer_rate", _status_timer_rate_);
  param_loader_->loadParam("safety/safety_timer_rate", _safety_timer_rate_);
  param_loader_->loadParam("safety/failsafe_timer_rate", _failsafe_timer_rate_);
  param_loader_->loadParam("safety/rc_emergency_handoff/enabled", _rc_emergency_handoff_);

  param_loader_->loadParam("safety/odometry_max_missing_time", _uav_state_max_missing_time_);
  param_loader_->loadParam("safety/odometry_innovation_eland/enabled", _odometry_innovation_check_enabled_);

  param_loader_->loadParam("safety/tilt_error_disarm/enabled", _tilt_error_disarm_enabled_);
  param_loader_->loadParam("safety/tilt_error_disarm/timeout", _tilt_error_disarm_timeout_);
  param_loader_->loadParam("safety/tilt_error_disarm/error_threshold", _tilt_error_disarm_threshold_);

  _tilt_error_disarm_threshold_ = M_PI * (_tilt_error_disarm_threshold_ / 180.0);

  if (_tilt_error_disarm_enabled_ && fabs(_tilt_error_disarm_threshold_) < 1e-3) {
    RCLCPP_ERROR(node_->get_logger(), "safety/tilt_error_disarm/enabled = 'TRUE' but the limit is too low");
    rclcpp::shutdown();
    exit(1);
  }

  // default constraints

  param_loader_->loadParam("default_constraints/horizontal/speed", current_constraints_.constraints.horizontal_speed);
  param_loader_->loadParam("default_constraints/horizontal/acceleration", current_constraints_.constraints.horizontal_acceleration);
  param_loader_->loadParam("default_constraints/horizontal/jerk", current_constraints_.constraints.horizontal_jerk);
  param_loader_->loadParam("default_constraints/horizontal/snap", current_constraints_.constraints.horizontal_snap);

  param_loader_->loadParam("default_constraints/vertical/ascending/speed", current_constraints_.constraints.vertical_ascending_speed);
  param_loader_->loadParam("default_constraints/vertical/ascending/acceleration", current_constraints_.constraints.vertical_ascending_acceleration);
  param_loader_->loadParam("default_constraints/vertical/ascending/jerk", current_constraints_.constraints.vertical_ascending_jerk);
  param_loader_->loadParam("default_constraints/vertical/ascending/snap", current_constraints_.constraints.vertical_ascending_snap);

  param_loader_->loadParam("default_constraints/vertical/descending/speed", current_constraints_.constraints.vertical_descending_speed);
  param_loader_->loadParam("default_constraints/vertical/descending/acceleration", current_constraints_.constraints.vertical_descending_acceleration);
  param_loader_->loadParam("default_constraints/vertical/descending/jerk", current_constraints_.constraints.vertical_descending_jerk);
  param_loader_->loadParam("default_constraints/vertical/descending/snap", current_constraints_.constraints.vertical_descending_snap);

  param_loader_->loadParam("default_constraints/heading/speed", current_constraints_.constraints.heading_speed);
  param_loader_->loadParam("default_constraints/heading/acceleration", current_constraints_.constraints.heading_acceleration);
  param_loader_->loadParam("default_constraints/heading/jerk", current_constraints_.constraints.heading_jerk);
  param_loader_->loadParam("default_constraints/heading/snap", current_constraints_.constraints.heading_snap);

  param_loader_->loadParam("default_constraints/angular_speed/roll", current_constraints_.constraints.roll_rate);
  param_loader_->loadParam("default_constraints/angular_speed/pitch", current_constraints_.constraints.pitch_rate);
  param_loader_->loadParam("default_constraints/angular_speed/yaw", current_constraints_.constraints.yaw_rate);

  param_loader_->loadParam("default_constraints/tilt", current_constraints_.constraints.tilt);

  current_constraints_.constraints.tilt = M_PI * (current_constraints_.constraints.tilt / 180.0);

  // joystick

  param_loader_->loadParam("joystick/enabled", _joystick_enabled_);
  param_loader_->loadParam("joystick/mode", _joystick_mode_);
  param_loader_->loadParam("joystick/carrot_distance", _joystick_carrot_distance_);
  param_loader_->loadParam("joystick/joystick_timer_rate", _joystick_timer_rate_);
  param_loader_->loadParam("joystick/attitude_control/tracker", _joystick_tracker_name_);
  param_loader_->loadParam("joystick/attitude_control/controller", _joystick_controller_name_);
  param_loader_->loadParam("joystick/attitude_control/fallback/tracker", _joystick_fallback_tracker_name_);
  param_loader_->loadParam("joystick/attitude_control/fallback/controller", _joystick_fallback_controller_name_);

  param_loader_->loadParam("joystick/channels/A", _channel_A_);
  param_loader_->loadParam("joystick/channels/B", _channel_B_);
  param_loader_->loadParam("joystick/channels/X", _channel_X_);
  param_loader_->loadParam("joystick/channels/Y", _channel_Y_);
  param_loader_->loadParam("joystick/channels/start", _channel_start_);
  param_loader_->loadParam("joystick/channels/back", _channel_back_);
  param_loader_->loadParam("joystick/channels/LT", _channel_LT_);
  param_loader_->loadParam("joystick/channels/RT", _channel_RT_);
  param_loader_->loadParam("joystick/channels/L_joy", _channel_L_joy_);
  param_loader_->loadParam("joystick/channels/R_joy", _channel_R_joy_);

  // load channels
  param_loader_->loadParam("joystick/channels/pitch", _channel_pitch_);
  param_loader_->loadParam("joystick/channels/roll", _channel_roll_);
  param_loader_->loadParam("joystick/channels/heading", _channel_heading_);
  param_loader_->loadParam("joystick/channels/throttle", _channel_throttle_);

  // load channel multipliers
  param_loader_->loadParam("joystick/channel_multipliers/pitch", _channel_mult_pitch_);
  param_loader_->loadParam("joystick/channel_multipliers/roll", _channel_mult_roll_);
  param_loader_->loadParam("joystick/channel_multipliers/heading", _channel_mult_heading_);
  param_loader_->loadParam("joystick/channel_multipliers/throttle", _channel_mult_throttle_);

  bool bumper_enabled;
  param_loader_->loadParam("obstacle_bumper/enabled", bumper_enabled);
  bumper_enabled_ = bumper_enabled;

  param_loader_->loadParam("obstacle_bumper/switch_tracker", _bumper_switch_tracker_);
  param_loader_->loadParam("obstacle_bumper/switch_controller", _bumper_switch_controller_);
  param_loader_->loadParam("obstacle_bumper/tracker", _bumper_tracker_name_);
  param_loader_->loadParam("obstacle_bumper/controller", _bumper_controller_name_);
  param_loader_->loadParam("obstacle_bumper/timer_rate", _bumper_timer_rate_);

  param_loader_->loadParam("obstacle_bumper/horizontal/min_distance_to_obstacle", _bumper_horizontal_distance_);
  param_loader_->loadParam("obstacle_bumper/horizontal/derived_from_dynamics", _bumper_horizontal_derive_from_dynamics_);

  param_loader_->loadParam("obstacle_bumper/vertical/min_distance_to_obstacle", _bumper_vertical_distance_);
  param_loader_->loadParam("obstacle_bumper/vertical/derived_from_dynamics", _bumper_vertical_derive_from_dynamics_);

  param_loader_->loadParam("obstacle_bumper/horizontal/overshoot", _bumper_horizontal_overshoot_);
  param_loader_->loadParam("obstacle_bumper/vertical/overshoot", _bumper_vertical_overshoot_);

  param_loader_->loadParam("safety/tracker_error_action", _tracker_error_action_);

  param_loader_->loadParam("trajectory_tracking/snap_to_safety_area", _snap_trajectory_to_safety_area_);

  // check the values of tracker error action
  if (_tracker_error_action_ != ELAND_STR && _tracker_error_action_ != EHOVER_STR) {
    RCLCPP_ERROR(node_->get_logger(), "the tracker_error_action parameter (%s) is not correct, requires {%s, %s}", _tracker_error_action_.c_str(), ELAND_STR,
                 EHOVER_STR);
    rclcpp::shutdown();
    exit(1);
  }

  param_loader_->loadParam("rc_joystick/enabled", _rc_goto_enabled_);
  param_loader_->loadParam("rc_joystick/channel_number", _rc_joystick_channel_);
  param_loader_->loadParam("rc_joystick/horizontal_speed", _rc_horizontal_speed_);
  param_loader_->loadParam("rc_joystick/vertical_speed", _rc_vertical_speed_);
  param_loader_->loadParam("rc_joystick/heading_rate", _rc_heading_rate_);

  param_loader_->loadParam("rc_joystick/channels/pitch", _rc_channel_pitch_);
  param_loader_->loadParam("rc_joystick/channels/roll", _rc_channel_roll_);
  param_loader_->loadParam("rc_joystick/channels/heading", _rc_channel_heading_);
  param_loader_->loadParam("rc_joystick/channels/throttle", _rc_channel_throttle_);

  param_loader_->loadParam("pirouette/speed", _pirouette_speed_);
  param_loader_->loadParam("pirouette/timer_rate", _pirouette_timer_rate_);

  param_loader_->loadParam("safety/parachute/enabled", _parachute_enabled_);

  // --------------------------------------------------------------
  // |             initialize the last control output             |
  // --------------------------------------------------------------

  initializeControlOutput();

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(node_);
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | ------------------- scope timer logger ------------------- |

  param_loader_->loadParam("scope_timer/enabled", scope_timer_enabled_);
  const std::string scope_timer_log_filename = param_loader_->loadParam2("scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<mrs_lib::ScopeTimerLogger>(node_, scope_timer_log_filename, scope_timer_enabled_);

  // bind transformer to trackers and controllers for use
  common_handlers_->transformer = transformer_;

  // bind scope timer to trackers and controllers for use
  common_handlers_->scope_timer.enabled = scope_timer_enabled_;
  common_handlers_->scope_timer.logger  = scope_timer_logger_;

  common_handlers_->safety_area.isPointInSafetyArea2d = std::bind(&ControlManager::isPointInSafetyArea2d, this, std::placeholders::_1);
  common_handlers_->safety_area.isPointInSafetyArea3d = std::bind(&ControlManager::isPointInSafetyArea3d, this, std::placeholders::_1);
  common_handlers_->safety_area.getMinZ               = std::bind(&ControlManager::getMinZ, this, std::placeholders::_1);
  common_handlers_->safety_area.getMaxZ               = std::bind(&ControlManager::getMaxZ, this, std::placeholders::_1);

  common_handlers_->getMass = std::bind(&ControlManager::getMass, this);

  common_handlers_->detailed_model_params = loadDetailedUavModelParams(node_, _platform_config_, _custom_config_);

  common_handlers_->control_output_modalities = _hw_api_inputs_;

  common_handlers_->uav_name = _uav_name_;

  common_handlers_->parent_node = node_;

  // --------------------------------------------------------------
  // |                        load trackers                       |
  // --------------------------------------------------------------

  std::vector<std::string> custom_trackers;

  param_loader_->loadParam("mrs_trackers", _tracker_names_);
  param_loader_->loadParam("trackers", custom_trackers);

  if (!custom_trackers.empty()) {
    _tracker_names_.insert(_tracker_names_.end(), custom_trackers.begin(), custom_trackers.end());
  }

  param_loader_->loadParam("null_tracker", _null_tracker_name_);
  param_loader_->loadParam("landing_takeoff_tracker", _landoff_tracker_name_);

  tracker_loader_ = std::make_unique<pluginlib::ClassLoader<mrs_uav_managers::Tracker>>("mrs_uav_managers", "mrs_uav_managers::Tracker");

  for (int i = 0; i < int(_tracker_names_.size()); i++) {

    std::string tracker_name = _tracker_names_.at(i);

    // load the controller parameters
    std::string address;
    std::string name_space;
    bool        human_switchable;

    param_loader_->loadParam(tracker_name + "/address", address);
    param_loader_->loadParam(tracker_name + "/namespace", name_space);
    param_loader_->loadParam(tracker_name + "/human_switchable", human_switchable);

    TrackerParams new_tracker(address, name_space, human_switchable);
    trackers_.insert(std::pair<std::string, TrackerParams>(tracker_name, new_tracker));

    try {
      RCLCPP_INFO(node_->get_logger(), "loading the tracker '%s'", new_tracker.address.c_str());
      tracker_list_.push_back(tracker_loader_->createSharedInstance(new_tracker.address.c_str()));
    }
    catch (pluginlib::CreateClassException &ex1) {
      RCLCPP_ERROR(node_->get_logger(), "CreateClassException for the tracker '%s'", new_tracker.address.c_str());
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex1.what());
      rclcpp::shutdown();
      exit(1);
    }
    catch (pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(node_->get_logger(), "PluginlibException for the tracker '%s'", new_tracker.address.c_str());
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex.what());
      rclcpp::shutdown();
      exit(1);
    }
  }

  RCLCPP_INFO(node_->get_logger(), "trackers were loaded");

  trackers_subnode_ = node_->create_sub_node("mrs_uav_trackers");
  param_loader_->setPrefix("");

  for (int i = 0; i < int(tracker_list_.size()); i++) {

    std::map<std::string, TrackerParams>::iterator it;
    it = trackers_.find(_tracker_names_.at(i));

    rclcpp::Node::SharedPtr subnode = trackers_subnode_->create_sub_node(it->second.name_space);

    // create private handlers
    std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers =
        std::make_shared<mrs_uav_managers::control_manager::PrivateHandlers_t>();

    private_handlers->name_space   = it->second.name_space;
    private_handlers->runtime_name = _tracker_names_.at(i);
    private_handlers->param_loader = std::make_unique<mrs_lib::ParamLoader>(subnode, _tracker_names_.at(i));
    private_handlers->param_loader->copyYamls(*param_loader_);
    private_handlers->parent_param_loader = param_loader_;

    bool success = false;

    try {
      RCLCPP_INFO(node_->get_logger(), "initializing the tracker '%s'", it->second.address.c_str());
      success = tracker_list_.at(i)->initialize(subnode, common_handlers_, private_handlers);
    }
    catch (std::runtime_error &ex) {
      RCLCPP_ERROR(node_->get_logger(), "exception caught during tracker initialization: '%s'", ex.what());
    }

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "failed to initialize the tracker '%s'", it->second.address.c_str());
      rclcpp::shutdown();
      exit(1);
    }
  }

  RCLCPP_INFO(node_->get_logger(), "trackers were initialized");

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
      RCLCPP_ERROR(node_->get_logger(), "the safety/hover_tracker (%s) is not within the loaded trackers", _ehover_tracker_name_.c_str());
      rclcpp::shutdown();
      exit(1);
    }
  }

  // | ----- check for the existence of the landoff tracker ----- |

  {
    auto idx = idxInVector(_landoff_tracker_name_, _tracker_names_);

    if (idx) {
      _landoff_tracker_idx_ = idx.value();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "the landoff tracker (%s) is not within the loaded trackers", _landoff_tracker_name_.c_str());
      rclcpp::shutdown();
      exit(1);
    }
  }

  // | ------- check for the existence of the null tracker ------ |

  {
    auto idx = idxInVector(_null_tracker_name_, _tracker_names_);

    if (idx) {
      _null_tracker_idx_ = idx.value();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "the null tracker (%s) is not within the loaded trackers", _null_tracker_name_.c_str());
      rclcpp::shutdown();
      exit(1);
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
      RCLCPP_ERROR(node_->get_logger(), "the joystick tracker (%s) is not within the loaded trackers", _joystick_tracker_name_.c_str());
      rclcpp::shutdown();
      exit(1);
    }
  }

  if (_bumper_switch_tracker_) {

    auto idx = idxInVector(_bumper_tracker_name_, _tracker_names_);

    if (!idx) {
      RCLCPP_ERROR(node_->get_logger(), "the bumper tracker (%s) is not within the loaded trackers", _bumper_tracker_name_.c_str());
      rclcpp::shutdown();
      exit(1);
    }
  }

  {
    auto idx = idxInVector(_joystick_fallback_tracker_name_, _tracker_names_);

    if (idx) {
      _joystick_fallback_tracker_idx_ = idx.value();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "the joystick fallback tracker (%s) is not within the loaded trackers", _joystick_fallback_tracker_name_.c_str());
      rclcpp::shutdown();
      exit(1);
    }
  }

  // --------------------------------------------------------------
  // |                    load the controllers                    |
  // --------------------------------------------------------------

  std::vector<std::string> custom_controllers;

  param_loader_->setPrefix("mrs_uav_managers/control_manager/");
  param_loader_->loadParam("mrs_controllers", _controller_names_);
  param_loader_->loadParam("controllers", custom_controllers);

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
    param_loader_->loadParam(controller_name + "/address", address);
    param_loader_->loadParam(controller_name + "/namespace", name_space);
    param_loader_->loadParam(controller_name + "/eland_threshold", eland_threshold);
    param_loader_->loadParam(controller_name + "/failsafe_threshold", failsafe_threshold);
    param_loader_->loadParam(controller_name + "/odometry_innovation_threshold", odometry_innovation_threshold);
    param_loader_->loadParam(controller_name + "/human_switchable", human_switchable);

    // check if the controller can output some of the required outputs
    {

      ControlOutputModalities_t outputs;
      param_loader_->loadParam(controller_name + "/outputs/actuators", outputs.actuators, false);
      param_loader_->loadParam(controller_name + "/outputs/control_group", outputs.control_group, false);
      param_loader_->loadParam(controller_name + "/outputs/attitude_rate", outputs.attitude_rate, false);
      param_loader_->loadParam(controller_name + "/outputs/attitude", outputs.attitude, false);
      param_loader_->loadParam(controller_name + "/outputs/acceleration_hdg_rate", outputs.acceleration_hdg_rate, false);
      param_loader_->loadParam(controller_name + "/outputs/acceleration_hdg", outputs.acceleration_hdg, false);
      param_loader_->loadParam(controller_name + "/outputs/velocity_hdg_rate", outputs.velocity_hdg_rate, false);
      param_loader_->loadParam(controller_name + "/outputs/velocity_hdg", outputs.velocity_hdg, false);
      param_loader_->loadParam(controller_name + "/outputs/position", outputs.position, false);

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

        RCLCPP_ERROR(node_->get_logger(), "the controller '%s' does not meet the control output requirements, which are some of the following",
                     controller_name.c_str());

        if (_hw_api_inputs_.actuators) {
          RCLCPP_ERROR(node_->get_logger(), "- actuators");
        }

        if (_hw_api_inputs_.control_group) {
          RCLCPP_ERROR(node_->get_logger(), "- control group");
        }

        if (_hw_api_inputs_.attitude_rate) {
          RCLCPP_ERROR(node_->get_logger(), "- attitude rate");
        }

        if (_hw_api_inputs_.attitude) {
          RCLCPP_ERROR(node_->get_logger(), "- attitude");
        }

        if (_hw_api_inputs_.acceleration_hdg_rate) {
          RCLCPP_ERROR(node_->get_logger(), "- acceleration+hdg rate");
        }

        if (_hw_api_inputs_.acceleration_hdg) {
          RCLCPP_ERROR(node_->get_logger(), "- acceleration+hdg");
        }

        if (_hw_api_inputs_.velocity_hdg_rate) {
          RCLCPP_ERROR(node_->get_logger(), "- velocity+hdg rate");
        }

        if (_hw_api_inputs_.velocity_hdg) {
          RCLCPP_ERROR(node_->get_logger(), "- velocity+hdg");
        }

        if (_hw_api_inputs_.position) {
          RCLCPP_ERROR(node_->get_logger(), "- position");
        }

        rclcpp::shutdown();
        exit(1);
      }

      if ((_hw_api_inputs_.actuators || _hw_api_inputs_.control_group) && !common_handlers_->detailed_model_params) {
        RCLCPP_ERROR(node_->get_logger(),
                     "the HW API supports 'actuators' or 'control_group' input, but the 'detailed uav model params' were not loaded sucessfully");
        rclcpp::shutdown();
        exit(1);
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
      RCLCPP_INFO(node_->get_logger(), "loading the controller '%s'", new_controller.address.c_str());
      controller_list_.push_back(controller_loader_->createSharedInstance(new_controller.address.c_str()));
    }
    catch (pluginlib::CreateClassException &ex1) {
      RCLCPP_ERROR(node_->get_logger(), "CreateClassException for the controller '%s'", new_controller.address.c_str());
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex1.what());
      rclcpp::shutdown();
      exit(1);
    }
    catch (pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(node_->get_logger(), "PluginlibException for the controller '%s'", new_controller.address.c_str());
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex.what());
      rclcpp::shutdown();
      exit(1);
    }
  }

  RCLCPP_INFO(node_->get_logger(), "controllers were loaded");

  controllers_subnode_ = node_->create_sub_node("mrs_uav_controllers");
  param_loader_->setPrefix("");

  for (int i = 0; i < int(controller_list_.size()); i++) {

    std::map<std::string, ControllerParams>::iterator it;
    it = controllers_.find(_controller_names_.at(i));

    rclcpp::Node::SharedPtr subnode = controllers_subnode_->create_sub_node(it->second.name_space);

    // create private handlers
    std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers =
        std::make_shared<mrs_uav_managers::control_manager::PrivateHandlers_t>();

    private_handlers->name_space   = it->second.name_space;
    private_handlers->runtime_name = _controller_names_.at(i);
    private_handlers->param_loader = std::make_unique<mrs_lib::ParamLoader>(subnode, _controller_names_.at(i));
    private_handlers->param_loader->copyYamls(*param_loader_);
    private_handlers->parent_param_loader = param_loader_;

    bool success = false;

    try {
      RCLCPP_INFO(node_->get_logger(), "initializing the controller '%s'", it->second.address.c_str());
      success = controller_list_.at(i)->initialize(subnode, common_handlers_, private_handlers);
    }
    catch (std::runtime_error &ex) {
      RCLCPP_ERROR(node_->get_logger(), "exception caught during controller initialization: '%s'", ex.what());
    }

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "failed to initialize the controller '%s'", it->second.address.c_str());
      rclcpp::shutdown();
      exit(1);
    }
  }

  RCLCPP_INFO(node_->get_logger(), "controllers were initialized");

  {
    auto idx = idxInVector(_failsafe_controller_name_, _controller_names_);

    if (idx) {
      _failsafe_controller_idx_ = idx.value();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "the failsafe controller (%s) is not within the loaded controllers", _failsafe_controller_name_.c_str());
      rclcpp::shutdown();
      exit(1);
    }
  }

  {
    auto idx = idxInVector(_eland_controller_name_, _controller_names_);

    if (idx) {
      _eland_controller_idx_ = idx.value();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "the eland controller (%s) is not within the loaded controllers", _eland_controller_name_.c_str());
      rclcpp::shutdown();
      exit(1);
    }
  }

  {
    auto idx = idxInVector(_joystick_controller_name_, _controller_names_);

    if (idx) {
      _joystick_controller_idx_ = idx.value();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "the joystick controller (%s) is not within the loaded controllers", _joystick_controller_name_.c_str());
      rclcpp::shutdown();
      exit(1);
    }
  }

  if (_bumper_switch_controller_) {

    auto idx = idxInVector(_bumper_controller_name_, _controller_names_);

    if (!idx) {
      RCLCPP_ERROR(node_->get_logger(), "the bumper controller (%s) is not within the loaded controllers", _bumper_controller_name_.c_str());
      rclcpp::shutdown();
      exit(1);
    }
  }

  {
    auto idx = idxInVector(_joystick_fallback_controller_name_, _controller_names_);

    if (idx) {
      _joystick_fallback_controller_idx_ = idx.value();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "the joystick fallback controller (%s) is not within the loaded controllers",
                   _joystick_fallback_controller_name_.c_str());
      rclcpp::shutdown();
      exit(1);
    }
  }

  // --------------------------------------------------------------
  // |                  activate the NullTracker                  |
  // --------------------------------------------------------------

  RCLCPP_INFO(node_->get_logger(), "activating the null tracker");

  tracker_list_.at(_null_tracker_idx_)->activate(last_tracker_cmd_);
  active_tracker_idx_ = _null_tracker_idx_;

  // --------------------------------------------------------------
  // |    activate the eland controller as the first controller   |
  // --------------------------------------------------------------

  RCLCPP_INFO(node_->get_logger(), "activating the the eland controller (%s) as the first controller", _controller_names_.at(_eland_controller_idx_).c_str());

  controller_list_.at(_eland_controller_idx_)->activate(last_control_output_);
  active_controller_idx_ = _eland_controller_idx_;

  // update the time
  {
    std::scoped_lock lock(mutex_controller_tracker_switch_time_);

    controller_tracker_switch_time_ = clock_->now();
  }

  output_enabled_ = false;

  // | --------------- set the default constraints -------------- |

  sanitized_constraints_ = current_constraints_;
  setConstraints(current_constraints_);

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(node_, "ControlManager", _profiler_enabled_);

  // | ----------------------- publishers ----------------------- |

  control_output_publisher_ = OutputPublisher(node_);

  ph_controller_diagnostics_ = mrs_lib::PublisherHandler<mrs_msgs::msg::ControllerDiagnostics>(node_, "~/controller_diagnostics_out");
  ph_tracker_cmd_            = mrs_lib::PublisherHandler<mrs_msgs::msg::TrackerCommand>(node_, "~/tracker_cmd_out");
  ph_mrs_odom_input_         = mrs_lib::PublisherHandler<mrs_msgs::msg::EstimatorInput>(node_, "~/estimator_input_out");
  ph_control_reference_odom_ = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(node_, "~/control_reference_out");
  ph_diagnostics_            = mrs_lib::PublisherHandler<mrs_msgs::msg::ControlManagerDiagnostics>(node_, "~/diagnostics_out");
  ph_offboard_on_            = mrs_lib::PublisherHandler<std_msgs::msg::Empty>(node_, "~/offboard_on_out");
  ph_tilt_error_             = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>(node_, "~/tilt_error_out");
  ph_mass_nominal_           = mrs_lib::PublisherHandler<std_msgs::msg::Float64>(node_, "~/mass_nominal_out"); // TODO latch
  ph_control_error_          = mrs_lib::PublisherHandler<mrs_msgs::msg::ControlError>(node_, "~/control_error_out");

  {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = 10.0;
    // TODO latch

    ph_throttle_ = mrs_lib::PublisherHandler<std_msgs::msg::Float64>(opts, "~/throttle_out");
  }

  {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = 100.0;

    ph_thrust_ = mrs_lib::PublisherHandler<std_msgs::msg::Float64>(opts, "~/thrust_out");
  }

  {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = 10.0;

    ph_mass_estimate_ = mrs_lib::PublisherHandler<std_msgs::msg::Float64>(opts, "~/mass_estimate_out");
  }

  {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = 10.0;

    ph_disturbances_markers_ = mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>(opts, "~/disturbances_markers_out");
  }

  ph_current_constraints_ = mrs_lib::PublisherHandler<mrs_msgs::msg::DynamicsConstraints>(node_, "~/current_constraints_out");
  ph_heading_             = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>(node_, "~/heading_out");
  ph_speed_               = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>(node_, "~/speed_out");

  {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = 1.0;
    // TODO latch

    pub_debug_original_trajectory_poses_   = mrs_lib::PublisherHandler<geometry_msgs::msg::PoseArray>(opts, "~/trajectory_original/poses_out");
    pub_debug_original_trajectory_markers_ = mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>(opts, "~/trajectory_original/markers_out");
  }

  // | ------------------ publish nominal mass ------------------ |

  {
    std_msgs::msg::Float64 nominal_mass;

    nominal_mass.data = _uav_mass_;

    ph_mass_nominal_.publish(nominal_mass);
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  if (_state_input_ == INPUT_UAV_STATE) {
    sh_uav_state_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavState>(shopts, "~/uav_state_in", &ControlManager::callbackUavState, this);
  } else if (_state_input_ == INPUT_ODOMETRY) {
    sh_odometry_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/odometry_in", &ControlManager::callbackOdometry, this);
  }

  if (_odometry_innovation_check_enabled_) {
    sh_odometry_innovation_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/odometry_innovation_in");
  }

  sh_bumper_           = mrs_lib::SubscriberHandler<mrs_msgs::msg::ObstacleSectors>(shopts, "~/bumper_sectors_in");
  sh_max_z_            = mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>(shopts, "~/max_z_in");
  sh_safety_area_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::SafetyAreaManagerDiagnostics>(shopts, "~/safety_area_diag_in");
  sh_joystick_         = mrs_lib::SubscriberHandler<sensor_msgs::msg::Joy>(shopts, "~/joystick_in", &ControlManager::callbackJoystick, this);
  sh_gnss_             = mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix>(shopts, "~/gnss_in", &ControlManager::callbackGNSS, this);
  sh_hw_api_rc_        = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiRcChannels>(shopts, "~/hw_api_rc_in", &ControlManager::callbackRC, this);

  sh_hw_api_status_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus>(shopts, "~/hw_api_status_in", &ControlManager::callbackHwApiStatus, this);

  // | -------------------- general services -------------------- |

  ss_switch_tracker_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::String>(
      node_, "~/switch_tracker_in", std::bind(&ControlManager::callbackSwitchTracker, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_switch_controller_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::String>(
      node_, "~/switch_controller_in", std::bind(&ControlManager::callbackSwitchController, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_reset_tracker_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/tracker_reset_static_in", std::bind(&ControlManager::callbackTrackerResetStatic, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_hover_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/hover_in", std::bind(&ControlManager::callbackHover, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS(),
      cbkgrp_ss_);
  ss_ehover_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/ehover_in", std::bind(&ControlManager::callbackEHover, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS(),
      cbkgrp_ss_);
  ss_failsafe_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/failsafe_in", std::bind(&ControlManager::callbackFailsafe, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS(),
      cbkgrp_ss_);
  ss_failsafe_escalating_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/failsafe_escalating_in", std::bind(&ControlManager::callbackFailsafeEscalating, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_toggle_output_ = mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>(
      node_, "~/toggle_output_in", std::bind(&ControlManager::callbackToggleOutput, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_arm_ = mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>(
      node_, "~/arm_in", std::bind(&ControlManager::callbackArm, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_enable_callbacks_ = mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>(
      node_, "~/enable_callbacks_in", std::bind(&ControlManager::callbackEnableCallbacks, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_set_constraints_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::DynamicsConstraintsSrv>(
      node_, "~/set_constraints_in", std::bind(&ControlManager::callbackSetConstraints, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_use_joystick_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/use_joystick_in", std::bind(&ControlManager::callbackUseJoystick, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_eland_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/eland_in", std::bind(&ControlManager::callbackEland, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS(),
      cbkgrp_ss_);
  ss_parachute_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/parachute_in", std::bind(&ControlManager::callbackParachute, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS(),
      cbkgrp_ss_);
  ss_transform_reference_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::TransformReferenceSrv>(
      node_, "~/transform_reference_in", std::bind(&ControlManager::callbackTransformReference, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_transform_reference_array_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::TransformReferenceArraySrv>(
      node_, "~/transform_reference_array_in", std::bind(&ControlManager::callbackTransformReferenceArray, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_transform_pose_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::TransformPoseSrv>(
      node_, "~/transform_pose_in", std::bind(&ControlManager::callbackTransformPose, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_transform_vector3_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::TransformVector3Srv>(
      node_, "~/transform_vector3_in", std::bind(&ControlManager::callbackTransformVector3, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_bumper_enabler_ = mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>(
      node_, "~/bumper_in", std::bind(&ControlManager::callbackEnableBumper, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS(),
      cbkgrp_ss_);
  ss_get_min_z_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::GetFloat64>(
      node_, "~/get_min_z_in", std::bind(&ControlManager::callbackGetMinZ, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS(),
      cbkgrp_ss_);
  ss_validate_reference_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::ValidateReference>(
      node_, "~/validate_reference_in", std::bind(&ControlManager::callbackValidateReference, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_validate_reference_2d_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::ValidateReference>(
      node_, "~/validate_reference_2d_in", std::bind(&ControlManager::callbackValidateReference2d, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_validate_reference_array_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::ValidateReferenceArray>(
      node_, "~/validate_reference_array_in", std::bind(&ControlManager::callbackValidateReferenceArray, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_start_trajectory_tracking_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/start_trajectory_tracking_in", std::bind(&ControlManager::callbackStartTrajectoryTracking, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_stop_trajectory_tracking_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/stop_trajectory_tracking_in", std::bind(&ControlManager::callbackStopTrajectoryTracking, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_resume_trajectory_tracking_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/resume_trajectory_tracking_in",
      std::bind(&ControlManager::callbackResumeTrajectoryTracking, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS(),
      cbkgrp_ss_);
  ss_goto_trajectory_start_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/goto_trajectory_start_in", std::bind(&ControlManager::callbackGotoTrajectoryStart, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  sch_arming_                  = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "~/hw_api_arming_out", cbkgrp_sc_);
  sch_eland_                   = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/eland_out", cbkgrp_sc_);
  sch_shutdown_                = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/shutdown_out", cbkgrp_sc_);
  sch_set_odometry_callbacks_  = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "~/set_odometry_callbacks_out", cbkgrp_sc_);
  sch_ungrip_                  = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/ungrip_out", cbkgrp_sc_);
  sch_parachute_               = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/parachute_out", cbkgrp_sc_);
  sch_point_in_safety_area_2d_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "~/point_in_safety_area_2d_out", cbkgrp_sc_);
  sch_point_in_safety_area_3d_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "~/point_in_safety_area_3d_out", cbkgrp_sc_);
  sch_path_to_point_in_safety_area_2d_ =
      mrs_lib::ServiceClientHandler<mrs_msgs::srv::ValidatePathToPointSrv>(node_, "~/path_in_safety_area_2d_out", cbkgrp_sc_);
  sch_path_to_point_in_safety_area_3d_ =
      mrs_lib::ServiceClientHandler<mrs_msgs::srv::ValidatePathToPointSrv>(node_, "~/path_in_safety_area_3d_out", cbkgrp_sc_);
  sch_get_min_z_              = mrs_lib::ServiceClientHandler<mrs_msgs::srv::GetReferenceStampedSrv>(node_, "~/get_min_z_out", cbkgrp_sc_);
  sch_get_max_z_              = mrs_lib::ServiceClientHandler<mrs_msgs::srv::GetReferenceStampedSrv>(node_, "~/get_max_z_out", cbkgrp_sc_);
  sch_is_safety_area_enabled_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::GetBoolSrv>(node_, "~/is_safety_area_enabled_out", cbkgrp_sc_);

  // | ---------------- setpoint command services --------------- |

  // human callable
  ss_goto_     = mrs_lib::ServiceServerHandler<mrs_msgs::srv::Vec4>(node_, "~/goto_in",
                                                                    std::bind(&ControlManager::callbackGoto, this, std::placeholders::_1, std::placeholders::_2),
                                                                    rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_goto_fcu_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::Vec4>(
      node_, "~/goto_fcu_in", std::bind(&ControlManager::callbackGotoFcu, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS(),
      cbkgrp_ss_);
  ss_goto_relative_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::Vec4>(
      node_, "~/goto_relative_in", std::bind(&ControlManager::callbackGotoRelative, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_goto_altitude_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::Vec1>(
      node_, "~/goto_altitude_in", std::bind(&ControlManager::callbackGotoAltitude, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_goto_heading_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::Vec1>(
      node_, "~/set_heading_in", std::bind(&ControlManager::callbackSetHeading, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_set_heading_relative_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::Vec1>(
      node_, "~/set_heading_relative_in", std::bind(&ControlManager::callbackSetHeadingRelative, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_reference_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv>(
      node_, "~/reference_in", std::bind(&ControlManager::callbackReferenceService, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  sh_reference_          = mrs_lib::SubscriberHandler<mrs_msgs::msg::ReferenceStamped>(shopts, "~/reference_in", &ControlManager::callbackReferenceTopic, this);
  ss_velocity_reference_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::VelocityReferenceStampedSrv>(
      node_, "~/velocity_reference_in", std::bind(&ControlManager::callbackVelocityReferenceService, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  sh_velocity_reference_   = mrs_lib::SubscriberHandler<mrs_msgs::msg::VelocityReferenceStamped>(shopts, "~/velocity_reference_in",
                                                                                                 &ControlManager::callbackVelocityReferenceTopic, this);
  ss_trajectory_reference_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::TrajectoryReferenceSrv>(
      node_, "~/trajectory_reference_in", std::bind(&ControlManager::callbackTrajectoryReferenceService, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  sh_trajectory_reference_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::TrajectoryReference>(shopts, "~/trajectory_reference_in",
                                                                                            &ControlManager::callbackTrajectoryReferenceTopic, this);

  // | --------------------- other services --------------------- |

  ss_emergency_reference_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv>(
      node_, "~/emergency_reference_in", std::bind(&ControlManager::callbackEmergencyReference, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  ss_pirouette_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/pirouette_in", std::bind(&ControlManager::callbackPirouette, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS(),
      cbkgrp_ss_);

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_timers_;

  mrs_lib::TimerHandlerOptions timer_opts_no_start;

  timer_opts_no_start.node           = node_;
  timer_opts_no_start.autostart      = false;
  timer_opts_no_start.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fcn = std::bind(&ControlManager::timerStatus, this);

    timer_status_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_status_timer_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&ControlManager::timerSafety, this);

    timer_safety_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_safety_timer_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&ControlManager::timerBumper, this);

    timer_bumper_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_bumper_timer_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&ControlManager::timerJoystick, this);

    timer_joystick_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_joystick_timer_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&ControlManager::timerEland, this);

    timer_eland_ = std::make_shared<TimerType>(timer_opts_no_start, rclcpp::Rate(_elanding_timer_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&ControlManager::timerFailsafe, this);

    timer_failsafe_ = std::make_shared<TimerType>(timer_opts_no_start, rclcpp::Rate(_failsafe_timer_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&ControlManager::timerPirouette, this);

    timer_pirouette_ = std::make_shared<TimerType>(timer_opts_no_start, rclcpp::Rate(_pirouette_timer_rate_, clock_), callback_fcn);
  }

  // | ----------------------- finish init ---------------------- |

  if (!param_loader_->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

/* shutdown() //{ */

void ControlManager::shutdown() {

  std::cout << "ControlManager: shutdown(): called" << std::endl;

  timer_status_->stop();
  timer_failsafe_->stop();
  timer_eland_->stop();
  timer_safety_->stop();
  timer_pirouette_->stop();
  timer_bumper_->stop();
  timer_joystick_->stop();

  std::cout << "ControlManager: unloading trackers" << std::endl;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    for (int i = 0; i < int(tracker_list_.size()); i++) {

      std::cout << "ControlManager: unloading " << _tracker_names_[i] << std::endl;

      tracker_list_.at(i).reset();
    }
  }

  std::cout << "ControlManager: unloading controllers" << std::endl;

  {
    std::scoped_lock lock(mutex_controller_list_);

    for (int i = 0; i < int(controller_list_.size()); i++) {

      std::cout << "ControlManager: unloading " << _controller_names_[i] << std::endl;

      controller_list_.at(i).reset();
    }
  }

  std::cout << "ControlManager: finished shutdown()" << std::endl;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerHwApiCapabilities() //{ */

void ControlManager::timerHwApiCapabilities() {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerHwApiCapabilities");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::timerHwApiCapabilities", scope_timer_logger_, scope_timer_enabled_);

  if (!sh_hw_api_capabilities_.hasMsg()) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "waiting for HW API capabilities");
    return;
  }

  auto hw_ap_capabilities = sh_hw_api_capabilities_.getMsg();

  RCLCPP_INFO(node_->get_logger(), "got HW API capabilities, the possible control modes are:");

  if (hw_ap_capabilities->accepts_actuator_cmd) {
    RCLCPP_INFO(node_->get_logger(), "- actuator command");
    _hw_api_inputs_.actuators = true;
  }

  if (hw_ap_capabilities->accepts_control_group_cmd) {
    RCLCPP_INFO(node_->get_logger(), "- control group command");
    _hw_api_inputs_.control_group = true;
  }

  if (hw_ap_capabilities->accepts_attitude_rate_cmd) {
    RCLCPP_INFO(node_->get_logger(), "- attitude rate command");
    _hw_api_inputs_.attitude_rate = true;
  }

  if (hw_ap_capabilities->accepts_attitude_cmd) {
    RCLCPP_INFO(node_->get_logger(), "- attitude command");
    _hw_api_inputs_.attitude = true;
  }

  if (hw_ap_capabilities->accepts_acceleration_hdg_rate_cmd) {
    RCLCPP_INFO(node_->get_logger(), "- acceleration+hdg rate command");
    _hw_api_inputs_.acceleration_hdg_rate = true;
  }

  if (hw_ap_capabilities->accepts_acceleration_hdg_cmd) {
    RCLCPP_INFO(node_->get_logger(), "- acceleration+hdg command");
    _hw_api_inputs_.acceleration_hdg = true;
  }

  if (hw_ap_capabilities->accepts_velocity_hdg_rate_cmd) {
    RCLCPP_INFO(node_->get_logger(), "- velocityhdg rate command");
    _hw_api_inputs_.velocity_hdg_rate = true;
  }

  if (hw_ap_capabilities->accepts_velocity_hdg_cmd) {
    RCLCPP_INFO(node_->get_logger(), "- velocityhdg command");
    _hw_api_inputs_.velocity_hdg = true;
  }

  if (hw_ap_capabilities->accepts_position_cmd) {
    RCLCPP_INFO(node_->get_logger(), "- position command");
    _hw_api_inputs_.position = true;
  }

  initialize();

  timer_hw_api_capabilities_->cancel();
}

//}

/* //{ timerStatus() */

void ControlManager::timerStatus() {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerStatus");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::timerStatus", scope_timer_logger_, scope_timer_enabled_);

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

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 5000,
                         "tracker: '%s', controller: '%s', mass: '%.2f kg', disturbances: body [%.2f, %.2f] N, world [%.2f, %.2f] N", tracker.c_str(),
                         controller.c_str(), mass, bx_b, by_b, wx_w, wy_w);
  }

  // --------------------------------------------------------------
  // |                   publish the diagnostics                  |
  // --------------------------------------------------------------

  publishDiagnostics();

  // --------------------------------------------------------------
  // |                publish if the offboard is on               |
  // --------------------------------------------------------------

  if (offboard_mode_) {

    std_msgs::msg::Empty offboard_on_out;

    ph_offboard_on_.publish(offboard_on_out);
  }

  // --------------------------------------------------------------
  // |                   publish the tilt error                   |
  // --------------------------------------------------------------
  {
    std::scoped_lock lock(mutex_attitude_error_);

    if (tilt_error_) {

      mrs_msgs::msg::Float64Stamped tilt_error_out;
      tilt_error_out.header.stamp    = clock_->now();
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

    mrs_msgs::msg::ControlError msg_out;

    msg_out.header.stamp    = clock_->now();
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

    std_msgs::msg::Float64 mass_estimate_out;
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

      mrs_msgs::msg::Float64Stamped heading_out;
      heading_out.header = uav_state.header;
      heading_out.value  = heading;

      ph_heading_.publish(heading_out);
    }
    catch (...) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "exception caught, could not transform heading");
    }
  }

  // --------------------------------------------------------------
  // |                  publish the current speed                 |
  // --------------------------------------------------------------

  if (_state_input_ == INPUT_UAV_STATE && sh_uav_state_.hasMsg()) {

    double speed = sqrt(pow(uav_state.velocity.linear.x, 2) + pow(uav_state.velocity.linear.y, 2) + pow(uav_state.velocity.linear.z, 2));

    mrs_msgs::msg::Float64Stamped speed_out;
    speed_out.header = uav_state.header;
    speed_out.value  = speed;

    ph_speed_.publish(speed_out);
  }

  // --------------------------------------------------------------
  // |              publish the disturbances markers              |
  // --------------------------------------------------------------

  if (last_control_output.diagnostics.disturbance_estimator && got_uav_state_) {

    visualization_msgs::msg::MarkerArray msg_out;

    double id = 0;

    double multiplier = 1.0;

    Eigen::Quaterniond quat_eigen = mrs_lib::AttitudeConverter(uav_state.pose.orientation);

    Eigen::Vector3d           vec3d;
    geometry_msgs::msg::Point point;

    /* world disturbance //{ */
    {

      visualization_msgs::msg::Marker marker;

      marker.header.frame_id = uav_state.header.frame_id;
      marker.header.stamp    = clock_->now();
      marker.ns              = "control_manager";
      marker.id              = id++;
      marker.type            = visualization_msgs::msg::Marker::ARROW;
      marker.action          = visualization_msgs::msg::Marker::ADD;

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

      visualization_msgs::msg::Marker marker;

      marker.header.frame_id = uav_state.header.frame_id;
      marker.header.stamp    = clock_->now();
      marker.ns              = "control_manager";
      marker.id              = id++;
      marker.type            = visualization_msgs::msg::Marker::ARROW;
      marker.action          = visualization_msgs::msg::Marker::ADD;

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

    mrs_msgs::msg::DynamicsConstraints constraints = sanitized_constraints.constraints;

    ph_current_constraints_.publish(constraints);
  }
}

//}

/* //{ timerSafety() */

void ControlManager::timerSafety() {

  mrs_lib::AtomicScopeFlag unset_running(running_safety_timer_);

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerSafety");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::timerSafety", scope_timer_logger_, scope_timer_enabled_);

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
    RCLCPP_WARN(node_->get_logger(), "timerSafety tried to run while odometry switch in progress");
    return;
  }

  // | ------------------------ timeouts ------------------------ |

  if (_state_input_ == INPUT_UAV_STATE && sh_uav_state_.hasMsg()) {
    double missing_for = (clock_->now() - sh_uav_state_.lastMsgTime()).seconds();

    if (missing_for > _uav_state_max_missing_time_) {
      timeoutUavState(missing_for);
    }
  }

  if (_state_input_ == INPUT_ODOMETRY && sh_odometry_.hasMsg()) {
    double missing_for = (clock_->now() - sh_odometry_.lastMsgTime()).seconds();

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

    if (last_tracker_cmd->use_position_horizontal && !std::holds_alternative<mrs_msgs::msg::HwApiPositionCmd>(last_control_output.control_output.value())) {

      position_error(0) = last_tracker_cmd->position.x - uav_state.pose.position.x;
      position_error(1) = last_tracker_cmd->position.y - uav_state.pose.position.y;

      position_error_set = true;
    }

    if (last_tracker_cmd->use_position_vertical && !std::holds_alternative<mrs_msgs::msg::HwApiPositionCmd>(last_control_output.control_output.value())) {

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

      if ((clock_->now() - controller_tracker_switch_time).seconds() > 1.0) {

        if (!failsafe_triggered_) {

          RCLCPP_ERROR(node_->get_logger(), "activating failsafe land: control_error=%.2f/%.2f m (x: %.2f, y: %.2f, z: %.2f)", position_error->norm(),
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

    std::optional<nav_msgs::msg::Odometry::ConstSharedPtr> innovation;

    if (sh_odometry_innovation_.hasMsg()) {
      innovation = {sh_odometry_innovation_.getMsg()};
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "missing estimator innnovation but the innovation check is enabled!");
    }

    if (innovation) {

      auto [x, y, z] = mrs_lib::getPosition(innovation.value());

      double heading = 0;
      try {
        heading = mrs_lib::getHeading(innovation.value());
      }
      catch (mrs_lib::AttitudeConverter::GetHeadingException &e) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "exception caught: '%s'", e.what());
      }

      double last_innovation = mrs_lib::geometry::dist(vec3_t(x, y, z), vec3_t(0, 0, 0));

      if (last_innovation > _odometry_innovation_threshold_ || radians::diff(heading, 0) > M_PI_2) {

        auto controller_tracker_switch_time = mrs_lib::get_mutexed(mutex_controller_tracker_switch_time_, controller_tracker_switch_time_);

        if ((clock_->now() - controller_tracker_switch_time).seconds() > 1.0) {

          if (!failsafe_triggered_ && !eland_triggered_) {

            RCLCPP_ERROR(node_->get_logger(), "activating emergency land: odometry innovation too large: %.2f/%.2f (x: %.2f, y: %.2f, z: %.2f, heading: %.2f)",
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

    if ((clock_->now() - controller_tracker_switch_time).seconds() > 1.0) {

      if (!failsafe_triggered_ && !eland_triggered_) {

        RCLCPP_ERROR(node_->get_logger(), "activating emergency land: tilt angle too large (%.2f/%.2f deg)", (180.0 / M_PI) * tilt_angle,
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

      if ((clock_->now() - controller_tracker_switch_time).seconds() > 1.0) {

        if (!failsafe_triggered_ && !eland_triggered_) {

          RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "releasing payload due to large position error");

          ungripSrv();
        }
      }
    }

    if (error_size > _eland_threshold_) {

      auto controller_tracker_switch_time = mrs_lib::get_mutexed(mutex_controller_tracker_switch_time_, controller_tracker_switch_time_);

      if ((clock_->now() - controller_tracker_switch_time).seconds() > 1.0) {

        if (!failsafe_triggered_ && !eland_triggered_) {

          RCLCPP_ERROR(node_->get_logger(), "activating emergency land: position error %.2f/%.2f m (error x: %.2f, y: %.2f, z: %.2f)", error_size,
                       _eland_threshold_, position_error.value()(0), position_error.value()(1), position_error.value()(2));

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

      if ((clock_->now() - controller_tracker_switch_time).seconds() > 1.0) {

        if (!failsafe_triggered_ && !eland_triggered_) {

          RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "releasing payload: yaw error %.2f/%.2f deg", (180.0 / M_PI) * yaw_error.value(),
                                (180.0 / M_PI) * _yaw_error_eland_ / 2.0);

          ungripSrv();
        }
      }
    }

    if (yaw_error.value() > _yaw_error_eland_) {

      auto controller_tracker_switch_time = mrs_lib::get_mutexed(mutex_controller_tracker_switch_time_, controller_tracker_switch_time_);

      if ((clock_->now() - controller_tracker_switch_time).seconds() > 1.0) {

        if (!failsafe_triggered_ && !eland_triggered_) {

          RCLCPP_ERROR(node_->get_logger(), "activating emergency land: yaw error %.2f/%.2f deg", (180.0 / M_PI) * yaw_error.value(),
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

    RCLCPP_ERROR(node_->get_logger(), "tilt angle too large, disarming: tilt angle=%.2f/%.2f deg", (180.0 / M_PI) * tilt_angle,
                 (180.0 / M_PI) * _tilt_limit_disarm_);

    arming(false);
  }

  // --------------------------------------------------------------
  // |     disarm the drone when tilt error exceeds the limit     |
  // --------------------------------------------------------------

  if (_tilt_error_disarm_enabled_ && tilt_error) {

    auto controller_tracker_switch_time = mrs_lib::get_mutexed(mutex_controller_tracker_switch_time_, controller_tracker_switch_time_);

    // the time from the last controller/tracker switch
    // fyi: we should not
    double time_from_ctrl_tracker_switch = (clock_->now() - controller_tracker_switch_time).seconds();

    // if the tile error is over the threshold
    // && we are not ramping up during takeoff
    if (fabs(tilt_error.value()) > _tilt_error_disarm_threshold_ && !last_control_output.diagnostics.ramping_up) {

      // only account for the error if some time passed from the last tracker/controller switch
      if (time_from_ctrl_tracker_switch > 1.0) {

        // if the threshold was not exceeded before
        if (!tilt_error_disarm_over_thr_) {

          tilt_error_disarm_over_thr_ = true;
          tilt_error_disarm_time_     = clock_->now();

          RCLCPP_WARN(node_->get_logger(), "tilt error exceeded threshold (%.2f/%.2f deg)", (180.0 / M_PI) * tilt_error.value(),
                      (180.0 / M_PI) * _tilt_error_disarm_threshold_);

          // if it was exceeded before, just keep it
        } else {

          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 100, "tilt error (%.2f deg) over threshold for %.2f s", (180.0 / M_PI) * tilt_error.value(),
                               (clock_->now() - tilt_error_disarm_time_).seconds());
        }

        // if the tile error is bad, but the controller just switched,
        // don't think its bad anymore
      } else {

        tilt_error_disarm_over_thr_ = false;
        tilt_error_disarm_time_     = clock_->now();
      }

      // if the tilt error is fine
    } else {

      // make it fine
      tilt_error_disarm_over_thr_ = false;
      tilt_error_disarm_time_     = clock_->now();
    }

    // calculate the time over the threshold
    double tot = (clock_->now() - tilt_error_disarm_time_).seconds();

    // if the tot exceeds the limit (and if we are actually over the threshold)
    if (tilt_error_disarm_over_thr_ && (tot > _tilt_error_disarm_timeout_)) {

      bool is_flying = offboard_mode_ && active_tracker_idx != _null_tracker_idx_;

      // only when flying and not in failsafe
      if (is_flying) {

        RCLCPP_ERROR(node_->get_logger(), "tilt error too large for %.2f s, disarming", tot);

        toggleOutput(false);
        arming(false);
      }
    }
  }

  // | --------- dropping out of OFFBOARD in mid flight --------- |

  // if we are not in offboard and the drone is in mid air (NullTracker is not active)
  if (offboard_mode_was_true_ && !offboard_mode_ && active_tracker_idx != _null_tracker_idx_) {

    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "we fell out of OFFBOARD in mid air, disabling output");

    toggleOutput(false);
  }
} // namespace control_manager

//}

/* //{ timerEland() */

void ControlManager::timerEland() {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerEland");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::timerEland", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto last_control_output = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);

  if (!last_control_output.control_output) {
    return;
  }

  auto throttle = extractThrottle(last_control_output);

  if (!throttle) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "TODO: implement landing detection mechanism for the current control modality");
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
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "timerEland: last_control_output has not been initialized, returning");
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "tip: the RC eland is probably triggered");
      return;
    }

    // recalculate the mass based on the throttle
    throttle_mass_estimate_ = mrs_lib::quadratic_throttle_model::throttleToForce(common_handlers_->throttle_model, throttle.value()) / common_handlers_->g;
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "landing: initial mass: %.2f throttle_mass_estimate: %.2f", landing_uav_mass_,
                         throttle_mass_estimate_);

    // condition for automatic motor turn off
    if (((throttle_mass_estimate_ < _elanding_cutoff_mass_factor_ * landing_uav_mass_) || throttle < 0.01)) {
      if (!throttle_under_threshold_) {

        throttle_mass_estimate_first_time_ = clock_->now();
        throttle_under_threshold_          = true;
      }

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 100, "throttle is under cutoff factor for %.2f s",
                           (clock_->now() - throttle_mass_estimate_first_time_).seconds());

    } else {
      throttle_mass_estimate_first_time_ = clock_->now();
      throttle_under_threshold_          = false;
    }

    if (throttle_under_threshold_ && ((clock_->now() - throttle_mass_estimate_first_time_).seconds() > _elanding_cutoff_timeout_)) {
      // enable callbacks? ... NO

      RCLCPP_INFO(node_->get_logger(), "reached cutoff throttle, disabling output");
      toggleOutput(false);

      // disarm the drone
      if (_eland_disarm_enabled_) {

        RCLCPP_INFO(node_->get_logger(), "calling for disarm");
        arming(false);
      }

      changeLandingState(IDLE_STATE);

      RCLCPP_WARN(node_->get_logger(), "emergency landing finished");

      RCLCPP_DEBUG(node_->get_logger(), "stopping eland timer");
      timer_eland_->stop();
      RCLCPP_DEBUG(node_->get_logger(), "eland timer stopped");

      // we should NOT set eland_triggered_=true
    }
  }
}

//}

/* //{ timerFailsafe() */

void ControlManager::timerFailsafe() {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "timerFailsafe() spinning");

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerFailsafe");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::timerFailsafe", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  updateControllers(uav_state);

  publish();

  auto last_control_output = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);

  if (!last_control_output.control_output) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "timerFailsafe: the control output produced by the failsafe controller is empty!");
    return;
  }

  auto throttle = extractThrottle(last_control_output);

  if (!throttle) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "FailsafeTimer: could not extract throttle out of the last control output");
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
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "failsafe: initial mass: %.2f throttle_mass_estimate: %.2f", landing_uav_mass_,
                       throttle_mass_estimate_);

  // condition for automatic motor turn off
  if (((throttle_mass_estimate_ < _elanding_cutoff_mass_factor_ * landing_uav_mass_))) {

    if (!throttle_under_threshold_) {

      throttle_mass_estimate_first_time_ = clock_->now();
      throttle_under_threshold_          = true;
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 100, "throttle is under cutoff factor for %.2f s",
                         (clock_->now() - throttle_mass_estimate_first_time_).seconds());

  } else {

    throttle_mass_estimate_first_time_ = clock_->now();
    throttle_under_threshold_          = false;
  }

  // condition for automatic motor turn off
  if (throttle_under_threshold_ && ((clock_->now() - throttle_mass_estimate_first_time_).seconds() > _elanding_cutoff_timeout_)) {

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "detecting zero throttle, disarming");

    toggleOutput(false);

    RCLCPP_INFO(node_->get_logger(), "calling for disarm");
    arming(false);

    RCLCPP_WARN(node_->get_logger(), "failsafe landing finished");

    RCLCPP_DEBUG(node_->get_logger(), "stopping failsafe timer");
    timer_failsafe_->stop();
    RCLCPP_DEBUG(node_->get_logger(), "failsafe timer stopped");
  }
}

//}

/* //{ timerJoystick() */

void ControlManager::timerJoystick() {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerJoystick");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::timerJoystick", scope_timer_logger_, scope_timer_enabled_);

  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  // if start was pressed and held for > 3.0 s
  if (joystick_start_pressed_ && joystick_start_press_time_.seconds() != 0 && (clock_->now() - joystick_start_press_time_).seconds() > 3.0) {

    joystick_start_press_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

    RCLCPP_INFO(node_->get_logger(), "transitioning to joystick control: activating '%s' and '%s'", _joystick_tracker_name_.c_str(),
                _joystick_controller_name_.c_str());

    joystick_start_pressed_ = false;

    switchTracker(_joystick_tracker_name_);
    switchController(_joystick_controller_name_);
  }

  // if RT+LT were pressed and held for > 0.1 s
  if (joystick_failsafe_pressed_ && joystick_failsafe_press_time_.seconds() != 0 && (clock_->now() - joystick_failsafe_press_time_).seconds() > 0.1) {

    joystick_failsafe_press_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

    RCLCPP_INFO(node_->get_logger(), "activating failsafe by joystick");

    joystick_failsafe_pressed_ = false;

    failsafe();
  }

  // if joypads were pressed and held for > 0.1 s
  if (joystick_eland_pressed_ && joystick_eland_press_time_.seconds() != 0 && (clock_->now() - joystick_eland_press_time_).seconds() > 0.1) {

    joystick_eland_press_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

    RCLCPP_INFO(node_->get_logger(), "activating eland by joystick");

    joystick_failsafe_pressed_ = false;

    eland();
  }

  // if back was pressed and held for > 0.1 s
  if (joystick_back_pressed_ && joystick_back_press_time_.seconds() != 0 && (clock_->now() - joystick_back_press_time_).seconds() > 0.1) {

    joystick_back_press_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

    // activate/deactivate the joystick goto functionality
    joystick_goto_enabled_ = !joystick_goto_enabled_;

    RCLCPP_INFO(node_->get_logger(), "joystick control %s", joystick_goto_enabled_ ? "activated" : "deactivated");
  }

  // if the GOTO functionality is enabled...
  if (joystick_goto_enabled_ && sh_joystick_.hasMsg()) {

    auto joystick_data = sh_joystick_.getMsg();

    // create the reference

    std::shared_ptr<mrs_msgs::srv::Vec4::Request> request = std::make_shared<mrs_msgs::srv::Vec4::Request>();

    if (fabs(joystick_data->axes.at(_channel_pitch_)) >= 0.05 || fabs(joystick_data->axes.at(_channel_roll_)) >= 0.05 ||
        fabs(joystick_data->axes.at(_channel_heading_)) >= 0.05 || fabs(joystick_data->axes.at(_channel_throttle_)) >= 0.05) {

      if (_joystick_mode_ == 0) {

        request->goal.at(REF_X)       = _channel_mult_pitch_ * joystick_data->axes.at(_channel_pitch_) * _joystick_carrot_distance_;
        request->goal.at(REF_Y)       = _channel_mult_roll_ * joystick_data->axes.at(_channel_roll_) * _joystick_carrot_distance_;
        request->goal.at(REF_Z)       = _channel_mult_throttle_ * joystick_data->axes.at(_channel_throttle_);
        request->goal.at(REF_HEADING) = _channel_mult_heading_ * joystick_data->axes.at(_channel_heading_);

        std::shared_ptr<mrs_msgs::srv::Vec4::Response> response = std::make_shared<mrs_msgs::srv::Vec4::Response>();

        callbackGotoFcu(request, response);

      } else if (_joystick_mode_ == 1) {

        mrs_msgs::msg::TrajectoryReference trajectory;

        double dt = 0.2;

        trajectory.fly_now         = true;
        trajectory.header.frame_id = "fcu_untilted";
        trajectory.use_heading     = true;
        trajectory.dt              = dt;

        mrs_msgs::msg::Reference point;
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

  if (rc_goto_active_ && !bumper_repulsing_ && last_tracker_cmd && sh_hw_api_rc_.hasMsg()) {

    // create the reference
    std::shared_ptr<mrs_msgs::srv::VelocityReferenceStampedSrv::Request> request = std::make_shared<mrs_msgs::srv::VelocityReferenceStampedSrv::Request>();

    double des_x       = 0;
    double des_y       = 0;
    double des_z       = 0;
    double des_heading = 0;

    bool nothing_to_do = true;

    // copy member variables
    mrs_msgs::msg::HwApiRcChannels::ConstSharedPtr rc_channels = sh_hw_api_rc_.getMsg();

    if (rc_channels->channels.size() < 4) {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "RC control channel numbers are out of range (the # of channels in rc/in topic is %d)",
                            int(rc_channels->channels.size()));
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "tip: this could be caused by the RC failsafe not being configured!");

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

      request->reference.header.frame_id = "fcu_untilted";

      request->reference.reference.use_heading_rate = true;

      request->reference.reference.velocity.x   = des_x;
      request->reference.reference.velocity.y   = des_y;
      request->reference.reference.velocity.z   = des_z;
      request->reference.reference.heading_rate = des_heading;

      std::shared_ptr<mrs_msgs::srv::VelocityReferenceStampedSrv::Response> response = std::make_shared<mrs_msgs::srv::VelocityReferenceStampedSrv::Response>();

      // disable callbacks of all trackers
      std::shared_ptr<std_srvs::srv::SetBool::Request> req_enable_callbacks = std::make_shared<std_srvs::srv::SetBool::Request>();

      // enable the callbacks for the active tracker
      req_enable_callbacks->data = true;
      {
        std::scoped_lock lock(mutex_tracker_list_);

        tracker_list_.at(active_tracker_idx_)->enableCallbacks(req_enable_callbacks);
      }

      callbacks_enabled_ = true;

      callbackVelocityReferenceService(request, response);

      callbacks_enabled_ = false;

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "goto by RC with speed x=%.2f, y=%.2f, z=%.2f, heading_rate=%.2f", des_x, des_y, des_z,
                           des_heading);

      // disable the callbacks back again
      req_enable_callbacks->data = false;
      {
        std::scoped_lock lock(mutex_tracker_list_);

        tracker_list_.at(active_tracker_idx_)->enableCallbacks(req_enable_callbacks);
      }
    }
  }
}

//}

/* //{ timerBumper() */

void ControlManager::timerBumper() {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerBumper");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::timerBumper", scope_timer_logger_, scope_timer_enabled_);

  // | ---------------------- preconditions --------------------- |

  if (!sh_bumper_.hasMsg()) {
    return;
  }

  if (!bumper_enabled_) {
    return;
  }

  // bumper should be only active when flying normally
  if (!isFlyingNormally()) {

    if (!bumper_repulsing_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "bumper can not function, not flying 'normally'");
      return;
    }
  }

  if (!got_uav_state_) {
    return;
  }

  if (sh_bumper_.hasMsg() && (clock_->now() - sh_bumper_.lastMsgTime()).seconds() > 1.0) {

    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "obstacle bumper is missing messages although we got them before");

    return;
  }

  // TODO why was this here?
  /* timer_bumper_.setPeriod(ros::Duration(1.0 / _bumper_timer_rate_)); */

  // | ------------------ call the bumper logic ----------------- |

  bumperPushFromObstacle();
}

//}

/* //{ timerPirouette() */

void ControlManager::timerPirouette() {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerPirouette");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::timerPirouette", scope_timer_logger_, scope_timer_enabled_);

  pirouette_iterator_++;

  double pirouette_duration  = (2 * M_PI) / _pirouette_speed_;
  double pirouette_n_steps   = pirouette_duration * _pirouette_timer_rate_;
  double pirouette_step_size = (2 * M_PI) / pirouette_n_steps;

  if (rc_escalating_failsafe_triggered_ || failsafe_triggered_ || eland_triggered_ || (pirouette_iterator_ > pirouette_duration * _pirouette_timer_rate_)) {

    _pirouette_enabled_ = false;
    timer_pirouette_->stop();

    setCallbacks(true);

    return;
  }

  // set the reference
  mrs_msgs::msg::ReferenceStamped reference_request;

  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  reference_request.header.frame_id      = "";
  reference_request.header.stamp         = rclcpp::Time(0, 0, clock_->get_clock_type());
  reference_request.reference.position.x = last_tracker_cmd->position.x;
  reference_request.reference.position.y = last_tracker_cmd->position.y;
  reference_request.reference.position.z = last_tracker_cmd->position.z;
  reference_request.reference.heading    = pirouette_initial_heading_ + pirouette_iterator_ * pirouette_step_size;

  // enable the callbacks for the active tracker
  {
    std::scoped_lock lock(mutex_tracker_list_);

    std::shared_ptr<std_srvs::srv::SetBool::Request> req_enable_callbacks = std::make_shared<std_srvs::srv::SetBool::Request>();
    req_enable_callbacks->data                                            = true;

    tracker_list_.at(active_tracker_idx_)->enableCallbacks(req_enable_callbacks);

    callbacks_enabled_ = true;
  }

  setReference(reference_request);

  {
    std::scoped_lock lock(mutex_tracker_list_);

    // disable the callbacks for the active tracker
    std::shared_ptr<std_srvs::srv::SetBool::Request> req_enable_callbacks = std::make_shared<std_srvs::srv::SetBool::Request>();
    req_enable_callbacks->data                                            = false;

    tracker_list_.at(active_tracker_idx_)->enableCallbacks(req_enable_callbacks);

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
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::asyncControl", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto uav_state           = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto current_constraints = mrs_lib::get_mutexed(mutex_constraints_, current_constraints_);

  if (!failsafe_triggered_) { // when failsafe is triggered, updateControllers() and publish() is called in timerFailsafe()

    // run the safety timer
    // in the case of large control errors, the safety mechanisms will be triggered before the controllers and trackers are updated...
    while (running_safety_timer_) {

      RCLCPP_DEBUG(node_->get_logger(), "waiting for safety timer to finish");
      clock_->sleep_for(0.001s);

      if (!running_safety_timer_) {
        RCLCPP_DEBUG(node_->get_logger(), "safety timer finished");
        break;
      }
    }

    timerSafety();

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

        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the controller '%s' is enforcing constraints over the ConstraintManager",
                             _controller_names_.at(active_controller_idx).c_str());

      } else if (!enforce && constraints_being_enforced_) {

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "constraint values returned to original values");

        constraints_being_enforced_ = false;

        mrs_lib::set_mutexed(mutex_constraints_, current_constraints, sanitized_constraints_);

        setConstraintsToTrackers(current_constraints);
      }
    }

    publish();
  }

  // if odometry switch happened, we finish it here and turn the safety timer back on
  if (odometry_switch_in_progress_) {

    RCLCPP_DEBUG(node_->get_logger(), "starting safety timer");
    timer_safety_->start();
    RCLCPP_DEBUG(node_->get_logger(), "safety timer started");
    odometry_switch_in_progress_ = false;

    {
      std::scoped_lock lock(mutex_uav_state_);

      RCLCPP_INFO(node_->get_logger(), "odometry after switch: x=%.2f, y=%.2f, z=%.2f, heading=%.2f", uav_state.pose.position.x, uav_state.pose.position.y,
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

void ControlManager::callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackOdometry");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::callbackOdometry", scope_timer_logger_, scope_timer_enabled_);

  // | ------------------ check for time stamp ------------------ |

  {
    std::scoped_lock lock(mutex_uav_state_);

    if (uav_state_.header.stamp == msg->header.stamp) {
      return;
    }
  }

  // | --------------------- check for nans --------------------- |

  if (!validateOdometry(node_, *msg, "callbackOdometry(): msg")) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "incoming 'odometry' contains invalid values, throwing it away");
    return;
  }

  // | ---------------------- frame switch ---------------------- |

  /* Odometry frame switch //{ */

  // | -- prepare an OdometryConstSharedPtr for trackers & controllers -- |

  mrs_msgs::msg::UavState uav_state_odom;

  uav_state_odom.header   = msg->header;
  uav_state_odom.pose     = msg->pose.pose;
  uav_state_odom.velocity = msg->twist.twist;

  // | ----- check for change in odometry frame of reference ---- |

  if (got_uav_state_) {

    if (msg->header.frame_id != uav_state_.header.frame_id) {

      RCLCPP_INFO(node_->get_logger(), "detecting switch of odometry frame");
      {
        std::scoped_lock lock(mutex_uav_state_);

        RCLCPP_INFO(node_->get_logger(), "odometry before switch: x=%.2f, y=%.2f, z=%.2f, heading=%.2f", uav_state_.pose.position.x, uav_state_.pose.position.y,
                    uav_state_.pose.position.z, uav_heading_);
      }

      odometry_switch_in_progress_ = true;

      // we have to stop safety timer, otherwise it will interfere
      RCLCPP_DEBUG(node_->get_logger(), "stopping the safety timer");
      timer_safety_->stop();
      RCLCPP_DEBUG(node_->get_logger(), "safety timer stopped");

      // wait for the safety timer to stop if its running
      while (running_safety_timer_) {

        RCLCPP_DEBUG(node_->get_logger(), "waiting for safety timer to finish");
        clock_->sleep_for(0.001s);

        if (!running_safety_timer_) {
          RCLCPP_DEBUG(node_->get_logger(), "safety timer finished");
          break;
        }
      }

      // we have to also for the oneshot control timer to finish
      while (running_async_control_) {

        RCLCPP_DEBUG(node_->get_logger(), "waiting for control timer to finish");
        clock_->sleep_for(0.001s);

        if (!running_async_control_) {
          RCLCPP_DEBUG(node_->get_logger(), "control timer finished");
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

    uav_state_ = mrs_msgs::msg::UavState();

    uav_state_.header           = msg->header;
    uav_state_.pose             = msg->pose.pose;
    uav_state_.velocity.angular = msg->twist.twist.angular;

    // transform the twist into the header's frame
    {
      // the velocity from the odometry
      geometry_msgs::msg::Vector3Stamped speed_child_frame;
      speed_child_frame.header.frame_id = msg->child_frame_id;
      speed_child_frame.header.stamp    = msg->header.stamp;
      speed_child_frame.vector.x        = msg->twist.twist.linear.x;
      speed_child_frame.vector.y        = msg->twist.twist.linear.y;
      speed_child_frame.vector.z        = msg->twist.twist.linear.z;

      auto res = transformer_->transformSingle(speed_child_frame, msg->header.frame_id);

      if (res) {
        uav_state_.velocity.linear.x = res->vector.x;
        uav_state_.velocity.linear.y = res->vector.y;
        uav_state_.velocity.linear.z = res->vector.z;
      } else {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "could not transform the odometry speed from '%s' to '%s'", msg->child_frame_id.c_str(),
                              msg->header.frame_id.c_str());
        return;
      }
    }

    // calculate the euler angles
    std::tie(uav_roll_, uav_pitch_, uav_yaw_) = mrs_lib::AttitudeConverter(msg->pose.pose.orientation);

    try {
      uav_heading_ = mrs_lib::AttitudeConverter(msg->pose.pose.orientation).getHeading();
    }
    catch (...) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "could not calculate UAV heading");
    }

    transformer_->setDefaultFrame(msg->header.frame_id);

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

void ControlManager::callbackUavState(const mrs_msgs::msg::UavState::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackUavState");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::callbackUavState", scope_timer_logger_, scope_timer_enabled_);

  // | ------------------ check for time stamp ------------------ |

  {
    std::scoped_lock lock(mutex_uav_state_);

    if (uav_state_.header.stamp == msg->header.stamp) {
      return;
    }
  }

  // | --------------------- check for nans --------------------- |

  if (!validateUavState(node_, *msg, "callbackUavState(): msg")) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "incoming 'uav_state' contains invalid values, throwing it away");
    return;
  }

  // | -------------------- check for hiccups ------------------- |

  /* hickup detection //{ */

  double alpha               = 0.99;
  double alpha2              = 0.666;
  double uav_state_count_lim = 1000;

  double uav_state_dt = (clock_->now() - previous_uav_state_.header.stamp).seconds();

  // belive only reasonable numbers
  if (uav_state_dt <= 1.0) {

    uav_state_avg_dt_ = alpha * uav_state_avg_dt_ + (1 - alpha) * uav_state_dt;

    if (uav_state_count_ < uav_state_count_lim) {
      uav_state_count_++;
    }
  }

  if (uav_state_count_ == uav_state_count_lim) {

    /* RCLCPP_INFO_STREAM(node_->get_logger(), "uav_state_dt = " << uav_state_dt); */

    if (uav_state_dt < uav_state_avg_dt_ && uav_state_dt > 0.0001) {

      uav_state_hiccup_factor_ = alpha2 * uav_state_hiccup_factor_ + (1 - alpha2) * (uav_state_avg_dt_ / uav_state_dt);

    } else if (uav_state_avg_dt_ > 0.0001) {

      uav_state_hiccup_factor_ = alpha2 * uav_state_hiccup_factor_ + (1 - alpha2) * (uav_state_dt / uav_state_avg_dt_);
    }

    if (uav_state_hiccup_factor_ > 3.141592653) {

      /* ROS_ERROR_STREAM_THROTTLE(0.1, "hiccup factor = " << uav_state_hiccup_factor_); */

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 2000, " ");
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 2000, "// | ------------------------- WARNING ------------------------ |");
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 2000, "// |                                                            |");
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 2000, "// |            UAV_STATE has a large hiccup factor!            |");
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 2000, "// |           hint, hint: you are probably rosbagging          |");
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 2000, "// |           lot of data or publishing lot of large           |");
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 2000, "// |          messages without mutual nodelet managers.         |");
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 2000, "// |                                                            |");
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 2000, "// | ------------------------- WARNING ------------------------ |");
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 2000, " ");
    }
  }

  //}

  // | ---------------------- frame switch ---------------------- |

  /* frame switch //{ */

  // | ----- check for change in odometry frame of reference ---- |

  if (got_uav_state_) {

    if (msg->estimator_iteration != uav_state_.estimator_iteration) {

      RCLCPP_INFO(node_->get_logger(), "detecting switch of odometry frame");
      {
        std::scoped_lock lock(mutex_uav_state_);

        RCLCPP_INFO(node_->get_logger(), "odometry before switch: x=%.2f, y=%.2f, z=%.2f, heading=%.2f", uav_state_.pose.position.x, uav_state_.pose.position.y,
                    uav_state_.pose.position.z, uav_heading_);
      }

      odometry_switch_in_progress_ = true;

      // we have to stop safety timer, otherwise it will interfere
      RCLCPP_DEBUG(node_->get_logger(), "stopping the safety timer");
      timer_safety_->stop();
      RCLCPP_DEBUG(node_->get_logger(), "safety timer stopped");

      // wait for the safety timer to stop if its running
      while (running_safety_timer_) {

        RCLCPP_DEBUG(node_->get_logger(), "waiting for safety timer to finish");
        clock_->sleep_for(0.001s);

        if (!running_safety_timer_) {
          RCLCPP_DEBUG(node_->get_logger(), "safety timer finished");
          break;
        }
      }

      // we have to also for the oneshot control timer to finish
      while (running_async_control_) {

        RCLCPP_DEBUG(node_->get_logger(), "waiting for control timer to finish");
        clock_->sleep_for(0.001s);

        if (!running_async_control_) {
          RCLCPP_DEBUG(node_->get_logger(), "control timer finished");
          break;
        }
      }

      {
        std::scoped_lock lock(mutex_controller_list_, mutex_tracker_list_);

        tracker_list_.at(active_tracker_idx_)->switchOdometrySource(*msg);
        controller_list_.at(active_controller_idx_)->switchOdometrySource(*msg);
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

    uav_state_ = *msg;

    std::tie(uav_roll_, uav_pitch_, uav_yaw_) = mrs_lib::AttitudeConverter(uav_state_.pose.orientation);

    try {
      uav_heading_ = mrs_lib::AttitudeConverter(uav_state_.pose.orientation).getHeading();
    }
    catch (...) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "could not calculate UAV heading, not updating it");
    }

    transformer_->setDefaultFrame(msg->header.frame_id);

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

void ControlManager::callbackGNSS(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackGNSS");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::callbackGNSS", scope_timer_logger_, scope_timer_enabled_);

  transformer_->setLatLon(msg->latitude, msg->longitude);
}

//}

/* callbackJoystick() //{ */

void ControlManager::callbackJoystick(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackJoystick");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::callbackJoystick", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto active_tracker_idx    = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);

  sensor_msgs::msg::Joy::ConstSharedPtr joystick_data = msg;

  // TODO check if the array is smaller than the largest idx
  if (joystick_data->buttons.size() == 0 || joystick_data->axes.size() == 0) {
    return;
  }

  // | ---- switching back to fallback tracker and controller --- |

  // if any of the A, B, X, Y buttons are pressed when flying with joystick, switch back to fallback controller and tracker
  if ((joystick_data->buttons.at(_channel_A_) == 1 || joystick_data->buttons.at(_channel_B_) == 1 || joystick_data->buttons.at(_channel_X_) == 1 ||
       joystick_data->buttons.at(_channel_Y_) == 1) &&
      active_tracker_idx == _joystick_tracker_idx_ && active_controller_idx == _joystick_controller_idx_) {

    RCLCPP_INFO(node_->get_logger(), "switching from joystick to normal control");

    switchTracker(_joystick_fallback_tracker_name_);
    switchController(_joystick_fallback_controller_name_);

    joystick_goto_enabled_ = false;
  }

  // | ------- joystick control activation ------- |

  // if start button was pressed
  if (joystick_data->buttons.at(_channel_start_) == 1) {

    if (!joystick_start_pressed_) {

      RCLCPP_INFO(node_->get_logger(), "joystick start button pressed");

      joystick_start_pressed_    = true;
      joystick_start_press_time_ = clock_->now();
    }

  } else if (joystick_start_pressed_) {

    RCLCPP_INFO(node_->get_logger(), "joystick start button released");

    joystick_start_pressed_    = false;
    joystick_start_press_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  }

  // | ---------------- Joystick goto activation ---------------- |

  // if back button was pressed
  if (joystick_data->buttons.at(_channel_back_) == 1) {

    if (!joystick_back_pressed_) {

      RCLCPP_INFO(node_->get_logger(), "joystick back button pressed");

      joystick_back_pressed_    = true;
      joystick_back_press_time_ = clock_->now();
    }

  } else if (joystick_back_pressed_) {

    RCLCPP_INFO(node_->get_logger(), "joystick back button released");

    joystick_back_pressed_    = false;
    joystick_back_press_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  }

  // | ------------------------ Failsafes ----------------------- |

  // if LT and RT buttons are both pressed down
  if (joystick_data->axes.at(_channel_LT_) < -0.99 && joystick_data->axes.at(_channel_RT_) < -0.99) {

    if (!joystick_failsafe_pressed_) {

      RCLCPP_INFO(node_->get_logger(), "joystick Failsafe pressed");

      joystick_failsafe_pressed_    = true;
      joystick_failsafe_press_time_ = clock_->now();
    }

  } else if (joystick_failsafe_pressed_) {

    RCLCPP_INFO(node_->get_logger(), "joystick Failsafe released");

    joystick_failsafe_pressed_    = false;
    joystick_failsafe_press_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  }

  // if left and right joypads are both pressed down
  if (joystick_data->buttons.at(_channel_L_joy_) == 1 && joystick_data->buttons.at(_channel_R_joy_) == 1) {

    if (!joystick_eland_pressed_) {

      RCLCPP_INFO(node_->get_logger(), "joystick eland pressed");

      joystick_eland_pressed_    = true;
      joystick_eland_press_time_ = clock_->now();
    }

  } else if (joystick_eland_pressed_) {

    RCLCPP_INFO(node_->get_logger(), "joystick eland released");

    joystick_eland_pressed_    = false;
    joystick_eland_press_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  }
}

//}

/* //{ callbackHwApiStatus() */

void ControlManager::callbackHwApiStatus(const mrs_msgs::msg::HwApiStatus::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackHwApiStatus");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::callbackHwApiStatus", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::msg::HwApiStatus::ConstSharedPtr state = msg;

  // | ------ detect and print the changes in offboard mode ----- |
  if (state->offboard) {

    if (!offboard_mode_) {
      offboard_mode_          = true;
      offboard_mode_was_true_ = true;
      RCLCPP_INFO(node_->get_logger(), "detected: OFFBOARD mode ON");
    }

  } else {

    if (offboard_mode_) {
      offboard_mode_ = false;
      RCLCPP_INFO(node_->get_logger(), "detected: OFFBOARD mode OFF");
    }
  }

  // | --------- detect and print the changes in arming --------- |
  if (state->armed == true) {

    if (!armed_) {
      armed_ = true;
      RCLCPP_INFO(node_->get_logger(), "detected: vehicle ARMED");
    }

  } else {

    if (armed_) {
      armed_ = false;
      RCLCPP_INFO(node_->get_logger(), "detected: vehicle DISARMED");
    }
  }
}

//}

/* //{ callbackRC() */

void ControlManager::callbackRC(const mrs_msgs::msg::HwApiRcChannels::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackRC");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::callbackRC", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::msg::HwApiRcChannels::ConstSharedPtr rc = msg;

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting RC channels");

  // | ------------------- rc joystic control ------------------- |

  // when the switch change its position
  if (_rc_goto_enabled_) {

    if (_rc_joystick_channel_ >= int(rc->channels.size())) {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "RC joystick activation channel number (%d) is out of range [0-%d]", _rc_joystick_channel_,
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

            RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "activating RC joystick");

            callbacks_enabled_ = false;

            /* std::shared_ptr<std_srvs::srv::SetBool::Request> req_goto_out = std::make_shared<std_srvs::srv::SetBool::Request>(); */
            /* req_goto_out->data = false; */

            std::shared_ptr<std_srvs::srv::SetBool::Request> req_enable_callbacks = std::make_shared<std_srvs::srv::SetBool::Request>();
            req_enable_callbacks->data                                            = callbacks_enabled_;

            {
              std::scoped_lock lock(mutex_tracker_list_);

              // disable callbacks of all trackers
              for (int i = 0; i < int(tracker_list_.size()); i++) {
                tracker_list_.at(i)->enableCallbacks(req_enable_callbacks);
              }
            }

            rc_goto_active_ = true;

          } else {

            RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "can not activate RC joystick, not flying normally");
          }

        } else if (channel_high && !rc_joystick_channel_was_low_) {

          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "can not activate RC joystick, the switch is ON from the beginning");
        }
      }

      // rc control deactivation
      if (rc_goto_active_ && channel_low) {

        RCLCPP_INFO(node_->get_logger(), "deactivating RC joystick");

        callbacks_enabled_ = true;

        /* std::shared_ptr<std_srvs::srv::SetBool::Request> req_goto_out = std::make_shared<std_srvs::srv::SetBool::Request>(); */
        /* req_goto_out->data = true; */

        std::shared_ptr<std_srvs::srv::SetBool::Request> req_enable_callbacks = std::make_shared<std_srvs::srv::SetBool::Request>();
        req_enable_callbacks->data                                            = callbacks_enabled_;

        {
          std::scoped_lock lock(mutex_tracker_list_);

          // enable callbacks of all trackers
          for (int i = 0; i < int(tracker_list_.size()); i++) {
            tracker_list_.at(i)->enableCallbacks(req_enable_callbacks);
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

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "RC eland channel number (%d) is out of range [0-%d]", _rc_escalating_failsafe_channel_,
                            int(rc->channels.size()));

    } else {

      if (rc->channels.at(_rc_escalating_failsafe_channel_) >= _rc_escalating_failsafe_threshold_) {

        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "triggering escalating failsafe by RC");

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

void ControlManager::timeoutUavState(const double &missing_for) {

  auto last_control_output = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);

  if (output_enabled_ && last_control_output.control_output && !failsafe_triggered_) {

    // We need to fire up timerFailsafe, which will regularly trigger the controllers
    // in place of the callbackUavState/callbackOdometry().

    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 100, "not receiving uav_state/odometry for %.3f s, initiating failsafe land", missing_for);

    failsafe();
  }
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackSwitchTracker() */

bool ControlManager::callbackSwitchTracker(const std::shared_ptr<mrs_msgs::srv::String::Request>  request,
                                           const std::shared_ptr<mrs_msgs::srv::String::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  if (failsafe_triggered_ || eland_triggered_) {

    std::stringstream ss;
    ss << "can not switch tracker, eland or failsafe active";

    response->message = ss.str();
    response->success = false;

    RCLCPP_WARN_STREAM(node_->get_logger(), "" << ss.str());

    return true;
  }

  if (rc_goto_active_) {

    std::stringstream ss;
    ss << "can not switch tracker, RC joystick is active";

    response->message = ss.str();
    response->success = false;

    RCLCPP_WARN_STREAM(node_->get_logger(), "" << ss.str());

    return true;
  }

  auto [success, tracker_response] = switchTracker(request->value);

  response->success = success;
  response->message = tracker_response;

  return true;
}

//}

/* callbackSwitchController() //{ */

bool ControlManager::callbackSwitchController(const std::shared_ptr<mrs_msgs::srv::String::Request>  request,
                                              const std::shared_ptr<mrs_msgs::srv::String::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  if (failsafe_triggered_ || eland_triggered_) {

    std::stringstream ss;
    ss << "can not switch controller, eland or failsafe active";

    response->message = ss.str();
    response->success = false;

    RCLCPP_WARN_STREAM(node_->get_logger(), "" << ss.str());

    return true;
  }

  if (rc_goto_active_) {

    std::stringstream ss;
    ss << "can not switch controller, RC joystick is active";

    response->message = ss.str();
    response->success = false;

    RCLCPP_WARN_STREAM(node_->get_logger(), "" << ss.str());

    return true;
  }

  auto [success, controller_response] = switchController(request->value);

  response->success = success;
  response->message = controller_response;

  return true;
}

//}

/* //{ callbackSwitchTracker() */

bool ControlManager::callbackTrackerResetStatic([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  std::stringstream message;

  if (failsafe_triggered_ || eland_triggered_) {

    message << "can not reset tracker, eland or failsafe active";

    response->message = message.str();
    response->success = false;

    RCLCPP_WARN_STREAM(node_->get_logger(), "" << message.str());

    return true;
  }

  // reactivate the current tracker
  {
    std::scoped_lock lock(mutex_tracker_list_);

    std::string tracker_name = _tracker_names_.at(active_tracker_idx_);

    bool succ = tracker_list_.at(active_tracker_idx_)->resetStatic();

    if (succ) {
      message << "the tracker '" << tracker_name << "' was reset";
      RCLCPP_INFO_STREAM(node_->get_logger(), "" << message.str());
    } else {
      message << "the tracker '" << tracker_name << "' reset failed!";
      RCLCPP_ERROR_STREAM(node_->get_logger(), "" << message.str());
    }
  }

  response->message = message.str();
  response->success = true;

  return true;
}

//}

/* //{ callbackEHover() */

bool ControlManager::callbackEHover([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  if (failsafe_triggered_ || eland_triggered_) {

    std::stringstream ss;
    ss << "can not switch controller, eland or failsafe active";

    response->message = ss.str();
    response->success = false;

    RCLCPP_WARN_STREAM(node_->get_logger(), "" << ss.str());

    return true;
  }

  RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "ehover trigger by callback");

  auto [success, message] = ehover();

  response->success = success;
  response->message = message;

  return true;
}

//}

/* callbackFailsafe() //{ */

bool ControlManager::callbackFailsafe([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  if (failsafe_triggered_) {

    std::stringstream ss;
    ss << "can not activate failsafe, it is already active";

    response->message = ss.str();
    response->success = false;

    RCLCPP_INFO_STREAM(node_->get_logger(), "" << ss.str());

    return true;
  }

  RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "failsafe triggered by callback");

  auto [success, message] = failsafe();

  response->success = success;
  response->message = message;

  return true;
}

//}

/* callbackFailsafeEscalating() //{ */

bool ControlManager::callbackFailsafeEscalating([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  if (_service_escalating_failsafe_enabled_) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "escalating failsafe triggered by callback");

    auto [success, message] = escalatingFailsafe();

    response->success = success;
    response->message = message;

  } else {

    std::stringstream ss;
    ss << "escalating failsafe is disabled";

    response->success = false;
    response->message = ss.str();

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "%s", ss.str().c_str());
  }

  return true;
}

//}

/* //{ callbackELand() */

bool ControlManager::callbackEland([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "eland triggered by callback");

  auto [success, message] = eland();

  response->success = success;
  response->message = message;

  return true;
}

//}

/* //{ callbackParachute() */

bool ControlManager::callbackParachute([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  if (!_parachute_enabled_) {

    std::stringstream ss;
    ss << "parachute disabled";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    response->message = ss.str();
    response->success = false;
  }

  RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "parachute triggered by callback");

  auto [success, message] = deployParachute();

  response->success = success;
  response->message = message;

  return true;
}

//}

//}

/* //{ callbackToggleOutput() */

bool ControlManager::callbackToggleOutput(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                                          const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "toggling output by service");

  // copy member variables
  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  std::stringstream ss;

  bool prereq_check = true;

  {
    if (sh_safety_area_diag_.hasMsg()) {
      if (!sh_safety_area_diag_.getMsg()->position_valid_2d) {
        ss << "cannot toggle output, the UAV is outside of the safety area!";
        prereq_check = false;
      }
    }
  }

  if (request->data && (failsafe_triggered_ || eland_triggered_ || rc_escalating_failsafe_triggered_)) {
    ss << "cannot toggle output ON, we landed in emergency";
    prereq_check = false;
  }

  if (!sh_hw_api_status_.hasMsg() || (clock_->now() - sh_hw_api_status_.lastMsgTime()).seconds() > 1.0) {
    ss << "cannot toggle output ON, missing HW API status!";
    prereq_check = false;
  }

  if (!prereq_check) {

    response->message = ss.str();
    response->success = false;

    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    return false;

  } else {

    toggleOutput(request->data);

    ss << "Output: " << (output_enabled_ ? "ON" : "OFF");
    response->message = ss.str();
    response->success = true;

    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    publishDiagnostics();

    return true;
  }
}

//}

/* callbackArm() //{ */

bool ControlManager::callbackArm(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                                 const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "arming by service");

  std::stringstream ss;

  if (failsafe_triggered_ || eland_triggered_) {

    ss << "can not " << (request->data ? "arm" : "disarm") << ", eland or failsafe active";

    response->message = ss.str();
    response->success = false;

    RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());

    return true;
  }

  if (request->data) {

    ss << "this service is not allowed to arm the UAV";
    response->success = false;
    RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());

  } else {

    auto [success, message] = arming(false);

    if (success) {

      ss << "disarmed";
      response->success = true;
      RCLCPP_INFO_STREAM(node_->get_logger(), "" << ss.str());

    } else {

      ss << "could not disarm: " << message;
      response->success = false;
      RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());
    }
  }

  response->message = ss.str();

  return true;
}

//}

/* //{ callbackEnableCallbacks() */

bool ControlManager::callbackEnableCallbacks(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                                             const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  setCallbacks(request->data);

  std::stringstream ss;

  ss << "callbacks " << (callbacks_enabled_ ? "enabled" : "disabled");

  response->message = ss.str();
  response->success = true;

  RCLCPP_INFO_STREAM(node_->get_logger(), "" << ss.str());

  return true;
}

//}

/* callbackSetConstraints() //{ */

bool ControlManager::callbackSetConstraints(const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request>  request,
                                            const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response> response) {

  if (!is_initialized_) {
    response->message = "not initialized";
    response->success = false;
    return true;
  }

  {
    std::scoped_lock lock(mutex_constraints_);

    current_constraints_ = *request;

    auto enforced = enforceControllersConstraints(current_constraints_);

    if (enforced) {
      sanitized_constraints_      = enforced.value();
      constraints_being_enforced_ = true;
    } else {
      sanitized_constraints_      = *request;
      constraints_being_enforced_ = false;
    }

    got_constraints_ = true;

    setConstraintsToControllers(current_constraints_);
    setConstraintsToTrackers(sanitized_constraints_);
  }

  response->message = "setting constraints";
  response->success = true;

  return true;
}

//}

/* //{ callbackEmergencyReference() */

bool ControlManager::callbackEmergencyReference(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request>  request,
                                                const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  callbacks_enabled_ = false;

  std::shared_ptr<mrs_msgs::srv::ReferenceSrv::Response> tracker_response = std::make_shared<mrs_msgs::srv::ReferenceSrv::Response>();

  std::stringstream ss;

  // transform the reference to the current frame
  mrs_msgs::msg::ReferenceStamped original_reference;
  original_reference.header    = request->header;
  original_reference.reference = request->reference;

  auto ret = transformer_->transformSingle(original_reference, uav_state.header.frame_id);

  if (!ret) {

    ss << "the emergency reference could not be transformed";

    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    response->message = ss.str();
    response->success = false;
    return true;
  }

  mrs_msgs::msg::ReferenceStamped transformed_reference = ret.value();

  std::shared_ptr<std_srvs::srv::SetBool::Request> req_enable_callbacks = std::make_shared<std_srvs::srv::SetBool::Request>();

  std::shared_ptr<mrs_msgs::srv::ReferenceSrv::Request> req_goto_out = std::make_shared<mrs_msgs::srv::ReferenceSrv::Request>();
  req_goto_out->reference                                            = transformed_reference.reference;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    // disable callbacks of all trackers
    req_enable_callbacks->data = false;
    for (int i = 0; i < int(tracker_list_.size()); i++) {
      tracker_list_.at(i)->enableCallbacks(req_enable_callbacks);
    }

    // enable the callbacks for the active tracker
    req_enable_callbacks->data = true;
    tracker_list_.at(active_tracker_idx_)->enableCallbacks(req_enable_callbacks);

    // call the setReference()
    tracker_response = tracker_list_.at(active_tracker_idx_)->setReference(req_goto_out);

    // disable the callbacks back again
    req_enable_callbacks->data = false;
    tracker_list_.at(active_tracker_idx_)->enableCallbacks(req_enable_callbacks);

    if (tracker_response != nullptr) {
      response->message = tracker_response->message;
      response->success = tracker_response->success;
    } else {
      ss << "the tracker '" << _tracker_names_.at(active_tracker_idx_) << "' does not implement the 'setReference()' function!";
      response->message = ss.str();
      response->success = false;
    }
  }

  return true;
}

//}

/* callbackPirouette() //{ */

bool ControlManager::callbackPirouette([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

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

    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    response->message = ss.str();
    response->success = false;

    return false;
  }

  if (_pirouette_enabled_) {
    response->success = false;
    response->message = "already active";
    return true;
  }

  if (failsafe_triggered_ || eland_triggered_ || rc_escalating_failsafe_triggered_) {

    std::stringstream ss;
    ss << "can not activate the pirouette, eland or failsafe active";

    response->message = ss.str();
    response->success = false;

    RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());

    return true;
  }

  _pirouette_enabled_ = true;

  setCallbacks(false);

  pirouette_initial_heading_ = uav_heading;
  pirouette_iterator_        = 0;
  timer_pirouette_->start();

  response->success = true;
  response->message = "activated";

  return true;
}

//}

/* callbackUseJoystick() //{ */

bool ControlManager::callbackUseJoystick([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  std::stringstream ss;

  {
    auto [success, tracker_response] = switchTracker(_joystick_tracker_name_);

    if (!success) {

      ss << "switching to '" << _joystick_tracker_name_ << "' was unsuccessfull: '" << tracker_response << "'";
      RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());

      response->success = false;
      response->message = ss.str();

      return true;
    }
  }

  auto [success, controller_response] = switchController(_joystick_controller_name_);

  if (!success) {

    ss << "switching to '" << _joystick_controller_name_ << "' was unsuccessfull: '" << controller_response << "'";
    RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());

    response->success = false;
    response->message = ss.str();

    // switch back to hover tracker
    switchTracker(_ehover_tracker_name_);

    // switch back to safety controller
    switchController(_eland_controller_name_);

    RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());

    return true;
  }

  ss << "switched to joystick control";

  response->success = true;
  response->message = ss.str();

  RCLCPP_INFO_STREAM(node_->get_logger(), "" << ss.str());

  return true;
}

//}

/* //{ callbackHover() */

bool ControlManager::callbackHover([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  auto [success, message] = hover();

  response->success = success;
  response->message = message;

  return true;
}

//}

/* //{ callbackStartTrajectoryTracking() */

bool ControlManager::callbackStartTrajectoryTracking([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                     const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  auto [success, message] = startTrajectoryTracking();

  response->success = success;
  response->message = message;

  return true;
}

//}

/* //{ callbackStopTrajectoryTracking() */

bool ControlManager::callbackStopTrajectoryTracking([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                    const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  auto [success, message] = stopTrajectoryTracking();

  response->success = success;
  response->message = message;

  return true;
}

//}

/* //{ callbackResumeTrajectoryTracking() */

bool ControlManager::callbackResumeTrajectoryTracking([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                      const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  auto [success, message] = resumeTrajectoryTracking();

  response->success = success;
  response->message = message;

  return true;
}

//}

/* //{ callbackGotoTrajectoryStart() */

bool ControlManager::callbackGotoTrajectoryStart([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                 const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  auto [success, message] = gotoTrajectoryStart();

  response->success = success;
  response->message = message;

  return true;
}

//}

/* //{ callbackTransformReference() */

bool ControlManager::callbackTransformReference(const std::shared_ptr<mrs_msgs::srv::TransformReferenceSrv::Request>  request,
                                                const std::shared_ptr<mrs_msgs::srv::TransformReferenceSrv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  // transform the reference to the current frame
  mrs_msgs::msg::ReferenceStamped transformed_reference = request->reference;

  if (auto ret = transformer_->transformSingle(transformed_reference, request->frame_id)) {

    response->reference = ret.value();
    response->message   = "transformation successful";
    response->success   = true;
    return true;

  } else {

    response->message = "the reference could not be transformed";
    response->success = false;
    return true;
  }

  return true;
}

//}

/* //{ callbackTransformReferenceArray() */

bool ControlManager::callbackTransformReferenceArray(const std::shared_ptr<mrs_msgs::srv::TransformReferenceArraySrv::Request>  request,
                                                     const std::shared_ptr<mrs_msgs::srv::TransformReferenceArraySrv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  // transform the reference array to the current frame
  const auto tf_opt = transformer_->getTransform(request->array.header.frame_id, request->to_frame_id, request->array.header.stamp);
  if (!tf_opt.has_value()) {
    response->message = "The reference array could not be transformed";
    response->success = false;
    return true;
  }
  const auto tf = tf_opt.value();

  response->array.header.stamp    = request->array.header.stamp;
  response->array.header.frame_id = request->to_frame_id;
  response->array.array.reserve(request->array.array.size());

  for (const auto &ref : request->array.array) {

    mrs_msgs::msg::ReferenceStamped ref_stamped;
    ref_stamped.header    = request->array.header;
    ref_stamped.reference = ref;

    if (auto ret = transformer_->transform(ref_stamped, tf)) {

      response->array.array.push_back(ret.value().reference);

    } else {

      response->message = "The reference array could not be transformed";
      response->success = false;
      return true;
    }
  }

  response->message = "transformation successful";
  response->success = true;
  return true;
}

//}

/* //{ callbackTransformPose() */

bool ControlManager::callbackTransformPose(const std::shared_ptr<mrs_msgs::srv::TransformPoseSrv::Request>  request,
                                           const std::shared_ptr<mrs_msgs::srv::TransformPoseSrv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  // transform the reference to the current frame
  geometry_msgs::msg::PoseStamped transformed_pose = request->pose;

  if (auto ret = transformer_->transformSingle(transformed_pose, request->frame_id)) {

    response->pose    = ret.value();
    response->message = "transformation successful";
    response->success = true;
    return true;

  } else {

    response->message = "the pose could not be transformed";
    response->success = false;
    return true;
  }

  return true;
}

//}

/* //{ callbackTransformVector3() */

bool ControlManager::callbackTransformVector3(const std::shared_ptr<mrs_msgs::srv::TransformVector3Srv::Request>  request,
                                              const std::shared_ptr<mrs_msgs::srv::TransformVector3Srv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  // transform the reference to the current frame
  geometry_msgs::msg::Vector3Stamped transformed_vector3 = request->vector;

  if (auto ret = transformer_->transformSingle(transformed_vector3, request->frame_id)) {

    response->vector  = ret.value();
    response->message = "transformation successful";
    response->success = true;
    return true;

  } else {

    response->message = "the twist could not be transformed";
    response->success = false;
    return true;
  }

  return true;
}

//}

/* //{ callbackEnableBumper() */

bool ControlManager::callbackEnableBumper(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                                          const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  bumper_enabled_ = request->data;

  std::stringstream ss;

  ss << "bumper " << (bumper_enabled_ ? "enalbed" : "disabled");

  RCLCPP_INFO_STREAM(node_->get_logger(), "" << ss.str());

  response->success = true;
  response->message = ss.str();

  return true;
}

//}

/* //{ callbackGetMinZ() */

bool ControlManager::callbackGetMinZ([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::GetFloat64::Request> request,
                                     const std::shared_ptr<mrs_msgs::srv::GetFloat64::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  response->success = true;
  response->value   = getMinZ(uav_state.header.frame_id);

  return true;
}

//}

/* //{ callbackValidateReference() */

bool ControlManager::callbackValidateReference(const std::shared_ptr<mrs_msgs::srv::ValidateReference::Request>  request,
                                               const std::shared_ptr<mrs_msgs::srv::ValidateReference::Response> response) {

  if (!is_initialized_) {
    response->message = "not initialized";
    response->success = false;
    return true;
  }

  if (!validateReference(node_, request->reference.reference, "callbackValidateReference(): request->reference.reference")) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "NaN detected in variable 'request->reference'!!!");
    response->message = "NaNs/infs in input!";
    response->success = false;
    return true;
  }

  // copy member variables
  auto uav_state        = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  // transform the reference to the current frame
  mrs_msgs::msg::ReferenceStamped original_reference;
  original_reference.header    = request->reference.header;
  original_reference.reference = request->reference.reference;

  auto ret = transformer_->transformSingle(original_reference, uav_state.header.frame_id);

  if (!ret) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the reference could not be transformed");
    response->message = "the reference could not be transformed";
    response->success = false;
    return true;
  }

  mrs_msgs::msg::ReferenceStamped transformed_reference = ret.value();

  if (!isPointInSafetyArea3d(transformed_reference)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "reference validation failed, the point is outside of the safety area!");
    response->message = "the point is outside of the safety area";
    response->success = false;
    return true;
  }

  if (last_tracker_cmd) {

    mrs_msgs::msg::ReferenceStamped from_point;
    from_point.header.frame_id      = uav_state.header.frame_id;
    from_point.reference.position.x = last_tracker_cmd->position.x;
    from_point.reference.position.y = last_tracker_cmd->position.y;
    from_point.reference.position.z = last_tracker_cmd->position.z;

    if (!isPathToPointInSafetyArea3d(from_point, transformed_reference)) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "reference validation failed, the path is going outside the safety area!");
      response->message = "the path is going outside the safety area";
      response->success = false;
      return true;
    }
  }

  response->message = "the reference is ok";
  response->success = true;
  return true;
}

//}

/* //{ callbackValidateReference2d() */

bool ControlManager::callbackValidateReference2d(const std::shared_ptr<mrs_msgs::srv::ValidateReference::Request>  request,
                                                 const std::shared_ptr<mrs_msgs::srv::ValidateReference::Response> response) {

  if (!is_initialized_) {
    response->message = "not initialized";
    response->success = false;
    return true;
  }

  if (!validateReference(node_, request->reference.reference, "callbackValidateReference2d(): request->reference.reference")) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "NaN detected in variable 'request->reference'!!!");
    response->message = "NaNs/infs in input!";
    response->success = false;
    return true;
  }

  // copy member variables
  auto uav_state        = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  // transform the reference to the current frame
  mrs_msgs::msg::ReferenceStamped original_reference;
  original_reference.header    = request->reference.header;
  original_reference.reference = request->reference.reference;

  auto ret = transformer_->transformSingle(original_reference, uav_state.header.frame_id);

  if (!ret) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the reference could not be transformed");
    response->message = "the reference could not be transformed";
    response->success = false;
    return true;
  }

  mrs_msgs::msg::ReferenceStamped transformed_reference = ret.value();

  if (!isPointInSafetyArea2d(transformed_reference)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "reference validation failed, the point is outside of the safety area!");
    response->message = "the point is outside of the safety area";
    response->success = false;
    return true;
  }

  if (last_tracker_cmd) {

    mrs_msgs::msg::ReferenceStamped from_point;
    from_point.header.frame_id      = uav_state.header.frame_id;
    from_point.reference.position.x = last_tracker_cmd->position.x;
    from_point.reference.position.y = last_tracker_cmd->position.y;
    from_point.reference.position.z = last_tracker_cmd->position.z;

    if (!isPathToPointInSafetyArea2d(from_point, transformed_reference)) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "reference validation failed, the path is going outside the safety area!");
      response->message = "the path is going outside the safety area";
      response->success = false;
      return true;
    }
  }

  response->message = "the reference is ok";
  response->success = true;
  return true;
}

//}

/* //{ callbackValidateReferenceArray() */

bool ControlManager::callbackValidateReferenceArray(const std::shared_ptr<mrs_msgs::srv::ValidateReferenceArray::Request>  request,
                                                    const std::shared_ptr<mrs_msgs::srv::ValidateReferenceArray::Response> response) {

  if (!is_initialized_) {
    response->message = "not initialized";
    return false;
  }

  // copy member variables
  auto uav_state        = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  // get the transformer
  auto opt_tf = transformer_->getTransform(uav_state.header.frame_id, request->array.header.frame_id, request->array.header.stamp);

  if (!opt_tf) {
    RCLCPP_WARN(node_->get_logger(), "could not find transform for the reference's frame_id");
    response->message = "could not find transform for the reference's frame_id";
    return true;
  }

  geometry_msgs::msg::TransformStamped tf = opt_tf.value();

  for (int i = 0; i < int(request->array.array.size()); i++) {

    response->success.push_back(true);

    // | ---------- check if the "number" are valid first --------- |

    mrs_msgs::msg::ReferenceStamped original_reference;
    original_reference.header    = request->array.header;
    original_reference.reference = request->array.array.at(i);

    if (!validateReference(node_, original_reference.reference, "callbackValidateReferenceArray(): original_reference.reference")) {

      RCLCPP_WARN(node_->get_logger(), "the reference for validation ([%d]) contains invalid numbers", i);
      response->success.at(i) = false;
    }

    // | --- transform the reference into current control frame --- |

    auto ret = transformer_->transform(original_reference, tf);

    if (!ret) {
      RCLCPP_WARN(node_->get_logger(), "the reference for validation ([%d]) could not be transformed", i);
      response->success.at(i) = false;
    }

    mrs_msgs::msg::ReferenceStamped transformed_reference = ret.value();

    // | --- check transformed reference agains the safety area --- |

    if (!isPointInSafetyArea3d(transformed_reference)) {
      response->success.at(i) = false;
    }

    // check the path from the current point (if flying) to the reference

    if (last_tracker_cmd) {

      mrs_msgs::msg::ReferenceStamped from_point;
      from_point.header.frame_id      = uav_state.header.frame_id;
      from_point.reference.position.x = last_tracker_cmd->position.x;
      from_point.reference.position.y = last_tracker_cmd->position.y;
      from_point.reference.position.z = last_tracker_cmd->position.z;

      if (!isPathToPointInSafetyArea3d(from_point, transformed_reference)) {
        response->success.at(i) = false;
      }
    }
  }

  response->message = "references were checked";
  return true;
}

//}

// | -------------- setpoint topics and services -------------- |

/* //{ callbackReferenceService() */

bool ControlManager::callbackReferenceService(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request>  request,
                                              const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response) {

  if (!is_initialized_) {
    response->message = "not initialized";
    response->success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackReferenceService");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::callbackReferenceService", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::msg::ReferenceStamped des_reference;
  des_reference.header    = request->header;
  des_reference.reference = request->reference;

  auto [success, message] = setReference(des_reference);

  response->success = success;
  response->message = message;

  return true;
}

//}

/* //{ callbackReferenceTopic() */

void ControlManager::callbackReferenceTopic(const mrs_msgs::msg::ReferenceStamped::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackReferenceTopic");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::callbackReferenceTopic", scope_timer_logger_, scope_timer_enabled_);

  setReference(*msg);
}

//}

/* //{ callbackVelocityReferenceService() */

bool ControlManager::callbackVelocityReferenceService(const std::shared_ptr<mrs_msgs::srv::VelocityReferenceStampedSrv::Request>  request,
                                                      const std::shared_ptr<mrs_msgs::srv::VelocityReferenceStampedSrv::Response> response) {

  if (!is_initialized_) {
    response->message = "not initialized";
    response->success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackVelocityReferenceService");
  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer(node_, "ControlManager::callbackVelocityReferenceService", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::msg::VelocityReferenceStamped des_reference;
  des_reference = request->reference;

  auto [success, message] = setVelocityReference(des_reference);

  response->success = success;
  response->message = message;

  return true;
}

//}

/* //{ callbackVelocityReferenceTopic() */

void ControlManager::callbackVelocityReferenceTopic(const mrs_msgs::msg::VelocityReferenceStamped::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackVelocityReferenceTopic");
  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer(node_, "ControlManager::callbackVelocityReferenceTopic", scope_timer_logger_, scope_timer_enabled_);

  setVelocityReference(*msg);
}

//}

/* //{ callbackTrajectoryReferenceService() */

bool ControlManager::callbackTrajectoryReferenceService(const std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Request>  request,
                                                        const std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Response> response) {

  if (!is_initialized_) {
    response->message = "not initialized";
    response->success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackTrajectoryReferenceService");
  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer(node_, "ControlManager::callbackTrajectoryReferenceService", scope_timer_logger_, scope_timer_enabled_);

  auto [success, message, modified, tracker_names, tracker_successes, tracker_messages] = setTrajectoryReference(request->trajectory);

  response->success          = success;
  response->message          = message;
  response->modified         = modified;
  response->tracker_names    = tracker_names;
  response->tracker_messages = tracker_messages;

  for (size_t i = 0; i < tracker_successes.size(); i++) {
    response->tracker_successes.push_back(tracker_successes.at(i));
  }

  return true;
}

//}

/* //{ callbackTrajectoryReferenceTopic() */

void ControlManager::callbackTrajectoryReferenceTopic(const mrs_msgs::msg::TrajectoryReference::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackTrajectoryReferenceTopic");
  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer(node_, "ControlManager::callbackTrajectoryReferenceTopic", scope_timer_logger_, scope_timer_enabled_);

  setTrajectoryReference(*msg);
}

//}

// | ------------- human-callable "goto" services ------------- |

/* //{ callbackGoto() */

bool ControlManager::callbackGoto(const std::shared_ptr<mrs_msgs::srv::Vec4::Request> request, const std::shared_ptr<mrs_msgs::srv::Vec4::Response> response) {

  if (!is_initialized_) {
    response->message = "not initialized";
    response->success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackGoto");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::callbackGoto", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::msg::ReferenceStamped des_reference;
  des_reference.header.frame_id      = "";
  des_reference.header.stamp         = rclcpp::Time(0, 0, clock_->get_clock_type());
  des_reference.reference.position.x = request->goal.at(REF_X);
  des_reference.reference.position.y = request->goal.at(REF_Y);
  des_reference.reference.position.z = request->goal.at(REF_Z);
  des_reference.reference.heading    = request->goal.at(REF_HEADING);

  auto [success, message] = setReference(des_reference);

  response->success = success;
  response->message = message;

  return true;
}

//}

/* //{ callbackGotoFcu() */

bool ControlManager::callbackGotoFcu(const std::shared_ptr<mrs_msgs::srv::Vec4::Request>  request,
                                     const std::shared_ptr<mrs_msgs::srv::Vec4::Response> response) {

  if (!is_initialized_) {
    response->message = "not initialized";
    response->success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackGotoFcu");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::callbackGotoFcu", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::msg::ReferenceStamped des_reference;
  des_reference.header.frame_id      = "fcu_untilted";
  des_reference.header.stamp         = rclcpp::Time(0, 0, clock_->get_clock_type());
  des_reference.reference.position.x = request->goal.at(REF_X);
  des_reference.reference.position.y = request->goal.at(REF_Y);
  des_reference.reference.position.z = request->goal.at(REF_Z);
  des_reference.reference.heading    = request->goal.at(REF_HEADING);

  auto [success, message] = setReference(des_reference);

  response->success = success;
  response->message = message;

  return true;
}

//}

/* //{ callbackGotoRelative() */

bool ControlManager::callbackGotoRelative(const std::shared_ptr<mrs_msgs::srv::Vec4::Request>  request,
                                          const std::shared_ptr<mrs_msgs::srv::Vec4::Response> response) {

  if (!is_initialized_) {
    response->message = "not initialized";
    response->success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackGotoRelative");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::callbackGotoRelative", scope_timer_logger_, scope_timer_enabled_);

  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  if (!last_tracker_cmd) {
    response->message = "not flying";
    response->success = false;
    return true;
  }

  mrs_msgs::msg::ReferenceStamped des_reference;
  des_reference.header.frame_id      = "";
  des_reference.header.stamp         = rclcpp::Time(0, 0, clock_->get_clock_type());
  des_reference.reference.position.x = last_tracker_cmd->position.x + request->goal.at(REF_X);
  des_reference.reference.position.y = last_tracker_cmd->position.y + request->goal.at(REF_Y);
  des_reference.reference.position.z = last_tracker_cmd->position.z + request->goal.at(REF_Z);
  des_reference.reference.heading    = last_tracker_cmd->heading + request->goal.at(REF_HEADING);

  auto [success, message] = setReference(des_reference);

  response->success = success;
  response->message = message;

  return true;
}

//}

/* //{ callbackGotoAltitude() */

bool ControlManager::callbackGotoAltitude(const std::shared_ptr<mrs_msgs::srv::Vec1::Request>  request,
                                          const std::shared_ptr<mrs_msgs::srv::Vec1::Response> response) {

  if (!is_initialized_) {
    response->message = "not initialized";
    response->success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackGotoAltitude");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::callbackGotoAltitude", scope_timer_logger_, scope_timer_enabled_);

  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  if (!last_tracker_cmd) {
    response->message = "not flying";
    response->success = false;
    return true;
  }

  mrs_msgs::msg::ReferenceStamped des_reference;
  des_reference.header.frame_id      = "";
  des_reference.header.stamp         = rclcpp::Time(0, 0, clock_->get_clock_type());
  des_reference.reference.position.x = last_tracker_cmd->position.x;
  des_reference.reference.position.y = last_tracker_cmd->position.y;
  des_reference.reference.position.z = request->goal;
  des_reference.reference.heading    = last_tracker_cmd->heading;

  auto [success, message] = setReference(des_reference);

  response->success = success;
  response->message = message;

  return true;
}

//}

/* //{ callbackSetHeading() */

bool ControlManager::callbackSetHeading(const std::shared_ptr<mrs_msgs::srv::Vec1::Request>  request,
                                        const std::shared_ptr<mrs_msgs::srv::Vec1::Response> response) {

  if (!is_initialized_) {
    response->message = "not initialized";
    response->success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackSetHeading");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::callbackSetHeading", scope_timer_logger_, scope_timer_enabled_);

  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  if (!last_tracker_cmd) {
    response->message = "not flying";
    response->success = false;
    return true;
  }

  mrs_msgs::msg::ReferenceStamped des_reference;
  des_reference.header.frame_id      = "";
  des_reference.header.stamp         = rclcpp::Time(0, 0, clock_->get_clock_type());
  des_reference.reference.position.x = last_tracker_cmd->position.x;
  des_reference.reference.position.y = last_tracker_cmd->position.y;
  des_reference.reference.position.z = last_tracker_cmd->position.z;
  des_reference.reference.heading    = request->goal;

  auto [success, message] = setReference(des_reference);

  response->success = success;
  response->message = message;

  return true;
}

//}

/* //{ callbackSetHeadingRelative() */

bool ControlManager::callbackSetHeadingRelative(const std::shared_ptr<mrs_msgs::srv::Vec1::Request>  request,
                                                const std::shared_ptr<mrs_msgs::srv::Vec1::Response> response) {

  if (!is_initialized_) {
    response->message = "not initialized";
    response->success = false;
    return true;
  }

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackSetHeadingRelative");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::callbackSetHeadingRelative", scope_timer_logger_, scope_timer_enabled_);

  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  if (!last_tracker_cmd) {
    response->message = "not flying";
    response->success = false;
    return true;
  }

  mrs_msgs::msg::ReferenceStamped des_reference;
  des_reference.header.frame_id      = "";
  des_reference.header.stamp         = rclcpp::Time(0, 0, clock_->get_clock_type());
  des_reference.reference.position.x = last_tracker_cmd->position.x;
  des_reference.reference.position.y = last_tracker_cmd->position.y;
  des_reference.reference.position.z = last_tracker_cmd->position.z;
  des_reference.reference.heading    = last_tracker_cmd->heading + request->goal;

  auto [success, message] = setReference(des_reference);

  response->success = success;
  response->message = message;

  return true;
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* setReference() //{ */

std::tuple<bool, std::string> ControlManager::setReference(const mrs_msgs::msg::ReferenceStamped reference_in) {

  std::stringstream ss;

  if (!callbacks_enabled_) {
    ss << "can not set the reference, the callbacks are disabled";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    return std::tuple(false, ss.str());
  }

  if (!validateReference(node_, reference_in.reference, "setReference(): reference_in.reference")) {
    ss << "incoming reference is not finite!!!";
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    return std::tuple(false, ss.str());
  }

  // copy member variables
  auto uav_state        = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  // transform the reference to the current frame
  auto ret = transformer_->transformSingle(reference_in, uav_state.header.frame_id);

  if (!ret) {

    ss << "the reference could not be transformed";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    return std::tuple(false, ss.str());
  }

  mrs_msgs::msg::ReferenceStamped transformed_reference = ret.value();

  // safety area check
  if (!isPointInSafetyArea3d(transformed_reference)) {
    ss << "failed to set the reference, the point is outside of the safety area!";
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    return std::tuple(false, ss.str());
  }

  if (last_tracker_cmd) {

    mrs_msgs::msg::ReferenceStamped from_point;
    from_point.header.frame_id      = uav_state.header.frame_id;
    from_point.reference.position.x = last_tracker_cmd->position.x;
    from_point.reference.position.y = last_tracker_cmd->position.y;
    from_point.reference.position.z = last_tracker_cmd->position.z;

    if (!isPathToPointInSafetyArea3d(from_point, transformed_reference)) {
      ss << "failed to set the reference, the path is going outside the safety area!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return std::tuple(false, ss.str());
    }
  }

  // prepare the message for current tracker
  std::shared_ptr<mrs_msgs::srv::ReferenceSrv::Request> reference_request = std::make_shared<mrs_msgs::srv::ReferenceSrv::Request>();
  reference_request->reference                                            = transformed_reference.reference;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    RCLCPP_INFO(node_->get_logger(), "setting reference to x=%.2f, y=%.2f, z=%.2f, hdg=%.2f (expressed in '%s')", transformed_reference.reference.position.x,
                transformed_reference.reference.position.y, transformed_reference.reference.position.z, transformed_reference.reference.heading,
                transformed_reference.header.frame_id.c_str());

    auto tracker_response = tracker_list_.at(active_tracker_idx_)->setReference(reference_request);

    if (tracker_response != nullptr) {

      return std::tuple(tracker_response->success, tracker_response->message);

    } else {

      ss << "the tracker '" << _tracker_names_.at(active_tracker_idx_) << "' does not implement the 'setReference()' function!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "failed to set the reference: " << ss.str());
      return std::tuple(false, ss.str());
    }
  }
}

//}

/* setVelocityReference() //{ */

std::tuple<bool, std::string> ControlManager::setVelocityReference(const mrs_msgs::msg::VelocityReferenceStamped &reference_in) {

  std::stringstream ss;

  if (!callbacks_enabled_) {
    ss << "can not set the reference, the callbacks are disabled";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    return std::tuple(false, ss.str());
  }

  if (!validateVelocityReference(node_, reference_in.reference, "setVelocityReference(): reference_in.reference")) {
    ss << "velocity command is not valid!";
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    return std::tuple(false, ss.str());
  }

  {
    std::scoped_lock lock(mutex_last_tracker_cmd_);

    if (!last_tracker_cmd_) {
      ss << "could not set velocity command, not flying!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return std::tuple(false, ss.str());
    }
  }

  // copy member variables
  auto uav_state        = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_tracker_cmd = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);

  // | -- transform the velocity reference to the current frame - |

  mrs_msgs::msg::VelocityReferenceStamped transformed_reference = reference_in;

  auto ret = transformer_->getTransform(reference_in.header.frame_id, uav_state.header.frame_id, reference_in.header.stamp);

  geometry_msgs::msg::TransformStamped tf;

  if (!ret) {
    ss << "could not find tf from " << reference_in.header.frame_id << " to " << uav_state.header.frame_id;
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    return std::tuple(false, ss.str());
  } else {
    tf = ret.value();
  }

  // transform the velocity
  {
    geometry_msgs::msg::Vector3Stamped velocity;
    velocity.header   = reference_in.header;
    velocity.vector.x = reference_in.reference.velocity.x;
    velocity.vector.y = reference_in.reference.velocity.y;
    velocity.vector.z = reference_in.reference.velocity.z;

    auto ret = transformer_->transform(velocity, tf);

    if (!ret) {

      ss << "the velocity reference could not be transformed";
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return std::tuple(false, ss.str());

    } else {
      transformed_reference.reference.velocity.x = ret->vector.x;
      transformed_reference.reference.velocity.y = ret->vector.y;
      transformed_reference.reference.velocity.z = ret->vector.z;
    }
  }

  // transform the z and the heading
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header           = reference_in.header;
    pose.pose.position.x  = 0;
    pose.pose.position.y  = 0;
    pose.pose.position.z  = reference_in.reference.altitude;
    pose.pose.orientation = mrs_lib::AttitudeConverter(0, 0, reference_in.reference.heading);

    auto ret = transformer_->transform(pose, tf);

    if (!ret) {

      ss << "the velocity reference could not be transformed";
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
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

  mrs_msgs::msg::ReferenceStamped eqivalent_reference = velocityReferenceToReference(transformed_reference);

  RCLCPP_DEBUG(node_->get_logger(), "equivalent reference: %.2f, %.2f, %.2f, %.2f", eqivalent_reference.reference.position.x,
               eqivalent_reference.reference.position.y, eqivalent_reference.reference.position.z, eqivalent_reference.reference.heading);

  // safety area check
  if (!isPointInSafetyArea3d(eqivalent_reference)) {
    ss << "failed to set the reference, the point is outside of the safety area!";
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    return std::tuple(false, ss.str());
  }

  if (last_tracker_cmd) {

    mrs_msgs::msg::ReferenceStamped from_point;
    from_point.header.frame_id      = uav_state.header.frame_id;
    from_point.reference.position.x = last_tracker_cmd->position.x;
    from_point.reference.position.y = last_tracker_cmd->position.y;
    from_point.reference.position.z = last_tracker_cmd->position.z;

    if (!isPathToPointInSafetyArea3d(from_point, eqivalent_reference)) {
      ss << "failed to set the reference, the path is going outside the safety area!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return std::tuple(false, ss.str());
    }
  }

  // prepare the message for current tracker
  std::shared_ptr<mrs_msgs::srv::VelocityReferenceSrv::Request> reference_request = std::make_shared<mrs_msgs::srv::VelocityReferenceSrv::Request>();
  reference_request->reference                                                    = transformed_reference.reference;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    auto tracker_response = tracker_list_.at(active_tracker_idx_)->setVelocityReference(reference_request);

    if (tracker_response != nullptr) {

      return std::tuple(tracker_response->success, tracker_response->message);

    } else {

      ss << "the tracker '" << _tracker_names_.at(active_tracker_idx_) << "' does not implement the 'setVelocityReference()' function!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "failed to set the velocity reference: " << ss.str());
      return std::tuple(false, ss.str());
    }
  }
}

//}

/* setTrajectoryReference() //{ */

std::tuple<bool, std::string, bool, std::vector<std::string>, std::vector<bool>, std::vector<std::string>>
ControlManager::setTrajectoryReference(const mrs_msgs::msg::TrajectoryReference trajectory_in) {

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  std::stringstream ss;

  if (!callbacks_enabled_) {
    ss << "can not set the reference, the callbacks are disabled";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());
  }

  /* validate the size and check for NaNs //{ */

  // check for the size 0, which is invalid
  if (trajectory_in.points.size() == 0) {

    ss << "can not load trajectory with size 0";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());
  }

  for (int i = 0; i < int(trajectory_in.points.size()); i++) {

    // check the point for NaN/inf
    bool valid = validateReference(node_, trajectory_in.points.at(i), "setTrajectoryReference(): trajectory_in.points.at(i),");

    if (!valid) {

      ss << "trajectory contains NaNs/infs.";
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());
    }
  }

  //}

  /* publish the debugging topics of the original trajectory //{ */

  {

    geometry_msgs::msg::PoseArray debug_trajectory_out;
    debug_trajectory_out.header = trajectory_in.header;

    debug_trajectory_out.header.frame_id = transformer_->resolveFrame(debug_trajectory_out.header.frame_id);

    if (debug_trajectory_out.header.stamp == rclcpp::Time(0, 0, clock_->get_clock_type())) {
      debug_trajectory_out.header.stamp = clock_->now();
    }

    for (int i = 0; i < int(trajectory_in.points.size()) - 1; i++) {

      geometry_msgs::msg::Pose new_pose;

      new_pose.position.x = trajectory_in.points.at(i).position.x;
      new_pose.position.y = trajectory_in.points.at(i).position.y;
      new_pose.position.z = trajectory_in.points.at(i).position.z;

      new_pose.orientation = mrs_lib::AttitudeConverter(0, 0, trajectory_in.points.at(i).heading);

      debug_trajectory_out.poses.push_back(new_pose);
    }

    pub_debug_original_trajectory_poses_.publish(debug_trajectory_out);

    visualization_msgs::msg::MarkerArray msg_out;

    visualization_msgs::msg::Marker marker;

    marker.header = trajectory_in.header;

    marker.header.frame_id = transformer_->resolveFrame(marker.header.frame_id);

    if (marker.header.frame_id == "") {
      marker.header.frame_id = uav_state.header.frame_id;
    }

    if (marker.header.stamp == rclcpp::Time(0, 0, clock_->get_clock_type())) {
      marker.header.stamp = clock_->now();
    }

    marker.type             = visualization_msgs::msg::Marker::LINE_LIST;
    marker.color.a          = 1;
    marker.scale.x          = 0.05;
    marker.color.r          = 0;
    marker.color.g          = 1;
    marker.color.b          = 0;
    marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

    for (int i = 0; i < int(trajectory_in.points.size()) - 1; i++) {

      geometry_msgs::msg::Point point1;

      point1.x = trajectory_in.points.at(i).position.x;
      point1.y = trajectory_in.points.at(i).position.y;
      point1.z = trajectory_in.points.at(i).position.z;

      marker.points.push_back(point1);

      geometry_msgs::msg::Point point2;

      point2.x = trajectory_in.points.at(i + 1).position.x;
      point2.y = trajectory_in.points.at(i + 1).position.y;
      point2.z = trajectory_in.points.at(i + 1).position.z;

      marker.points.push_back(point2);
    }

    msg_out.markers.push_back(marker);

    pub_debug_original_trajectory_markers_.publish(msg_out);
  }

  //}

  mrs_msgs::msg::TrajectoryReference processed_trajectory = trajectory_in;

  int trajectory_size = int(processed_trajectory.points.size());

  bool trajectory_modified = false;

  /* safety area check //{ */

  if (sh_safety_area_diag_.hasMsg()) {
    auto msg = sh_safety_area_diag_.getMsg();

    if (msg->safety_area_enabled) {

      int last_valid_idx    = 0;
      int first_invalid_idx = -1;

      double min_z = getMinZ(processed_trajectory.header.frame_id);
      double max_z = getMaxZ(processed_trajectory.header.frame_id);

      for (int i = 0; i < trajectory_size; i++) {

        if (_snap_trajectory_to_safety_area_) {

          // saturate the trajectory to min and max Z
          if (processed_trajectory.points.at(i).position.z < min_z) {

            processed_trajectory.points.at(i).position.z = min_z;
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the trajectory violates the minimum Z!");
            trajectory_modified = true;
          }

          if (processed_trajectory.points.at(i).position.z > max_z) {

            processed_trajectory.points.at(i).position.z = max_z;
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the trajectory violates the maximum Z!");
            trajectory_modified = true;
          }
        }

        // check the point against the safety area
        mrs_msgs::msg::ReferenceStamped des_reference;
        des_reference.header    = processed_trajectory.header;
        des_reference.reference = processed_trajectory.points.at(i);

        if (!isPointInSafetyArea3d(des_reference)) {

          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the trajectory contains points outside of the safety area!");
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
              RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
              return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());

              // we have a valid point in the past
            } else {

              if (!_snap_trajectory_to_safety_area_) {
                break;
              }

              bool interpolation_success = true;

              // iterpolate between the last valid point and this new valid
              // point
              double angle = atan2((processed_trajectory.points.at(i).position.y - processed_trajectory.points.at(last_valid_idx).position.y),
                                   (processed_trajectory.points.at(i).position.x - processed_trajectory.points.at(last_valid_idx).position.x));

              double dist_two_points = mrs_lib::geometry::dist(
                  vec2_t(processed_trajectory.points.at(i).position.x, processed_trajectory.points.at(i).position.y),
                  vec2_t(processed_trajectory.points.at(last_valid_idx).position.x, processed_trajectory.points.at(last_valid_idx).position.y));
              double step = dist_two_points / (i - last_valid_idx);

              for (int j = last_valid_idx; j < i; j++) {
                mrs_msgs::msg::ReferenceStamped temp_point;
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

        // special case, the trajectory does not end with a valid point
        if (first_invalid_idx != -1) {

          // super special case, the whole trajectory is invalid
          if (first_invalid_idx == 0) {

            ss << "the whole trajectory is outside of the safety area!";
            RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
            return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());

            // there is a good portion of the trajectory in the beginning
          } else {

            trajectory_size = last_valid_idx + 1;
            processed_trajectory.points.resize(trajectory_size);
            trajectory_modified = true;
          }
        }
      }
    }

    if (trajectory_size == 0) {

      ss << "the trajectory happened to be empty after all the checks! This message should not appear!";
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());
    }

    //}

    /* transform the trajectory to the current control frame //{ */

    std::optional<geometry_msgs::msg::TransformStamped> tf_traj_state;

    if (rclcpp::Time(processed_trajectory.header.stamp).seconds() > clock_->now().seconds()) {
      tf_traj_state = transformer_->getTransform(processed_trajectory.header.frame_id, "", processed_trajectory.header.stamp);
    } else {
      tf_traj_state = transformer_->getTransform(processed_trajectory.header.frame_id, "", uav_state_.header.stamp);
    }

    if (!tf_traj_state) {
      ss << "could not create TF transformer for the trajectory";
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());
    }

    processed_trajectory.header.frame_id = transformer_->frame_to(*tf_traj_state);

    for (int i = 0; i < trajectory_size; i++) {

      mrs_msgs::msg::ReferenceStamped trajectory_point;
      trajectory_point.header    = processed_trajectory.header;
      trajectory_point.reference = processed_trajectory.points.at(i);

      auto ret = transformer_->transform(trajectory_point, *tf_traj_state);

      if (!ret) {

        ss << "trajectory cannnot be transformed";
        RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
        return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());

      } else {

        // transform the points in the trajectory to the current frame
        processed_trajectory.points.at(i) = ret.value().reference;
      }
    }

    //}

    std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Request> request = std::make_shared<mrs_msgs::srv::TrajectoryReferenceSrv::Request>();

    // check for empty trajectory
    if (processed_trajectory.points.size() == 0) {
      ss << "reference trajectory was processing and it is now empty, this should not happen!";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
      return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());
    }

    // prepare the message for current tracker
    request->trajectory = processed_trajectory;

    bool                     success;
    std::string              message;
    bool                     modified;
    std::vector<std::string> tracker_names;
    std::vector<bool>        tracker_successes;
    std::vector<std::string> tracker_messages;

    {
      std::scoped_lock lock(mutex_tracker_list_);

      // set the trajectory to the currently active tracker
      auto response = tracker_list_.at(active_tracker_idx_)->setTrajectoryReference(request);

      tracker_names.push_back(_tracker_names_.at(active_tracker_idx_));

      if (response != nullptr) {

        success  = response->success;
        message  = response->message;
        modified = response->modified || trajectory_modified;
        tracker_successes.push_back(response->success);
        tracker_messages.push_back(response->message);

      } else {

        ss << "the active tracker '" << _tracker_names_.at(active_tracker_idx_) << "' does not implement the 'setTrajectoryReference()' function!";
        RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

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

          response = tracker_list_.at(i)->setTrajectoryReference(request);

          if (response != nullptr) {

            tracker_successes.push_back(response->success);
            tracker_messages.push_back(response->message);

            if (response->success) {
              std::stringstream ss;
              ss << "trajectory loaded to non-active tracker '" << _tracker_names_.at(i);
              RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
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
  } else {

    ss << "safety area diagnostics message is not available, can not set trajectory";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    return std::tuple(false, ss.str(), false, std::vector<std::string>(), std::vector<bool>(), std::vector<std::string>());
  }
}

//}

/* isOffboard() //{ */

bool ControlManager::isOffboard(void) {

  if (!sh_hw_api_status_.hasMsg()) {
    return false;
  }

  auto hw_api_status = sh_hw_api_status_.getMsg();

  return hw_api_status->connected && hw_api_status->offboard;
}

//}

/* setCallbacks() //{ */

void ControlManager::setCallbacks(bool in) {

  callbacks_enabled_ = in;

  std::shared_ptr<std_srvs::srv::SetBool::Request> req_enable_callbacks = std::make_shared<std_srvs::srv::SetBool::Request>();
  req_enable_callbacks->data                                            = callbacks_enabled_;

  {
    std::scoped_lock lock(mutex_tracker_list_);

    // set callbacks to all trackers
    for (int i = 0; i < int(tracker_list_.size()); i++) {
      tracker_list_.at(i)->enableCallbacks(req_enable_callbacks);
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
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::publishDiagnostics", scope_timer_logger_, scope_timer_enabled_);

  mrs_msgs::msg::ControlManagerDiagnostics diagnostics_msg;

  diagnostics_msg.stamp    = clock_->now();
  diagnostics_msg.uav_name = _uav_name_;

  diagnostics_msg.desired_uav_state_rate = desired_uav_state_rate_;

  diagnostics_msg.output_enabled = output_enabled_;

  diagnostics_msg.joystick_active = rc_goto_active_;

  diagnostics_msg.flying_normally = isFlyingNormally();

  diagnostics_msg.bumper_active = bumper_repulsing_;

  // | ----------------- fill the tracker status ---------------- |

  {
    std::scoped_lock lock(mutex_tracker_list_);

    mrs_msgs::msg::TrackerStatus tracker_status;

    diagnostics_msg.active_tracker = _tracker_names_.at(active_tracker_idx_);
    diagnostics_msg.tracker_status = tracker_list_.at(active_tracker_idx_)->getStatus();
  }

  // | --------------- fill the controller status --------------- |

  {
    std::scoped_lock lock(mutex_controller_list_);

    mrs_msgs::msg::ControllerStatus controller_status;

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

void ControlManager::setConstraintsToTrackers(const mrs_msgs::srv::DynamicsConstraintsSrv::Request &constraints) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("setConstraintsToTrackers");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::setConstraintsToTrackers", scope_timer_logger_, scope_timer_enabled_);

  std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> request = std::make_shared<mrs_msgs::srv::DynamicsConstraintsSrv::Request>(constraints);

  {
    std::scoped_lock lock(mutex_tracker_list_);

    // for each tracker
    for (int i = 0; i < int(tracker_list_.size()); i++) {

      // if it is the active one, update and retrieve the command
      auto response = tracker_list_.at(i)->setConstraints(request);
    }
  }
}

//}

/* setConstraintsToControllers() //{ */

void ControlManager::setConstraintsToControllers(const mrs_msgs::srv::DynamicsConstraintsSrv::Request &constraints) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("setConstraintsToControllers");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::setConstraintsToControllers", scope_timer_logger_, scope_timer_enabled_);

  std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> request = std::make_shared<mrs_msgs::srv::DynamicsConstraintsSrv::Request>(constraints);

  {
    std::scoped_lock lock(mutex_controller_list_);

    // for each controller
    for (int i = 0; i < int(controller_list_.size()); i++) {

      // if it is the active one, update and retrieve the command
      auto response = controller_list_.at(i)->setConstraints(request);
    }
  }
}

//}

/* setConstraints() //{ */

void ControlManager::setConstraints(const mrs_msgs::srv::DynamicsConstraintsSrv::Request &constraints) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("setConstraints");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::setConstraints", scope_timer_logger_, scope_timer_enabled_);

  setConstraintsToTrackers(constraints);

  setConstraintsToControllers(constraints);
}

//}

/* enforceControllerConstraints() //{ */

std::optional<mrs_msgs::srv::DynamicsConstraintsSrv::Request>
ControlManager::enforceControllersConstraints(const mrs_msgs::srv::DynamicsConstraintsSrv::Request &constraints) {

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

  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);
  auto active_tracker_idx    = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);

  // clang-format off
  return
      callbacks_enabled_
      && output_enabled_
      && offboard_mode_
      && armed_
      && (_controller_names_.size() == 1 || ((active_controller_idx != _eland_controller_idx_) && (active_controller_idx != _failsafe_controller_idx_)))
      && (_tracker_names_.size() == 1 || ((active_tracker_idx != _null_tracker_idx_) && (active_tracker_idx != _landoff_tracker_idx_) && (active_tracker_idx != _ehover_tracker_idx_)));
  // clang-format on
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

// | ----------------------- safety area ---------------------- |

/* //{ isPointInSafetyArea3d() */
bool ControlManager::isPointInSafetyArea3d(const mrs_msgs::msg::ReferenceStamped &point) {
  std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();

  request->header    = point.header;
  request->reference = point.reference;

  auto response = sch_point_in_safety_area_3d_.callSync(request);
  if (!response) {
    RCLCPP_WARN(node_->get_logger(), "SafetyArea: Service call to check if the point is in the safety area failed");
    return false;
  }
  return response.value()->success;
}

//}

/* //{ isPointInSafetyArea2d() */
bool ControlManager::isPointInSafetyArea2d(const mrs_msgs::msg::ReferenceStamped &point) {

  std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();

  request->header    = point.header;
  request->reference = point.reference;

  auto response = sch_point_in_safety_area_2d_.callSync(request);
  if (!response) {
    RCLCPP_WARN(node_->get_logger(), "SafetyArea: Service call to check if the point is in the safety area failed");
    return false;
  }

  return response.value()->success;
}

//}

/* //{ isPathToPointInSafetyArea3d() */
bool ControlManager::isPathToPointInSafetyArea3d(const mrs_msgs::msg::ReferenceStamped &start, const mrs_msgs::msg::ReferenceStamped &end) {
  std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Request> request = std::make_shared<mrs_msgs::srv::ValidatePathToPointSrv::Request>();

  request->start.header = start.header;
  request->start.point  = start.reference.position;

  request->end.header = end.header;
  request->end.point  = end.reference.position;

  auto response = sch_path_to_point_in_safety_area_3d_.callSync(request);
  if (!response) {
    RCLCPP_WARN(node_->get_logger(), "SafetyArea: Service call to check if the point is in the safety area failed");
    return false;
  }
  return response.value()->success;
}


//}

/* //{ isPathToPointInSafetyArea2d() */
bool ControlManager::isPathToPointInSafetyArea2d(const mrs_msgs::msg::ReferenceStamped &start, const mrs_msgs::msg::ReferenceStamped &end) {
  std::shared_ptr<mrs_msgs::srv::ValidatePathToPointSrv::Request> request = std::make_shared<mrs_msgs::srv::ValidatePathToPointSrv::Request>();
  request->start.header                                                   = start.header;
  request->start.point                                                    = start.reference.position;

  request->end.header = end.header;
  request->end.point  = end.reference.position;

  auto response = sch_path_to_point_in_safety_area_2d_.callSync(request);
  if (!response) {
    RCLCPP_WARN(node_->get_logger(), "SafetyArea: Service call to check if the point is in the safety area failed");
    return false;
  }
  return response.value()->success;
}

//}

/* //{ getMaxZ() */

double ControlManager::getMaxZ(const std::string &frame_id) {

  double safety_area_max_z = std::numeric_limits<float>::max();

  {
    // | ------- first, get max_z from the safety area ------ |
    std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Request> request = std::make_shared<mrs_msgs::srv::GetReferenceStampedSrv::Request>();


    auto response = sch_get_max_z_.callSync(request);

    if (!response) {
      RCLCPP_WARN(node_->get_logger(), "SafetyArea: Service call to get max_z from the safety area timed out");
    } else {
      geometry_msgs::msg::PointStamped point;
      point.header  = response.value()->reference.header;
      point.point.x = 0;
      point.point.y = 0;
      point.point.z = response.value()->reference.reference.position.z;
      auto ret      = transformer_->transformSingle(point, frame_id);

      if (!ret) {
        RCLCPP_WARN(node_->get_logger(), "SafetyArea: Could not transform safety area's max_z to '%s'", frame_id.c_str());
      }
      safety_area_max_z = ret->point.z;
    }
  }

  // | ------------ overwrite from estimation manager ----------- |

  double estimation_manager_max_z = std::numeric_limits<float>::max();

  {
    // if possible, override it with max z from the estimation manager
    if (sh_max_z_.hasMsg()) {

      auto msg = sh_max_z_.getMsg();

      // transform it into the safety area frame
      geometry_msgs::msg::PointStamped point;
      point.header  = msg->header;
      point.point.x = 0;
      point.point.y = 0;
      point.point.z = msg->value;

      auto ret = transformer_->transformSingle(point, frame_id);

      if (!ret) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "SafetyArea: Could not transform estimation manager's max_z to the current control frame");
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

double ControlManager::getMinZ(const std::string &frame_id) {

  {
    std::shared_ptr<mrs_msgs::srv::GetReferenceStampedSrv::Request> request = std::make_shared<mrs_msgs::srv::GetReferenceStampedSrv::Request>();

    auto response = sch_get_min_z_.callSync(request);

    if (!response) {
      RCLCPP_WARN(node_->get_logger(), "SafetyArea: Service call to get min_z from the safety area timed out");
      return std::numeric_limits<double>::lowest();
    }

    geometry_msgs::msg::PointStamped point;
    point.header  = response.value()->reference.header;
    point.point.x = 0;
    point.point.y = 0;
    point.point.z = response.value()->reference.reference.position.z;
    auto ret      = transformer_->transformSingle(point, frame_id);

    if (!ret) {
      RCLCPP_WARN(node_->get_logger(), "SafetyArea: Could not transform safety area's min_z to '%s'", frame_id.c_str());
      return std::numeric_limits<double>::lowest();
    }

    return ret->point.z;
  }
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
  mrs_msgs::msg::ObstacleSectors::ConstSharedPtr bumper_data = sh_bumper_.getMsg();
  auto                                           uav_state   = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

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

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "Bumper: found potential collision (sector %lu vs. %d), obstacle distance: %.2f, repulsing",
                         min_sector_id, oposite_sector_idx, bumper_data->sectors.at(min_sector_id));

    repulsion_distance = min_distance_horizontal + _bumper_horizontal_overshoot_ - bumper_data->sectors.at(min_sector_id);

    horizontal_collision_detected = true;
  }

  double vertical_repulsion_distance = 0;

  // check for vertical collision down
  if (bumper_data->sectors.at(bumper_data->n_horizontal_sectors) > 0 && bumper_data->sectors.at(bumper_data->n_horizontal_sectors) <= min_distance_vertical) {

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "Bumper: potential collision below, obstacle distance: %.2f, limit: %.2f",
                         bumper_data->sectors.at(bumper_data->n_horizontal_sectors), min_distance_vertical);
    vertical_collision_detected = true;
    vertical_repulsion_distance = min_distance_vertical - bumper_data->sectors.at(bumper_data->n_horizontal_sectors) + _bumper_vertical_overshoot_;
  }

  // check for vertical collision up
  if (bumper_data->sectors.at(bumper_data->n_horizontal_sectors + 1) > 0 &&
      bumper_data->sectors.at(bumper_data->n_horizontal_sectors + 1) <= min_distance_vertical) {

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "Bumper: potential collision above, obstacle distance: %.2f, limit: %.2f",
                         bumper_data->sectors.at(bumper_data->n_horizontal_sectors + 1), min_distance_vertical);
    vertical_collision_detected = true;
    vertical_repulsion_distance = -(min_distance_vertical - bumper_data->sectors.at(bumper_data->n_horizontal_sectors + 1) + _bumper_vertical_overshoot_);
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

    std::shared_ptr<std_srvs::srv::SetBool::Request> req_enable_callbacks = std::make_shared<std_srvs::srv::SetBool::Request>();

    // create the reference in the fcu_untilted frame
    mrs_msgs::msg::ReferenceStamped reference_fcu_untilted;

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
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "Bumper: bumper reference could not be transformed");
        return;
      }

      reference_fcu_untilted = ret.value();

      // copy the reference into the service type message
      std::shared_ptr<mrs_msgs::srv::ReferenceSrv::Request> req_goto_out = std::make_shared<mrs_msgs::srv::ReferenceSrv::Request>();
      req_goto_out->reference                                            = reference_fcu_untilted.reference;

      // disable callbacks of all trackers
      req_enable_callbacks->data = false;
      for (size_t i = 0; i < tracker_list_.size(); i++) {
        tracker_list_.at(i)->enableCallbacks(req_enable_callbacks);
      }

      // enable the callbacks for the active tracker
      req_enable_callbacks->data = true;
      tracker_list_.at(active_tracker_idx_)->enableCallbacks(req_enable_callbacks);

      // call the goto
      auto tracker_response = tracker_list_.at(active_tracker_idx_)->setReference(req_goto_out);

      // disable the callbacks back again
      req_enable_callbacks->data = false;
      tracker_list_.at(active_tracker_idx_)->enableCallbacks(req_enable_callbacks);
    }
  }

  // if repulsing_ and the distance is safe once again
  if (bumper_repulsing_ && !horizontal_collision_detected && !vertical_collision_detected) {

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "Bumper: no more collision, stopping repulsion");

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

    std::shared_ptr<std_srvs::srv::SetBool::Request> req_enable_callbacks = std::make_shared<std_srvs::srv::SetBool::Request>();

    {
      std::scoped_lock lock(mutex_tracker_list_);

      // enable callbacks of all trackers
      req_enable_callbacks->data = true;
      for (size_t i = 0; i < tracker_list_.size(); i++) {
        tracker_list_.at(i)->enableCallbacks(req_enable_callbacks);
      }
    }

    callbacks_enabled_ = true;

    bumper_repulsing_ = false;
  }
}

//}

/* bumperGetSectorId() //{ */

int ControlManager::bumperGetSectorId(const double &x, const double &y, [[maybe_unused]] const double &z) {

  // copy member variables
  auto bumper_data = sh_bumper_.getMsg();

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

    RCLCPP_DEBUG(node_->get_logger(), "starting eland timer");
    timer_eland_->start();
    RCLCPP_DEBUG(node_->get_logger(), "eland timer started");
    eland_triggered_ = true;
    bumper_enabled_  = false;

    landing_uav_mass_ = getMass();
  }

  break;
  }

  RCLCPP_INFO(node_->get_logger(), "switching emergency landing state %s -> %s", state_names[previous_state_landing_], state_names[current_state_landing_]);
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

    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto response = tracker_list_.at(active_tracker_idx_)->hover(request);

    if (response != nullptr) {

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
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

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
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

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
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    }
  }

  std::stringstream ss;
  ss << "ehover activated";
  RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

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
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

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
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

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
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    }
  }

  // | ----------------- call the eland service ----------------- |

  std::stringstream ss;
  bool              success;

  if (elandSrv()) {

    changeLandingState(LANDING_STATE);

    odometryCallbacksSrv(false);

    ss << "eland activated";
    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    success = true;

    callbacks_enabled_ = false;

  } else {

    ss << "error during activation of eland";
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

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
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
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
      RCLCPP_INFO_STREAM(node_->get_logger(), "" << ss.str());

      return std::tuple(true, ss.str());

    } else {

      std::stringstream ss;
      ss << "could not deploy parachute: '" << message << "', continuing with normal failsafe";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    }
  }

  if (_failsafe_controller_idx_ != active_controller_idx) {

    try {

      std::scoped_lock lock(mutex_controller_list_);

      RCLCPP_INFO(node_->get_logger(), "activating the controller '%s'", _failsafe_controller_name_.c_str());
      controller_list_.at(_failsafe_controller_idx_)->activate(last_control_output);

      {
        std::scoped_lock lock(mutex_controller_tracker_switch_time_);

        // update the time (used in failsafe)
        controller_tracker_switch_time_ = clock_->now();
      }

      failsafe_triggered_ = true;
      RCLCPP_DEBUG(node_->get_logger(), "stopping eland timer");
      timer_eland_->stop();
      RCLCPP_DEBUG(node_->get_logger(), "eland timer stopped");

      landing_uav_mass_ = getMass();

      eland_triggered_ = false;
      RCLCPP_DEBUG(node_->get_logger(), "starting failsafe timer");
      timer_failsafe_->start();
      RCLCPP_DEBUG(node_->get_logger(), "failsafe timer started");

      bumper_enabled_ = false;

      odometryCallbacksSrv(false);

      callbacks_enabled_ = false;

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "the controller '%s' was activated", _failsafe_controller_name_.c_str());

      // super important, switch the active controller idx
      try {
        controller_list_.at(active_controller_idx_)->deactivate();
        active_controller_idx_ = _failsafe_controller_idx_;
      }
      catch (std::runtime_error &exrun) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "could not deactivate the controller '%s'",
                              _controller_names_.at(active_controller_idx_).c_str());
      }
    }
    catch (std::runtime_error &exrun) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "error during activation of the controller '%s'", _failsafe_controller_name_.c_str());
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "exception: '%s'", exrun.what());
    }
  }

  publishDiagnostics();

  return std::tuple(true, "failsafe activated");
}

//}

/* escalatingFailsafe() //{ */

std::tuple<bool, std::string> ControlManager::escalatingFailsafe(void) {

  std::stringstream ss;

  if ((clock_->now() - escalating_failsafe_time_).seconds() < _escalating_failsafe_timeout_) {

    ss << "too soon for escalating failsafe";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 100, "" << ss.str());

    return std::tuple(false, ss.str());
  }

  if (!output_enabled_) {

    ss << "not escalating failsafe, output is disabled";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 100, "" << ss.str());

    return std::tuple(false, ss.str());
  }

  RCLCPP_WARN(node_->get_logger(), "escalating failsafe triggered");

  auto active_tracker_idx    = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);

  std::string active_tracker_name    = _tracker_names_.at(active_tracker_idx);
  std::string active_controller_name = _controller_names_.at(active_controller_idx);

  EscalatingFailsafeStates_t next_state = getNextEscFailsafeState();

  escalating_failsafe_time_ = clock_->now();

  switch (next_state) {

  case ESC_NONE_STATE: {

    ss << "escalating failsafe has run to impossible situation";
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 100, "" << ss.str());

    return std::tuple(false, "escalating failsafe has run to impossible situation");

    break;
  }

  case ESC_EHOVER_STATE: {

    ss << "escalating failsafe escalates to ehover";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 100, "" << ss.str());

    auto [success, message] = ehover();

    if (success) {
      state_escalating_failsafe_ = ESC_EHOVER_STATE;
    }

    return {success, message};

    break;
  }

  case ESC_ELAND_STATE: {

    ss << "escalating failsafe escalates to eland";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 100, "" << ss.str());

    auto [success, message] = eland();

    if (success) {
      state_escalating_failsafe_ = ESC_ELAND_STATE;
    }

    return {success, message};

    break;
  }

  case ESC_FAILSAFE_STATE: {

    escalating_failsafe_time_ = clock_->now();

    ss << "escalating failsafe escalates to failsafe";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 100, "" << ss.str());

    auto [success, message] = failsafe();

    if (success) {
      state_escalating_failsafe_ = ESC_FINISHED_STATE;
    }

    return {success, message};

    break;
  }

  case ESC_FINISHED_STATE: {

    escalating_failsafe_time_ = clock_->now();

    ss << "escalating failsafe has nothing more to do";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 100, "" << ss.str());

    return std::tuple(false, "escalating failsafe has nothing more to do");

    break;
  }

  default: {

    break;
  }
  }

  RCLCPP_ERROR(node_->get_logger(), "escalatingFailsafe() reached the final return, this should not happen!");

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

  RCLCPP_ERROR(node_->get_logger(), "getNextEscFailsafeState() reached the final return, this should not happen!");

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

    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto response = tracker_list_.at(active_tracker_idx_)->startTrajectoryTracking(request);

    if (response != nullptr) {

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

    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto response = tracker_list_.at(active_tracker_idx_)->stopTrajectoryTracking(request);

    if (response != nullptr) {

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

    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto response = tracker_list_.at(active_tracker_idx_)->resumeTrajectoryTracking(request);

    if (response != nullptr) {

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

    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto response = tracker_list_.at(active_tracker_idx_)->gotoTrajectoryStart(request);

    if (response != nullptr) {

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
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    return std::tuple(false, ss.str());
  }

  if (!input && !isOffboard()) {

    ss << "can not disarm, not in OFFBOARD mode";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    return std::tuple(false, ss.str());
  }

  if (!input && _rc_emergency_handoff_) {

    toggleOutput(false);

    return std::tuple(true, "RC emergency handoff is ON, disabling output");
  }

  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();

  request->data = input ? 1 : 0; // arm or disarm?

  RCLCPP_INFO(node_->get_logger(), "calling for %s", input ? "arming" : "disarming");

  auto response = sch_arming_.callSync(request);

  if (response) {

    if (response.value()->success) {

      ss << "service call for " << (input ? "arming" : "disarming") << " was successful";
      RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

      if (!input) {

        toggleOutput(false);

        RCLCPP_DEBUG(node_->get_logger(), "stopping failsafe timer");
        timer_failsafe_->stop();
        RCLCPP_DEBUG(node_->get_logger(), "failsafe timer stopped");

        RCLCPP_DEBUG(node_->get_logger(), "stopping the eland timer");
        timer_eland_->stop();
        RCLCPP_DEBUG(node_->get_logger(), "eland timer stopped");
      }

    } else {
      ss << "service call for " << (input ? "arming" : "disarming") << " failed";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    }

  } else {
    ss << "calling for " << (input ? "arming" : "disarming") << " resulted in failure: '" << response.value()->message << "'";
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
  }

  return std::tuple(response.value()->success, ss.str());
}

//}

/* odometryCallbacksSrv() //{ */

void ControlManager::odometryCallbacksSrv(const bool input) {

  RCLCPP_INFO(node_->get_logger(), "switching odometry callbacks to %s", input ? "ON" : "OFF");

  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();

  request->data = input;

  auto response = sch_set_odometry_callbacks_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_WARN(node_->get_logger(), "service call for toggle odometry callbacks returned: '%s'", response.value()->message.c_str());
    }

  } else {
    RCLCPP_ERROR(node_->get_logger(), "service call for toggle odometry callbacks failed!");
  }
}

//}

/* elandSrv() //{ */

bool ControlManager::elandSrv(void) {

  RCLCPP_INFO(node_->get_logger(), "calling for eland");

  std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto response = sch_eland_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_WARN(node_->get_logger(), "service call for eland returned: '%s'", response.value()->message.c_str());
    }

    return response.value()->success;

  } else {

    RCLCPP_ERROR(node_->get_logger(), "service call for eland failed!");

    return false;
  }
}

//}

/* parachuteSrv() //{ */

bool ControlManager::parachuteSrv(void) {

  RCLCPP_INFO(node_->get_logger(), "calling for parachute deployment");

  std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto response = sch_parachute_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_WARN(node_->get_logger(), "service call for parachute deployment returned: '%s'", response.value()->message.c_str());
    }

    return response.value()->success;

  } else {

    RCLCPP_ERROR(node_->get_logger(), "service call for parachute deployment failed!");

    return false;
  }
}

//}

/* ungripSrv() //{ */

void ControlManager::ungripSrv(void) {

  std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto response = sch_ungrip_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for ungripping payload returned: '%s'", response.value()->message.c_str());
    }

  } else {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for ungripping payload failed!");
  }
}

//}

// | ------------------------ routines ------------------------ |

/* toggleOutput() //{ */

void ControlManager::toggleOutput(const bool &input) {

  if (input == output_enabled_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 100, "output is already %s", input ? "ON" : "OFF");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "switching output %s", input ? "ON" : "OFF");

  output_enabled_ = input;

  // if switching output off, switch to NullTracker
  if (!output_enabled_) {

    RCLCPP_INFO(node_->get_logger(), "switching to 'NullTracker' after switching output off");

    switchTracker(_null_tracker_name_);

    RCLCPP_INFO_STREAM(node_->get_logger(), "switching to the controller '" << _eland_controller_name_ << "' after switching output off");

    switchController(_eland_controller_name_);

    // | --------- deactivate all trackers and controllers -------- |

    for (int i = 0; i < int(tracker_list_.size()); i++) {

      std::map<std::string, TrackerParams>::iterator it;
      it = trackers_.find(_tracker_names_.at(i));

      try {
        RCLCPP_INFO(node_->get_logger(), "deactivating the tracker '%s'", it->second.address.c_str());
        tracker_list_.at(i)->deactivate();
      }
      catch (std::runtime_error &ex) {
        RCLCPP_ERROR(node_->get_logger(), "exception caught during tracker deactivation: '%s'", ex.what());
      }
    }

    for (int i = 0; i < int(controller_list_.size()); i++) {

      std::map<std::string, ControllerParams>::iterator it;
      it = controllers_.find(_controller_names_.at(i));

      try {
        RCLCPP_INFO(node_->get_logger(), "deactivating the controller '%s'", it->second.address.c_str());
        controller_list_.at(i)->deactivate();
      }
      catch (std::runtime_error &ex) {
        RCLCPP_ERROR(node_->get_logger(), "exception caught during controller deactivation: '%s'", ex.what());
      }
    }

    timer_failsafe_->stop();
    timer_eland_->stop();
    timer_pirouette_->stop();

    offboard_mode_was_true_ = false;
  }
}

//}

/* switchTracker() //{ */

std::tuple<bool, std::string> ControlManager::switchTracker(const std::string &tracker_name) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("switchTracker");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::switchTracker", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto last_tracker_cmd   = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);
  auto active_tracker_idx = mrs_lib::get_mutexed(mutex_tracker_list_, active_tracker_idx_);

  std::stringstream ss;

  if (!got_uav_state_) {

    ss << "can not switch tracker, missing odometry!";
    RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());
    return std::tuple(false, ss.str());
  }

  if (_state_input_ == INPUT_UAV_STATE && _odometry_innovation_check_enabled_ && !sh_odometry_innovation_.hasMsg()) {

    ss << "can not switch tracker, missing odometry innovation!";
    RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());
    return std::tuple(false, ss.str());
  }

  auto new_tracker_idx = idxInVector(tracker_name, _tracker_names_);

  // check if the tracker exists
  if (!new_tracker_idx) {
    ss << "the tracker '" << tracker_name << "' does not exist!";
    RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());
    return std::tuple(false, ss.str());
  }

  // check if the tracker is already active
  if (new_tracker_idx.value() == active_tracker_idx) {
    ss << "not switching, the tracker '" << tracker_name << "' is already active!";
    RCLCPP_INFO_STREAM(node_->get_logger(), "" << ss.str());
    return std::tuple(true, ss.str());
  }

  {
    std::scoped_lock lock(mutex_tracker_list_);

    try {

      RCLCPP_INFO(node_->get_logger(), "activating the tracker '%s'", _tracker_names_.at(new_tracker_idx.value()).c_str());

      auto [success, message] = tracker_list_.at(new_tracker_idx.value())->activate(last_tracker_cmd);

      if (!success) {

        ss << "the tracker '" << tracker_name << "' could not be activated: '" << message << "'";
        RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());
        return std::tuple(false, ss.str());

      } else {

        ss << "the tracker '" << tracker_name << "' was activated";
        RCLCPP_INFO_STREAM(node_->get_logger(), "" << ss.str());

        {
          std::scoped_lock lock(mutex_controller_tracker_switch_time_);

          // update the time (used in failsafe)
          controller_tracker_switch_time_ = clock_->now();
        }

        // super important, switch the active tracker idx
        try {

          RCLCPP_INFO(node_->get_logger(), "deactivating '%s'", _tracker_names_.at(active_tracker_idx_).c_str());
          tracker_list_.at(active_tracker_idx_)->deactivate();

          // if switching from null tracker, re-activate the already active the controller
          if (active_tracker_idx_ == _null_tracker_idx_) {

            RCLCPP_INFO(node_->get_logger(), "reactivating '%s' due to switching from 'NullTracker'", _controller_names_.at(active_controller_idx_).c_str());
            {
              std::scoped_lock lock(mutex_controller_list_);

              initializeControlOutput();

              auto last_control_output = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);

              controller_list_.at(active_controller_idx_)->activate(last_control_output);

              {
                std::scoped_lock lock(mutex_controller_tracker_switch_time_);

                // update the time (used in failsafe)
                controller_tracker_switch_time_ = clock_->now();
              }
            }

            // if switching to null tracker, deactivate the active controller
          } else if (new_tracker_idx == _null_tracker_idx_) {

            RCLCPP_INFO(node_->get_logger(), "deactivating '%s' due to switching to 'NullTracker'", _controller_names_.at(active_controller_idx_).c_str());
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
        catch (std::runtime_error &exrun) {
          RCLCPP_ERROR(node_->get_logger(), "could not deactivate the tracker '%s'", _tracker_names_.at(active_tracker_idx_).c_str());
        }
      }
    }
    catch (std::runtime_error &exrun) {
      RCLCPP_ERROR(node_->get_logger(), "error during activation of the tracker '%s'", tracker_name.c_str());
      RCLCPP_ERROR(node_->get_logger(), "exception: '%s'", exrun.what());
    }
  }

  publishDiagnostics();

  return std::tuple(true, ss.str());
}

//}

/* switchController() //{ */

std::tuple<bool, std::string> ControlManager::switchController(const std::string &controller_name) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("switchController");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::switchController", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto last_control_output   = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);
  auto active_controller_idx = mrs_lib::get_mutexed(mutex_controller_list_, active_controller_idx_);

  std::stringstream ss;

  if (!got_uav_state_) {

    ss << "can not switch controller, missing odometry!";
    RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());
    return std::tuple(false, ss.str());
  }

  if (_state_input_ == INPUT_UAV_STATE && _odometry_innovation_check_enabled_ && !sh_odometry_innovation_.hasMsg()) {

    ss << "can not switch controller, missing odometry innovation!";
    RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());
    return std::tuple(false, ss.str());
  }

  auto new_controller_idx = idxInVector(controller_name, _controller_names_);

  // check if the controller exists
  if (!new_controller_idx) {
    ss << "the controller '" << controller_name << "' does not exist!";
    RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());
    return std::tuple(false, ss.str());
  }

  // check if the controller is not active
  if (new_controller_idx.value() == active_controller_idx) {

    ss << "not switching, the controller '" << controller_name << "' is already active!";
    RCLCPP_INFO_STREAM(node_->get_logger(), "" << ss.str());
    return std::tuple(true, ss.str());
  }

  {
    std::scoped_lock lock(mutex_controller_list_);

    try {

      RCLCPP_INFO(node_->get_logger(), "activating the controller '%s'", _controller_names_.at(new_controller_idx.value()).c_str());
      if (!controller_list_.at(new_controller_idx.value())->activate(last_control_output)) {

        ss << "the controller '" << controller_name << "' was not activated";
        RCLCPP_ERROR_STREAM(node_->get_logger(), "" << ss.str());
        return std::tuple(false, ss.str());

      } else {

        ss << "the controller '" << controller_name << "' was activated";
        RCLCPP_INFO_STREAM(node_->get_logger(), "" << ss.str());

        RCLCPP_INFO(node_->get_logger(), "triggering hover after switching to '%s', re-activating '%s'",
                    _controller_names_.at(new_controller_idx.value()).c_str(), _tracker_names_.at(active_tracker_idx_).c_str());

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
          controller_tracker_switch_time_ = clock_->now();
        }

        // super important, switch the active controller idx
        try {

          controller_list_.at(active_controller_idx_)->deactivate();
          active_controller_idx_ = new_controller_idx.value();
        }
        catch (std::runtime_error &exrun) {
          RCLCPP_ERROR(node_->get_logger(), "could not deactivate controller '%s'", _controller_names_.at(active_controller_idx_).c_str());
        }
      }
    }
    catch (std::runtime_error &exrun) {
      RCLCPP_ERROR(node_->get_logger(), "error during activation of controller '%s'", controller_name.c_str());
      RCLCPP_ERROR(node_->get_logger(), "exception: '%s'", exrun.what());
    }
  }

  mrs_msgs::srv::DynamicsConstraintsSrv::Request sanitized_constraints;

  {
    std::scoped_lock lock(mutex_constraints_);

    sanitized_constraints_ = current_constraints_;
    sanitized_constraints  = sanitized_constraints_;
  }

  setConstraintsToControllers(sanitized_constraints);

  publishDiagnostics();

  return std::tuple(true, ss.str());
}

//}

/* updateTrackers() //{ */

void ControlManager::updateTrackers(void) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("updateTrackers");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::updateTrackers", scope_timer_logger_, scope_timer_enabled_);

  // copy member variables
  auto uav_state           = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto last_control_output = mrs_lib::get_mutexed(mutex_last_control_output_, last_control_output_);

  // --------------------------------------------------------------
  // |                     Update the trackers                    |
  // --------------------------------------------------------------

  std::optional<mrs_msgs::msg::TrackerCommand> tracker_command;

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
        catch (std::runtime_error &exrun) {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "caught an exception while updating the active tracker (%s)",
                                _tracker_names_.at(active_tracker_idx).c_str());
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "the exception: '%s'", exrun.what());
          tracker_command = {};
        }

      } else {

        try {
          // nonactive tracker => just update without retrieving the command
          tracker_list_.at(i)->update(uav_state, last_control_output);
        }
        catch (std::runtime_error &exrun) {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "caught an exception while updating the tracker '%s'", _tracker_names_.at(i).c_str());
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "the exception: '%s'", exrun.what());
        }
      }
    }

    if (active_tracker_idx == _null_tracker_idx_) {
      return;
    }
  }

  if (validateTrackerCommand(node_, tracker_command, "tracker_command")) {

    std::scoped_lock lock(mutex_last_tracker_cmd_);

    last_tracker_cmd_ = tracker_command;

    // | --------- fill in the potentially missing header --------- |

    if (last_tracker_cmd_->header.frame_id == "") {
      last_tracker_cmd_->header.frame_id = uav_state.header.frame_id;
    }

    if (rclcpp::Time(last_tracker_cmd_->header.stamp).seconds() == 0) {
      last_tracker_cmd_->header.stamp = clock_->now();
    }

  } else {

    if (active_tracker_idx == _ehover_tracker_idx_) {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "the emergency tracker '%s' returned empty or invalid command!",
                            _tracker_names_.at(active_tracker_idx).c_str());
      failsafe();

    } else {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "the tracker '%s' returned empty or invalid command!",
                            _tracker_names_.at(active_tracker_idx).c_str());

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

void ControlManager::updateControllers(const mrs_msgs::msg::UavState &uav_state) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("updateControllers");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::updateControllers", scope_timer_logger_, scope_timer_enabled_);

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
        catch (std::runtime_error &exrun) {

          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "an exception while updating the active controller (%s)",
                                _controller_names_.at(active_controller_idx).c_str());
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "the exception: '%s'", exrun.what());
        }

      } else {

        try {
          // nonactive controller => just update without retrieving the command
          controller_list_.at(i)->updateInactive(uav_state, last_tracker_cmd);
        }
        catch (std::runtime_error &exrun) {

          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "exception while updating the controller '%s'", _controller_names_.at(i).c_str());
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "exception: '%s'", exrun.what());
        }
      }
    }
  }

  // normally, the active controller returns a valid command
  if (validateControlOutput(node_, control_output, _hw_api_inputs_, "control_output")) {

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

        RCLCPP_ERROR(node_->get_logger(), "disabling control, the active controller returned an empty command when failsafe was active");

        toggleOutput(false);

      } else if (eland_triggered_) {

        RCLCPP_ERROR(node_->get_logger(), "triggering failsafe, the active controller returned an empty command when eland was active");

        failsafe();

      } else {

        RCLCPP_ERROR(node_->get_logger(), "triggering eland, the active controller returned an empty command");

        eland();
      }
    }
  }
}

//}

/* publish() //{ */

void ControlManager::publish(void) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("publish");
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer(node_, "ControlManager::publish", scope_timer_logger_, scope_timer_enabled_);

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

  mrs_msgs::msg::HwApiAttitudeCmd attitude_target;
  attitude_target.stamp = clock_->now();

  if (!output_enabled_) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "output is disabled");

  } else if (active_tracker_idx == _null_tracker_idx_) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 5000, "'NullTracker' is active, not controlling");

    Controller::HwApiOutputVariant output =
        initializeDefaultOutput(node_, _hw_api_inputs_, uav_state, _min_throttle_null_tracker_, common_handlers_->throttle_model.n_motors);

    {
      std::scoped_lock lock(mutex_last_control_output_);

      last_control_output_.control_output = output;
    }

    control_output_publisher_.publish(output);

  } else if (active_tracker_idx != _null_tracker_idx_ && !last_control_output.control_output) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the controller '%s' returned nil command, not publishing anything",
                         _controller_names_.at(active_controller_idx).c_str());

    Controller::HwApiOutputVariant output =
        initializeDefaultOutput(node_, _hw_api_inputs_, uav_state, _min_throttle_null_tracker_, common_handlers_->throttle_model.n_motors);

    control_output_publisher_.publish(output);

  } else if (last_control_output.control_output) {

    if (validateHwApiAttitudeCmd(node_, attitude_target, "publish(): attitude_target")) {
      control_output_publisher_.publish(last_control_output.control_output.value());
    } else {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "the attitude cmd is not valid just before publishing!");
      return;
    }

  } else {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "not publishing a control command");
  }

  // | ----------- publish the controller diagnostics ----------- |

  ph_controller_diagnostics_.publish(last_control_output.diagnostics);

  // | --------- publish the applied throttle and thrust -------- |

  auto throttle = extractThrottle(last_control_output);

  if (throttle) {

    {
      std_msgs::msg::Float64 msg;
      msg.data = throttle.value();
      ph_throttle_.publish(msg);
    }

    double thrust = mrs_lib::quadratic_throttle_model::throttleToForce(common_handlers_->throttle_model, throttle.value());

    {
      std_msgs::msg::Float64 msg;
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

    mrs_msgs::msg::EstimatorInput msg;

    msg.header.frame_id = _uav_name_ + "/fcu";
    msg.header.stamp    = clock_->now();

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

mrs_msgs::msg::ReferenceStamped ControlManager::velocityReferenceToReference(const mrs_msgs::msg::VelocityReferenceStamped &vel_reference) {

  auto last_tracker_cmd    = mrs_lib::get_mutexed(mutex_last_tracker_cmd_, last_tracker_cmd_);
  auto uav_state           = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);
  auto current_constraints = mrs_lib::get_mutexed(mutex_constraints_, current_constraints_);

  mrs_msgs::msg::ReferenceStamped reference_out;

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

    if (vel_reference.reference.velocity.z >= 0) {
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

void ControlManager::publishControlReferenceOdom(const std::optional<mrs_msgs::msg::TrackerCommand> &tracker_command,
                                                 const Controller::ControlOutput                    &control_output) {

  if (!tracker_command || !control_output.control_output) {
    return;
  }

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  nav_msgs::msg::Odometry msg;

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

    geometry_msgs::msg::Vector3Stamped velocity;
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
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "could not transform the reference speed from '%s' to '%s'", velocity.header.frame_id.c_str(),
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
  if (std::holds_alternative<mrs_msgs::msg::HwApiAttitudeRateCmd>(control_output.control_output.value())) {

    auto attitude_cmd = std::get<mrs_msgs::msg::HwApiAttitudeRateCmd>(control_output.control_output.value());

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

} // namespace control_manager

} // namespace mrs_uav_managers

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_managers::control_manager::ControlManager)
