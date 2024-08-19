/*//{ includes */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <pluginlib/class_loader.h>
#include <nodelet/loader.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <nav_msgs/Odometry.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <mrs_msgs/UavState.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/EstimationDiagnostics.h>
#include <mrs_msgs/HwApiCapabilities.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/SafetyAreaManagerDiagnostics.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/gps_conversions.h>
#include <mrs_lib/scope_timer.h>
/* #include <mrs_lib/profiler.h> */

#include <mrs_errorgraph/error_publisher.h>

#include <mrs_uav_managers/state_estimator.h>
#include <mrs_uav_managers/agl_estimator.h>
#include <mrs_uav_managers/estimation_manager/support.h>
#include <mrs_uav_managers/estimation_manager/common_handlers.h>
#include <mrs_uav_managers/estimation_manager/private_handlers.h>
/*//}*/

namespace mrs_uav_managers
{

namespace estimation_manager
{

// --------------------------------------------------------------
// |                           classes                          |
// --------------------------------------------------------------

/*//{ class StateMachine */
class StateMachine {

  /*//{ states */
public:
  typedef enum
  {

    UNINITIALIZED_STATE,
    INITIALIZED_STATE,
    READY_FOR_FLIGHT_STATE,
    TAKING_OFF_STATE,
    FLYING_STATE,
    HOVER_STATE,
    LANDING_STATE,
    LANDED_STATE,
    ESTIMATOR_SWITCHING_STATE,
    DUMMY_STATE,
    EMERGENCY_STATE,
    FAILSAFE_STATE,
    ERROR_STATE

  } SMState_t;

  /*//}*/

public:
  StateMachine(const std::string& nodelet_name) : nodelet_name_(nodelet_name) {
  }

  bool isInState(const SMState_t& state) const {
    return mrs_lib::get_mutexed(mtx_state_, current_state_) == state;
  }

  bool isInitialized() const {
    return mrs_lib::get_mutexed(mtx_state_, current_state_) != UNINITIALIZED_STATE;
  }

  bool isInPublishableState() const {
    const SMState_t current_state = mrs_lib::get_mutexed(mtx_state_, current_state_);
    return current_state == READY_FOR_FLIGHT_STATE || current_state == TAKING_OFF_STATE || current_state == HOVER_STATE || current_state == FLYING_STATE ||
           current_state == LANDING_STATE || current_state == DUMMY_STATE || current_state == FAILSAFE_STATE;
  }

  bool isInSwitchableState() const {
    const SMState_t current_state = mrs_lib::get_mutexed(mtx_state_, current_state_);
    return current_state == READY_FOR_FLIGHT_STATE || current_state == TAKING_OFF_STATE || current_state == HOVER_STATE || current_state == FLYING_STATE ||
           current_state == LANDING_STATE;
  }

  bool isInTheAir() const {
    const SMState_t current_state = mrs_lib::get_mutexed(mtx_state_, current_state_);
    return current_state == TAKING_OFF_STATE || current_state == HOVER_STATE || current_state == FLYING_STATE || current_state == LANDING_STATE;
  }

  SMState_t getCurrentState() const {
    return mrs_lib::get_mutexed(mtx_state_, current_state_);
  }

  std::string getCurrentStateString() const {
    return mrs_lib::get_mutexed(mtx_state_, sm_state_names_[current_state_]);
  }

  std::string getStateAsString(const SMState_t& state) const {
    return sm_state_names_[state];
  }

  /*//{ changeState() */
  bool changeState(const SMState_t& target_state) {

    if (target_state == current_state_) {

      ROS_WARN("[%s]: requested change to same state %s -> %s", getPrintName().c_str(), getStateAsString(current_state_).c_str(),
               getStateAsString(target_state).c_str());
      return true;
    }

    switch (target_state) {

      case UNINITIALIZED_STATE: {
        ROS_ERROR_THROTTLE(1.0, "[%s]: transition to %s is not possible from any state", getPrintName().c_str(), getStateAsString(UNINITIALIZED_STATE).c_str());
        return false;
        break;
      }

      case INITIALIZED_STATE: {
        if (current_state_ != UNINITIALIZED_STATE) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: transition to %s is possible only from %s", getPrintName().c_str(), getStateAsString(INITIALIZED_STATE).c_str(),
                             getStateAsString(UNINITIALIZED_STATE).c_str());
          return false;
        }
        break;
      }

      case READY_FOR_FLIGHT_STATE: {
        if (current_state_ != INITIALIZED_STATE && current_state_ != LANDED_STATE && current_state_ != ESTIMATOR_SWITCHING_STATE) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: transition to %s is possible only from %s, %s or %s", getPrintName().c_str(),
                             getStateAsString(READY_FOR_FLIGHT_STATE).c_str(), getStateAsString(INITIALIZED_STATE).c_str(),
                             getStateAsString(LANDED_STATE).c_str(), getStateAsString(ESTIMATOR_SWITCHING_STATE).c_str());
          return false;
        }
        break;
      }

      case TAKING_OFF_STATE: {
        if (current_state_ != READY_FOR_FLIGHT_STATE && current_state_ != ESTIMATOR_SWITCHING_STATE) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: transition to %s is possible only from %s or %s", getPrintName().c_str(), getStateAsString(TAKING_OFF_STATE).c_str(),
                             getStateAsString(READY_FOR_FLIGHT_STATE).c_str(), getStateAsString(ESTIMATOR_SWITCHING_STATE).c_str());
          return false;
        }
        break;
      }

      case FLYING_STATE: {
        if (current_state_ != TAKING_OFF_STATE && current_state_ != READY_FOR_FLIGHT_STATE && current_state_ != HOVER_STATE && current_state_ != ESTIMATOR_SWITCHING_STATE) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: transition to %s is possible only from %s or %s or %s", getPrintName().c_str(), getStateAsString(FLYING_STATE).c_str(),
                             getStateAsString(TAKING_OFF_STATE).c_str(), getStateAsString(HOVER_STATE).c_str(),
                             getStateAsString(ESTIMATOR_SWITCHING_STATE).c_str());
          return false;
        }
        break;
      }

      case HOVER_STATE: {
        if (current_state_ != FLYING_STATE && current_state_ != ESTIMATOR_SWITCHING_STATE) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: transition to %s is possible only from %s or %s", getPrintName().c_str(), getStateAsString(HOVER_STATE).c_str(),
                             getStateAsString(FLYING_STATE).c_str(), getStateAsString(ESTIMATOR_SWITCHING_STATE).c_str());
          return false;
        }
        break;
      }

      case ESTIMATOR_SWITCHING_STATE: {
        if (current_state_ != READY_FOR_FLIGHT_STATE && current_state_ != TAKING_OFF_STATE  && current_state_ != HOVER_STATE && current_state_ != FLYING_STATE && current_state_ != LANDING_STATE) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: transition to %s is possible only from %s, %s, %s, %s or %s", getPrintName().c_str(),
                             getStateAsString(ESTIMATOR_SWITCHING_STATE).c_str(), getStateAsString(READY_FOR_FLIGHT_STATE).c_str(), getStateAsString(TAKING_OFF_STATE).c_str(), getStateAsString(FLYING_STATE).c_str(),
                             getStateAsString(HOVER_STATE).c_str(), getStateAsString(FLYING_STATE).c_str());
          return false;
        }
        pre_switch_state_ = current_state_;
        break;
      }

      case LANDING_STATE: {
        if (current_state_ != FLYING_STATE && current_state_ != HOVER_STATE && current_state_ != ESTIMATOR_SWITCHING_STATE) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: transition to %s is possible only from %s, %s or %s", getPrintName().c_str(), getStateAsString(LANDING_STATE).c_str(),
                             getStateAsString(FLYING_STATE).c_str(), getStateAsString(HOVER_STATE).c_str(), getStateAsString(ESTIMATOR_SWITCHING_STATE).c_str());
          return false;
        }
        break;
      }

      case LANDED_STATE: {
        if (current_state_ != LANDING_STATE) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: transition to %s is possible only from %s", getPrintName().c_str(), getStateAsString(LANDED_STATE).c_str(),
                             getStateAsString(LANDING_STATE).c_str());
          return false;
        }
        break;
      }

      case DUMMY_STATE: {
        if (current_state_ != INITIALIZED_STATE) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: transition to %s is possible only from %s", getPrintName().c_str(), getStateAsString(DUMMY_STATE).c_str(),
                             getStateAsString(INITIALIZED_STATE).c_str());
          return false;
        }
        break;
      }
      case EMERGENCY_STATE: {
        ROS_WARN("[%s]: transition to %s", getPrintName().c_str(), getStateAsString(EMERGENCY_STATE).c_str());
        break;
      }

      case ERROR_STATE: {
        ROS_WARN("[%s]: transition to %s", getPrintName().c_str(), getStateAsString(ERROR_STATE).c_str());
        break;
      }

      case FAILSAFE_STATE: {
        ROS_WARN("[%s]: transition to %s", getPrintName().c_str(), getStateAsString(FAILSAFE_STATE).c_str());
        break;
      }

      default: {
        ROS_ERROR_THROTTLE(1.0, "[%s]: rejected transition to unknown state id %d", getPrintName().c_str(), target_state);
        return false;
        break;
      }
    }

    std::scoped_lock lock(mtx_state_);
    {
      previous_state_ = current_state_;
      current_state_  = target_state;
    }

    ROS_INFO("[%s]: successfully changed states %s -> %s", getPrintName().c_str(), getStateAsString(previous_state_).c_str(),
             getStateAsString(current_state_).c_str());

    return true;
  }
  /*//}*/

  /*//{ changeToPreSwitchState() */
  void changeToPreSwitchState() {
    changeState(pre_switch_state_);
  }
  /*//}*/

private:
  const std::string name_ = "StateMachine";
  const std::string nodelet_name_;

  SMState_t current_state_    = UNINITIALIZED_STATE;
  SMState_t previous_state_   = UNINITIALIZED_STATE;
  SMState_t pre_switch_state_ = UNINITIALIZED_STATE;

  mutable std::mutex mtx_state_;

  std::string getName() const {
    return name_;
  }

  std::string getPrintName() const {
    return nodelet_name_ + "/" + name_;
  }

  // clang-format off
  const std::vector<std::string> sm_state_names_ = {
  "UNINITIALIZED_STATE",
  "INITIALIZED_STATE",
  "READY_FOR_FLIGHT_STATE",
  "TAKING_OFF_STATE",
  "FLYING_STATE",
  "HOVER_STATE",
  "LANDING_STATE",
  "LANDED_STATE",
  "ESTIMATOR_SWITCHING_STATE",
  "DUMMY_STATE",
  "EMERGENCY_STATE",
  "FAILSAFE_STATE",
  "ERROR_STATE"
  };
  // clang-format on

};
/*//}*/

/*//{ class EstimationManager */
class EstimationManager : public nodelet::Nodelet {

private:
  const std::string nodelet_name_ = "EstimationManager";
  const std::string package_name_ = "mrs_uav_managers";

  ros::NodeHandle nh_;
  std::atomic<bool> is_initialized_ = false;

  std::string _custom_config_;
  std::string _platform_config_;
  std::string _world_config_;

  // | ----------------------- errorgraph ----------------------- |
  enum class error_type_t : uint16_t
  {
    placeholder
  };

  std::unique_ptr<mrs_errorgraph::ErrorPublisher> error_publisher_;

  std::shared_ptr<CommonHandlers_t> ch_;

  std::shared_ptr<StateMachine> sm_;

  std::string after_midair_activation_tracker_name_;
  std::string takeoff_tracker_name_;

  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>    sh_control_manager_diag_;

  mrs_lib::SubscribeHandler<mrs_msgs::SafetyAreaManagerDiagnostics> sh_safety_area_manager_diag;
  void callbackSafetyAreaManagerDiagnostics(const mrs_msgs::SafetyAreaManagerDiagnostics::ConstPtr msg); 

  mrs_lib::SubscribeHandler<mrs_msgs::EstimatorInput>               sh_control_input_;

  mrs_lib::PublisherHandler<mrs_msgs::EstimationDiagnostics> ph_diagnostics_;
  mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>        ph_max_flight_z_;
  mrs_lib::PublisherHandler<mrs_msgs::UavState>              ph_uav_state_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry>              ph_odom_main_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry>              ph_odom_slow_;  // use topic throttler instead?
  mrs_lib::PublisherHandler<nav_msgs::Odometry>              ph_innovation_;

  mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped> ph_altitude_amsl_;
  mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped> ph_altitude_agl_;

  mrs_lib::PublisherHandler<geometry_msgs::QuaternionStamped> ph_orientation_;

  ros::Timer timer_publish_;
  void       timerPublish(const ros::TimerEvent& event);

  ros::Timer timer_publish_diagnostics_;
  void       timerPublishDiagnostics(const ros::TimerEvent& event);

  ros::Timer timer_check_health_;
  void       timerCheckHealth(const ros::TimerEvent& event);

  ros::Timer timer_initialization_;
  void       timerInitialization(const ros::TimerEvent& event);

  ros::ServiceServer srvs_change_estimator_;
  bool               callbackChangeEstimator(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);
  int                estimator_switch_count_ = 0;

  ros::ServiceServer srvs_reset_estimator_;
  bool               callbackResetEstimator(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);


  ros::ServiceServer srvs_toggle_callbacks_;
  bool               callbackToggleServiceCallbacks(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool               callbacks_enabled_             = false;
  bool               callbacks_disabled_by_service_ = false;

  bool                                             callFailsafeService();
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> srvch_failsafe_;
  bool                                             failsafe_call_succeeded_ = false;

  // TODO service clients
  /* mrs_lib::ServiceClientHandler<std_srvs::Trigger> srvc_hover_; */
  /* mrs_lib::ServiceClientHandler<mrs_msgs::ReferenceStampedSrv> srvc_reference_; */
  /* mrs_lib::ServiceClientHandler<std_srvs::Trigger> srvc_ehover_; */
  /* mrs_lib::ServiceClientHandler<std_srvs::SetBool> srvc_enable_callbacks_; */

  // | ------------- dynamic loading of estimators ------------- |
  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_managers::StateEstimator>> state_estimator_loader_;  // pluginlib loader of dynamically loaded estimators
  std::vector<std::string>                                                  estimator_names_;         // list of estimator names
  std::vector<boost::shared_ptr<mrs_uav_managers::StateEstimator>>          estimator_list_;          // list of estimators
  std::mutex                                                                mutex_estimator_list_;
  std::vector<std::string>                                                  switchable_estimator_names_;
  std::mutex                                                                mutex_switchable_estimator_names_;
  std::string                                                               initial_estimator_name_ = "UNDEFINED_INITIAL_ESTIMATOR";
  boost::shared_ptr<mrs_uav_managers::StateEstimator>                       initial_estimator_;
  boost::shared_ptr<mrs_uav_managers::StateEstimator>                       active_estimator_;
  std::mutex                                                                mutex_active_estimator_;

  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_managers::AglEstimator>> agl_estimator_loader_;  // pluginlib loader of dynamically loaded estimators
  std::string                                                             est_alt_agl_name_ = "UNDEFINED_AGL_ESTIMATOR";
  boost::shared_ptr<mrs_uav_managers::AglEstimator>                       est_alt_agl_;
  bool                                                                    is_using_agl_estimator_;

  double max_flight_z_;

  bool switchToHealthyEstimator();
  void switchToEstimator(const boost::shared_ptr<mrs_uav_managers::StateEstimator>& target_estimator);

  bool loadConfigFile(const std::string& file_path);

public:
  EstimationManager() {
  }

  void onInit();

  std::string getName() const;
};
/*//}*/

/*//{ onInit() */
void EstimationManager::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[%s]: initializing", getName().c_str());

  error_publisher_ = std::make_unique<mrs_errorgraph::ErrorPublisher>(nh_, "EstimationManager", "main");

  sm_ = std::make_shared<StateMachine>(nodelet_name_);

  ch_ = std::make_shared<CommonHandlers_t>();

  ch_->nodelet_name = nodelet_name_;
  ch_->package_name = package_name_;

  // finish initialization in a oneshot timer, so that we don't block loading of other nodelets by the nodelet manager
  timer_initialization_ = nh_.createTimer(ros::Rate(1.0), &EstimationManager::timerInitialization, this, true, true);
}
/*//}*/

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | --------------------- topic callbacks -------------------- |

/* //{ callbackSafetyAreaManagerDiagnostics() */

void EstimationManager::callbackSafetyAreaManagerDiagnostics(const mrs_msgs::SafetyAreaManagerDiagnostics::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  //Do we need the profiler?
  /* mrs_lib::Routine    profiler_routine = profiler_.createRoutine("callbackSafetyAreaManagerDiagnostics"); */
  /* mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("EstimationManager::callbackSafetyAreaManagerDiagnostics", scope_timer_logger_, scope_timer_enabled_); */

  mrs_msgs::SafetyAreaManagerDiagnosticsConstPtr safety_area = msg;

  /* ch_->world_origin.x = safety_area->safety_area.origin_x; */ 
  /* ch_->world_origin.y = safety_area->safety_area.origin_y; */ 

}

//}

// | --------------------- timer callbacks -------------------- |

/*//{ timerPublish() */
void EstimationManager::timerPublish([[maybe_unused]] const ros::TimerEvent& event) {

  if (!sm_->isInitialized()) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("EstimationManager::timerPublish", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  if (sm_->isInState(StateMachine::ESTIMATOR_SWITCHING_STATE)) {
    ROS_WARN("[%s]: Not publishing during estimator switching.", getName().c_str());
    return;
  }

  if (!sm_->isInPublishableState()) {
    ROS_WARN_THROTTLE(1.0, "[%s]: not publishing uav state in %s", getName().c_str(), sm_->getCurrentStateString().c_str());
    return;
  }

  mrs_msgs::UavState uav_state;
  auto               ret = active_estimator_->getUavState();
  if (ret) {
    uav_state = ret.value();
  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Active estimator did not provide uav_state.", getName().c_str());
    return;
  }

  if (!Support::noNans(uav_state.pose.orientation)) {
    ROS_ERROR("[%s]: nan in uav state orientation", getName().c_str());
    return;
  }


  if (active_estimator_->isMitigatingJump()) {
    estimator_switch_count_++;
  }
  uav_state.estimator_iteration = estimator_switch_count_;

  scope_timer.checkpoint("get uav state");
  // TODO state health checks

  ph_uav_state_.publish(uav_state);

  scope_timer.checkpoint("pub uav state");
  nav_msgs::Odometry odom_main = Support::uavStateToOdom(uav_state);

  scope_timer.checkpoint("uav state to odom");
  const std::vector<double> pose_covariance = active_estimator_->getPoseCovariance();
  for (size_t i = 0; i < pose_covariance.size(); i++) {
    odom_main.pose.covariance[i] = pose_covariance[i];
  }

  const std::vector<double> twist_covariance = active_estimator_->getTwistCovariance();
  for (size_t i = 0; i < twist_covariance.size(); i++) {
    odom_main.twist.covariance[i] = twist_covariance[i];
  }

  scope_timer.checkpoint("get covariance");
  ph_odom_main_.publish(odom_main);

  nav_msgs::Odometry innovation = active_estimator_->getInnovation();
  ph_innovation_.publish(innovation);


  mrs_msgs::Float64Stamped agl_height;

  if (is_using_agl_estimator_) {
    agl_height = est_alt_agl_->getUavAglHeight();
    ph_altitude_agl_.publish(agl_height);
  }
}
/*//}*/

/*//{ timerPublishDiagnostics() */
void EstimationManager::timerPublishDiagnostics([[maybe_unused]] const ros::TimerEvent& event) {

  if (!sm_->isInitialized()) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("EstimationManager::timerPublishDiagnostics", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  if (sm_->isInState(StateMachine::ESTIMATOR_SWITCHING_STATE)) {
    ROS_WARN("[%s]: Not publishing diagnostics during estimator switching.", getName().c_str());
    return;
  }

  if (!sm_->isInPublishableState()) {
    ROS_WARN_THROTTLE(1.0, "[%s]: not publishing uav state in %s", getName().c_str(), sm_->getCurrentStateString().c_str());
    return;
  }

  mrs_msgs::UavState uav_state;
  auto               ret = active_estimator_->getUavState();
  if (ret) {
    uav_state = ret.value();
  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Active estimator did not provide uav_state.", getName().c_str());
    return;
  }

  if (!Support::noNans(uav_state.pose.orientation)) {
    ROS_ERROR("[%s]: nan in uav state orientation", getName().c_str());
    return;
  }

  mrs_msgs::Float64Stamped agl_height;

  if (is_using_agl_estimator_) {
    agl_height = est_alt_agl_->getUavAglHeight();
    ph_altitude_agl_.publish(agl_height);
  }

  mrs_msgs::Float64Stamped max_flight_z_msg;
  max_flight_z_msg.header.stamp = ros::Time::now();
  if (active_estimator_ && active_estimator_->isInitialized()) {
    max_flight_z_msg.header.frame_id = active_estimator_->getFrameId();
    max_flight_z_msg.value           = active_estimator_->getMaxFlightZ();
  }
  max_flight_z_ = max_flight_z_msg.value;
  ph_max_flight_z_.publish(max_flight_z_msg);

  // publish diagnostics
  mrs_msgs::EstimationDiagnostics diagnostics;

  diagnostics.header.stamp   = uav_state.header.stamp;
  diagnostics.child_frame_id = uav_state.child_frame_id;

  diagnostics.pose         = uav_state.pose;
  diagnostics.velocity     = uav_state.velocity;
  diagnostics.acceleration = uav_state.acceleration;

  diagnostics.sm_state                 = sm_->getCurrentStateString();
  diagnostics.max_flight_z             = max_flight_z_msg.value;
  diagnostics.estimator_iteration      = estimator_switch_count_;
  diagnostics.estimation_rate          = ch_->desired_uav_state_rate;
  diagnostics.running_state_estimators = estimator_names_;

  {
    std::scoped_lock lock(mutex_switchable_estimator_names_);
    diagnostics.switchable_state_estimators = switchable_estimator_names_;
  }


  if (active_estimator_ && active_estimator_->isInitialized()) {
    diagnostics.header.frame_id         = active_estimator_->getFrameId();
    diagnostics.current_state_estimator = active_estimator_->getName();
  } else {
    diagnostics.header.frame_id         = "";
    diagnostics.current_state_estimator = "";
  }

  diagnostics.estimator_horizontal = uav_state.estimator_horizontal;
  diagnostics.estimator_vertical   = uav_state.estimator_vertical;
  diagnostics.estimator_heading    = uav_state.estimator_heading;

  if (is_using_agl_estimator_) {
    diagnostics.estimator_agl_height = est_alt_agl_name_;
    diagnostics.agl_height           = agl_height.value;
  } else {
    diagnostics.estimator_agl_height = "NOT_USED";
    diagnostics.agl_height           = std::nanf("");
  }

  diagnostics.platform_config = _platform_config_;
  diagnostics.custom_config   = _custom_config_;

  ph_diagnostics_.publish(diagnostics);

  ROS_INFO_THROTTLE(5.0, "[%s]: %s. pos: [%.2f, %.2f, %.2f] m. Estimator: %s. Max. z.: %.2f m. Estimator switches: %d.", getName().c_str(),
                    sm_->getCurrentStateString().c_str(), uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z,
                    active_estimator_->getName().c_str(), max_flight_z_, estimator_switch_count_);
}
/*//}*/

/*//{ timerCheckHealth() */
void EstimationManager::timerCheckHealth([[maybe_unused]] const ros::TimerEvent& event) {

  if (!sm_->isInitialized()) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("EstimationManager::timerCheckHealth", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  /*//{ start ready estimators, check switchable estimators */
  std::vector<std::string> switchable_estimator_names;
  for (auto estimator : estimator_list_) {

    if (estimator->isReady()) {
      try {
        ROS_INFO_THROTTLE(1.0, "[%s]: starting the estimator '%s'", getName().c_str(), estimator->getName().c_str());
        estimator->start();
      }
      catch (std::runtime_error& ex) {
        ROS_ERROR("[%s]: exception caught during estimator starting: '%s'", getName().c_str(), ex.what());
      error_publisher_->addOneshotError("Estimator start error: runtime exception.");
      error_publisher_->flushAndShutdown();
      return;
      }
    }

    if (estimator->isRunning() && estimator->getName() != "dummy" && estimator->getName() != "ground_truth") {
      switchable_estimator_names.push_back(estimator->getName());
    }
  }

  {
    std::scoped_lock lock(mutex_switchable_estimator_names_);
    switchable_estimator_names_ = switchable_estimator_names;
  }

  if (is_using_agl_estimator_ && est_alt_agl_->isReady()) {
    est_alt_agl_->start();
  }

  /*//}*/

  if (!callbacks_disabled_by_service_ &&
      (sm_->isInState(StateMachine::FLYING_STATE) || sm_->isInState(StateMachine::HOVER_STATE) || sm_->isInState(StateMachine::READY_FOR_FLIGHT_STATE))) {
    callbacks_enabled_ = true;
  } else {
    callbacks_enabled_ = false;
  }

  // TODO fuj if, zmenit na switch
  // activate initial estimator
  if (sm_->isInState(StateMachine::INITIALIZED_STATE) && initial_estimator_->isRunning()) {
    std::scoped_lock lock(mutex_active_estimator_);
    ROS_INFO_THROTTLE(1.0, "[%s]: activating the initial estimator %s", getName().c_str(), initial_estimator_->getName().c_str());
    active_estimator_ = initial_estimator_;
    if (active_estimator_->getName() == "dummy") {
      sm_->changeState(StateMachine::DUMMY_STATE);
    } else {
      if (!is_using_agl_estimator_ || est_alt_agl_->isRunning()) {
        sm_->changeState(StateMachine::READY_FOR_FLIGHT_STATE);
      } else {
        ROS_INFO_THROTTLE(1.0, "[%s]: %s agl estimator: %s to be running", getName().c_str(), Support::waiting_for_string.c_str(),
                          est_alt_agl_->getName().c_str());
      }
    }
  }

  // active estimator is in faulty state, we need to switch to healthy estimator
  if (sm_->isInSwitchableState() && active_estimator_->isError()) {
    sm_->changeState(StateMachine::ESTIMATOR_SWITCHING_STATE);
  }

  if (sm_->isInState(StateMachine::ESTIMATOR_SWITCHING_STATE)) {
    if (switchToHealthyEstimator()) {
      sm_->changeToPreSwitchState();
    } else {  // cannot switch to healthy estimator - failsafe necessary
      ROS_ERROR_THROTTLE(1.0, "[%s]: Cannot switch to any healthy estimator. Triggering failsafe.", getName().c_str());
      sm_->changeState(StateMachine::FAILSAFE_STATE);
    }
  }

  if (sm_->isInState(StateMachine::FAILSAFE_STATE)) {
    if (!failsafe_call_succeeded_ && callFailsafeService()) {
      ROS_INFO_THROTTLE(1.0, "[%s]: failsafe called successfully", getName().c_str());
      failsafe_call_succeeded_ = true;
    }
    ROS_ERROR_THROTTLE(1.0, "[%s]: we are in failsafe state", getName().c_str());
  }

  // standard takeoff
  if (sm_->isInState(StateMachine::READY_FOR_FLIGHT_STATE)) {
    if (sh_control_manager_diag_.hasMsg() && sh_control_manager_diag_.getMsg()->active_tracker == takeoff_tracker_name_) {
      sm_->changeState(StateMachine::TAKING_OFF_STATE);
    }
  }

  // midair activation
  if (sm_->isInState(StateMachine::READY_FOR_FLIGHT_STATE)) {
    if (sh_control_manager_diag_.hasMsg() && sh_control_manager_diag_.getMsg()->active_tracker == after_midair_activation_tracker_name_) {
      sm_->changeState(StateMachine::FLYING_STATE);
    }
  }

  if (sm_->isInState(StateMachine::TAKING_OFF_STATE)) {
    if (sh_control_manager_diag_.hasMsg() && sh_control_manager_diag_.getMsg()->active_tracker != takeoff_tracker_name_) {
      sm_->changeState(StateMachine::FLYING_STATE);
    }
  }

  if (sm_->isInState(StateMachine::FLYING_STATE)) {
    if (!sh_control_input_.hasMsg()) {
      ROS_WARN_THROTTLE(1.0, "[%s]: not received control input since starting EstimationManager, estimation suboptimal, potentially unstable", getName().c_str());
    } else if ((ros::Time::now() - sh_control_input_.lastMsgTime()).toSec() > 0.1) {
      ROS_WARN_THROTTLE(1.0, "[%s]: not received control input for %.4fs, estimation suboptimal, potentially unstable", getName().c_str(),
                        (ros::Time::now() - sh_control_input_.lastMsgTime()).toSec());
    }
  }
}
/*//}*/

/*//{ timerInitialization() */
void EstimationManager::timerInitialization([[maybe_unused]] const ros::TimerEvent& event) {

  mrs_lib::ParamLoader param_loader(nh_, getName());

  param_loader.loadParam("custom_config", _custom_config_);
  param_loader.loadParam("platform_config", _platform_config_);
  param_loader.loadParam("world_config", _world_config_);

  if (_custom_config_ != "") {
    param_loader.addYamlFile(_custom_config_);
  }

  if (_platform_config_ != "") {
    param_loader.addYamlFile(_platform_config_);
  }

  if (_world_config_ != "") {
    param_loader.addYamlFile(_world_config_);
  }

  param_loader.addYamlFileFromParam("private_config");
  param_loader.addYamlFileFromParam("public_config");
  param_loader.addYamlFileFromParam("uav_manager_config");
  param_loader.addYamlFileFromParam("estimators_config");
  param_loader.addYamlFileFromParam("active_estimators_config");

  param_loader.loadParam("uav_name", ch_->uav_name);

  /*//{ load world_origin parameters */

  std::string world_origin_units;
  bool        is_origin_param_ok = true;
  double      world_origin_x     = 0;
  double      world_origin_y     = 0;

  param_loader.loadParam("world_origin/units", world_origin_units);

  if (Support::toLowercase(world_origin_units) == "utm") {
    ROS_INFO("[%s]: Loading world origin in UTM units.", getName().c_str());
    is_origin_param_ok &= param_loader.loadParam("world_origin/origin_x", world_origin_x);
    is_origin_param_ok &= param_loader.loadParam("world_origin/origin_y", world_origin_y);

  } else if (Support::toLowercase(world_origin_units) == "latlon") {
    double lat, lon;
    ROS_INFO("[%s]: Loading world origin in LatLon units.", getName().c_str());
    is_origin_param_ok &= param_loader.loadParam("world_origin/origin_x", lat);
    is_origin_param_ok &= param_loader.loadParam("world_origin/origin_y", lon);
    mrs_lib::UTM(lat, lon, &world_origin_x, &world_origin_y);
    ROS_INFO("[%s]: Converted to UTM x: %f, y: %f.", getName().c_str(), world_origin_x, world_origin_y);

  } else {
    ROS_ERROR("[%s]: world_origin_units must be (\"UTM\"|\"LATLON\"). Got '%s'", getName().c_str(), world_origin_units.c_str());
    error_publisher_->addOneshotError("Wrong world_origin_units value.");
    error_publisher_->flushAndShutdown();
    return;
  }

  ch_->world_origin.x = world_origin_x;
  ch_->world_origin.y = world_origin_y;

  if (!is_origin_param_ok) {
    ROS_ERROR("[%s]: Could not load all mandatory parameters from world file. Please check your world file.", getName().c_str());
    error_publisher_->addOneshotError("Bad world file.");
    error_publisher_->flushAndShutdown();
    return;
  }
  /*//}*/

  /*//{ load tracker names */
  param_loader.loadParam("mrs_uav_managers/uav_manager/takeoff/during_takeoff/tracker", takeoff_tracker_name_);
  param_loader.loadParam("mrs_uav_managers/uav_manager/midair_activation/after_activation/tracker", after_midair_activation_tracker_name_);
  /*//}*/

  param_loader.setPrefix(ch_->package_name + "/" + Support::toSnakeCase(ch_->nodelet_name) + "/");

  /*//{ load parameters into common handlers */
  param_loader.loadParam("frame_id/fcu", ch_->frames.fcu);
  ch_->frames.ns_fcu = ch_->uav_name + "/" + ch_->frames.fcu;

  param_loader.loadParam("frame_id/fcu_untilted", ch_->frames.fcu_untilted);
  ch_->frames.ns_fcu_untilted = ch_->uav_name + "/" + ch_->frames.fcu_untilted;

  param_loader.loadParam("frame_id/rtk_antenna", ch_->frames.rtk_antenna);
  ch_->frames.ns_rtk_antenna = ch_->uav_name + "/" + ch_->frames.rtk_antenna;

  ch_->transformer = std::make_shared<mrs_lib::Transformer>(nh_, getName());
  ch_->transformer->retryLookupNewest(true);

  param_loader.loadParam("rate/diagnostics", ch_->desired_diagnostics_rate);

  /*//}*/

  /*//{ load parameters */

  /*//{ publish debug topics parameters */
  param_loader.loadParam("debug_topics/input", ch_->debug_topics.input);
  param_loader.loadParam("debug_topics/output", ch_->debug_topics.output);
  param_loader.loadParam("debug_topics/state", ch_->debug_topics.state);
  param_loader.loadParam("debug_topics/covariance", ch_->debug_topics.covariance);
  param_loader.loadParam("debug_topics/innovation", ch_->debug_topics.innovation);
  param_loader.loadParam("debug_topics/diagnostics", ch_->debug_topics.diag);
  param_loader.loadParam("debug_topics/correction", ch_->debug_topics.correction);
  param_loader.loadParam("debug_topics/correction_delay", ch_->debug_topics.corr_delay);
  /*//}*/

  /*//}*/

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = getName();
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  /*//{ wait for hw api message */

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiCapabilities> sh_hw_api_capabilities_ =
      mrs_lib::SubscribeHandler<mrs_msgs::HwApiCapabilities>(shopts, "hw_api_capabilities_in");
  while (!sh_hw_api_capabilities_.hasMsg()) {
    ROS_INFO("[%s]: %s hw_api_capabilities message at topic: %s", getName().c_str(), Support::waiting_for_string.c_str(),
             sh_hw_api_capabilities_.topicName().c_str());
    error_publisher_->addWaitingForNodeError("HwApiManager", "main");
    ros::Duration(1.0).sleep();
  }

  mrs_msgs::HwApiCapabilitiesConstPtr hw_api_capabilities = sh_hw_api_capabilities_.getMsg();
  /*//}*/

  /*//{ wait for desired uav_state rate */
  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in");
  while (!sh_control_manager_diag_.hasMsg()) {
    ROS_INFO("[%s]: %s control_manager_diagnostics message at topic: %s", getName().c_str(), Support::waiting_for_string.c_str(),
             sh_control_manager_diag_.topicName().c_str());
    ros::Duration(1.0).sleep();
  }

  mrs_msgs::ControlManagerDiagnosticsConstPtr control_manager_diag_msg = sh_control_manager_diag_.getMsg();
  ch_->desired_uav_state_rate                                          = control_manager_diag_msg->desired_uav_state_rate;
  ROS_INFO("[%s]: The estimation is running at: %.2f Hz", getName().c_str(), ch_->desired_uav_state_rate);
  /*//}*/

  /*//{ initialize subscribers */

  sh_control_input_ = mrs_lib::SubscribeHandler<mrs_msgs::EstimatorInput>(shopts, "control_input_in");

  sh_safety_area_manager_diag = mrs_lib::SubscribeHandler<mrs_msgs::SafetyAreaManagerDiagnostics>(shopts, "safety_area_manager_diagnostics_in", &EstimationManager::callbackSafetyAreaManagerDiagnostics, this);

  /*//}*/

  /*//{ load state estimator plugins */
  param_loader.loadParam("state_estimators", estimator_names_);

  state_estimator_loader_ = std::make_unique<pluginlib::ClassLoader<mrs_uav_managers::StateEstimator>>("mrs_uav_managers", "mrs_uav_managers::StateEstimator");

  for (size_t i = 0; i < estimator_names_.size(); i++) {

    const std::string estimator_name = estimator_names_[i];

    // load the estimator parameters
    std::string address;
    param_loader.loadParam(estimator_name + "/address", address);

    try {
      ROS_INFO("[%s]: loading the estimator '%s'", getName().c_str(), address.c_str());
      estimator_list_.push_back(state_estimator_loader_->createInstance(address.c_str()));
    }
    catch (pluginlib::CreateClassException& ex1) {
      ROS_ERROR("[%s]: CreateClassException for the estimator '%s'", getName().c_str(), address.c_str());
      ROS_ERROR("[%s]: Error: %s", getName().c_str(), ex1.what());
      error_publisher_->addOneshotError("Initialization error: plugin exception.");
      error_publisher_->flushAndShutdown();
      return;
    }
    catch (pluginlib::PluginlibException& ex) {
      ROS_ERROR("[%s]: PluginlibException for the estimator '%s'", getName().c_str(), address.c_str());
      ROS_ERROR("[%s]: Error: %s", getName().c_str(), ex.what());
      error_publisher_->addOneshotError("Initialization error: plugin exception.");
      error_publisher_->flushAndShutdown();
      return;
    }
  }

  /*//{ load agl estimator plugins */
  param_loader.loadParam("agl_height_estimator", est_alt_agl_name_);
  is_using_agl_estimator_ = est_alt_agl_name_ != "";
  ROS_WARN_COND(!is_using_agl_estimator_, "[%s]: not using AGL estimator for min height safe checking", getName().c_str());


  if (is_using_agl_estimator_) {

    agl_estimator_loader_ = std::make_unique<pluginlib::ClassLoader<mrs_uav_managers::AglEstimator>>("mrs_uav_managers", "mrs_uav_managers::AglEstimator");

    // load the estimator parameters
    std::string address;
    param_loader.loadParam(est_alt_agl_name_ + "/address", address);

    try {
      ROS_INFO("[%s]: loading the estimator '%s'", getName().c_str(), address.c_str());
      est_alt_agl_ = agl_estimator_loader_->createInstance(address.c_str());
    }
    catch (pluginlib::CreateClassException& ex1) {
      ROS_ERROR("[%s]: CreateClassException for the estimator '%s'", getName().c_str(), address.c_str());
      ROS_ERROR("[%s]: Error: %s", getName().c_str(), ex1.what());
      error_publisher_->addOneshotError("Initialization error: plugin exception.");
      error_publisher_->flushAndShutdown();
      return;
    }
    catch (pluginlib::PluginlibException& ex) {
      ROS_ERROR("[%s]: PluginlibException for the estimator '%s'", getName().c_str(), address.c_str());
      ROS_ERROR("[%s]: Error: %s", getName().c_str(), ex.what());
      error_publisher_->addOneshotError("Initialization error: plugin exception.");
      error_publisher_->flushAndShutdown();
      return;
    }
  }
  /*//}*/

  ROS_INFO("[%s]: estimators were loaded", getName().c_str());
  /*//}*/

  /*//{ check whether initial estimator was loaded */
  param_loader.loadParam("initial_state_estimator", initial_estimator_name_);
  bool initial_estimator_found = false;
  for (auto estimator : estimator_list_) {
    if (estimator->getName() == initial_estimator_name_) {
      initial_estimator_      = estimator;
      initial_estimator_found = true;
      break;
    }
  }

  if (!initial_estimator_found) {
    ROS_ERROR("[%s]: initial estimator %s could not be found among loaded estimators. shutting down", getName().c_str(), initial_estimator_name_.c_str());
    error_publisher_->addOneshotError("Initial estimator not found.");
    error_publisher_->flushAndShutdown();
    return;
  }
  /*//}*/

  /*//{ initialize estimators */
  for (auto estimator : estimator_list_) {

    // create private handlers
    std::shared_ptr<mrs_uav_managers::estimation_manager::PrivateHandlers_t> ph = std::make_shared<mrs_uav_managers::estimation_manager::PrivateHandlers_t>();

    ph->loadConfigFile = boost::bind(&EstimationManager::loadConfigFile, this, _1);
    ph->param_loader   = std::make_unique<mrs_lib::ParamLoader>(ros::NodeHandle(nh_, estimator->getName()), "EstimationManager/" + estimator->getName());

    if (_custom_config_ != "") {
      ph->param_loader->addYamlFile(_custom_config_);
    }

    if (_platform_config_ != "") {
      ph->param_loader->addYamlFile(_platform_config_);
    }

    if (_world_config_ != "") {
      ph->param_loader->addYamlFile(_world_config_);
    }

    try {
      ROS_INFO("[%s]: initializing the estimator '%s'", getName().c_str(), estimator->getName().c_str());
      estimator->initialize(nh_, ch_, ph);
    }
    catch (std::runtime_error& ex) {
      ROS_ERROR("[%s]: exception caught during estimator initialization: '%s'", getName().c_str(), ex.what());
      error_publisher_->addOneshotError("Initialization error: runtime exception.");
      error_publisher_->flushAndShutdown();
      return;
    }

    if (!estimator->isCompatibleWithHwApi(hw_api_capabilities)) {
      ROS_ERROR("[%s]: estimator %s is not compatible with the hw api. Shutting down.", getName().c_str(), estimator->getName().c_str());
      error_publisher_->addOneshotError("Initialization error: estimator not compatible with HW API.");
      error_publisher_->flushAndShutdown();
      return;
    }
  }

  // | ----------- agl height estimator initialization ---------- |
  if (is_using_agl_estimator_) {

    std::shared_ptr<mrs_uav_managers::estimation_manager::PrivateHandlers_t> ph = std::make_shared<mrs_uav_managers::estimation_manager::PrivateHandlers_t>();

    ph->loadConfigFile = boost::bind(&EstimationManager::loadConfigFile, this, _1);
    ph->param_loader   = std::make_unique<mrs_lib::ParamLoader>(ros::NodeHandle(nh_, est_alt_agl_->getName()), "EstimationManager/" + est_alt_agl_->getName());

    if (_custom_config_ != "") {
      ph->param_loader->addYamlFile(_custom_config_);
    }

    if (_platform_config_ != "") {
      ph->param_loader->addYamlFile(_platform_config_);
    }

    if (_world_config_ != "") {
      ph->param_loader->addYamlFile(_world_config_);
    }

    try {
      ROS_INFO("[%s]: initializing the estimator '%s'", getName().c_str(), est_alt_agl_->getName().c_str());
      est_alt_agl_->initialize(nh_, ch_, ph);
    }
    catch (std::runtime_error& ex) {
      ROS_ERROR("[%s]: exception caught during estimator initialization: '%s'", getName().c_str(), ex.what());
      error_publisher_->addOneshotError("Initialization error: runtime exception.");
      error_publisher_->flushAndShutdown();
      return;
    }

    if (!est_alt_agl_->isCompatibleWithHwApi(hw_api_capabilities)) {
      ROS_ERROR("[%s]: estimator %s is not compatible with the hw api. Shutting down.", getName().c_str(), est_alt_agl_->getName().c_str());
      error_publisher_->addOneshotError("Initialization error: estimator not compatible with HW API.");
      error_publisher_->flushAndShutdown();
      return;
    }
  }

  ROS_INFO("[%s]: estimators were initialized", getName().c_str());

  /*//}*/

  /*//{ initialize publishers */
  ph_uav_state_    = mrs_lib::PublisherHandler<mrs_msgs::UavState>(nh_, "uav_state_out", 10);
  ph_odom_main_    = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, "odom_main_out", 10);
  ph_innovation_   = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, "innovation_out", 10);
  ph_diagnostics_  = mrs_lib::PublisherHandler<mrs_msgs::EstimationDiagnostics>(nh_, "diagnostics_out", 10);
  ph_max_flight_z_ = mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>(nh_, "max_flight_z_agl_out", 10);
  ph_altitude_agl_ = mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>(nh_, "height_agl_out", 10);

  /*//}*/

  /*//{ initialize timers */
  timer_publish_ = nh_.createTimer(ros::Rate(ch_->desired_uav_state_rate), &EstimationManager::timerPublish, this);

  timer_publish_diagnostics_ = nh_.createTimer(ros::Rate(ch_->desired_diagnostics_rate), &EstimationManager::timerPublishDiagnostics, this);

  timer_check_health_ = nh_.createTimer(ros::Rate(ch_->desired_uav_state_rate), &EstimationManager::timerCheckHealth, this);
  /*//}*/

  /*//{ initialize service clients */

  srvch_failsafe_.initialize(nh_, "failsafe_out");

  /*//}*/

  /*//{ initialize service servers */
  srvs_change_estimator_ = nh_.advertiseService("change_estimator_in", &EstimationManager::callbackChangeEstimator, this);
  srvs_reset_estimator_ = nh_.advertiseService("reset_estimator_in", &EstimationManager::callbackResetEstimator, this);
  srvs_toggle_callbacks_ = nh_.advertiseService("toggle_service_callbacks_in", &EstimationManager::callbackToggleServiceCallbacks, this);
  /*//}*/

  /*//{ initialize scope timer */
  param_loader.loadParam("scope_timer/enabled", ch_->scope_timer.enabled);
  std::string       filepath;
  const std::string time_logger_filepath = ros::package::getPath(package_name_) + "/scope_timer/scope_timer.txt";
  ch_->scope_timer.logger                = std::make_shared<mrs_lib::ScopeTimerLogger>(time_logger_filepath, ch_->scope_timer.enabled);
  /*//}*/

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getName().c_str());
    error_publisher_->addOneshotError("Could not load all parameters.");
    error_publisher_->flushAndShutdown();
    return;
  }

  sm_->changeState(StateMachine::INITIALIZED_STATE);

  ROS_INFO("[%s]: initialized", getName().c_str());
  is_initialized_ = true;
}
/*//}*/

// | -------------------- service callbacks ------------------- |

/*//{ callbackChangeEstimator() */
bool EstimationManager::callbackChangeEstimator(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res) {

  if (!sm_->isInitialized()) {
    return false;
  }

  // enable switching out from vins_kickoff estimator during takeoff
  if (!callbacks_enabled_ && active_estimator_->getName() != "vins_kickoff") {
    res.success = false;
    res.message = ("Service callbacks are disabled");
    ROS_WARN("[%s]: Ignoring service call. Callbacks are disabled.", getName().c_str());
    return true;
  }

  // switching into these estimators during flight is dangerous with realhw, so we don't allow it 
  if (req.value == "dummy" || req.value == "ground_truth" || req.value == "vins_kickoff") {
    res.success = false;
    std::stringstream ss;
    ss << "Switching to " << req.value << " estimator is not allowed.";
    res.message = ss.str();
    ROS_WARN("[%s]: Switching to %s estimator is not allowed.", getName().c_str(), req.value.c_str());
    return true;
  }

  // we do not want the switch to be disturbed by a service call
  callbacks_enabled_ = false;

  bool                                                target_estimator_found = false;
  boost::shared_ptr<mrs_uav_managers::StateEstimator> target_estimator;
  for (auto estimator : estimator_list_) {
    if (estimator->getName() == req.value) {
      target_estimator       = estimator;
      target_estimator_found = true;
      break;
    }
  }

  if (target_estimator_found) {

    if (target_estimator->isRunning()) {
      sm_->changeState(StateMachine::ESTIMATOR_SWITCHING_STATE);
      switchToEstimator(target_estimator);
      sm_->changeToPreSwitchState();
    } else {
      ROS_WARN("[%s]: Switch to not running estimator %s requested", getName().c_str(), req.value.c_str());
      res.success = false;
      res.message = ("Requested estimator is not running");
      return true;
    }

  } else {
    ROS_WARN("[%s]: Switch to invalid estimator %s requested", getName().c_str(), req.value.c_str());
    res.success = false;
    res.message = ("Not a valid estimator type");
    return true;
  }

  res.success = true;
  res.message = "Estimator switch successful";

  // allow service calllbacks after switch again
  callbacks_enabled_ = true;

  return true;
}
/*//}*/

/*//{ callbackResetEstimator() */
bool EstimationManager::callbackResetEstimator(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res) {

  if (!sm_->isInitialized()) {
    return false;
  }

  if (!callbacks_enabled_) {
    res.success = false;
    res.message = ("Service callbacks are disabled");
    ROS_WARN("[%s]: Ignoring service call. Callbacks are disabled.", getName().c_str());
    return true;
  }


  bool                                                target_estimator_found = false;
  boost::shared_ptr<mrs_uav_managers::StateEstimator> target_estimator;
  for (auto estimator : estimator_list_) {
    if (estimator->getName() == req.value) {
      target_estimator       = estimator;
      target_estimator_found = true;
      break;
    }
  }

  if (target_estimator_found) {


    if (target_estimator->getName() == active_estimator_->getName()) {
      res.success = false;
      res.message = ("Cannot reset active estimator");
      ROS_WARN("[%s]: Ignoring service call. Cannot reset active estimator.", getName().c_str());
      return true;
    }

      target_estimator->reset();
      ROS_INFO("[EstimationManager]: Estimator %s reset", target_estimator->getName().c_str());

      double t_wait_left = 5;
      while (t_wait_left > 0) {
        ROS_INFO("[EstimationManager]: Attempting starting %s estimator", target_estimator->getName().c_str());
        target_estimator->start();

        if (target_estimator->isRunning()) {
          ROS_INFO("[EstimationManager]: Reset of %s estimator successful", target_estimator->getName().c_str());
          break;
        } 
        
        const double start_period = 1.0;
        ros::Duration(start_period).sleep();
        t_wait_left -= start_period;

      }

  } else {
    ROS_WARN("[%s]: Reset of invalid estimator %s requested", getName().c_str(), req.value.c_str());
    res.success = false;
    res.message = ("Not a valid estimator type");
    return true;
  }

  res.success = true;
  res.message = "Estimator reset successful";

  return true;
}
/*//}*/

/* //{ callbackToggleServiceCallbacks() */
bool EstimationManager::callbackToggleServiceCallbacks(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  if (!sm_->isInitialized()) {
    ROS_ERROR("[%s]: service for toggling callbacks is not available before initialization.", getName().c_str());
    return false;
  }

  callbacks_disabled_by_service_ = !req.data;

  res.success = true;
  res.message = (callbacks_disabled_by_service_ ? "Service callbacks disabled" : "Service callbacks enabled");

  if (callbacks_disabled_by_service_) {

    ROS_INFO("[%s]: Service callbacks disabled.", getName().c_str());

  } else {

    ROS_INFO("[%s]: Service callbacks enabled", getName().c_str());
  }

  return true;
}

//}


// --------------------------------------------------------------
// |                        other methods                       |
// --------------------------------------------------------------
//
/*//{ switchToHealthyEstimator() */
bool EstimationManager::switchToHealthyEstimator() {

  // available estimators should be specified in decreasing priority order in config file
  for (auto estimator : estimator_list_) {
    if (estimator->isRunning()) {
      switchToEstimator(estimator);
      return true;
    }
  }
  return false;  // no other estimator is running
}
/*//}*/

/*//{ switchToEstimator() */
void EstimationManager::switchToEstimator(const boost::shared_ptr<mrs_uav_managers::StateEstimator>& target_estimator) {

  std::scoped_lock lock(mutex_active_estimator_);
  ROS_INFO("[%s]: switching estimator from %s to %s", getName().c_str(), active_estimator_->getName().c_str(), target_estimator->getName().c_str());
  active_estimator_ = target_estimator;
  estimator_switch_count_++;
}
/*//}*/

/*//{ callFailsafeService() */
bool EstimationManager::callFailsafeService() {
  std_srvs::Trigger srv_out;
  return srvch_failsafe_.call(srv_out);
}
/*//}*/

/*//{ getName() */
std::string EstimationManager::getName() const {
  return nodelet_name_;
}
/*//}*/

/* loadConfigFile() //{ */

bool EstimationManager::loadConfigFile(const std::string& file_path) {

  const std::string name_space = nh_.getNamespace() + "/";

  ROS_INFO("[%s]: loading '%s' under the namespace '%s'", getName().c_str(), file_path.c_str(), name_space.c_str());

  // load the user-requested file
  {
    std::string command = "rosparam load " + file_path + " " + name_space;
    int         result  = std::system(command.c_str());

    if (result != 0) {
      ROS_ERROR("[%s]: failed to load '%s'", getName().c_str(), file_path.c_str());
      return false;
    }
  }

  // load the platform config
  if (_platform_config_ != "") {
    std::string command = "rosparam load " + _platform_config_ + " " + name_space;
    int         result  = std::system(command.c_str());

    if (result != 0) {
      ROS_ERROR("[%s]: failed to load the platform config file '%s'", getName().c_str(), _platform_config_.c_str());
      return false;
    }
  }

  // load the custom config
  if (_custom_config_ != "") {
    std::string command = "rosparam load " + _custom_config_ + " " + name_space;
    int         result  = std::system(command.c_str());

    if (result != 0) {
      ROS_ERROR("[%s]: failed to load the custom config file '%s'", getName().c_str(), _custom_config_.c_str());
      return false;
    }
  }

  return true;
}

//}

}  // namespace estimation_manager

}  // namespace mrs_uav_managers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::estimation_manager::EstimationManager, nodelet::Nodelet)
