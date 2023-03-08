#define VERSION "0.0.0.1"

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

/*//{ FIXME: delete after merge with new uav system */
#include <mrs_msgs/OdometryDiag.h>
/*//}*/

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>


#include "mrs_uav_managers/state_estimator.h"
#include "mrs_uav_managers/agl_estimator.h"
#include "estimation_manager/support.h"
#include "estimation_manager/common_handlers.h"
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
    READY_FOR_TAKEOFF_STATE,
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
    std::scoped_lock lock(mtx_state_);
    return state == current_state_;
  }

  bool isInitialized() const {
    std::scoped_lock lock(mtx_state_);
    return current_state_ != UNINITIALIZED_STATE;
  }

  bool isInPublishableState() const {
    std::scoped_lock lock(mtx_state_);
    return current_state_ == READY_FOR_TAKEOFF_STATE || current_state_ == TAKING_OFF_STATE || current_state_ == HOVER_STATE || current_state_ == FLYING_STATE ||
           current_state_ == LANDING_STATE || current_state_ == DUMMY_STATE;
  }

  bool isInTheAir() const {
    std::scoped_lock lock(mtx_state_);
    return current_state_ == TAKING_OFF_STATE || current_state_ == HOVER_STATE || current_state_ == FLYING_STATE || current_state_ == LANDING_STATE;
  }

  SMState_t getCurrentState() const {
    std::scoped_lock lock(mtx_state_);
    return current_state_;
  }

  std::string getCurrentStateString() const {
    std::scoped_lock lock(mtx_state_);
    return sm_state_names_[current_state_];
  }

  std::string getStateAsString(const SMState_t& state) const {
    return sm_state_names_[state];
  }

  /*//{ changeState() */
  bool changeState(const SMState_t& target_state) {

    switch (target_state) {

      case UNINITIALIZED_STATE: {
        ROS_ERROR("[%s]: transition to %s is not possible from any state", getPrintName().c_str(), getStateAsString(UNINITIALIZED_STATE).c_str());
        return false;
        break;
      }

      case INITIALIZED_STATE: {
        if (current_state_ != UNINITIALIZED_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s", getPrintName().c_str(), getStateAsString(INITIALIZED_STATE).c_str(),
                    getStateAsString(UNINITIALIZED_STATE).c_str());
          return false;
        }
        break;
      }

      case READY_FOR_TAKEOFF_STATE: {
        if (current_state_ != INITIALIZED_STATE && current_state_ != LANDED_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s or %s", getPrintName().c_str(), getStateAsString(READY_FOR_TAKEOFF_STATE).c_str(),
                    getStateAsString(INITIALIZED_STATE).c_str(), getStateAsString(LANDED_STATE).c_str());
          return false;
        }
        break;
      }

      case TAKING_OFF_STATE: {
        if (current_state_ != READY_FOR_TAKEOFF_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s", getPrintName().c_str(), getStateAsString(TAKING_OFF_STATE).c_str(),
                    getStateAsString(READY_FOR_TAKEOFF_STATE).c_str());
          return false;
        }
        break;
      }

      case FLYING_STATE: {
        if (current_state_ != TAKING_OFF_STATE && current_state_ != HOVER_STATE && current_state_ != ESTIMATOR_SWITCHING_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s or %s or %s", getPrintName().c_str(), getStateAsString(FLYING_STATE).c_str(),
                    getStateAsString(TAKING_OFF_STATE).c_str(), getStateAsString(HOVER_STATE).c_str(), getStateAsString(ESTIMATOR_SWITCHING_STATE).c_str());
          return false;
        }
        break;
      }

      case HOVER_STATE: {
        if (current_state_ != FLYING_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s", getPrintName().c_str(), getStateAsString(HOVER_STATE).c_str(),
                    getStateAsString(FLYING_STATE).c_str());
          return false;
        }
        break;
      }

      case ESTIMATOR_SWITCHING_STATE: {
        if (current_state_ != FLYING_STATE && current_state_ != HOVER_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s or %s", getPrintName().c_str(), getStateAsString(ESTIMATOR_SWITCHING_STATE).c_str(),
                    getStateAsString(FLYING_STATE).c_str(), getStateAsString(HOVER_STATE).c_str());
          return false;
        }
        pre_switch_state_ = current_state_;
        break;
      }

      case LANDING_STATE: {
        if (current_state_ != FLYING_STATE && current_state_ != HOVER_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s or %s", getPrintName().c_str(), getStateAsString(LANDING_STATE).c_str(),
                    getStateAsString(FLYING_STATE).c_str(), getStateAsString(HOVER_STATE).c_str());
          return false;
        }
        break;
      }

      case LANDED_STATE: {
        if (current_state_ != LANDING_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s", getPrintName().c_str(), getStateAsString(LANDED_STATE).c_str(),
                    getStateAsString(LANDING_STATE).c_str());
          return false;
        }
        break;
      }

      case DUMMY_STATE: {
        if (current_state_ != INITIALIZED_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s", getPrintName().c_str(), getStateAsString(DUMMY_STATE).c_str(),
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

      default: {
        ROS_ERROR("[%s]: rejected transition to unknown state id %d", getPrintName().c_str(), target_state);
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
  "READY_FOR_TAKEOFF_STATE",
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

  std::string version_;

  std::shared_ptr<CommonHandlers_t> ch_;

  std::shared_ptr<StateMachine> sm_;

  mrs_lib::PublisherHandler<mrs_msgs::EstimationDiagnostics> ph_diagnostics_;
  mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>        ph_max_flight_altitude_agl_;
  mrs_lib::PublisherHandler<mrs_msgs::UavState>              ph_uav_state_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry>              ph_odom_main_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry>              ph_odom_slow_;  // use topic throttler instead?
  mrs_lib::PublisherHandler<nav_msgs::Odometry>              ph_innovation_;

  mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped> ph_altitude_amsl_;
  mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped> ph_altitude_agl_;

  mrs_lib::PublisherHandler<geometry_msgs::QuaternionStamped> ph_orientation_;

  /*//{ FIXME: delete after merge with new uav system */
  mrs_lib::PublisherHandler<mrs_msgs::OdometryDiag> ph_diagnostics_legacy_;
  /*//}*/

  ros::Timer timer_publish_;
  double     timer_rate_publish_;
  void       timerPublish(const ros::TimerEvent& event);

  ros::Timer timer_check_health_;
  double     timer_rate_check_health_;
  void       timerCheckHealth(const ros::TimerEvent& event);

  ros::ServiceServer srvs_change_estimator_;
  bool               callbackChangeEstimator(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);
  int                estimator_switch_count_ = 0;


  ros::ServiceServer srvs_toggle_callbacks_;
  bool               callbackToggleServiceCallbacks(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool               callbacks_enabled_             = false;
  bool               callbacks_disabled_by_service_ = false;

  bool                                             callFailsafeService();
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> srvch_failsafe_;
  // TODO service clients
  /* mrs_lib::ServiceClientHandler<std_srvs::Trigger> srvc_hover_; */
  /* mrs_lib::ServiceClientHandler<mrs_msgs::ReferenceStampedSrv> srvc_reference_; */
  /* mrs_lib::ServiceClientHandler<std_srvs::Trigger> srvc_ehover_; */
  /* mrs_lib::ServiceClientHandler<std_srvs::SetBool> srvc_enable_callbacks_; */

  // | ------------- dynamic loading of estimators ------------- |
  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_managers::StateEstimator>> state_estimator_loader_;  // pluginlib loader of dynamically loaded estimators
  std::vector<std::string>                                                  estimator_names_;         // list of estimator names
  /* std::map<std::string, EstimatorParams>                               estimator_params_;        // map between estimator names and estimator params */
  std::vector<boost::shared_ptr<mrs_uav_managers::StateEstimator>> estimator_list_;  // list of estimators
  std::mutex                                                       mutex_estimator_list_;
  std::vector<std::string>                                         switchable_estimator_names_;
  /* int                                                                      active_estimator_idx_; */
  std::string                                         initial_estimator_name_ = "UNDEFINED_INITIAL_ESTIMATOR";
  boost::shared_ptr<mrs_uav_managers::StateEstimator> initial_estimator_;
  boost::shared_ptr<mrs_uav_managers::StateEstimator> active_estimator_;
  std::mutex                                          mutex_active_estimator_;

  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_managers::AglEstimator>> agl_estimator_loader_;  // pluginlib loader of dynamically loaded estimators
  std::string                                                             est_alt_agl_name_ = "UNDEFINED_AGL_ESTIMATOR";
  boost::shared_ptr<mrs_uav_managers::AglEstimator>                         est_alt_agl_;

  double max_safety_area_altitude_;

  bool switchToHealthyEstimator();
  void switchToEstimator(const boost::shared_ptr<mrs_uav_managers::StateEstimator>& target_estimator);

public:
  EstimationManager() {
  }

  void onInit();

  std::string getName() const;
};
/*//}*/

/*//{ onInit() */
void EstimationManager::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[%s]: initializing", getName().c_str());

  sm_ = std::make_shared<StateMachine>(nodelet_name_);

  ch_ = std::make_shared<CommonHandlers_t>();

  ch_->nodelet_name = nodelet_name_;
  ch_->package_name = package_name_;

  mrs_lib::ParamLoader param_loader(nh, getName());

  // load maximum flight altitude from safety area
  bool use_safety_area;
  param_loader.loadParam("safety_area/use_safety_area", use_safety_area);
  if (use_safety_area) {
    param_loader.loadParam("safety_area/max_height", max_safety_area_altitude_);
  } else {
    ROS_WARN("[%s]: NOT USING SAFETY AREA!!!", getName().c_str());
    max_safety_area_altitude_ = 100;
  }

  // load common parameters into the common handlers structure
  param_loader.loadParam("uav_name", ch_->uav_name);
  param_loader.loadParam("frame_id/fcu", ch_->frames.fcu);
  ch_->frames.ns_fcu = ch_->uav_name + "/" + ch_->frames.fcu;

  param_loader.loadParam("frame_id/fcu_untilted", ch_->frames.fcu_untilted);
  ch_->frames.ns_fcu_untilted = ch_->uav_name + "/" + ch_->frames.fcu_untilted;

  param_loader.loadParam("frame_id/rtk_antenna", ch_->frames.rtk_antenna);
  ch_->frames.ns_rtk_antenna = ch_->uav_name + "/" + ch_->frames.rtk_antenna;

  ch_->transformer = std::make_shared<mrs_lib::Transformer>(nh, getName());
  ch_->transformer->retryLookupNewest(true);

  /*//{ check version */
  param_loader.loadParam("version", version_);

  if (version_ != VERSION) {

    ROS_ERROR("[%s]: the version of the binary (%s) does not match the config file (%s), please build me!", getName().c_str(), VERSION, version_.c_str());
    ros::shutdown();
  }
  /*//}*/

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getName();
  shopts.no_message_timeout = ros::Duration(0.5);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiCapabilities> sh_hw_api_capabilities_ =
      mrs_lib::SubscribeHandler<mrs_msgs::HwApiCapabilities>(shopts, "hw_api_capabilities_in");
  while (!sh_hw_api_capabilities_.hasMsg()) {
    ROS_INFO("[%s]: waiting for hw_api_capabilities message at topic: %s", getName().c_str(), sh_hw_api_capabilities_.topicName().c_str());
    ros::Duration(1.0).sleep();
  }

  mrs_msgs::HwApiCapabilitiesConstPtr hw_api_capabilities = sh_hw_api_capabilities_.getMsg();

  /*//{ load estimators */
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
      ros::shutdown();
    }
    catch (pluginlib::PluginlibException& ex) {
      ROS_ERROR("[%s]: PluginlibException for the estimator '%s'", getName().c_str(), address.c_str());
      ROS_ERROR("[%s]: Error: %s", getName().c_str(), ex.what());
      ros::shutdown();
    }
  }

  // initialize standalone height estimator
  /* est_alt_agl_ = std::make_unique<AltGeneric>(est_alt_agl_name_, "agl_origin", Support::toSnakeCase(getName())); */
  /* est_alt_agl_->initialize(nh, ch_); */
  /* est_alt_agl_->setInputCoeff(0.0);  // no input, just corrections */

  param_loader.loadParam("agl_height_estimator", est_alt_agl_name_);

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
    ros::shutdown();
  }
  catch (pluginlib::PluginlibException& ex) {
    ROS_ERROR("[%s]: PluginlibException for the estimator '%s'", getName().c_str(), address.c_str());
    ROS_ERROR("[%s]: Error: %s", getName().c_str(), ex.what());
    ros::shutdown();
  }

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
    ros::shutdown();
  }
  /*//}*/

  /*//{ initialize estimators */
  for (auto estimator : estimator_list_) {

    try {
      ROS_INFO("[%s]: initializing the estimator '%s'", getName().c_str(), estimator->getName().c_str());
      estimator->initialize(nh, ch_);
    }
    catch (std::runtime_error& ex) {
      ROS_ERROR("[%s]: exception caught during estimator initialization: '%s'", getName().c_str(), ex.what());
    }

    if (!estimator->isCompatibleWithHwApi(hw_api_capabilities)) {
      ROS_ERROR("[%s]: estimator %s is not compatible with the hw api. Shutting down.", getName().c_str(), estimator->getName().c_str());
      ros::shutdown();
    }
  }

  // | ----------- agl height estimator initialization ---------- |
  try {
    ROS_INFO("[%s]: initializing the estimator '%s'", getName().c_str(), est_alt_agl_->getName().c_str());
    est_alt_agl_->initialize(nh, ch_);
  }
  catch (std::runtime_error& ex) {
    ROS_ERROR("[%s]: exception caught during estimator initialization: '%s'", getName().c_str(), ex.what());
  }

  if (!est_alt_agl_->isCompatibleWithHwApi(hw_api_capabilities)) {
    ROS_ERROR("[%s]: estimator %s is not compatible with the hw api. Shutting down.", getName().c_str(), est_alt_agl_->getName().c_str());
    ros::shutdown();
  }

  ROS_INFO("[%s]: estimators were initialized", getName().c_str());

  /*//}*/

  /*//{ initialize publishers */
  ph_uav_state_               = mrs_lib::PublisherHandler<mrs_msgs::UavState>(nh, "uav_state_out", 1);
  ph_odom_main_               = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, "odom_main_out", 1);
  ph_innovation_              = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, "innovation_out", 1);
  ph_diagnostics_             = mrs_lib::PublisherHandler<mrs_msgs::EstimationDiagnostics>(nh, "diagnostics_out", 1);
  ph_max_flight_altitude_agl_ = mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>(nh, "max_flight_altitude_agl_out", 1);
  ph_altitude_agl_            = mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>(nh, "height_agl_out", 1);

  /*//{ FIXME: delete after merge with new uav system */
  ph_diagnostics_legacy_ = mrs_lib::PublisherHandler<mrs_msgs::OdometryDiag>(nh, "diagnostics_legacy_out", 1);
  /*//}*/
  /*//}*/

  /*//{ initialize timers */
  param_loader.loadParam("rate/uav_state", timer_rate_publish_);
  timer_publish_ = nh.createTimer(ros::Rate(timer_rate_publish_), &EstimationManager::timerPublish, this);

  param_loader.loadParam("rate/health", timer_rate_check_health_);
  timer_check_health_ = nh.createTimer(ros::Rate(timer_rate_check_health_), &EstimationManager::timerCheckHealth, this);
  /*//}*/

  /*//{ initialize service clients */

  srvch_failsafe_.initialize(nh, "failsafe_out");

  /*//}*/

  /*//{ initialize service servers */
  srvs_change_estimator_ = nh.advertiseService("change_estimator_in", &EstimationManager::callbackChangeEstimator, this);
  srvs_toggle_callbacks_ = nh.advertiseService("toggle_service_callbacks_in", &EstimationManager::callbackToggleServiceCallbacks, this);
  /*//}*/

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getName().c_str());
    ros::shutdown();
  }

  sm_->changeState(StateMachine::INITIALIZED_STATE);

  ROS_INFO("[%s]: initialized", getName().c_str());
}
/*//}*/

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | --------------------- timer callbacks -------------------- |

/*//{ timerPublish() */
void EstimationManager::timerPublish([[maybe_unused]] const ros::TimerEvent& event) {

  if (!sm_->isInitialized()) {
    return;
  }

  if (!sm_->isInState(StateMachine::ESTIMATOR_SWITCHING_STATE)) {

    double max_flight_altitude_agl = max_safety_area_altitude_;
    if (active_estimator_ && active_estimator_->isInitialized()) {
      max_flight_altitude_agl = std::min(max_flight_altitude_agl, active_estimator_->getMaxFlightAltitudeAgl());
    }

    // publish diagnostics
    mrs_msgs::Float64Stamped max_altitude_msg;
    max_altitude_msg.header.stamp = ros::Time::now();
    max_altitude_msg.value        = max_flight_altitude_agl;

    mrs_msgs::EstimationDiagnostics diagnostics;
    diagnostics.header.stamp                = ros::Time::now();
    diagnostics.current_sm_state            = sm_->getCurrentStateString();
    diagnostics.running_state_estimators    = estimator_names_;
    diagnostics.switchable_state_estimators = switchable_estimator_names_;
    diagnostics.max_flight_altitude_agl     = max_flight_altitude_agl;
    if (active_estimator_ && active_estimator_->isInitialized()) {
      max_altitude_msg.header.frame_id    = active_estimator_->getFrameId();
      diagnostics.header.frame_id         = active_estimator_->getFrameId();
      diagnostics.current_state_estimator = active_estimator_->getName();
    } else {
      max_altitude_msg.header.frame_id    = "";
      diagnostics.header.frame_id         = "";
      diagnostics.current_state_estimator = "";
    }
    ph_max_flight_altitude_agl_.publish(max_altitude_msg);
    ph_diagnostics_.publish(diagnostics);

    /*//{ FIXME: delete after merge with new uav system */
    mrs_msgs::OdometryDiag legacy_odom_diag_msg;
    legacy_odom_diag_msg.header.stamp = ros::Time::now();
    mrs_msgs::EstimatorType est_type;
    est_type.name                       = "OTHER";
    est_type.type                       = 1;
    legacy_odom_diag_msg.estimator_type = est_type;
    mrs_msgs::AltitudeType alt_type;
    alt_type.name                      = "OTHER";
    alt_type.type                      = 1;
    legacy_odom_diag_msg.altitude_type = alt_type;
    mrs_msgs::HeadingType hdg_type;
    hdg_type.name                                 = "OTHER";
    hdg_type.type                                 = 1;
    legacy_odom_diag_msg.heading_type             = hdg_type;
    legacy_odom_diag_msg.available_lat_estimators = {""};
    legacy_odom_diag_msg.available_hdg_estimators = {""};
    legacy_odom_diag_msg.available_alt_estimators = {""};

    legacy_odom_diag_msg.max_altitude = max_flight_altitude_agl;
    ph_diagnostics_legacy_.publish(legacy_odom_diag_msg);

    /*//}*/


    if (sm_->isInPublishableState()) {

      mrs_msgs::UavState uav_state = active_estimator_->getUavState();

      uav_state.estimator_iteration = estimator_switch_count_;

      // TODO state health checks

      ph_uav_state_.publish(uav_state);

      nav_msgs::Odometry odom_main = Support::uavStateToOdom(uav_state, ch_->transformer);

      const std::vector<double> pose_covariance = active_estimator_->getPoseCovariance();
      for (size_t i = 0; i < pose_covariance.size(); i++) {
        odom_main.pose.covariance[i] = pose_covariance[i];
      }

      const std::vector<double> twist_covariance = active_estimator_->getTwistCovariance();
      for (size_t i = 0; i < twist_covariance.size(); i++) {
        odom_main.twist.covariance[i] = twist_covariance[i];
      }

      ph_odom_main_.publish(odom_main);

      nav_msgs::Odometry innovation = active_estimator_->getInnovation();
      ph_innovation_.publish(innovation);

      ph_altitude_agl_.publish(est_alt_agl_->getUavAglHeight());

      ROS_INFO_THROTTLE(5.0, "[%s]: pos: [%.2f, %.2f, %.2f] m. Estimator: %s. Max. alt.: %.2f m. Estimator switches: %d.", getName().c_str(),
                        uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z, active_estimator_->getName().c_str(),
                        max_flight_altitude_agl, estimator_switch_count_);

    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: not publishing uav state in %s", getName().c_str(), sm_->getCurrentStateString().c_str());
    }
  }
}
/*//}*/

/*//{ timerCheckHealth() */
void EstimationManager::timerCheckHealth([[maybe_unused]] const ros::TimerEvent& event) {

  if (!sm_->isInitialized()) {
    return;
  }

  /*//{ start ready estimators, check switchable estimators */
  switchable_estimator_names_.clear();
  for (auto estimator : estimator_list_) {

    if (estimator->isReady()) {
      try {
        ROS_INFO("[%s]: starting the estimator '%s'", getName().c_str(), estimator->getName().c_str());
        estimator->start();
      }
      catch (std::runtime_error& ex) {
        ROS_ERROR("[%s]: exception caught during estimator starting: '%s'", getName().c_str(), ex.what());
      }
    }

    if (estimator->isRunning() && estimator->getName() != "dummy" && estimator->getName() != "ground_truth") {
      switchable_estimator_names_.push_back(estimator->getName());
    }
  }

  if (est_alt_agl_->isReady()) {
    est_alt_agl_->start();
  }

  /*//}*/

  if (!callbacks_disabled_by_service_ && (sm_->isInState(StateMachine::FLYING_STATE) || sm_->isInState(StateMachine::HOVER_STATE))) {
    callbacks_enabled_ = true;
  } else {
    callbacks_enabled_ = false;
  }

  // TODO fuj if, zmenit na switch
  // activate initial estimator
  if (sm_->isInState(StateMachine::INITIALIZED_STATE) && initial_estimator_->isRunning()) {
    std::scoped_lock lock(mutex_active_estimator_);
    ROS_INFO("[%s]: activating the initial estimator %s", getName().c_str(), initial_estimator_->getName().c_str());
    active_estimator_ = initial_estimator_;
    if (active_estimator_->getName() == "dummy") {
      sm_->changeState(StateMachine::DUMMY_STATE);
    } else {
      sm_->changeState(StateMachine::READY_FOR_TAKEOFF_STATE);
    }
  }

  // active estimator is in faulty state, we need to switch to healthy estimator
  if (sm_->isInTheAir() && active_estimator_->isError()) {
    sm_->changeState(StateMachine::ESTIMATOR_SWITCHING_STATE);
  }

  if (sm_->isInState(StateMachine::ESTIMATOR_SWITCHING_STATE)) {
    if (switchToHealthyEstimator()) {
      sm_->changeToPreSwitchState();
    } else {  // cannot switch to healthy estimator - failsafe necessary
      ROS_ERROR("[%s]: Cannot switch to any healthy estimator. Triggering failsafe.", getName().c_str());
      if (callFailsafeService()) {
        ROS_INFO("[%s]: failsafe called successfully", getName().c_str());
        sm_->changeState(StateMachine::FAILSAFE_STATE);
      }
    }
  }

  if (sm_->isInState(StateMachine::READY_FOR_TAKEOFF_STATE)) {
    sm_->changeState(StateMachine::TAKING_OFF_STATE);
  }

  if (sm_->isInState(StateMachine::TAKING_OFF_STATE)) {
    sm_->changeState(StateMachine::FLYING_STATE);
  }
}
/*//}*/

// | -------------------- service callbacks ------------------- |

/*//{ callbackChangeEstimator() */
bool EstimationManager::callbackChangeEstimator(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res) {

  if (!sm_->isInitialized()) {
    return false;
  }

  if (!callbacks_enabled_) {
    res.success = false;
    res.message = ("Service callbacks are disabled");
    ROS_WARN("[%s]: Ignoring service call. Callbacks are disabled.", getName().c_str());
    return true;
  }

  if (req.value == "dummy" || req.value == "ground_truth") {
    res.success = false;
    std::stringstream ss;
    ss << "Switching to " << req.value << " estimator is not allowed.";
    res.message = ss.str();
    ROS_WARN("[%s]: Switching to %s estimator is not allowed.", getName().c_str(), req.value.c_str());
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

    if (target_estimator->isRunning()) {
      sm_->changeState(StateMachine::ESTIMATOR_SWITCHING_STATE);
      switchToEstimator(target_estimator);
      sm_->changeToPreSwitchState();
    } else {
      ROS_WARN("[%s]: Not running estimator %s requested", getName().c_str(), req.value.c_str());
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

}  // namespace estimation_manager

}  // namespace mrs_uav_managers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::estimation_manager::EstimationManager, nodelet::Nodelet)
