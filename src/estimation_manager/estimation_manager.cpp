/*//{ includes */

#include <rclcpp/rclcpp.hpp>

#include <pluginlib/class_loader.hpp>

#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <mrs_msgs/msg/uav_state.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>
#include <mrs_msgs/srv/string.hpp>
#include <mrs_msgs/msg/estimation_diagnostics.hpp>
#include <mrs_msgs/msg/hw_api_capabilities.hpp>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>

#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/gps_conversions.h>
#include <mrs_lib/scope_timer.h>

#include <mrs_uav_managers/state_estimator.h>
#include <mrs_uav_managers/agl_estimator.h>
#include <mrs_uav_managers/estimation_manager/support.h>
#include <mrs_uav_managers/estimation_manager/common_handlers.h>
#include <mrs_uav_managers/estimation_manager/private_handlers.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

/*//}*/

/* using //{ */

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

namespace estimation_manager
{

// --------------------------------------------------------------
// |                           classes                          |
// --------------------------------------------------------------

/* class StateMachine //{ */

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
  StateMachine(const rclcpp::Node::SharedPtr &node, const std::string &nodelet_name) : nodelet_name_(nodelet_name) {
    node_  = node;
    clock_ = node_->get_clock();
  }

  bool isInState(const SMState_t &state) const {
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

  std::string getStateAsString(const SMState_t &state) const {
    return sm_state_names_[state];
  }

  /*//{ changeState() */
  bool changeState(const SMState_t &target_state) {

    if (target_state == current_state_) {

      RCLCPP_WARN(node_->get_logger(), "[%s]: requested change to same state %s -> %s", getPrintName().c_str(), getStateAsString(current_state_).c_str(),
                  getStateAsString(target_state).c_str());
      return true;
    }

    switch (target_state) {

    case UNINITIALIZED_STATE: {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: transition to %s is not possible from any state", getPrintName().c_str(),
                            getStateAsString(UNINITIALIZED_STATE).c_str());
      return false;
      break;
    }

    case INITIALIZED_STATE: {
      if (current_state_ != UNINITIALIZED_STATE) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: transition to %s is possible only from %s", getPrintName().c_str(),
                              getStateAsString(INITIALIZED_STATE).c_str(), getStateAsString(UNINITIALIZED_STATE).c_str());
        return false;
      }
      break;
    }

    case READY_FOR_FLIGHT_STATE: {
      if (current_state_ != INITIALIZED_STATE && current_state_ != LANDED_STATE && current_state_ != ESTIMATOR_SWITCHING_STATE) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: transition to %s is possible only from %s, %s or %s", getPrintName().c_str(),
                              getStateAsString(READY_FOR_FLIGHT_STATE).c_str(), getStateAsString(INITIALIZED_STATE).c_str(),
                              getStateAsString(LANDED_STATE).c_str(), getStateAsString(ESTIMATOR_SWITCHING_STATE).c_str());
        return false;
      }
      break;
    }

    case TAKING_OFF_STATE: {
      if (current_state_ != READY_FOR_FLIGHT_STATE && current_state_ != ESTIMATOR_SWITCHING_STATE) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: transition to %s is possible only from %s or %s", getPrintName().c_str(),
                              getStateAsString(TAKING_OFF_STATE).c_str(), getStateAsString(READY_FOR_FLIGHT_STATE).c_str(),
                              getStateAsString(ESTIMATOR_SWITCHING_STATE).c_str());
        return false;
      }
      break;
    }

    case FLYING_STATE: {
      if (current_state_ != TAKING_OFF_STATE && current_state_ != READY_FOR_FLIGHT_STATE && current_state_ != HOVER_STATE &&
          current_state_ != ESTIMATOR_SWITCHING_STATE) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: transition to %s is possible only from %s or %s or %s", getPrintName().c_str(),
                              getStateAsString(FLYING_STATE).c_str(), getStateAsString(TAKING_OFF_STATE).c_str(), getStateAsString(HOVER_STATE).c_str(),
                              getStateAsString(ESTIMATOR_SWITCHING_STATE).c_str());
        return false;
      }
      break;
    }

    case HOVER_STATE: {
      if (current_state_ != FLYING_STATE && current_state_ != ESTIMATOR_SWITCHING_STATE) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: transition to %s is possible only from %s or %s", getPrintName().c_str(),
                              getStateAsString(HOVER_STATE).c_str(), getStateAsString(FLYING_STATE).c_str(),
                              getStateAsString(ESTIMATOR_SWITCHING_STATE).c_str());
        return false;
      }
      break;
    }

    case ESTIMATOR_SWITCHING_STATE: {
      if (current_state_ != READY_FOR_FLIGHT_STATE && current_state_ != TAKING_OFF_STATE && current_state_ != HOVER_STATE && current_state_ != FLYING_STATE &&
          current_state_ != LANDING_STATE) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: transition to %s is possible only from %s, %s, %s, %s or %s", getPrintName().c_str(),
                              getStateAsString(ESTIMATOR_SWITCHING_STATE).c_str(), getStateAsString(READY_FOR_FLIGHT_STATE).c_str(),
                              getStateAsString(TAKING_OFF_STATE).c_str(), getStateAsString(FLYING_STATE).c_str(), getStateAsString(HOVER_STATE).c_str(),
                              getStateAsString(FLYING_STATE).c_str());
        return false;
      }
      pre_switch_state_ = current_state_;
      break;
    }

    case LANDING_STATE: {
      if (current_state_ != FLYING_STATE && current_state_ != HOVER_STATE && current_state_ != ESTIMATOR_SWITCHING_STATE) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: transition to %s is possible only from %s, %s or %s", getPrintName().c_str(),
                              getStateAsString(LANDING_STATE).c_str(), getStateAsString(FLYING_STATE).c_str(), getStateAsString(HOVER_STATE).c_str(),
                              getStateAsString(ESTIMATOR_SWITCHING_STATE).c_str());
        return false;
      }
      break;
    }

    case LANDED_STATE: {
      if (current_state_ != LANDING_STATE) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: transition to %s is possible only from %s", getPrintName().c_str(),
                              getStateAsString(LANDED_STATE).c_str(), getStateAsString(LANDING_STATE).c_str());
        return false;
      }
      break;
    }

    case DUMMY_STATE: {
      if (current_state_ != INITIALIZED_STATE) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: transition to %s is possible only from %s", getPrintName().c_str(),
                              getStateAsString(DUMMY_STATE).c_str(), getStateAsString(INITIALIZED_STATE).c_str());
        return false;
      }
      break;
    }
    case EMERGENCY_STATE: {
      RCLCPP_WARN(node_->get_logger(), "[%s]: transition to %s", getPrintName().c_str(), getStateAsString(EMERGENCY_STATE).c_str());
      break;
    }

    case ERROR_STATE: {
      RCLCPP_WARN(node_->get_logger(), "[%s]: transition to %s", getPrintName().c_str(), getStateAsString(ERROR_STATE).c_str());
      break;
    }

    case FAILSAFE_STATE: {
      RCLCPP_WARN(node_->get_logger(), "[%s]: transition to %s", getPrintName().c_str(), getStateAsString(FAILSAFE_STATE).c_str());
      break;
    }

    default: {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: rejected transition to unknown state id %d", getPrintName().c_str(), target_state);
      return false;
      break;
    }
    }

    std::scoped_lock lock(mtx_state_);
    {
      previous_state_ = current_state_;
      current_state_  = target_state;
    }

    RCLCPP_INFO(node_->get_logger(), "[%s]: successfully changed states %s -> %s", getPrintName().c_str(), getStateAsString(previous_state_).c_str(),
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
  const std::string        name_ = "StateMachine";
  const std::string        nodelet_name_;
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;


  SMState_t current_state_    = UNINITIALIZED_STATE;
  SMState_t previous_state_   = UNINITIALIZED_STATE;
  SMState_t pre_switch_state_ = UNINITIALIZED_STATE;

  mutable std::mutex mtx_state_;

  std::string getName() const {
    return name_;
  }

  std::string getPrintName() const {
    return name_;
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

//}

/* class EstimationManager //{ */

class EstimationManager : public mrs_lib::Node {

private:
  const std::string nodelet_name_ = "EstimationManager";
  const std::string package_name_ = "mrs_uav_managers";

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  std::string _custom_config_;
  std::string _platform_config_;
  std::string _world_config_;

  std::shared_ptr<CommonHandlers_t> ch_;

  std::shared_ptr<StateMachine> sm_;

  std::string after_midair_activation_tracker_name_;
  std::string takeoff_tracker_name_;

  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimatorInput>            sh_control_input_;

  mrs_lib::PublisherHandler<mrs_msgs::msg::EstimationDiagnostics> ph_diagnostics_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>        ph_max_flight_z_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::UavState>              ph_uav_state_;
  mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>              ph_odom_main_;
  mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>              ph_odom_slow_; // use topic throttler instead?
  mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>              ph_innovation_;

  mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped> ph_altitude_amsl_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped> ph_altitude_agl_;

  mrs_lib::PublisherHandler<geometry_msgs::msg::QuaternionStamped> ph_orientation_;

  rclcpp::Time                 time_preinit_started_;
  rclcpp::TimerBase::SharedPtr timer_preinit_;
  void                         timerPreinit();

  std::shared_ptr<TimerType> timer_publish_;
  void                       timerPublish();

  std::shared_ptr<TimerType> timer_publish_diagnostics_;
  void                       timerPublishDiagnostics();

  std::shared_ptr<TimerType> timer_check_health_;
  void                       timerCheckHealth();

  void initialize();

  void shutdown();

  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiCapabilities> sh_hw_api_capabilities_;

  mrs_lib::ServiceServerHandler<mrs_msgs::srv::String> srvs_change_estimator_;
  bool callbackChangeEstimator(const std::shared_ptr<mrs_msgs::srv::String::Request> request, const std::shared_ptr<mrs_msgs::srv::String::Response> response);
  int  estimator_switch_count_ = 0;

  mrs_lib::ServiceServerHandler<mrs_msgs::srv::String> srvs_reset_estimator_;
  bool callbackResetEstimator(const std::shared_ptr<mrs_msgs::srv::String::Request> request, const std::shared_ptr<mrs_msgs::srv::String::Response> response);

  mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv> srvs_set_world_origin_;
  bool callbackSetWorldOrigin(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request>  request,
                              const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response);

  mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool> srvs_toggle_callbacks_;

  bool callbackToggleServiceCallbacks(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                                      const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  bool callbacks_enabled_             = false;
  bool callbacks_disabled_by_service_ = false;

  bool                                                  callFailsafeService();
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> srvch_failsafe_;
  bool                                                  failsafe_call_succeeded_ = false;

  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv> srvch_set_world_origin_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv> srvch_update_sa_mgr_world_origin_;

  // | ------------- dynamic loading of estimators ------------- |

  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_managers::StateEstimator>> state_estimator_loader_; // pluginlib loader of dynamically loaded estimators
  std::vector<std::string>                                                  estimator_names_;        // list of estimator names
  std::vector<std::shared_ptr<mrs_uav_managers::StateEstimator>>            estimator_list_;         // list of estimators
  std::mutex                                                                mutex_estimator_list_;
  std::vector<std::string>                                                  switchable_estimator_names_;
  std::mutex                                                                mutex_switchable_estimator_names_;
  std::string                                                               initial_estimator_name_ = "UNDEFINED_INITIAL_ESTIMATOR";
  std::shared_ptr<mrs_uav_managers::StateEstimator>                         initial_estimator_;
  std::shared_ptr<mrs_uav_managers::StateEstimator>                         active_estimator_;
  std::mutex                                                                mutex_active_estimator_;

  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_managers::AglEstimator>> agl_estimator_loader_; // pluginlib loader of dynamically loaded estimators
  std::string                                                             est_alt_agl_name_ = "UNDEFINED_AGL_ESTIMATOR";
  std::shared_ptr<mrs_uav_managers::AglEstimator>                         est_alt_agl_;
  bool                                                                    is_using_agl_estimator_;

  double max_flight_z_;

  bool switchToHealthyEstimator();
  void switchToEstimator(const std::shared_ptr<mrs_uav_managers::StateEstimator> &target_estimator);

  bool loadConfigFile(const std::string &file_path);

public:
  EstimationManager(rclcpp::NodeOptions options);

  std::string getName() const;
};

//}

/* EstimationManager() //{ */

EstimationManager::EstimationManager(rclcpp::NodeOptions options) : mrs_lib::Node("estimation_manager", options) {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.node_name                           = getName();
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_hw_api_capabilities_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiCapabilities>(shopts, "~/hw_api_capabilities_in");

  sh_control_manager_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(shopts, "~/control_manager_diagnostics_in");

  time_preinit_started_ = clock_->now();

  timer_preinit_ = node_->create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&EstimationManager::timerPreinit, this));
}

//}

/* timerPreinit() //{ */

void EstimationManager::timerPreinit() {

  bool got_data = true;

  if (!sh_hw_api_capabilities_.hasMsg()) {
    RCLCPP_INFO(node_->get_logger(), "%s hw_api_capabilities message at topic: %s", Support::waiting_for_string.c_str(),
                sh_hw_api_capabilities_.topicName().c_str());

    got_data = false;
  }

  // let's wait for the diagnostics 10 seconds and then pass to fallback
  if (!sh_control_manager_diag_.hasMsg() && (clock_->now() - time_preinit_started_).seconds() < 10.0) {

    RCLCPP_INFO(node_->get_logger(), "%s control_manager_diagnostics message at topic: %s", Support::waiting_for_string.c_str(),
                sh_control_manager_diag_.topicName().c_str());

    got_data = false;
  }

  if (got_data) {

    timer_preinit_->cancel();

    initialize();
  }
}

//}

/* initialize() //{ */

void EstimationManager::initialize() {

  rclcpp::on_shutdown([this]() { this->shutdown(); });

  sm_ = std::make_shared<StateMachine>(node_, nodelet_name_);

  ch_ = std::make_shared<CommonHandlers_t>();

  ch_->nodelet_name = nodelet_name_;
  ch_->package_name = package_name_;

  mrs_lib::ParamLoader param_loader(node_);

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

  param_loader.loadParam("mrs_uav_managers/world_origin/units", world_origin_units);

  if (Support::toLowercase(world_origin_units) == "utm") {
    RCLCPP_INFO(node_->get_logger(), "Loading world origin in UTM units.");
    is_origin_param_ok &= param_loader.loadParam("mrs_uav_managers/world_origin/origin_x", world_origin_x);
    is_origin_param_ok &= param_loader.loadParam("mrs_uav_managers/world_origin/origin_y", world_origin_y);

  } else if (Support::toLowercase(world_origin_units) == "latlon") {
    double lat, lon;
    RCLCPP_INFO(node_->get_logger(), "Loading world origin in LatLon units.");
    is_origin_param_ok &= param_loader.loadParam("mrs_uav_managers/world_origin/origin_x", lat);
    is_origin_param_ok &= param_loader.loadParam("mrs_uav_managers/world_origin/origin_y", lon);
    mrs_lib::UTM(lat, lon, &world_origin_x, &world_origin_y);
    RCLCPP_INFO(node_->get_logger(), "Converted to UTM x: %f, y: %f.", world_origin_x, world_origin_y);

  } else {
    RCLCPP_ERROR(node_->get_logger(), "world_origin_units must be (\"UTM\"|\"LATLON\"). Got '%s'", world_origin_units.c_str());
    rclcpp::shutdown();
    exit(1);
  }

  ch_->world_origin.x = world_origin_x;
  ch_->world_origin.y = world_origin_y;

  if (!is_origin_param_ok) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all mandatory parameters from world file. Please check your world file.");
    rclcpp::shutdown();
    exit(1);
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

  ch_->transformer = std::make_shared<mrs_lib::Transformer>(node_);
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

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.node_name                           = getName();
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  /*//{ wait for hw api message */

  mrs_msgs::msg::HwApiCapabilities::ConstSharedPtr hw_api_capabilities = sh_hw_api_capabilities_.getMsg();

  /*//}*/

  /*//{ wait for desired uav_state rate */

  if (!sh_control_manager_diag_.hasMsg()) {

    RCLCPP_WARN(node_->get_logger(), "Not received control_manager_diagnostics, setting fallback estimation rate");
    ch_->desired_uav_state_rate = 100;

  } else {

    RCLCPP_INFO(node_->get_logger(), "[%s]: received control_manager_diagnostics", Support::waiting_for_string.c_str());
    mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr control_manager_diag_msg = sh_control_manager_diag_.getMsg();
    ch_->desired_uav_state_rate                                                       = control_manager_diag_msg->desired_uav_state_rate;
  }

  RCLCPP_INFO(node_->get_logger(), "The estimation is running at: %.2f Hz", ch_->desired_uav_state_rate);

  /*//}*/

  /*//{ initialize subscribers */

  sh_control_input_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimatorInput>(shopts, "~/control_input_in");

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
      RCLCPP_INFO(node_->get_logger(), "loading the estimator '%s'", address.c_str());
      estimator_list_.push_back(state_estimator_loader_->createSharedInstance(address.c_str()));
    }
    catch (pluginlib::CreateClassException &ex1) {
      RCLCPP_ERROR(node_->get_logger(), "CreateClassException for the estimator '%s'", address.c_str());
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex1.what());
      rclcpp::shutdown();
      exit(1);
    }
    catch (pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(node_->get_logger(), "PluginlibException for the estimator '%s'", address.c_str());
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex.what());
      rclcpp::shutdown();
      exit(1);
    }
  }

  /*//{ load agl estimator plugins */
  param_loader.loadParam("agl_height_estimator", est_alt_agl_name_);
  is_using_agl_estimator_ = est_alt_agl_name_ != "";

  if (!is_using_agl_estimator_) {
    RCLCPP_WARN(node_->get_logger(), "not using AGL estimator for min height safe checking");
  }

  if (is_using_agl_estimator_) {

    agl_estimator_loader_ = std::make_unique<pluginlib::ClassLoader<mrs_uav_managers::AglEstimator>>("mrs_uav_managers", "mrs_uav_managers::AglEstimator");

    // load the estimator parameters
    std::string address;
    param_loader.loadParam(est_alt_agl_name_ + "/address", address);

    try {
      RCLCPP_INFO(node_->get_logger(), "loading the estimator '%s'", address.c_str());
      est_alt_agl_ = agl_estimator_loader_->createSharedInstance(address.c_str());
    }
    catch (pluginlib::CreateClassException &ex1) {
      RCLCPP_ERROR(node_->get_logger(), "CreateClassException for the estimator '%s'", address.c_str());
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex1.what());
      rclcpp::shutdown();
      exit(1);
    }
    catch (pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(node_->get_logger(), "PluginlibException for the estimator '%s'", address.c_str());
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex.what());
      rclcpp::shutdown();
      exit(1);
    }
  }
  /*//}*/

  RCLCPP_INFO(node_->get_logger(), "estimators were loaded");
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
    RCLCPP_ERROR(node_->get_logger(), "initial estimator %s could not be found among loaded estimators. shutting down", initial_estimator_name_.c_str());
    rclcpp::shutdown();
    exit(1);
  }
  /*//}*/

  /*//{ initialize estimators */
  for (auto estimator : estimator_list_) {
    rclcpp::Node::SharedPtr subnode = node_->create_sub_node(estimator->getName());

    // create private handlers
    std::shared_ptr<mrs_uav_managers::estimation_manager::PrivateHandlers_t> ph = std::make_shared<mrs_uav_managers::estimation_manager::PrivateHandlers_t>();

    ph->loadConfigFile = std::bind(&EstimationManager::loadConfigFile, this, std::placeholders::_1);
    ph->param_loader   = std::make_unique<mrs_lib::ParamLoader>(subnode, estimator->getName());
    ph->param_loader->copyYamls(param_loader);
    ph->param_loader->setPrefix(ch_->package_name + "/" + Support::toSnakeCase(ch_->nodelet_name) + "/");

    try {
      RCLCPP_INFO(node_->get_logger(), "initializing the estimator '%s'", estimator->getName().c_str());
      estimator->initialize(subnode, ch_, ph);
    }
    catch (std::runtime_error &ex) {
      RCLCPP_ERROR(node_->get_logger(), "exception caught during estimator initialization: '%s'", ex.what());
      rclcpp::shutdown();
      exit(1);
    }

    if (!estimator->isCompatibleWithHwApi(hw_api_capabilities)) {
      RCLCPP_ERROR(node_->get_logger(), "estimator %s is not compatible with the hw api. Shutting down.", estimator->getName().c_str());
      rclcpp::shutdown();
      exit(1);
    }
  }

  // | ----------- agl height estimator initialization ---------- |
  if (is_using_agl_estimator_) {
    rclcpp::Node::SharedPtr subnode = node_->create_sub_node(est_alt_agl_->getName());

    std::shared_ptr<mrs_uav_managers::estimation_manager::PrivateHandlers_t> ph = std::make_shared<mrs_uav_managers::estimation_manager::PrivateHandlers_t>();

    ph->loadConfigFile = std::bind(&EstimationManager::loadConfigFile, this, std::placeholders::_1);
    ph->param_loader   = std::make_unique<mrs_lib::ParamLoader>(subnode, est_alt_agl_->getName());
    ph->param_loader->copyYamls(param_loader);
    ph->param_loader->setPrefix(ch_->package_name + "/" + Support::toSnakeCase(ch_->nodelet_name) + "/");

    try {
      RCLCPP_INFO(node_->get_logger(), "initializing the estimator '%s'", est_alt_agl_->getName().c_str());
      est_alt_agl_->initialize(subnode, ch_, ph);
    }
    catch (std::runtime_error &ex) {
      RCLCPP_ERROR(node_->get_logger(), "exception caught during estimator initialization: '%s'", ex.what());
      rclcpp::shutdown();
      exit(1);
    }

    if (!est_alt_agl_->isCompatibleWithHwApi(hw_api_capabilities)) {
      RCLCPP_ERROR(node_->get_logger(), "estimator %s is not compatible with the hw api. Shutting down.", est_alt_agl_->getName().c_str());
      rclcpp::shutdown();
      exit(1);
    }
  }

  RCLCPP_INFO(node_->get_logger(), "estimators were initialized");

  /*//}*/

  /*//{ initialize publishers */
  ph_uav_state_    = mrs_lib::PublisherHandler<mrs_msgs::msg::UavState>(node_, "~/uav_state_out");
  ph_odom_main_    = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(node_, "~/odom_main_out");
  ph_innovation_   = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(node_, "~/innovation_out");
  ph_diagnostics_  = mrs_lib::PublisherHandler<mrs_msgs::msg::EstimationDiagnostics>(node_, "~/diagnostics_out");
  ph_max_flight_z_ = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>(node_, "~/max_flight_z_agl_out");
  ph_altitude_agl_ = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>(node_, "~/height_agl_out");

  /*//}*/

  /*//{ initialize timers */

  mrs_lib::TimerHandlerOptions opts;

  opts.node           = node_;
  opts.autostart      = true;
  opts.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fcn = std::bind(&EstimationManager::timerPublish, this);

    timer_publish_ = std::make_shared<TimerType>(opts, rclcpp::Rate(ch_->desired_uav_state_rate, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&EstimationManager::timerPublishDiagnostics, this);

    timer_publish_diagnostics_ = std::make_shared<TimerType>(opts, rclcpp::Rate(ch_->desired_diagnostics_rate, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&EstimationManager::timerCheckHealth, this);

    timer_check_health_ = std::make_shared<TimerType>(opts, rclcpp::Rate(ch_->desired_uav_state_rate, clock_), callback_fcn);
  }

  /*//}*/

  /*//{ initialize service clients */

  srvch_failsafe_                   = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/failsafe_out", cbkgrp_sc_);
  srvch_set_world_origin_           = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "~/set_world_origin_out", cbkgrp_sc_);
  srvch_update_sa_mgr_world_origin_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "~/update_world_origin_out", cbkgrp_sc_);

  /*//}*/

  /*//{ initialize service servers */

  srvs_change_estimator_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::String>(
      node_, "~/change_estimator_in", std::bind(&EstimationManager::callbackChangeEstimator, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  srvs_reset_estimator_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::String>(
      node_, "~/reset_estimator_in", std::bind(&EstimationManager::callbackResetEstimator, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  srvs_set_world_origin_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv>(
      node_, "~/set_world_origin_in", std::bind(&EstimationManager::callbackSetWorldOrigin, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  srvs_toggle_callbacks_ = mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>(
      node_, "~/toggle_service_callbacks_in", std::bind(&EstimationManager::callbackToggleServiceCallbacks, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  /*//}*/

  /*//{ initialize scope timer */

  param_loader.loadParam("scope_timer/enabled", ch_->scope_timer.enabled);
  std::string       filepath;
  const std::string time_logger_filepath = ament_index_cpp::get_package_share_directory(package_name_) + "/scope_timer/scope_timer.txt";
  ch_->scope_timer.logger                = std::make_shared<mrs_lib::ScopeTimerLogger>(node_, time_logger_filepath, ch_->scope_timer.enabled);

  /*//}*/

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all non-optional parameters. Shutting down.");
    rclcpp::shutdown();
    exit(1);
  }

  sm_->changeState(StateMachine::INITIALIZED_STATE);

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | --------------------- timer callbacks -------------------- |

/* timerPublish() //{ */

void EstimationManager::timerPublish() {

  if (!sm_->isInitialized()) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer(node_, "EstimationManager::timerPublish", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  if (sm_->isInState(StateMachine::ESTIMATOR_SWITCHING_STATE)) {
    RCLCPP_WARN(node_->get_logger(), "Not publishing during estimator switching.");
    return;
  }

  if (!sm_->isInPublishableState()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "not publishing uav state in %s", sm_->getCurrentStateString().c_str());
    return;
  }

  mrs_msgs::msg::UavState uav_state;
  auto                    ret = active_estimator_->getUavState();
  if (ret) {
    uav_state = ret.value();
  } else {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "Active estimator did not provide uav_state.");
    return;
  }

  if (!Support::noNans(uav_state.pose.orientation)) {
    RCLCPP_ERROR(node_->get_logger(), "NaN in uav state orientation");
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
  nav_msgs::msg::Odometry odom_main = Support::uavStateToOdom(uav_state);

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

  nav_msgs::msg::Odometry innovation = active_estimator_->getInnovation();
  ph_innovation_.publish(innovation);


  mrs_msgs::msg::Float64Stamped agl_height;

  if (is_using_agl_estimator_) {
    agl_height = est_alt_agl_->getUavAglHeight();
    ph_altitude_agl_.publish(agl_height);
  }
}

//}

/* timerPublishDiagnostics() //{ */

void EstimationManager::timerPublishDiagnostics() {

  if (!sm_->isInitialized()) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer(node_, "EstimationManager::timerPublishDiagnostics", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  if (sm_->isInState(StateMachine::ESTIMATOR_SWITCHING_STATE)) {
    RCLCPP_WARN(node_->get_logger(), "Not publishing diagnostics during estimator switching.");
    return;
  }

  if (!sm_->isInPublishableState()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "not publishing uav state in %s", sm_->getCurrentStateString().c_str());
    return;
  }

  mrs_msgs::msg::UavState uav_state;
  auto                    ret = active_estimator_->getUavState();
  if (ret) {
    uav_state = ret.value();
  } else {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "Active estimator did not provide uav_state.");
    return;
  }

  if (!Support::noNans(uav_state.pose.orientation)) {
    RCLCPP_ERROR(node_->get_logger(), "NaN in uav state orientation");
    return;
  }

  mrs_msgs::msg::Float64Stamped agl_height;

  if (is_using_agl_estimator_) {
    agl_height = est_alt_agl_->getUavAglHeight();
    ph_altitude_agl_.publish(agl_height);
  }

  mrs_msgs::msg::Float64Stamped max_flight_z_msg;
  max_flight_z_msg.header.stamp = clock_->now();
  if (active_estimator_ && active_estimator_->isInitialized()) {
    max_flight_z_msg.header.frame_id = active_estimator_->getFrameId();
    max_flight_z_msg.value           = active_estimator_->getMaxFlightZ();
  }
  max_flight_z_ = max_flight_z_msg.value;
  ph_max_flight_z_.publish(max_flight_z_msg);

  // publish diagnostics
  mrs_msgs::msg::EstimationDiagnostics diagnostics;

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

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 5000, "%s. pos: [%.2f, %.2f, %.2f] m. Estimator: %s. Max. z.: %.2f m. Estimator switches: %d.",
                       sm_->getCurrentStateString().c_str(), uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z,
                       active_estimator_->getName().c_str(), max_flight_z_, estimator_switch_count_);
}

//}

/* timerCheckHealth() //{ */

void EstimationManager::timerCheckHealth() {

  if (!sm_->isInitialized()) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer(node_, "EstimationManager::timerCheckHealth", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  /*//{ start ready estimators, check switchable estimators */

  std::vector<std::string> switchable_estimator_names;
  for (auto estimator : estimator_list_) {

    if (estimator->isReady()) {
      try {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "starting the estimator '%s'", estimator->getName().c_str());
        estimator->start();
      }
      catch (std::runtime_error &ex) {
        RCLCPP_ERROR(node_->get_logger(), "exception caught during estimator starting: '%s'", ex.what());
        rclcpp::shutdown();
        exit(1);
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
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "activating the initial estimator %s", initial_estimator_->getName().c_str());
    active_estimator_ = initial_estimator_;
    active_estimator_->setActive(true);
    if (active_estimator_->getName() == "dummy") {
      sm_->changeState(StateMachine::DUMMY_STATE);
    } else {
      if (!is_using_agl_estimator_ || est_alt_agl_->isRunning()) {
        sm_->changeState(StateMachine::READY_FOR_FLIGHT_STATE);
      } else {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "%s agl estimator: %s to be running", Support::waiting_for_string.c_str(),
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
    } else { // cannot switch to healthy estimator - failsafe necessary
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "Cannot switch to any healthy estimator. Triggering failsafe.");
      sm_->changeState(StateMachine::FAILSAFE_STATE);
    }
  }

  if (sm_->isInState(StateMachine::FAILSAFE_STATE)) {
    if (!failsafe_call_succeeded_ && callFailsafeService()) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "failsafe called successfully");
      failsafe_call_succeeded_ = true;
    }
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "we are in failsafe state");
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
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000,
                           "not received control input since starting EstimationManager, estimation suboptimal, potentially unstable");
    } else if ((clock_->now() - sh_control_input_.lastMsgTime()).seconds() > 0.1) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "not received control input for %.4fs, estimation suboptimal, potentially unstable",
                           (clock_->now() - sh_control_input_.lastMsgTime()).seconds());
    }
  }
}

//}

/* shutdown() //{ */

void EstimationManager::shutdown() {

  std::cout << "EstimationManager: shutdown(): called" << std::endl;

  timer_check_health_->stop();
  timer_publish_->stop();
  timer_publish_diagnostics_->stop();

  std::cout << "EstimationManager: unloading estimators" << std::endl;

  for (int i = 0; i < int(estimator_list_.size()); i++) {

    std::cout << "EstimationManager: stopping " << estimator_names_[i] << std::endl;

    estimator_list_.at(i)->pause();

    std::cout << "EstimationManager: reseting pointer to " << estimator_names_[i] << std::endl;

    estimator_list_.at(i).reset();

    std::cout << "EstimationManager: pointer to " << estimator_names_[i] << " was reset" << std::endl;
  }

  initial_estimator_.reset();
  active_estimator_.reset();

  std::cout << "EstimationManager: unloading agl estimator" << std::endl;

  if (est_alt_agl_) {

    std::cout << "EstimationManager: stopping agl estimator" << std::endl;

    est_alt_agl_->pause();

    std::cout << "EstimationManager: reseting pointer to agl estimator" << std::endl;

    est_alt_agl_.reset();

    std::cout << "EstimationManager: pointer to agl estimator was reset" << std::endl;
  }

  std::cout << "EstimationManager: shutdown() finished" << std::endl;
}

//}

// | -------------------- service callbacks ------------------- |

/* callbackChangeEstimator() //{ */

bool EstimationManager::callbackChangeEstimator(const std::shared_ptr<mrs_msgs::srv::String::Request>  request,
                                                const std::shared_ptr<mrs_msgs::srv::String::Response> response) {

  if (!sm_->isInitialized()) {
    return false;
  }

  // enable switching out from vins_kickoff estimator during takeoff
  if (!callbacks_enabled_ && active_estimator_->getName() != "vins_kickoff") {
    response->success = false;
    response->message = ("Service callbacks are disabled");
    RCLCPP_WARN(node_->get_logger(), "Ignoring service call. Callbacks are disabled.");
    return true;
  }

  // switching into these estimators during flight is dangerous with realhw, so we don't allow it
  if (request->value == "dummy" || request->value == "ground_truth" || request->value == "vins_kickoff") {
    response->success = false;
    std::stringstream ss;
    ss << "Switching to " << request->value << " estimator is not allowed.";
    response->message = ss.str();
    RCLCPP_WARN(node_->get_logger(), "Switching to %s estimator is not allowed.", request->value.c_str());
    return true;
  }

  // we do not want the switch to be disturbed by a service call
  callbacks_enabled_ = false;

  bool                                              target_estimator_found = false;
  std::shared_ptr<mrs_uav_managers::StateEstimator> target_estimator;
  for (auto estimator : estimator_list_) {
    if (estimator->getName() == request->value) {
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
      RCLCPP_WARN(node_->get_logger(), "Switch to not running estimator %s requested", request->value.c_str());
      response->success = false;
      response->message = ("Requested estimator is not running");
      return true;
    }

  } else {
    RCLCPP_WARN(node_->get_logger(), "Switch to invalid estimator %s requested", request->value.c_str());
    response->success = false;
    response->message = ("Not a valid estimator type");
    return true;
  }

  response->success = true;
  response->message = "Estimator switch successful";

  // allow service calllbacks after switch again
  callbacks_enabled_ = true;

  return true;
}

//}

/* callbackResetEstimator() //{ */

bool EstimationManager::callbackResetEstimator(const std::shared_ptr<mrs_msgs::srv::String::Request>  request,
                                               const std::shared_ptr<mrs_msgs::srv::String::Response> response) {

  if (!sm_->isInitialized()) {
    return false;
  }

  if (!callbacks_enabled_) {
    response->success = false;
    response->message = ("Service callbacks are disabled");
    RCLCPP_WARN(node_->get_logger(), "Ignoring service call. Callbacks are disabled.");
    return true;
  }

  bool                                              target_estimator_found = false;
  std::shared_ptr<mrs_uav_managers::StateEstimator> target_estimator;
  for (auto estimator : estimator_list_) {
    if (estimator->getName() == request->value) {
      target_estimator       = estimator;
      target_estimator_found = true;
      break;
    }
  }

  if (target_estimator_found) {

    if (target_estimator->getName() == active_estimator_->getName()) {
      response->success = false;
      response->message = ("Cannot reset active estimator");
      RCLCPP_WARN(node_->get_logger(), "Ignoring service call. Cannot reset active estimator.");
      return true;
    }

    target_estimator->reset();
    RCLCPP_INFO(node_->get_logger(), "Estimator %s reset", target_estimator->getName().c_str());

    double t_wait_left = 5;
    while (t_wait_left > 0) {
      RCLCPP_INFO(node_->get_logger(), "Attempting starting %s estimator", target_estimator->getName().c_str());
      target_estimator->start();

      if (target_estimator->isRunning()) {
        RCLCPP_INFO(node_->get_logger(), "Reset of %s estimator successful", target_estimator->getName().c_str());
        break;
      }

      const double start_period = 1.0;
      clock_->sleep_for(std::chrono::duration<double>(start_period));
      t_wait_left -= start_period;
    }

  } else {
    RCLCPP_WARN(node_->get_logger(), "Reset of invalid estimator %s requested", request->value.c_str());
    response->success = false;
    response->message = ("Not a valid estimator type");
    return true;
  }

  response->success = true;
  response->message = "Estimator reset successful";

  return true;
}

/*//{ callbackSetWorldOrigin() */
bool EstimationManager::callbackSetWorldOrigin(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request>  request,
                                               const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response) {

  if (!sm_->isInitialized()) {
    return false;
  }

  if (!callbacks_enabled_) {
    response->success = false;
    response->message = ("Service callbacks are disabled");
    RCLCPP_WARN(node_->get_logger(), "[%s]: Ignoring service call. Callbacks are disabled.", getName().c_str());
    return true;
  }

  if (sm_->isInState(StateMachine::INITIALIZED_STATE) || sm_->isInState(StateMachine::READY_FOR_FLIGHT_STATE)) {

    double world_origin_x, world_origin_y;

    if (request->header.frame_id.find("latlon_origin") != std::string::npos) {
      const double lat = request->reference.position.x;
      const double lon = request->reference.position.y;
      mrs_lib::UTM(lat, lon, &world_origin_x, &world_origin_y);

      RCLCPP_INFO(node_->get_logger(), "Setting world origin to lat: %.6f lon: %.6f by service callback", request->reference.position.x,
                  request->reference.position.y);

    } else if (request->header.frame_id.find("utm_origin") != std::string::npos) {
      world_origin_x = request->reference.position.x;
      world_origin_y = request->reference.position.y;

      RCLCPP_INFO(node_->get_logger(), "Setting world origin to x: %.2f y: %.2f UTM by service callback", request->reference.position.x,
                  request->reference.position.y);

    } else {
      RCLCPP_INFO(node_->get_logger(), "Requested unsupported frame_id: \"%s\" in set_world_origin service. Supported are: latlon_origin, utm_origin",
                  request->header.frame_id.c_str());
      response->success = false;
      response->message = "Requested unsupported frame_id. Supported are: latlon_origin, utm_origin";
      return true;
    }

    ch_->world_origin.x = world_origin_x;
    ch_->world_origin.y = world_origin_y;

    for (auto estimator : estimator_list_) {

      estimator->reset();
      RCLCPP_INFO(node_->get_logger(), "Estimator %s reset", estimator->getName().c_str());

      double t_wait_left = 5;
      while (t_wait_left > 0) {
        RCLCPP_INFO(node_->get_logger(), "Attempting starting %s estimator", estimator->getName().c_str());
        estimator->start();

        if (estimator->isRunning()) {
          RCLCPP_INFO(node_->get_logger(), "Reset of %s estimator successful", estimator->getName().c_str());
          break;
        }

        const double start_period = 0.2;
        clock_->sleep_for(std::chrono::duration<double>(start_period));
        t_wait_left -= start_period;
      }
    }

    auto res = srvch_set_world_origin_.callSync(request);

    if (!res.has_value() || !res.value()->success) {
      RCLCPP_WARN(node_->get_logger(), "Could not call TransformManager set_world_origin service.");
      response->success = false;
      response->message = "Could not call TransformManager set_world_origin service.";
      return true;
    }

    if (!res.value()->success) {
      RCLCPP_WARN(node_->get_logger(), "TransformManager could not set world origin.");
      response->success = false;
      response->message = "TransformManager could not set world origin.";
      return true;
    }

    // update Safety Area manager world origin
    auto res_update = srvch_update_sa_mgr_world_origin_.callSync(request);
    if (!res_update.has_value() || !res_update.value()->success) {
      RCLCPP_WARN(node_->get_logger(), "Could not call Safety Area Manager update_world_origin service.");
      response->success = false;
      response->message = "Could not call Safety Area Manager update_world_origin service.";
      return true;
    }

    if (!res_update.value()->success) {
      RCLCPP_WARN(node_->get_logger(), "Safety Area Manager could not update world origin.");
      response->success = false;
      response->message = "SA Manager could not update world origin.";
      return true;
    }

    response->success = true;
    response->message = "World origin set successfully";

  } else {

    response->success = false;
    response->message = "Cannot set world origin while flying";
  }


  return true;
}
/*//}*/

/* //{ callbackToggleServiceCallbacks() */
bool EstimationManager::callbackToggleServiceCallbacks(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                                                       const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  if (!sm_->isInitialized()) {
    RCLCPP_ERROR(node_->get_logger(), "service for toggling callbacks is not available before initialization.");
    return false;
  }

  callbacks_disabled_by_service_ = !request->data;

  response->success = true;
  response->message = (callbacks_disabled_by_service_ ? "Service callbacks disabled" : "Service callbacks enabled");

  if (callbacks_disabled_by_service_) {

    RCLCPP_INFO(node_->get_logger(), "Service callbacks disabled.");

  } else {

    RCLCPP_INFO(node_->get_logger(), "Service callbacks enabled");
  }

  return true;
}

//}

// --------------------------------------------------------------
// |                        other methods                       |
// --------------------------------------------------------------

/* switchToHealthyEstimator() //{ */

bool EstimationManager::switchToHealthyEstimator() {

  // available estimators should be specified in decreasing priority order in config file
  for (auto estimator : estimator_list_) {
    if (estimator->isRunning()) {
      switchToEstimator(estimator);
      return true;
    }
  }

  return false; // no other estimator is running
}

//}

/* switchToEstimator() //{ */

void EstimationManager::switchToEstimator(const std::shared_ptr<mrs_uav_managers::StateEstimator> &target_estimator) {

  std::scoped_lock lock(mutex_active_estimator_);

  RCLCPP_INFO(node_->get_logger(), "switching estimator from %s to %s", active_estimator_->getName().c_str(), target_estimator->getName().c_str());

  active_estimator_->setActive(false);
  active_estimator_ = target_estimator;
  active_estimator_->setActive(true);
  estimator_switch_count_++;
}

//}

/* callFailsafeService() //{ */

bool EstimationManager::callFailsafeService() {

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto response = srvch_failsafe_.callSync(request);

  if (!response.has_value() || !response.value()->success) {
    return false;
  }

  return true;
}

//}

/* getName() //{ */

std::string EstimationManager::getName() const {
  return nodelet_name_;
}

//}

/* loadConfigFile() //{ */

bool EstimationManager::loadConfigFile(const std::string &file_path) {

  const std::string name_space = std::string(node_->get_namespace()) + "/";

  RCLCPP_INFO(node_->get_logger(), "'%s' under the namespace '%s'", file_path.c_str(), name_space.c_str());

  // load the user-requested file
  {
    std::string command = "rosparam load " + file_path + " " + name_space;
    int         result  = std::system(command.c_str());

    if (result != 0) {
      RCLCPP_ERROR(node_->get_logger(), "failed to load '%s'", file_path.c_str());
      return false;
    }
  }

  // load the platform config
  if (_platform_config_ != "") {
    std::string command = "rosparam load " + _platform_config_ + " " + name_space;
    int         result  = std::system(command.c_str());

    if (result != 0) {
      RCLCPP_ERROR(node_->get_logger(), "failed to load the platform config file '%s'", _platform_config_.c_str());
      return false;
    }
  }

  // load the custom config
  if (_custom_config_ != "") {
    std::string command = "rosparam load " + _custom_config_ + " " + name_space;
    int         result  = std::system(command.c_str());

    if (result != 0) {
      RCLCPP_ERROR(node_->get_logger(), "failed to load the custom config file '%s'", _custom_config_.c_str());
      return false;
    }
  }

  return true;
}

//}

} // namespace estimation_manager

} // namespace mrs_uav_managers

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_managers::estimation_manager::EstimationManager)
