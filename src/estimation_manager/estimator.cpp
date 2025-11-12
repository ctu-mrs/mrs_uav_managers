#include <mrs_uav_managers/estimation_manager/estimator.h>

namespace mrs_uav_managers
{

/* changeState() //{ */

bool Estimator::changeState(SMStates_t new_state) {

  if (new_state == getCurrentSmState()) {
    return true;
  }

  previous_sm_state_ = getCurrentSmState();
  setCurrentSmState(new_state);

  RCLCPP_INFO(node_->get_logger(), "[%s]: Switching sm state %s -> %s", getPrintName().c_str(), getSmStateString(previous_sm_state_).c_str(),
              getCurrentSmStateString().c_str());
  return true;
}

//}

/* isInState() //{ */

bool Estimator::isInState(const SMStates_t &state_in) const {
  return state_in == getCurrentSmState();
}

//}

/* isInitialized() //{ */

bool Estimator::isInitialized() const {
  return !isInState(UNINITIALIZED_STATE);
}

//}

/* isReady() //{ */

bool Estimator::isReady() const {
  return isInState(READY_STATE);
}

//}

/* isStarted() //{ */

bool Estimator::isStarted() const {
  return isInState(STARTED_STATE);
}

//}

/* isRunning() //{ */

bool Estimator::isRunning() const {
  return isInState(SMStates_t::RUNNING_STATE);
}

//}

/* isStopped() //{ */

bool Estimator::isStopped() const {
  return isInState(STOPPED_STATE);
}

//}

/* isError() //{ */

bool Estimator::isError() const {
  return isInState(ERROR_STATE);
}

//}

/* getCurrentSmState() //{ */

SMStates_t Estimator::getCurrentSmState() const {
  return current_sm_state_;
}

//}

/* setCurrentSmState() //{ */

void Estimator::setCurrentSmState(const SMStates_t &new_state) {
  std::scoped_lock lock(mutex_current_state_);
  current_sm_state_ = new_state;
}

//}

/* getSmStateString() //{ */

std::string Estimator::getSmStateString(const SMStates_t &state) const {
  return sm::state_names[state];
}

//}

/* getCurrentSmStateString() //{ */

std::string Estimator::getCurrentSmStateString(void) const {
  return getSmStateString(getCurrentSmState());
}

//}

/* isMitigatingJump() //{ */

bool Estimator::isMitigatingJump(void) {

  if (is_mitigating_jump_) {
    is_mitigating_jump_ = false;
    return true;
  } else {
    return false;
  }
}

//}

/* getName() //{ */

std::string Estimator::getName(void) const {
  return name_;
}

//}

/* getPrintName() //{ */

std::string Estimator::getPrintName(void) const {
  return name_;
}

//}

/* getType() //{ */

std::string Estimator::getType(void) const {
  return type_;
}

//}

/* getFrameId() //{ */

std::string Estimator::getFrameId(void) const {
  return ns_frame_id_;
}

//}

/* getMaxFlightZ() //{ */

double Estimator::getMaxFlightZ(void) const {
  return max_flight_z_;
}

//}

/* publishDiagnostics() //{ */

void Estimator::publishDiagnostics() const {

  if (!ch_->debug_topics.diag) {
    return;
  }

  mrs_msgs::msg::EstimatorDiagnostics msg;
  msg.header.stamp       = clock_->now();
  msg.header.frame_id    = getFrameId();
  msg.estimator_name     = getName();
  msg.estimator_type     = getType();
  msg.estimator_sm_state = getCurrentSmStateString();

  ph_diagnostics_.publish(msg);
}

//}

/* getAccGlobal() //{ */

tf2::Vector3 Estimator::getAccGlobal(const sensor_msgs::msg::Imu::ConstSharedPtr &input_msg, const double hdg) {

  geometry_msgs::msg::Vector3Stamped acc_stamped;
  acc_stamped.vector = input_msg->linear_acceleration;
  acc_stamped.header = input_msg->header;

  return getAccGlobal(acc_stamped, hdg);
}

tf2::Vector3 Estimator::getAccGlobal(const mrs_msgs::msg::EstimatorInput::ConstSharedPtr &input_msg, const double hdg) {

  geometry_msgs::msg::Vector3Stamped acc_stamped;
  acc_stamped.vector = input_msg->control_acceleration;
  acc_stamped.header = input_msg->header;

  return getAccGlobal(acc_stamped, hdg);
}

tf2::Vector3 Estimator::getAccGlobal(const geometry_msgs::msg::Vector3Stamped &acc_stamped, const double hdg) {

  // untilt the desired acceleration vector
  geometry_msgs::msg::Vector3Stamped des_acc;
  geometry_msgs::msg::Vector3        des_acc_untilted;
  des_acc.vector.x        = acc_stamped.vector.x;
  des_acc.vector.y        = acc_stamped.vector.y;
  des_acc.vector.z        = acc_stamped.vector.z;
  des_acc.header.frame_id = ch_->frames.ns_fcu;
  des_acc.header.stamp    = acc_stamped.header.stamp;

  auto response_acc = ch_->transformer->transformSingle(des_acc, ch_->frames.ns_fcu_untilted);

  if (response_acc) {
    des_acc_untilted.x = response_acc.value().vector.x;
    des_acc_untilted.y = response_acc.value().vector.y;
    des_acc_untilted.z = response_acc.value().vector.z;
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: transform from %s to %s failed", getPrintName().c_str(), des_acc.header.frame_id.c_str(),
                         ch_->frames.ns_fcu_untilted.c_str());
  }

  // rotate the desired acceleration vector to global frame
  const tf2::Vector3 des_acc_global = Support::rotateVecByHdg(des_acc_untilted, hdg);

  return des_acc_global;
}

//}

/* getHeadingRate() //{ */

std::optional<double> Estimator::getHeadingRate(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg) {

  double hdg_rate;

  try {
    hdg_rate = mrs_lib::AttitudeConverter(odom_msg->pose.pose.orientation).getHeadingRate(odom_msg->twist.twist.angular);
  }
  catch (...) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: failed getting heading rate", getPrintName().c_str());
    return {};
  }

  return hdg_rate;
}

//}

} // namespace mrs_uav_managers
