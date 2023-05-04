#ifndef MRS_UAV_MANAGERS_ESTIMATION_MANAGER_ESTIMATOR_H
#define MRS_UAV_MANAGERS_ESTIMATION_MANAGER_ESTIMATOR_H

/* includes //{ */

#include <ros/ros.h>

#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>

#include <mrs_msgs/UavState.h>
#include <mrs_msgs/EstimatorInput.h>
#include <mrs_msgs/EstimatorDiagnostics.h>

#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>


#include "estimation_manager/types.h"
#include "estimation_manager/support.h"
#include "estimation_manager/common_handlers.h"

//}

namespace mrs_uav_managers
{

/* using namespace estimation_manager; */

using namespace mrs_uav_managers::estimation_manager;

class Estimator {

protected:
  mutable mrs_lib::PublisherHandler<mrs_msgs::EstimatorDiagnostics> ph_diagnostics_;

  const std::string type_;
  const std::string name_;
  const std::string package_name_;

  std::string frame_id_;  // cannot be constant - must remain overridable by loaded parameter
  std::string ns_frame_id_;

  std::shared_ptr<CommonHandlers_t> ch_;

  double max_flight_z_ = -1.0;
  double pos_innovation_limit_;

private:
  SMStates_t previous_sm_state_ = UNINITIALIZED_STATE;
  SMStates_t current_sm_state_  = UNINITIALIZED_STATE;

protected:
  Estimator(const std::string &type, const std::string &name, const std::string &frame_id) : type_(type), name_(name), frame_id_(frame_id) {
  }

  virtual ~Estimator(void) {
  }

public:
  // virtual methods
  virtual void initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) = 0;
  virtual bool start(void)                                                                  = 0;
  virtual bool pause(void)                                                                  = 0;
  virtual bool reset(void)                                                                  = 0;

  // implemented methods
  // access methods
  std::string getName(void) const;
  std::string getPrintName(void) const;
  std::string getType(void) const;
  std::string getFrameId(void) const;
  double      getMaxFlightZ(void) const;
  std::string getSmStateString(const SMStates_t &state) const;
  std::string getCurrentSmStateString(void) const;
  SMStates_t  getCurrentSmState() const;

  // state machine methods
  bool changeState(SMStates_t new_state);
  bool isInState(const SMStates_t &state_in) const;
  bool isInitialized() const;
  bool isReady() const;
  bool isStarted() const;
  bool isRunning() const;
  bool isStopped() const;
  bool isError() const;

  void publishDiagnostics() const;

  tf2::Vector3          getAccGlobal(const sensor_msgs::Imu::ConstPtr &input_msg, const double hdg);
  tf2::Vector3          getAccGlobal(const mrs_msgs::EstimatorInput::ConstPtr &input_msg, const double hdg);
  tf2::Vector3          getAccGlobal(const geometry_msgs::Vector3Stamped &acc_stamped, const double hdg);
  std::optional<double> getHeadingRate(const nav_msgs::OdometryConstPtr &odom_msg);
};

}  // namespace mrs_uav_managers

#endif  // MRS_UAV_MANAGERS_ESTIMATION_MANAGER_ESTIMATOR_H
