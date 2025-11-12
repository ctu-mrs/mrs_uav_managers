#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

#include <mrs_msgs/msg/obstacle_sectors.hpp>

#include <mrs_lib/mutex.h>

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester();

  mrs_lib::PublisherHandler<mrs_msgs::msg::ObstacleSectors> ph_bumper_;

  std::shared_ptr<TimerType> timer_bumper_;
  void                       timerBumper();

  mrs_msgs::msg::ObstacleSectors bumper_data_;
  std::mutex                     mutex_bumper_data_;

  Eigen::Vector3d getBodyVelocity();

  void setBumperFrontSector(const double distance);

  bool test(void);

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh_;

  const std::string _uav_name_ = "uav1";
};

/* Tester() //{ */

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  ph_bumper_ = mrs_lib::PublisherHandler<mrs_msgs::msg::ObstacleSectors>(node_, "/" + _uav_name_ + "/bumper/obstacle_sectors");

  bumper_data_.n_horizontal_sectors = 8;
  bumper_data_.sectors_vertical_fov = 20;

  for (unsigned int i = 0; i < bumper_data_.n_horizontal_sectors + 2; i++) {
    bumper_data_.sectors.push_back(10.0);
  }

  std::function<void()> callback_fcn = std::bind(&Tester::timerBumper, this);

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node      = node_;
  timer_opts_start.autostart = true;

  timer_bumper_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(100.0, clock_), callback_fcn);
}

//}

/* setBumperFrontSector() //{ */

void Tester::setBumperFrontSector(const double distance) {

  std::scoped_lock lock(mutex_bumper_data_);

  bumper_data_.sectors[0] = distance;
}

//}

/* getBodyVelocity() //{ */

Eigen::Vector3d Tester::getBodyVelocity() {

  geometry_msgs::msg::Vector3Stamped vel;

  auto uav_state = uh_->sh_uav_state_.getMsg();

  vel.header = uav_state->header;
  vel.vector = uav_state->velocity.linear;

  auto result = this->transformer_->transformSingle(vel, _uav_name_ + "/fcu_untilted");

  if (result) {
    return Eigen::Vector3d(result->vector.x, result->vector.y, result->vector.z);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "could not transform body velocity");
    return Eigen::Vector3d(0, 0, 0);
  }
}

//}

/* timerBumper() //{ */

void Tester::timerBumper() {

  auto bumper_data = mrs_lib::get_mutexed(mutex_bumper_data_, bumper_data_);

  ph_bumper_.publish(bumper_data);
}

//}

bool Tester::test(void) {

  {
    auto [uhopt, message] = getUAVHandler(_uav_name_);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", _uav_name_.c_str(), message.c_str());
      return false;
    }

    uh_ = uhopt.value();
  }

  {
    auto [success, message] = uh_->activateMidAir();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "midair activation failed with message: '%s'", message.c_str());
      return false;
    }
  }

  this->sleep(2.0);

  RCLCPP_INFO(node_->get_logger(), "set the bumper front sector to 'close obstacle'");

  setBumperFrontSector(0.5);

  this->sleep(1.0);

  {
    auto ctrl_diag = uh_->sh_control_manager_diag_.getMsg();

    if (!(!ctrl_diag->flying_normally && ctrl_diag->bumper_active)) {
      RCLCPP_ERROR(node_->get_logger(), "missing the signs of the bumper being active");
      return false;
    }
  }

  this->sleep(5.0);

  {
    auto body_vel = getBodyVelocity();

    if (!(body_vel[0] < -2.0 && abs(body_vel[1]) < 0.5 && abs(body_vel[2]) < 0.5)) {
      RCLCPP_ERROR(node_->get_logger(), "body velocity is not suggesting that we are moving backwards (%.2f, %.2f, %.2f)", body_vel[0], body_vel[1],
                   body_vel[2]);
      return false;
    }
  }

  RCLCPP_INFO(node_->get_logger(), "set the bumper front sector to 'none obstacle'");

  setBumperFrontSector(10.0);

  RCLCPP_INFO(node_->get_logger(), "waiting for normal flight conditions");

  while (true) {

    auto ctrl_diag = uh_->sh_control_manager_diag_.getMsg();

    if (!rclcpp::ok()) {
      return false;
    }

    if (ctrl_diag->flying_normally && !ctrl_diag->bumper_active) {
      break;
    }
  }

  this->sleep(1.0);

  RCLCPP_INFO(node_->get_logger(), "testing goto");

  uh_->gotoRel(10, 0, 0, 0);

  RCLCPP_INFO(node_->get_logger(), "goto finished");

  if (uh_->isFlyingNormally()) {
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "not flying normally");
    return false;
  }
}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Tester tester;

  test_result &= tester.test();

  tester.sleep(2.0);

  std::cout << "Test: reporting test results" << std::endl;

  tester.reportTestResult(test_result);

  tester.join();
}
