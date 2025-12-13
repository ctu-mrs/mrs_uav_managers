#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

#include <mrs_msgs/msg/obstacle_sectors.hpp>

#include <mrs_lib/mutex.h>

#include <mrs_msgs/msg/hw_api_rc_channels.hpp>

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

#define AIL 0
#define THR 1
#define ELE 2
#define RUD 3

#define ACTIVATION 6

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

  std::shared_ptr<TimerType> timer_rc_;
  void                       timerRc();

  mrs_msgs::msg::HwApiRcChannels rc_;
  std::mutex                     mutex_rc_;

  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiRcChannels> ph_rc_channels_;

  void activate();
  void deactivate();
  void moveForward();
};

/* activate() //{ */

void Tester::activate() {

  {
    std::scoped_lock lock(mutex_rc_);

    rc_.channels[ACTIVATION] = 1.0;
  }
}

//}

/* deactivate() //{ */

void Tester::deactivate() {

  {
    std::scoped_lock lock(mutex_rc_);

    rc_.channels[ACTIVATION] = 0.0;
  }
}

//}

/* moveForward() //{ */

void Tester::moveForward() {

  {
    std::scoped_lock lock(mutex_rc_);

    rc_.channels[ELE] = 1.0;
    rc_.channels[AIL] = 0.5;
    rc_.channels[RUD] = 0.5;
    rc_.channels[THR] = 0.5;
  }
}

//}

/* timerRc() //{ */

void Tester::timerRc() {

  {
    std::scoped_lock lock(mutex_rc_);

    ph_rc_channels_.publish(rc_);
  }
}

//}

/* Tester() //{ */

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  ph_bumper_ = mrs_lib::PublisherHandler<mrs_msgs::msg::ObstacleSectors>(node_, "/" + _uav_name_ + "/bumper/obstacle_sectors");

  bumper_data_.n_horizontal_sectors = 8;
  bumper_data_.sectors_vertical_fov = 20;

  for (unsigned int i = 0; i < bumper_data_.n_horizontal_sectors + 2; i++) {
    bumper_data_.sectors.push_back(10.0);
  }

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node      = node_;
  timer_opts_start.autostart = true;

  ph_rc_channels_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiRcChannels>(node_, "/" + _uav_name_ + "/hw_api/rc_channels");

  rc_.channels.push_back(0.5);
  rc_.channels.push_back(0.5);
  rc_.channels.push_back(0.5);
  rc_.channels.push_back(0.5);
  rc_.channels.push_back(0.0);
  rc_.channels.push_back(0.0);
  rc_.channels.push_back(0.0);
  rc_.channels.push_back(0.0);
  rc_.channels.push_back(0.0);
  rc_.channels.push_back(0.0);

  {
    std::function<void()> callback_fcn = std::bind(&Tester::timerBumper, this);

    timer_bumper_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(100.0, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&Tester::timerRc, this);

    timer_rc_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(100.0, clock_), callback_fcn);
  }
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

  sleep(2.0);

  // | -------------- activate the RC joystick mode ------------- |

  activate();

  sleep(0.5);

  {
    auto ctrl_diag = uh_->sh_control_manager_diag_.getMsg();

    if (!ctrl_diag->joystick_active) {
      RCLCPP_ERROR(node_->get_logger(), "RC joystick not active when it should");
      return false;
    }
  }

  // | --------------------- flight forward --------------------- |

  moveForward();

  sleep(5.0);

  {
    auto body_vel = getBodyVelocity();

    if (body_vel[0] < 1.0) {
      RCLCPP_ERROR(node_->get_logger(), "forward velocity too small");
      return false;
    }
  }

  // | ------------------- activate the bumper ------------------ |

  RCLCPP_INFO(node_->get_logger(), "set the bumper front sector to 'close obstacle'");

  setBumperFrontSector(0.5);

  sleep(5.0);

  {
    auto ctrl_diag = uh_->sh_control_manager_diag_.getMsg();

    if (!(!ctrl_diag->flying_normally && ctrl_diag->bumper_active)) {
      RCLCPP_ERROR(node_->get_logger(), "missing the signs of the bumper being active");
      return false;
    }
  }

  {
    auto body_vel = getBodyVelocity();

    if (body_vel[0] > 0.0) {
      RCLCPP_ERROR(node_->get_logger(), "forward velocity is positive, but it should not be");
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

    auto body_vel = getBodyVelocity();

    if (ctrl_diag->joystick_active && !ctrl_diag->bumper_active && body_vel[0] > 1.0) {
      break;
    }

    sleep(0.1);
  }

  deactivate();

  sleep(5.0);

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
