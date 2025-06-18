#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <remote_control_test.h>

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

class Tester : public RemoteControlTest {

public:
  Tester();

  void activate();
  void deactivate();

  void moveForward();
  void moveBackward();
  void moveLeft();
  void moveRight();
  void moveUp();
  void moveDown();
  void rotateLeft();
  void rotateRight();

  void stop();

  bool getControllerDynamics(double &horizontal_speed, double &vertical_speed, double &heading_rate);

  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiRcChannels> ph_rc_channels_;

  std::shared_ptr<TimerType> timer_rc_;
  void                       timerRc();

  mrs_msgs::msg::HwApiRcChannels rc_;
  std::mutex                     mutex_rc_;
};

Tester::Tester() {

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

  std::function<void()> callback_fcn = std::bind(&Tester::timerRc, this);

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node      = node_;
  timer_opts_start.autostart = true;

  timer_rc_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(100.0, clock_), callback_fcn);
}

void Tester::timerRc() {

  {
    std::scoped_lock lock(mutex_rc_);

    ph_rc_channels_.publish(rc_);
  }
}

bool Tester::getControllerDynamics(double &horizontal_speed, double &vertical_speed, double &heading_rate) {

  pl_->addYamlFileFromParam("control_manager_config");

  pl_->loadParam("mrs_uav_managers/control_manager/rc_joystick/horizontal_speed", horizontal_speed);
  pl_->loadParam("mrs_uav_managers/control_manager/rc_joystick/vertical_speed", vertical_speed);
  pl_->loadParam("mrs_uav_managers/control_manager/rc_joystick/heading_rate", heading_rate);

  if (!pl_->loadedSuccessfully()) {
    return false;
  }

  return true;
}

void Tester::activate() {

  {
    std::scoped_lock lock(mutex_rc_);

    rc_.channels[ACTIVATION] = 1.0;
  }
}

void Tester::deactivate() {

  {
    std::scoped_lock lock(mutex_rc_);

    rc_.channels[ACTIVATION] = 0.0;
  }
}

void Tester::moveForward() {

  {
    std::scoped_lock lock(mutex_rc_);

    rc_.channels[ELE] = 1.0;
    rc_.channels[AIL] = 0.5;
    rc_.channels[RUD] = 0.5;
    rc_.channels[THR] = 0.5;
  }
}

void Tester::moveBackward() {

  {
    std::scoped_lock lock(mutex_rc_);

    rc_.channels[ELE] = 0.0;
    rc_.channels[AIL] = 0.5;
    rc_.channels[RUD] = 0.5;
    rc_.channels[THR] = 0.5;
  }
}

void Tester::moveLeft() {

  {
    std::scoped_lock lock(mutex_rc_);

    rc_.channels[ELE] = 0.5;
    rc_.channels[AIL] = 0.0;
    rc_.channels[RUD] = 0.5;
    rc_.channels[THR] = 0.5;
  }
}

void Tester::moveRight() {

  {
    std::scoped_lock lock(mutex_rc_);

    rc_.channels[ELE] = 0.5;
    rc_.channels[AIL] = 1.0;
    rc_.channels[RUD] = 0.5;
    rc_.channels[THR] = 0.5;
  }
}

void Tester::moveUp() {

  {
    std::scoped_lock lock(mutex_rc_);

    rc_.channels[ELE] = 0.5;
    rc_.channels[AIL] = 0.5;
    rc_.channels[RUD] = 0.5;
    rc_.channels[THR] = 1.0;
  }
}

void Tester::moveDown() {

  {
    std::scoped_lock lock(mutex_rc_);

    rc_.channels[ELE] = 0.5;
    rc_.channels[AIL] = 0.5;
    rc_.channels[RUD] = 0.5;
    rc_.channels[THR] = 0.0;
  }
}

void Tester::rotateLeft() {

  {
    std::scoped_lock lock(mutex_rc_);

    rc_.channels[ELE] = 0.5;
    rc_.channels[AIL] = 0.5;
    rc_.channels[RUD] = 0.0;
    rc_.channels[THR] = 0.5;
  }
}

void Tester::rotateRight() {

  {
    std::scoped_lock lock(mutex_rc_);

    rc_.channels[ELE] = 0.5;
    rc_.channels[AIL] = 0.5;
    rc_.channels[RUD] = 1.0;
    rc_.channels[THR] = 0.5;
  }
}

void Tester::stop() {

  {
    std::scoped_lock lock(mutex_rc_);

    rc_.channels[ELE] = 0.5;
    rc_.channels[AIL] = 0.5;
    rc_.channels[RUD] = 0.5;
    rc_.channels[THR] = 0.5;
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
