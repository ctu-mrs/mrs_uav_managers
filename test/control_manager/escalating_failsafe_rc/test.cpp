#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <escalating_failsafe_test.h>

#include <mrs_msgs/msg/hw_api_rc_channels.hpp>

using namespace std::chrono_literals;

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

class Tester : public EscalatingFailsafeTest {

public:
  Tester();

  std::optional<std::tuple<bool, std::string>> escalatingFailsafe();

  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiRcChannels> ph_rc_channels_;

  std::shared_ptr<TimerType> timer_rc_;
  void                       timerRc();

  mrs_msgs::msg::HwApiRcChannels rc_;
  std::mutex                     mutex_rc_;
};

Tester::Tester() : EscalatingFailsafeTest() {

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

std::optional<std::tuple<bool, std::string>> Tester::escalatingFailsafe() {

  {
    std::scoped_lock lock(mutex_rc_);

    rc_.channels[7] = 1.0;
  }

  sleep(0.2);

  {
    std::scoped_lock lock(mutex_rc_);

    rc_.channels[7] = 0.0;
  }

  return {};
}

void Tester::timerRc() {

  {
    std::scoped_lock lock(mutex_rc_);

    ph_rc_channels_.publish(rc_);
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
