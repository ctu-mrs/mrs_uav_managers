#include <gtest/gtest.h>

#include <escalating_failsafe_test.h>

#include <mrs_msgs/HwApiRcChannels.h>

class Tester : public EscalatingFailsafeTest {

public:
  Tester();

  mrs_lib::PublisherHandler<mrs_msgs::HwApiRcChannels> ph_rc_channels_;

  std::optional<std::tuple<bool, std::string>> escalatingFailsafe();

  ros::Timer timer_rc_;
  void       timerRc(const ros::TimerEvent& event);

  mrs_msgs::HwApiRcChannels rc_;
  std::mutex                mutex_rc_;
};

Tester::Tester() {

  ph_rc_channels_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiRcChannels>(nh_, "/" + _uav_name_ + "/hw_api/rc_channels");

  rc_.channels.push_back(0.0);
  rc_.channels.push_back(0.0);
  rc_.channels.push_back(0.0);
  rc_.channels.push_back(0.0);
  rc_.channels.push_back(0.0);
  rc_.channels.push_back(0.0);
  rc_.channels.push_back(0.0);
  rc_.channels.push_back(0.0);
  rc_.channels.push_back(0.0);
  rc_.channels.push_back(0.0);

  timer_rc_ = nh_.createTimer(ros::Rate(100.0), &Tester::timerRc, this, false, true);
}

void Tester::timerRc([[maybe_unused]] const ros::TimerEvent& event) {

  {
    std::scoped_lock lock(mutex_rc_);

    ph_rc_channels_.publish(rc_);
  }
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

TEST(TESTSuite, test) {

  Tester tester;

  bool result = tester.test();

  if (result) {
    GTEST_SUCCEED();
  } else {
    GTEST_FAIL();
  }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "test");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
