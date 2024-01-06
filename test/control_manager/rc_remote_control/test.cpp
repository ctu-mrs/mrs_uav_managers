#include <gtest/gtest.h>

#include <remote_control_test.h>

#include <mrs_msgs/HwApiRcChannels.h>

#define AIL 0
#define THR 1
#define ELE 2
#define RUD 3

#define ACTIVATION 6

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

  mrs_lib::PublisherHandler<mrs_msgs::HwApiRcChannels> ph_rc_channels_;

  ros::Timer timer_rc_;
  void       timerRc(const ros::TimerEvent& event);

  mrs_msgs::HwApiRcChannels rc_;
  std::mutex                mutex_rc_;
};

Tester::Tester() {

  ph_rc_channels_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiRcChannels>(nh_, "/" + _uav_name_ + "/hw_api/rc_channels");

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

  timer_rc_ = nh_.createTimer(ros::Rate(100.0), &Tester::timerRc, this, false, true);
}

void Tester::timerRc([[maybe_unused]] const ros::TimerEvent& event) {

  {
    std::scoped_lock lock(mutex_rc_);

    ph_rc_channels_.publish(rc_);
  }
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
