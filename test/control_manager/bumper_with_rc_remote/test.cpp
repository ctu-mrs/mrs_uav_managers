#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

#include <remote_control_test.h>

#include <mrs_msgs/ObstacleSectors.h>

#include <mrs_lib/mutex.h>

#include <mrs_msgs/HwApiRcChannels.h>

#define AIL 0
#define THR 1
#define ELE 2
#define RUD 3

#define ACTIVATION 6

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester();

  bool test();

private:
  mrs_lib::PublisherHandler<mrs_msgs::ObstacleSectors> ph_bumper_;

  ros::Timer timer_bumper_;
  void       timerBumper(const ros::TimerEvent& event);

  void setBumperFrontSector(const double distance);

  mrs_msgs::ObstacleSectors bumper_data_;
  std::mutex                mutex_bumper_data_;

  Eigen::Vector3d getBodyVelocity();

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh_;

  mrs_lib::PublisherHandler<mrs_msgs::HwApiRcChannels> ph_rc_channels_;

  ros::Timer timer_rc_;
  void       timerRc(const ros::TimerEvent& event);

  mrs_msgs::HwApiRcChannels rc_;
  std::mutex                mutex_rc_;

  void activate();
  void deactivate();
  void moveForward();
};

/* getBodyVelocity() //{ */

Eigen::Vector3d Tester::getBodyVelocity() {

  geometry_msgs::Vector3Stamped vel;

  auto uav_state = uh_->sh_uav_state_.getMsg();

  vel.header = uav_state->header;
  vel.vector = uav_state->velocity.linear;

  auto result = this->transformer_->transformSingle(vel, _uav_name_ + "/fcu_untilted");

  if (result) {
    return Eigen::Vector3d(result->vector.x, result->vector.y, result->vector.z);
  } else {
    ROS_ERROR("[%s]: could not transform body velocity", ros::this_node::getName().c_str());
    return Eigen::Vector3d(0, 0, 0);
  }
}

//}

/* setBumeprFrontSector() //{ */

void Tester::setBumperFrontSector(const double distance) {

  std::scoped_lock lock(mutex_bumper_data_);

  bumper_data_.sectors[0] = distance;
}

//}

/* constructor Tester() //{ */

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  ph_bumper_ = mrs_lib::PublisherHandler<mrs_msgs::ObstacleSectors>(nh_, "/" + _uav_name_ + "/bumper/obstacle_sectors");

  timer_bumper_ = nh_.createTimer(ros::Rate(20.0), &Tester::timerBumper, this);

  bumper_data_.n_horizontal_sectors = 8;
  bumper_data_.sectors_vertical_fov = 20;

  for (unsigned int i = 0; i < bumper_data_.n_horizontal_sectors + 2; i++) {
    bumper_data_.sectors.push_back(10.0);
  }

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

//}

/* timerBumper() //{ */

void Tester::timerBumper([[maybe_unused]] const ros::TimerEvent& event) {

  auto bumper_data = mrs_lib::get_mutexed(mutex_bumper_data_, bumper_data_);

  ph_bumper_.publish(bumper_data);
}

//}

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

bool Tester::test() {

  {
    auto [uhopt, message] = getUAVHandler(_uav_name_);

    if (!uhopt) {
      ROS_ERROR("[%s]: Failed obtain handler for '%s': '%s'", ros::this_node::getName().c_str(), _uav_name_.c_str(), message.c_str());
      return false;
    }

    uh_ = uhopt.value();
  }

  {
    auto [success, message] = uh_->activateMidAir();

    if (!success) {
      ROS_ERROR("[%s]: midair activation failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  this->sleep(2.0);

  // | -------------- activate the RC joystick mode ------------- |

  activate();

  this->sleep(0.5);

  {
    auto ctrl_diag = uh_->sh_control_manager_diag_.getMsg();

    if (!ctrl_diag->joystick_active) {
      ROS_ERROR("[%s]: RC joystick not active when it should", ros::this_node::getName().c_str());
      return false;
    }
  }

  // | --------------------- flight forward --------------------- |

  moveForward();

  this->sleep(5.0);

  {
    auto body_vel = getBodyVelocity();

    if (body_vel[0] < 1.0) {
      ROS_ERROR("[%s]: forward velocity too small", ros::this_node::getName().c_str());
      return false;
    }
  }

  // | ------------------- activate the bumper ------------------ |

  ROS_INFO("[%s]: set the bumper front sector to 'close obstacle'", ros::this_node::getName().c_str());

  setBumperFrontSector(0.5);

  this->sleep(5.0);

  {
    auto ctrl_diag = uh_->sh_control_manager_diag_.getMsg();

    if (!(!ctrl_diag->flying_normally && ctrl_diag->bumper_active)) {
      ROS_ERROR("[%s]: missing the signs of the bumper being active", ros::this_node::getName().c_str());
      return false;
    }
  }

  {
    auto body_vel = getBodyVelocity();

    if (body_vel[0] > 0.0) {
      ROS_ERROR("[%s]: forward velocity is positive, but it should not be", ros::this_node::getName().c_str());
      return false;
    }
  }

  ROS_INFO("[%s]: set the bumper front sector to 'none obstacle'", ros::this_node::getName().c_str());

  setBumperFrontSector(10.0);

  ROS_INFO("[%s]: waiting for normal flight conditions", ros::this_node::getName().c_str());

  while (true) {

    auto ctrl_diag = uh_->sh_control_manager_diag_.getMsg();

    if (!ros::ok()) {
      return false;
    }

    auto body_vel = getBodyVelocity();

    if (ctrl_diag->flying_normally && !ctrl_diag->bumper_active && body_vel[0] > 1.0) {
      break;
    }
  }

  if (uh_->isFlyingNormally()) {
    return true;
  } else {
    ROS_ERROR("[%s]: not flying normally", ros::this_node::getName().c_str());
    return false;
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
