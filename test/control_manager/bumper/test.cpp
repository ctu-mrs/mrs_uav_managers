#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

#include <mrs_msgs/ObstacleSectors.h>

#include <mrs_lib/mutex.h>

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
};

Eigen::Vector3d Tester::getBodyVelocity() {

  geometry_msgs::Vector3Stamped vel;

  auto uav_state = this->sh_uav_state_.getMsg();

  vel.header = uav_state->header;
  vel.vector = uav_state->velocity.linear;

  auto result = this->transformer_->transformSingle(vel, "fcu_untilted");

  if (result) {
    return Eigen::Vector3d(result->vector.x, result->vector.y, result->vector.z);
  } else {
    ROS_ERROR("[%s]: could not transform body velocity", ros::this_node::getName().c_str());
    return Eigen::Vector3d(0, 0, 0);
  }
}

void Tester::setBumperFrontSector(const double distance) {

  std::scoped_lock lock(mutex_bumper_data_);

  bumper_data_.sectors[0] = distance;
}

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  ph_bumper_ = mrs_lib::PublisherHandler<mrs_msgs::ObstacleSectors>(nh_, "/" + _uav_name_ + "/bumper/obstacle_sectors");

  bumper_data_.n_horizontal_sectors = 8;
  bumper_data_.sectors_vertical_fov = 20;

  for (unsigned int i = 0; i < bumper_data_.n_horizontal_sectors + 2; i++) {
    bumper_data_.sectors.push_back(10.0);
  }
}

void Tester::timerBumper([[maybe_unused]] const ros::TimerEvent& event) {

  auto bumper_data = mrs_lib::get_mutexed(mutex_bumper_data_, bumper_data_);

  ph_bumper_.publish(bumper_data);
}

bool Tester::test() {

  {
    auto [success, message] = activateMidAir();

    if (!success) {
      ROS_ERROR("[%s]: midair activation failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  {
    auto [success, message] = this->gotoRel(1000, 0, 0.0, 0);

    if (!success) {
      ROS_ERROR("[%s]: goto failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  this->sleep(10.0);

  setBumperFrontSector(0.5);

  this->sleep(1.0);

  auto ctrl_diag = this->sh_control_manager_diag_.getMsg();

  if (!(!ctrl_diag->flying_normally && ctrl_diag->bumper_active)) {
    ROS_ERROR("[%s]: missing the signs of the bumper being active", ros::this_node::getName().c_str());
    return false;
  }

  this->sleep(5.0);

  auto body_vel = getBodyVelocity();

  if (!(body_vel[0] > 0.5 && abs(body_vel[1]) < 0.1 && abs(body_vel[2]) < 0.1)) {
    ROS_ERROR("[%s]: body velocity is not suggesting that we are moving backwards", ros::this_node::getName().c_str());
    return false;
  }

  setBumperFrontSector(10.0);

  this->sleep(1.0);

  if (!(ctrl_diag->flying_normally && !ctrl_diag->bumper_active)) {
    ROS_ERROR("[%s]: looks like the bumper is still active when it should not be", ros::this_node::getName().c_str());
    return false;
  }

  this->gotoRel(10, 0, 0, 0);

  if (this->isFlyingNormally()) {
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
