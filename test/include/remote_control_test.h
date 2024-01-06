#include <mrs_uav_testing/test_generic.h>

class RemoteControlTest : public mrs_uav_testing::TestGeneric {

public:
  bool test();

  RemoteControlTest() : mrs_uav_testing::TestGeneric(){};

  /* interface to be implemented by the particular test //{ */

  virtual void activate()   = 0;
  virtual void deactivate() = 0;

  virtual void moveForward()  = 0;
  virtual void moveBackward() = 0;
  virtual void moveLeft()     = 0;
  virtual void moveRight()    = 0;
  virtual void moveUp()       = 0;
  virtual void moveDown()     = 0;
  virtual void rotateLeft()   = 0;
  virtual void rotateRight()  = 0;

  virtual void stop() = 0;

  //}

  bool testMoveForward();
  bool testMoveBackward();
  bool testMoveLeft();
  bool testMoveRight();
  bool testMoveUp();
  bool testMoveDown();
  bool testRotateLeft();
  bool testRotateRight();
  bool testStop();

  Eigen::Vector3d getFcuUntiltedVelocity();

  bool setGotoReference();
};

/* getFcuUntiltedVelocity() //{ */

Eigen::Vector3d RemoteControlTest::getFcuUntiltedVelocity() {

  auto uav_state = sh_uav_state_.getMsg();

  geometry_msgs::Vector3Stamped vel_world;
  vel_world.header = uav_state->header;
  vel_world.vector = uav_state->velocity.linear;

  auto vel_fcu_untilted = transformer_->transformSingle(vel_world, "fcu_untilted");

  if (vel_fcu_untilted) {
    return Eigen::Vector3d(vel_fcu_untilted->vector.x, vel_fcu_untilted->vector.y, vel_fcu_untilted->vector.z);
  } else {
    return Eigen::Vector3d::Zero();
  }
}

//}

/* testMoveForward() //{ */

bool RemoteControlTest::testMoveForward() {

  ros::Time started = ros::Time::now();

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    moveForward();

    auto velocity = getFcuUntiltedVelocity();

    if (velocity[0] > 0.5 && abs(velocity[1]) < 0.1 && abs(velocity[2]) < 0.1) {
      return true;
    }

    if ((ros::Time::now() - started).toSec() > 5.0) {
      return false;
    }

    sleep(0.1);
  }
}

//}

/* testMoveBackward() //{ */

bool RemoteControlTest::testMoveBackward() {

  ros::Time started = ros::Time::now();

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    moveBackward();

    auto velocity = getFcuUntiltedVelocity();

    if (velocity[0] < -0.5 && abs(velocity[1]) < 0.1 && abs(velocity[2]) < 0.1) {
      return true;
    }

    if ((ros::Time::now() - started).toSec() > 5.0) {
      return false;
    }

    sleep(0.1);
  }
}

//}

/* testMoveLeft() //{ */

bool RemoteControlTest::testMoveLeft() {

  ros::Time started = ros::Time::now();

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    moveLeft();

    auto velocity = getFcuUntiltedVelocity();

    if (abs(velocity[0]) < 0.1 && velocity[1] > 0.5 && abs(velocity[2]) < 0.1) {
      return true;
    }

    if ((ros::Time::now() - started).toSec() > 5.0) {
      return false;
    }

    sleep(0.1);
  }
}

//}

/* testMoveRight() //{ */

bool RemoteControlTest::testMoveRight() {

  ros::Time started = ros::Time::now();

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    moveRight();

    auto velocity = getFcuUntiltedVelocity();

    if (abs(velocity[0]) < 0.1 && velocity[1] < -0.5 && abs(velocity[2]) < 0.1) {
      return true;
    }

    if ((ros::Time::now() - started).toSec() > 5.0) {
      return false;
    }

    sleep(0.1);
  }
}

//}

/* testMoveUp() //{ */

bool RemoteControlTest::testMoveUp() {

  ros::Time started = ros::Time::now();

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    moveUp();

    auto velocity = getFcuUntiltedVelocity();

    if (abs(velocity[0]) < 0.1 && abs(velocity[1]) < 0.1 && velocity[2] > 0.5) {
      return true;
    }

    if ((ros::Time::now() - started).toSec() > 5.0) {
      return false;
    }

    sleep(0.1);
  }
}

//}

/* testMoveDown() //{ */

bool RemoteControlTest::testMoveDown() {

  ros::Time started = ros::Time::now();

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    moveDown();

    auto velocity = getFcuUntiltedVelocity();

    if (abs(velocity[0]) < 0.1 && abs(velocity[1]) < 0.1 && velocity[2] < -0.5) {
      return true;
    }

    if ((ros::Time::now() - started).toSec() > 5.0) {
      return false;
    }

    sleep(0.1);
  }
}

//}

/* testRotateLeft() //{ */

bool RemoteControlTest::testRotateLeft() {

  ros::Time started = ros::Time::now();

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    rotateLeft();

    auto uav_state = sh_uav_state_.getMsg();

    if (uav_state->velocity.angular.z > 0.2) {
      return true;
    }

    if ((ros::Time::now() - started).toSec() > 5.0) {
      return false;
    }

    sleep(0.1);
  }
}

//}

/* testRotateRight() //{ */

bool RemoteControlTest::testRotateRight() {

  ros::Time started = ros::Time::now();

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    rotateRight();

    auto uav_state = sh_uav_state_.getMsg();

    if (uav_state->velocity.angular.z < -0.2) {
      return true;
    }

    if ((ros::Time::now() - started).toSec() > 5.0) {
      return false;
    }

    sleep(0.1);
  }
}

//}

/* testStop() //{ */

bool RemoteControlTest::testStop() {

  ros::Time started = ros::Time::now();

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    stop();

    auto velocity  = getFcuUntiltedVelocity();
    auto uav_state = sh_uav_state_.getMsg();

    if (abs(velocity[0]) < 0.1 && abs(velocity[1]) < 0.1 && abs(velocity[2]) < 0.1 && abs(uav_state->velocity.angular.z) < 0.1) {
      return true;
    }

    if ((ros::Time::now() - started).toSec() > 10.0) {
      return false;
    }

    sleep(0.1);
  }
}

//}

/* setGotoReference() //{ */

bool RemoteControlTest::setGotoReference() {

  mrs_msgs::Vec4 srv;

  srv.request.goal[0] = 1000;
  srv.request.goal[1] = 0;
  srv.request.goal[2] = 5;
  srv.request.goal[3] = 0;

  {
    bool service_call = sch_goto_relative_.call(srv);

    if (!service_call || !srv.response.success) {
      ROS_ERROR("[%s]: failed to call the service", ros::this_node::getName().c_str());
      return false;
    }
  }

  sleep(5.0);

  // check if we gained velocity

  auto uav_state = sh_uav_state_.getMsg();

  if (uav_state->velocity.linear.x > 1.0) {
    return true;
  } else {
    ROS_ERROR("[%s]: velocity not reached, %.2f, %.2f, %.2f", ros::this_node::getName().c_str(), uav_state->velocity.linear.x, uav_state->velocity.linear.y,
              uav_state->velocity.linear.z);
    return false;
  }
}

//}

bool RemoteControlTest::test() {

  {
    auto [success, message] = activateMidAir();

    if (!success) {
      ROS_ERROR("[%s]: midair activation failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ------------- activate the remote controller ------------- |

  activate();

  // | -------- wait for indication from the diagnostics -------- |

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    if (sh_control_manager_diag_.getMsg()->rc_mode) {
      break;
    }

    sleep(0.01);
  }

  // | -------------------- test the motions -------------------- |

  {
    bool success = testRotateLeft();

    if (!success) {
      ROS_ERROR("[%s]: left rotation motion test failed", ros::this_node::getName().c_str());
      return false;
    }
  }

  {
    bool success = testRotateRight();

    if (!success) {
      ROS_ERROR("[%s]: right rotation motion test failed", ros::this_node::getName().c_str());
      return false;
    }
  }

  {
    bool success = testMoveForward();

    if (!success) {
      ROS_ERROR("[%s]: forward motion test failed", ros::this_node::getName().c_str());
      return false;
    }
  }

  {
    bool success = testMoveBackward();

    if (!success) {
      ROS_ERROR("[%s]: backward motion test failed", ros::this_node::getName().c_str());
      return false;
    }
  }

  {
    bool success = testMoveLeft();

    if (!success) {
      ROS_ERROR("[%s]: left motion test failed", ros::this_node::getName().c_str());
      return false;
    }
  }

  {
    bool success = testMoveRight();

    if (!success) {
      ROS_ERROR("[%s]: right motion test failed", ros::this_node::getName().c_str());
      return false;
    }
  }

  {
    bool success = testMoveUp();

    if (!success) {
      ROS_ERROR("[%s]: up motion test failed", ros::this_node::getName().c_str());
      return false;
    }
  }

  {
    bool success = testMoveDown();

    if (!success) {
      ROS_ERROR("[%s]: down motion test failed", ros::this_node::getName().c_str());
      return false;
    }
  }

  deactivate();

  // | -------- wait for indication from the diagnostics -------- |

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    if (!sh_control_manager_diag_.getMsg()->rc_mode) {
      break;
    }

    sleep(0.01);
  }

  sleep(2.0);

  // | ------ test if we can override a pre-existing motion ----- |

  {
    bool success = setGotoReference();

    if (!success) {
      ROS_ERROR("[%s]: failed to set goto reference", ros::this_node::getName().c_str());
      return false;
    }
  }

  activate();

  {
    bool success = testMoveUp();

    if (!success) {
      ROS_ERROR("[%s]: up motion test failed", ros::this_node::getName().c_str());
      return false;
    }
  }

  {
    bool success = testStop();

    if (!success) {
      ROS_ERROR("[%s]: stop motion test failed", ros::this_node::getName().c_str());
      return false;
    }
  }

  deactivate();

  // | ------------- activate the remote controller ------------- |

  return true;
}
