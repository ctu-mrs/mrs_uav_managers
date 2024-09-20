#include <gtest/gtest.h>

#include <trajectory_tracking_test.h>

class Tester : public TrajectoryTrackingTest {

public:
  Tester();

  bool test();
};

Tester::Tester() {
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

  {
    auto [success, message] = uh_->gotoAbs(0, 0, 2.0, 0);

    if (!success) {
      ROS_ERROR("[%s]: goto failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ----------------- prepare the trajectory ----------------- |

  Eigen::Vector4d from(0, 0, 2, 0);
  Eigen::Vector4d to(10, 10, 2, 3.14);
  const double    dt = 0.2;

  std::vector<Eigen::Vector4d> traj = sampleTrajectory(from, to, dt, 2.0);

  // | ----------------- publish the trajectory ----------------- |

  {

    mrs_msgs::TrajectoryReference msg_out;

    msg_out.dt          = dt;
    msg_out.fly_now     = true;
    msg_out.use_heading = true;

    for (auto point : traj) {

      mrs_msgs::Reference traj_point;

      traj_point.position.x = point(0);
      traj_point.position.y = point(1);
      traj_point.position.z = point(2);
      traj_point.heading    = point(3);

      msg_out.points.push_back(traj_point);
    }

    uh_->ph_trajectory_.publish(msg_out);
  }

  // | ------------- check following the trajectory ------------- |

  {
    auto [success, message] = checkTrajectoryFlythrough(traj, 1.0);

    if (!success) {
      ROS_ERROR("[%s]: trajectory was not followed: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | -------------- are we still flygin normally? ------------- |

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

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv) {

  ros::init(argc, argv, "test");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
