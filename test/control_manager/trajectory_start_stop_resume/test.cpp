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

  // | ----------------- prepare the trajectory ----------------- |

  Eigen::Vector4d from(20, 10, 2, 1.2);
  Eigen::Vector4d to(100, 10, 2, 1.2);
  const double    dt = 1.0;

  std::vector<Eigen::Vector4d> traj = sampleTrajectory(from, to, dt, 4.0);

  // | ----------------- publish the trajectory ----------------- |

  {
    mrs_msgs::TrajectoryReference msg_out;

    msg_out.dt          = dt;
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

  // | --------------- start the trajectory check --------------- |

  ROS_INFO("[%s]: pes", ros::this_node::getName().c_str());

  auto future_flythrough_check = std::async(std::launch::async, &Tester::checkTrajectoryFlythrough, this, traj, 1.0);

  ROS_INFO("[%s]: kocka", ros::this_node::getName().c_str());

  // | --------- goto the first point of the trajectory --------- |

  {
    auto [success, message] = uh_->gotoTrajectoryStart();

    if (!success) {
      ROS_ERROR("[%s]: failed to goto the first point of the trajectory: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | -------- check if we are really at the first point ------- |

  {
    if (!uh_->isAtPosition(from(0), from(1), from(2), from(3), 1.0)) {
      ROS_ERROR("[%s]: failed to reach the first point of the trajectory", ros::this_node::getName().c_str());
      return false;
    }
  }

  // | ---------------- start trajectory tracking --------------- |

  {
    auto [success, message] = uh_->startTrajectoryTracking();

    if (!success) {
      ROS_ERROR("[%s]: failed to start trajectory tracking: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ------------------- pause the following ------------------ |

  {
    auto [success, message] = uh_->stopTrajectoryTracking();

    if (!success) {
      ROS_ERROR("[%s]: failed to stop trajectory tracking: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | --------------- check if we are stationary --------------- |

  while (true) {

    if (!ros::ok()) {
      return false;
      ROS_ERROR("[%s]: stopped from outside", ros::this_node::getName().c_str());
    }

    std::optional<double> speed = uh_->getSpeed();

    if (speed && speed.value() < 0.3) {
      break;
    }
  }

  // | --------------- resume trajectory tracking --------------- |

  {
    auto [success, message] = uh_->resumeTrajectoryTracking();

    if (!success) {
      ROS_ERROR("[%s]: failed to stop trajectory tracking: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ------- wait till we finish the trajectory tracking ------ |

  while (true) {

    if (!ros::ok()) {
      return false;
      ROS_ERROR("[%s]: stopped from outside", ros::this_node::getName().c_str());
    }

    if (future_flythrough_check.valid()) {

      auto [success, message] = future_flythrough_check.get();

      if (!success) {
        return false;
        ROS_ERROR("[%s]: the trajectory was not tracked: '%s'", ros::this_node::getName().c_str(), message.c_str());
      } else {
        break;
      }
    }
  }

  // | -------------- are we still flying normally? ------------- |

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
