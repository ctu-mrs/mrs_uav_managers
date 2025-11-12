#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <trajectory_tracking_test.h>

using namespace std::chrono_literals;

class Tester : public TrajectoryTrackingTest {

public:
  Tester() : TrajectoryTrackingTest() {
  }

  bool test(void);
};

bool Tester::test(void) {

  const std::string uav_name = "uav1";

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
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

  // | ----------------- prepare the trajectory ----------------- |

  Eigen::Vector4d from(20, 10, 2, 1.2);
  Eigen::Vector4d to(100, 10, 2, 1.2);
  const double    dt = 1.0;

  std::vector<Eigen::Vector4d> traj = sampleTrajectory(from, to, dt, 4.0);

  // | ----------------- publish the trajectory ----------------- |

  {
    mrs_msgs::msg::TrajectoryReference msg_out;

    msg_out.dt          = dt;
    msg_out.use_heading = true;

    for (auto point : traj) {

      mrs_msgs::msg::Reference traj_point;

      traj_point.position.x = point(0);
      traj_point.position.y = point(1);
      traj_point.position.z = point(2);
      traj_point.heading    = point(3);

      msg_out.points.push_back(traj_point);
    }

    uh_->ph_trajectory_.publish(msg_out);
  }

  this->sleep(0.5);

  // | --------------- start the trajectory check --------------- |

  auto future_flythrough_check = std::async(std::launch::async, &Tester::checkTrajectoryFlythrough, this, traj, 1.0);

  // | --------- goto the first point of the trajectory --------- |

  {
    auto [success, message] = uh_->gotoTrajectoryStart();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "failed to goto the first point of the trajectory: '%s'", message.c_str());
      return false;
    }
  }

  // | -------- check if we are really at the first point ------- |

  {
    if (!uh_->isAtPosition(from(0), from(1), from(2), from(3), 1.0)) {
      RCLCPP_ERROR(node_->get_logger(), "failed to reach the first point of the trajectory");
      return false;
    }
  }

  // | ---------------- start trajectory tracking --------------- |

  {
    auto [success, message] = uh_->startTrajectoryTracking();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "failed to start trajectory tracking: '%s'", message.c_str());
      return false;
    }
  }

  // | ------------------- pause the following ------------------ |

  {
    auto [success, message] = uh_->stopTrajectoryTracking();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "failed to stop trajectory tracking: '%s'", message.c_str());
      return false;
    }
  }

  // | --------------- check if we are stationary --------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return false;
      RCLCPP_ERROR(node_->get_logger(), "stopped from outside");
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
      RCLCPP_ERROR(node_->get_logger(), "failed to stop trajectory tracking: '%s'", message.c_str());
      return false;
    }
  }

  // | ------- wait till we finish the trajectory tracking ------ |

  while (true) {

    if (!rclcpp::ok()) {
      return false;
      RCLCPP_ERROR(node_->get_logger(), "stopped from outside");
    }

    if (future_flythrough_check.valid()) {

      auto [success, message] = future_flythrough_check.get();

      if (!success) {
        return false;
        RCLCPP_ERROR(node_->get_logger(), "the trajectory was not tracked: '%s'", message.c_str());
      } else {
        break;
      }
    }
  }

  // | -------------- are we still flying normally? ------------- |

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
