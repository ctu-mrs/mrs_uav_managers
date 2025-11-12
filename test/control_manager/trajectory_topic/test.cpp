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

  {
    auto [success, message] = uh_->gotoAbs(0, 0, 2.0, 0);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "goto failed with message: '%s'", message.c_str());
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

    mrs_msgs::msg::TrajectoryReference msg_out;

    msg_out.dt          = dt;
    msg_out.fly_now     = true;
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

  // | ------------- check following the trajectory ------------- |

  {
    auto [success, message] = checkTrajectoryFlythrough(traj, 1.0);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "trajectory was not followed: '%s'", message.c_str());
      return false;
    }
  }

  // | -------------- are we still flygin normally? ------------- |

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
