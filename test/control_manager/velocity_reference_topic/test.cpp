#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {
  }

  bool test(void);
};

bool Tester::test(void) {

  const std::string uav_name = "uav1";

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh;

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
      return false;
    }

    uh = uhopt.value();
  }

  {
    auto [success, message] = uh->activateMidAir();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "midair activation failed with message: '%s'", message.c_str());
      return false;
    }
  }

  std::string desired_frame_id = uh->sh_uav_state_.getMsg()->header.frame_id;

  while (true) {

    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "stopped from the outside");
      return false;
    }

    // | ------------------ publish the reference ----------------- |

    mrs_msgs::msg::VelocityReferenceStamped msg;

    msg.header.frame_id = desired_frame_id;

    msg.reference.velocity.x = 1.2;
    msg.reference.velocity.y = 1.5;
    msg.reference.velocity.z = 0.5;

    uh->ph_velocity_reference_.publish(msg);

    // | ------------------- check for the state ------------------ |

    Eigen::Vector3d vel;

    {
      auto opt_vel = uh->getVelocity(desired_frame_id);

      if (!opt_vel) {
        RCLCPP_ERROR(node_->get_logger(), "could not obtain UAV velocity");
        return false;
      }

      vel = opt_vel.value();
    }

    if (std::abs(msg.reference.velocity.x - vel(0)) < 0.1 && std::abs(msg.reference.velocity.y - vel(1)) < 0.1 &&
        std::abs(msg.reference.velocity.z - vel(2)) < 0.1) {
      RCLCPP_INFO(node_->get_logger(), "reached the desired velocity");
      break;
    }

    sleep(0.01);
  }

  // | ----------- check if we become stationary again ---------- |

  while (true) {

    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "stopped from the outside");
      return false;
    }

    std::optional<double> speed = uh->getSpeed();

    if (speed && speed.value() < 0.1) {
      RCLCPP_INFO(node_->get_logger(), "reached stopping speed");
      break;
    }
  }

  this->sleep(5.0);

  if (uh->isFlyingNormally()) {
    RCLCPP_INFO(node_->get_logger(), "still flying normally");
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
