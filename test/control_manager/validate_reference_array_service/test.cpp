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

  mrs_msgs::srv::ValidateReferenceArray::Request req;

  req.array.header.frame_id = uav_name + "/world_origin";

  {
    mrs_msgs::msg::Reference msg;

    msg.position.x = 0;
    msg.position.y = 0;
    msg.position.z = 2;
    msg.heading    = 0;

    req.array.array.push_back(msg);
  }

  {
    mrs_msgs::msg::Reference msg;

    msg.position.x = 100;
    msg.position.y = 0;
    msg.position.z = 2;
    msg.heading    = 0;

    req.array.array.push_back(msg);
  }

  {
    mrs_msgs::msg::Reference msg;

    msg.position.x = 0;
    msg.position.y = 0;
    msg.position.z = -100;
    msg.heading    = 0;

    req.array.array.push_back(msg);
  }

  {
    mrs_msgs::msg::Reference msg;

    msg.position.x = 10;
    msg.position.y = 10;
    msg.position.z = 3;
    msg.heading    = 0;

    req.array.array.push_back(msg);
  }

  {
    auto [success, response] = uh->validateReferenceArray(req);

    if (!success) {

      RCLCPP_ERROR(node_->get_logger(), "reference array validation service call failed: '%s'", response->message.c_str());
      return false;

    } else {

      if (!response->success[0]) {
        RCLCPP_ERROR(node_->get_logger(), "reference #1 validation failed");
        return false;
      }

      if (response->success[1]) {
        RCLCPP_ERROR(node_->get_logger(), "reference #2 validation failed");
        return false;
      }

      if (response->success[2]) {
        RCLCPP_ERROR(node_->get_logger(), "reference #3 validation failed");
        return false;
      }

      if (!response->success[3]) {
        RCLCPP_ERROR(node_->get_logger(), "reference #4 validation failed");
        return false;
      }
    }
  }

  // | ---------------- now try with bad frame id --------------- |

  {
    req.array.header.frame_id = "frame_that_does_not_exist";

    auto [success, response] = uh->validateReferenceArray(req);

    if (success) {

      if (response.value().success.size() > 0) {

        RCLCPP_ERROR(node_->get_logger(), "the succes[] array has the wrong size (%d != 0)", int(response.value().success.size()));
        return false;
      }

    } else {

      RCLCPP_ERROR(node_->get_logger(), "reference array validation service call failed");
      return false;
    }
  }

  if (uh->isFlyingNormally()) {
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
