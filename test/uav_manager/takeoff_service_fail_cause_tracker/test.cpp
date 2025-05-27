#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

#include <failed_takeoff_test.h>

using namespace std::chrono_literals;

class Tester : public FailedTakeoffTest {

public:
  Tester() : FailedTakeoffTest(){};
};

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Tester tester;

  test_result &= tester.test();

  tester.sleep(2.0);

  tester.reportTestResult(test_result);

  rclcpp::shutdown();

  exit(0);
}
