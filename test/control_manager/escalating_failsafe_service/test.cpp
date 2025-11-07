#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <escalating_failsafe_test.h>

using namespace std::chrono_literals;

class Tester : public EscalatingFailsafeTest {

public:
  Tester() : EscalatingFailsafeTest() {
  }

  std::optional<std::tuple<bool, std::string>> escalatingFailsafe();
};

std::optional<std::tuple<bool, std::string>> Tester::escalatingFailsafe() {

  auto [success, message] = uh_->escalatingFailsafe();

  return {{success, message}};
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
