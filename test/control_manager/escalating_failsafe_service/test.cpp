#include <gtest/gtest.h>

#include <escalating_failsafe_test.h>

class Tester : public EscalatingFailsafeTest {

public:
  Tester();

  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_esc_failsafe_;

  std::optional<std::tuple<bool, std::string>> escalatingFailsafe();
};

Tester::Tester() {

  sch_esc_failsafe_ = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/control_manager/failsafe_escalating");
}

std::optional<std::tuple<bool, std::string>> Tester::escalatingFailsafe() {

  {
    std_srvs::Trigger srv;

    {
      bool service_call = sch_esc_failsafe_.call(srv);

      if (!service_call || !srv.response.success) {
        return {{false, "escalating failsafe service call failed"}};
      }
    }
  }

  return {{true, "service called"}};
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
