#include <gtest/gtest.h>

#include <failed_takeoff_test.h>

class Tester : public FailedTakeoffTest {

public:
  Tester() : FailedTakeoffTest(){};
};

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

