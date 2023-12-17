#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test();
};

bool Tester::test() {

  {
    auto [success, message] = activateMidAir();

    if (!success) {
      ROS_ERROR("[%s]: midair activation failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | --------------- goto to violate min height --------------- |

  {
    auto [success, message] = this->gotoAbs(0, 0, 100, 0);

    if (success) {
      ROS_ERROR("[%s]: goto should fail", ros::this_node::getName().c_str());
      return false;
    }
  }

  // | --------- wait till we are flying normally again --------- |

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    if (this->isFlyingNormally()) {
      break;
    }
  }

  // | ------------------- check the altitude ------------------- |

  auto height = this->getHeightAgl();

  // TODO substitude with subscribed max agl height
  double max_height_agl = 40;

  if (height) {
    if (height.value() < max_height_agl) {
      return true;
    }
  }

  return false;
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
