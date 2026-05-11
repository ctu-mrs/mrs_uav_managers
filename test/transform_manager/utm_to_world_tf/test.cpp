#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test();

  void printTf(const geometry_msgs::TransformStamped& tf);
  void printUavState(std::shared_ptr<mrs_uav_testing::UAVHandler> uh);
};

bool Tester::test() {

  const ros::Time t_start = ros::Time::now();

  bool test_success = true;

  double goto_x = 5;
  double goto_y = 5;
  double goto_z = 5;
  double goto_hdg = 1.57;

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh;

  {
    auto [uhopt, message] = getUAVHandler(_uav_name_);

    if (!uhopt) {
      ROS_ERROR("[%s]: Failed obtain handler for '%s': '%s'", ros::this_node::getName().c_str(), _uav_name_.c_str(), message.c_str());
      return false;
    }

    uh = uhopt.value();
  }

  {
    auto [success, message] = uh->activateMidAir();

    if (!success) {
      ROS_ERROR("[%s]: midair activation failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  {
    auto [success, message] = uh->switchController("Se3Controller");

    if (!success) {
      ROS_ERROR("[%s]: switch controller failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  {
    auto [success, message] = uh->setConstraints("fast");

    if (!success) {
      ROS_ERROR("[%s]: set constraints failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  {
    auto [success, message] = uh->gotoService(goto_x, goto_y, goto_z, goto_hdg);

    if (!success) {
      ROS_ERROR("[%s]: goto failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  const std::string utm_frame_id = _uav_name_ + "/utm_origin";
  const std::string world_frame_id = _uav_name_ + "/world_origin";

  const double tf_pos_eps = 0.01;
  const double t_sleep = 0.01;
  const double t_stop = 30;

  while (true) {
    
    if (uh->isAtPosition(goto_x, goto_y, goto_z, goto_hdg, 0.3, world_frame_id)) {
      goto_x *= -1;
      goto_y *= -1;
      goto_hdg *= -1;
      {
        auto [success, message] = uh->gotoService(goto_x, goto_y, goto_z, goto_hdg);

        if (!success) {
          ROS_ERROR("[%s]: goto failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
          return false;
        }
      }
    }
    
    const auto tf_opt1 = this->transformer_->getTransform(utm_frame_id, world_frame_id, ros::Time::now());

    const auto t_now = ros::Time::now();

    this->sleep(t_sleep);
    
    const auto tf_opt2 = this->transformer_->getTransform(utm_frame_id, world_frame_id, ros::Time());

    if (std::fabs(tf_opt1.value().transform.translation.x - tf_opt2.value().transform.translation.x) > tf_pos_eps ||
        std::fabs(tf_opt1.value().transform.translation.y - tf_opt2.value().transform.translation.y) > tf_pos_eps ||
        std::fabs(tf_opt1.value().transform.translation.z - tf_opt2.value().transform.translation.z) > tf_pos_eps) {
      ROS_ERROR("[%s]: the tf from utm_origin to world_origin is not constant.", ros::this_node::getName().c_str());
      printTf(tf_opt1.value());
      printTf(tf_opt2.value());
      printUavState(uh);
      test_success = false;
    }

    if ((ros::Time::now() - t_start).toSec() > t_stop) {
      break;
    } 
  }

  if (!test_success) {
    return false;
  }


  if (uh->isFlyingNormally()) {
    return true;
  } else {
    ROS_ERROR("[%s]: not flying normally", ros::this_node::getName().c_str());
    return false;
  }
}

void Tester::printTf(const geometry_msgs::TransformStamped& tf) {

  ROS_INFO("[%s]: got the transform", ros::this_node::getName().c_str());

  std::cout << "from: " << mrs_lib::Transformer::frame_from(tf) << ", to: " << mrs_lib::Transformer::frame_to(tf) << ", stamp: " << tf.header.stamp << std::endl;
  std::cout << tf << std::endl;
}

void Tester::printUavState(std::shared_ptr<mrs_uav_testing::UAVHandler> uh) {

    const auto uav_state = uh->sh_uav_state_.getMsg();

    tf2::Quaternion q(uav_state->pose.orientation.x, uav_state->pose.orientation.y, uav_state->pose.orientation.z, uav_state->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ROS_INFO("[%s]: got the uav_state", ros::this_node::getName().c_str());
    std::cout << "xyz: " << uav_state->pose.position.x << " " << uav_state->pose.position.y << " " << uav_state->pose.position.z << " rpy: " << roll << " " << pitch << " " << yaw << std::endl;
    
}

TEST(TESTSuite, test) {

  Tester tester;

  bool result = tester.test();

  if (result) {
    GTEST_SUCCEED();
  } else {
    ROS_ERROR("[%s]: The tf from utm_origin to world_origin was not constant during the test.", ros::this_node::getName().c_str());
    GTEST_FAIL();
  }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "test");
  ROS_INFO("[%s]: RUNNING TEST!", ros::this_node::getName().c_str());

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
