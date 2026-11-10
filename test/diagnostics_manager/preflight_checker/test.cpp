#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_managers/diagnostics_manager/preflight_checker.hpp>

#include <mrs_msgs/msg/safety_area_manager_diagnostics.hpp>
#include <std_msgs/msg/empty.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

using namespace mrs_uav_managers::diagnostics_manager;
using namespace std::chrono_literals;

namespace
{

// the default preflight_check config, with time_window and the topic check timeout shortened to 1 s and the topic check
// watching /preflight_checker_test/heartbeat
class PreflightCheckerTest : public ::testing::Test {

protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"preflight_check_config", std::string(PREFLIGHT_CHECK_CONFIG)},
        {"custom_config", std::string(CUSTOM_CONFIG)},
    });

    node_    = std::make_shared<rclcpp::Node>("preflight_checker_test", options);
    checker_ = std::make_unique<PreflightChecker>(node_, "uav1", rclcpp::Duration(1s));
    ASSERT_TRUE(rclcpp::ok()) << "PreflightChecker failed to load its parameters";

    executor_.add_node(node_);
    // a flag instead of executor_.cancel(): a cancel that comes before spin() starts is lost and spin() never returns
    spinner_ = std::thread([this] {
      while (!stop_spinning_) {
        executor_.spin_once(50ms);
      }
    });
  }

  void TearDown() override {
    stop_spinning_ = true;
    spinner_.join();
    executor_.remove_node(node_);
    checker_.reset();
    node_.reset();
  }

  // calls pred() every 50 ms until it holds or timeout runs out
  bool waitFor(const std::function<bool()> &pred, const std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(50ms);
    }
    return pred();
  }

  // a UAV standing still on the ground, inside the safety area
  static PreflightChecker::PreflightInputs onTheGround() {
    PreflightChecker::PreflightInputs inputs;
    inputs.velocity              = geometry_msgs::msg::Vector3();
    inputs.distance_sensor_range = sensor_msgs::msg::Range();
    inputs.angular_rate          = geometry_msgs::msg::Vector3();
    inputs.has_distance_sensor   = true;
    inputs.has_imu               = true;
    inputs.position_valid        = true;
    inputs.position_known        = true;
    return inputs;
  }

  rclcpp::Node::SharedPtr                   node_;
  std::unique_ptr<PreflightChecker>         checker_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread                               spinner_;
  std::atomic<bool>                         stop_spinning_{false};
};

} // namespace

// | ----------------- speed, height and gyro ----------------- |

TEST_F(PreflightCheckerTest, StandingStillPasses) {
  const auto result = checker_->runPreflightChecks(onTheGround());
  EXPECT_TRUE(result.speed_ok);
  EXPECT_TRUE(result.height_ok);
  EXPECT_TRUE(result.gyro_ok);
}

TEST_F(PreflightCheckerTest, MovingFails) {
  auto inputs                         = onTheGround();
  inputs.velocity->x                  = 1.0;  // max_speed 0.3 m/s
  inputs.distance_sensor_range->range = 2.0;  // max_height 0.8 m
  inputs.angular_rate->z              = -2.0; // max_rate 1.0 rad/s, either sign

  const auto result = checker_->runPreflightChecks(inputs);
  EXPECT_FALSE(result.speed_ok);
  EXPECT_FALSE(result.height_ok);
  EXPECT_FALSE(result.gyro_ok);
  EXPECT_FALSE(result.can_takeoff);
}

TEST_F(PreflightCheckerTest, NanFails) {
  auto inputs                         = onTheGround();
  inputs.velocity->x                  = std::nan("");
  inputs.distance_sensor_range->range = std::nan("");
  inputs.angular_rate->y              = std::nan("");

  const auto result = checker_->runPreflightChecks(inputs);
  EXPECT_FALSE(result.speed_ok);
  EXPECT_FALSE(result.height_ok);
  EXPECT_FALSE(result.gyro_ok);
}

TEST_F(PreflightCheckerTest, MissingDataFails) {
  auto inputs = onTheGround();
  inputs.velocity.reset();
  inputs.distance_sensor_range.reset();
  inputs.angular_rate.reset();

  const auto result = checker_->runPreflightChecks(inputs);
  EXPECT_FALSE(result.speed_ok);
  EXPECT_FALSE(result.height_ok);
  EXPECT_FALSE(result.gyro_ok);
}

TEST_F(PreflightCheckerTest, NoSensorSkipsHeightAndGyro) {
  auto inputs                         = onTheGround();
  inputs.has_distance_sensor          = false;
  inputs.has_imu                      = false;
  inputs.distance_sensor_range->range = 2.0;
  inputs.angular_rate->z              = 2.0;

  const auto result = checker_->runPreflightChecks(inputs);
  EXPECT_TRUE(result.height_ok);
  EXPECT_TRUE(result.gyro_ok);
}

TEST_F(PreflightCheckerTest, ViolationHoldsForTimeWindow) {
  auto moving                         = onTheGround();
  moving.velocity->x                  = 1.0;
  moving.distance_sensor_range->range = 2.0;
  moving.angular_rate->z              = 2.0;
  checker_->runPreflightChecks(moving);

  // standing still again, but within time_window (1 s) of the violation
  auto result = checker_->runPreflightChecks(onTheGround());
  EXPECT_FALSE(result.speed_ok);
  EXPECT_FALSE(result.height_ok);
  EXPECT_FALSE(result.gyro_ok);

  std::this_thread::sleep_for(1200ms);

  result = checker_->runPreflightChecks(onTheGround());
  EXPECT_TRUE(result.speed_ok);
  EXPECT_TRUE(result.height_ok);
  EXPECT_TRUE(result.gyro_ok);
}

// | ----------------------- topic check ---------------------- |

TEST_F(PreflightCheckerTest, TopicCheckFollowsTheHeartbeat) {
  const auto inputs = onTheGround();

  EXPECT_FALSE(checker_->runPreflightChecks(inputs).topics_ok) << "a topic never seen must fail";

  auto pub   = node_->create_publisher<std_msgs::msg::Empty>("/preflight_checker_test/heartbeat", 10);
  auto timer = node_->create_wall_timer(50ms, [&pub] { pub->publish(std_msgs::msg::Empty()); });

  EXPECT_TRUE(waitFor([&] { return checker_->runPreflightChecks(inputs).topics_ok; }, 3s)) << "the topic is publishing";

  timer->cancel();

  // stale after topic_check/timeout (1 s)
  EXPECT_TRUE(checker_->runPreflightChecks(inputs).topics_ok) << "fresh right after the last message";
  EXPECT_TRUE(waitFor([&] { return !checker_->runPreflightChecks(inputs).topics_ok; }, 3s)) << "the topic went silent";
}

// | ------- position, passed on from SafetyAreaManager ------- |

TEST_F(PreflightCheckerTest, PositionFollowsSafetyAreaManager) {
  auto run = [this] { return checker_->runPreflightChecks(); };

  auto result = run();
  EXPECT_FALSE(result.position_known) << "no SafetyAreaManager diagnostics yet";
  EXPECT_FALSE(result.can_takeoff);

  std::atomic<bool> inside{true};

  auto pub   = node_->create_publisher<mrs_msgs::msg::SafetyAreaManagerDiagnostics>("/preflight_checker_test/safety_area_manager_diagnostics_in", 10);
  auto timer = node_->create_wall_timer(50ms, [&] {
    mrs_msgs::msg::SafetyAreaManagerDiagnostics msg;
    msg.safety_area_enabled = true;
    msg.position_valid_2d   = inside;
    msg.position_valid_3d   = inside;
    pub->publish(msg);
  });

  EXPECT_TRUE(waitFor([&] { return run().position_known && run().position_valid; }, 3s)) << "inside the safety area";

  inside = false;
  EXPECT_TRUE(waitFor(
      [&] {
        const auto r = run();
        return r.position_known && !r.position_valid && !r.can_takeoff;
      },
      3s))
      << "outside the safety area";

  timer->cancel();

  // not_reporting_timeout (1 s)
  EXPECT_TRUE(waitFor([&] { return !run().position_known; }, 3s)) << "SafetyAreaManager went silent";
}
