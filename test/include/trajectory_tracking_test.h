#ifndef TRAJECTORY_TRACKING_TEST_H
#define TRAJECTORY_TRACKING_TEST_H

#include <mrs_uav_testing/test_generic.h>

#include <mrs_lib/geometry/cyclic.h>

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

class TrajectoryTrackingTest : public mrs_uav_testing::TestGeneric {

public:
  TrajectoryTrackingTest();

  std::tuple<bool, std::string> checkTrajectoryFlythrough(const std::vector<Eigen::Vector4d> &points, const double tolerance);

  std::vector<Eigen::Vector4d> sampleTrajectory(const Eigen::Vector4d from, const Eigen::Vector4d to, const double dt, const double speed);

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh_;
};

TrajectoryTrackingTest::TrajectoryTrackingTest() : mrs_uav_testing::TestGeneric() {};

/* checkTrajectoryFlythrough() //{ */

std::tuple<bool, std::string> TrajectoryTrackingTest::checkTrajectoryFlythrough(const std::vector<Eigen::Vector4d> &points, const double tolerance) {

  unsigned long current_idx = 0;

  if (points.size() == 0) {
    return {false, "trajectory is empty"};
  }

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "terminated form outside"};
    }

    if (uh_->isAtPosition(points.at(current_idx)(0), points.at(current_idx)(1), points.at(current_idx)(2), points.at(current_idx)(3), tolerance)) {
      current_idx++;
    }

    if (current_idx == points.size()) {
      return {true, "points passed"};
    }

    sleep(0.01);
  }
}

//}

/* sampleTrajectory() //{ */

std::vector<Eigen::Vector4d> TrajectoryTrackingTest::sampleTrajectory(const Eigen::Vector4d from, const Eigen::Vector4d to, const double dt,
                                                                      const double speed) {

  const Eigen::Vector3d from_pos = from.head(3);
  const Eigen::Vector3d to_pos   = to.head(3);

  const Eigen::Vector3d vec_to = to_pos - from_pos;

  const double dist_to = vec_to.norm();

  const Eigen::Vector3d dir_to = vec_to.normalized();

  const int    n_points  = floor((dist_to / speed) / dt);
  const double step_size = dist_to / n_points;

  std::vector<Eigen::Vector4d> trajectory;

  const double heading_dist = sradians::dist(from(3), to(3));

  trajectory.push_back(from);

  const double heading_step = heading_dist / n_points;

  Eigen::Vector4d current_point = from;

  for (int i = 1; i < n_points; i++) {

    current_point.head(3) += dir_to * step_size;
    current_point(3) += heading_step;

    trajectory.push_back(current_point);
  }

  trajectory.push_back(to);

  return trajectory;
}

//}

#endif // TRAJECTORY_TRACKING_TEST_H
