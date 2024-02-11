#include <mrs_uav_testing/test_generic.h>

class FailedTakeoffTest : public mrs_uav_testing::TestGeneric {

public:
  bool test();

  FailedTakeoffTest();

private:
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_takeoff_;
};

FailedTakeoffTest::FailedTakeoffTest() : mrs_uav_testing::TestGeneric() {

  sch_takeoff_ = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/uav_manager/takeoff");
}

bool FailedTakeoffTest::test() {

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh;

  {
    auto [uhopt, message] = getUAVHandler(_uav_name_);

    if (!uhopt) {
      ROS_ERROR("[%s]: Failed obtain handler for '%s': '%s'", ros::this_node::getName().c_str(), _uav_name_.c_str(), message.c_str());
      return false;
    }

    uh = uhopt.value();
  }

  // | ---------------- wait for ready to takeoff --------------- |

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the MRS UAV System", name_.c_str());

    if (uh->mrsSystemReady()) {
      ROS_INFO("[%s]: MRS UAV System is ready", name_.c_str());
      break;
    }

    sleep(0.01);
  }

  // | ---------------------- arm the drone --------------------- |

  ROS_INFO("[%s]: arming the drone", name_.c_str());

  {
    std_srvs::SetBool srv;
    srv.request.data = true;

    {
      bool service_call = uh->sch_arming_.call(srv);

      if (!service_call || !srv.response.success) {
        return false;
      }
    }
  }

  // | ---------------------- wait a second --------------------- |

  sleep(2.0);

  // | --------------------- check if armed --------------------- |

  if (!uh->sh_hw_api_status_.getMsg()->armed) {
    return false;
  }

  // | ------------------- switch to offboard ------------------- |

  {
    std_srvs::Trigger srv;

    {
      bool service_call = uh->sch_offboard_.call(srv);

      if (!service_call || !srv.response.success) {
        return false;
      }
    }
  }

  // | -------------------------- wait -------------------------- |

  sleep(2.0);

  // | ------------------ check if in offboard ------------------ |

  if (!uh->sh_hw_api_status_.getMsg()->offboard) {
    return false;
  }

  sleep(2.0);

  // | ------------------------- takeoff ------------------------ |

  {
    std_srvs::Trigger srv;

    {
      bool service_call = sch_takeoff_.call(srv);

      if (service_call && srv.response.success) {
        ROS_ERROR("[%s]: takeoff call success, this should not happen", ros::this_node::getName().c_str());
        return false;
      }
    }
  }

  // | -------------------- check if disarmed ------------------- |

  sleep(0.1);

  if (uh->sh_hw_api_status_.getMsg()->armed) {
    ROS_ERROR("[%s]: the uav is still armed", ros::this_node::getName().c_str());
    return false;
  }

  return true;
}
