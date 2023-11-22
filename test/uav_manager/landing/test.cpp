#include <gtest/gtest.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>

#include <std_msgs/Bool.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/EstimationDiagnostics.h>
#include <mrs_msgs/Vec4.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

TEST(TESTSuite, landing) {

  // | ------------------ initialize test node ------------------ |

  ros::NodeHandle nh = ros::NodeHandle("~");

  ROS_INFO("[%s]: ROS node initialized", ros::this_node::getName().c_str());

  ros::Time::waitForValid();

  ros::AsyncSpinner spinner(2);
  spinner.start();

  std::string uav_name = "uav1";

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = "Test";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag =
      mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "/" + uav_name + "/control_manager/diagnostics");

  mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics> sh_estim_manager_diag =
      mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>(shopts, "/" + uav_name + "/estimation_manager/diagnostics");

  ROS_INFO("[%s]: subscribers initialized", ros::this_node::getName().c_str());

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<std_srvs::SetBool> sch_arming = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh, "/" + uav_name + "/hw_api/arming");

  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_midair =
      mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh, "/" + uav_name + "/uav_manager/midair_activation");

  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_land = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh, "/" + uav_name + "/uav_manager/land");

  mrs_lib::ServiceClientHandler<mrs_msgs::Vec4> sch_goto = mrs_lib::ServiceClientHandler<mrs_msgs::Vec4>(nh, "/" + uav_name + "/control_manager/goto");

  ROS_INFO("[%s]: service client initialized", ros::this_node::getName().c_str());

  // | ---------------- wait for ready to takeoff --------------- |

  while (ros::ok()) {

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the MRS UAV System", ros::this_node::getName().c_str());

    if (sh_control_manager_diag.hasMsg() && sh_estim_manager_diag.hasMsg()) {
      break;
    }

    ros::Duration(0.01).sleep();
  }

  ROS_INFO("[%s]: MRS UAV System is ready", ros::this_node::getName().c_str());

  ros::Duration(1.0).sleep();

  // | ---------------------- arm the drone --------------------- |

  ROS_INFO("[%s]: arming the edrone", ros::this_node::getName().c_str());

  std_srvs::SetBool arming;
  arming.request.data = true;

  {
    bool service_call = sch_arming.call(arming);

    if (!service_call || !arming.response.success) {
      ROS_ERROR("[%s]: arming service call failed", ros::this_node::getName().c_str());
      GTEST_FAIL();
    }
  }

  // | -------------------------- wait -------------------------- |

  ros::Duration(0.2).sleep();

  // | -------------------- midair activation ------------------- |

  ROS_INFO("[%s]: activating the drone 'in mid air'", ros::this_node::getName().c_str());

  std_srvs::Trigger midair;

  {
    bool service_call = sch_midair.call(midair);

    if (!service_call || !midair.response.success) {
      ROS_ERROR("[%s]: midair activation service call failed", ros::this_node::getName().c_str());
      GTEST_FAIL();
    }
  }

  // | ----------------- sleep before the action ---------------- |

  ros::Duration(1.0).sleep();

  // | -------------------------- goto -------------------------- |

  mrs_msgs::Vec4 goto_cmd;

  goto_cmd.request.goal[0] = 5;
  goto_cmd.request.goal[1] = 5;
  goto_cmd.request.goal[2] = 8;
  goto_cmd.request.goal[3] = 1;

  ROS_INFO("[%s]: calling goto", ros::this_node::getName().c_str());

  {
    bool service_call = sch_goto.call(goto_cmd);

    if (!service_call || !goto_cmd.response.success) {
      ROS_ERROR("[%s]: goto service call failed", ros::this_node::getName().c_str());
      GTEST_FAIL();
    }
  }

  ros::Duration(1.0).sleep();

  // | ------------ check for the result of the goto ------------ |

  while (ros::ok()) {

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for goto", ros::this_node::getName().c_str());

    if (!sh_control_manager_diag.getMsg()->tracker_status.have_goal) {

      ROS_INFO("[%s]: goto finished", ros::this_node::getName().c_str());
      break;
    }

    ros::Duration(0.01).sleep();
  }

  // | ------------------ initiate the landing ------------------ |

  ROS_INFO("[%s]: calling for landing", ros::this_node::getName().c_str());

  std_srvs::Trigger landing;

  {
    bool service_call = sch_land.call(landing);

    if (!service_call || !landing.response.success) {
      ROS_ERROR("[%s]: landing service call failed", ros::this_node::getName().c_str());
      GTEST_FAIL();
    }
  }

  // | ------------ waiting for the landing to finish ----------- |

  while (ros::ok()) {

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the landing to finish", ros::this_node::getName().c_str());

    if (!sh_control_manager_diag.getMsg()->output_enabled) {

      ROS_INFO("[%s]: finished", ros::this_node::getName().c_str());

      GTEST_SUCCEED();
      return;
    }

    ros::Duration(0.01).sleep();
  }

  GTEST_FAIL();
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "takeoff_test");

  return RUN_ALL_TESTS();
}
