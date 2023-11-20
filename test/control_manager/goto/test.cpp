#include <ros/ros.h>
#include <ros/package.h>

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/attitude_converter.h>

#include <std_msgs/Bool.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/EstimationDiagnostics.h>
#include <mrs_msgs/Vec4.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <gtest/gtest.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

/* TEST(TESTSuite, goto) //{ */

TEST(TESTSuite, goto) {

  int result = 0;

  // | ------------------ initialize test node ------------------ |

  ros::NodeHandle nh = ros::NodeHandle("~");

  ROS_INFO("[%s]: ROS node initialized", ros::this_node::getName().c_str());

  ros::Time::waitForValid();

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

  std::cout << "[Test]: subscribers initialized" << std::endl;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<std_srvs::SetBool> sch_arming = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh, "/" + uav_name + "/hw_api/arming");
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_midair =
      mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh, "/" + uav_name + "/uav_manager/midair_activation");
  mrs_lib::ServiceClientHandler<mrs_msgs::Vec4> sch_goto = mrs_lib::ServiceClientHandler<mrs_msgs::Vec4>(nh, "/" + uav_name + "/control_manager/goto");

  std::cout << "[Test]: service clients initialized" << std::endl;

  // | ---------------- wait for ready to takeoff --------------- |

  while (ros::ok()) {

    std::cout << "[Test]: waiting for 'ready to midair activation'" << std::endl;

    if (sh_control_manager_diag.hasMsg() && sh_estim_manager_diag.hasMsg()) {
      break;
    }

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  std::cout << "[Test]: we are ready 'for midair activation'" << std::endl;

  ros::Duration(1.0).sleep();

  // | ---------------------- arm the drone --------------------- |

  std::cout << "[Test]: arming the drone" << std::endl;

  std_srvs::SetBool arming;
  arming.request.data = true;

  sch_arming.call(arming);

  // | -------------------------- wait -------------------------- |

  ros::Duration(0.2).sleep();

  // | -------------------- midair activation ------------------- |

  std::cout << "[Test]: activating the drone 'in mid air'" << std::endl;

  std_srvs::Trigger midair;

  sch_midair.call(midair);

  // | ----------------- sleep before the action ---------------- |

  ros::Duration(1.0).sleep();

  // --------------------------------------------------------------
  // |                          test goto                         |
  // --------------------------------------------------------------

  mrs_msgs::Vec4 goto_cmd;

  goto_cmd.request.goal[0] = -10;
  goto_cmd.request.goal[1] = -20;
  goto_cmd.request.goal[2] = 5.5;
  goto_cmd.request.goal[3] = 2.2;

  std::cout << "[Test]: calling goto" << std::endl;

  sch_goto.call(goto_cmd);

  // | ------------------ check for the result ------------------ |

  while (ros::ok()) {

    std::cout << "[Test]: waiting for the goto" << std::endl;

    auto diag = sh_estim_manager_diag.getMsg();

    auto hdg = mrs_lib::AttitudeConverter(diag->pose.orientation).getHeading();

    auto flying_normally = sh_control_manager_diag.getMsg()->flying_normally;

    if (std::abs(goto_cmd.request.goal[0] - diag->pose.position.x) < 1e-1 && std::abs(goto_cmd.request.goal[1] - diag->pose.position.y) < 1e-1 &&
        std::abs(goto_cmd.request.goal[2] - diag->pose.position.z) < 1e-1 && std::abs(goto_cmd.request.goal[3] - hdg) < 1e-1 && flying_normally) {
      result = 1;
      break;
    }

    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  ROS_INFO("[%s]: finished", ros::this_node::getName().c_str());

  EXPECT_TRUE(result);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "goto_test");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
