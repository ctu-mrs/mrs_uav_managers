#include <gtest/gtest.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>

#include <std_msgs/Bool.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/EstimationDiagnostics.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <ros/package.h>

#include <boost/process.hpp>

/* TEST(TESTSuite, takeoff) //{ */

TEST(TESTSuite, takeoff) {

  int result = 0;

  // | ------------------ initialize test node ------------------ |

  std::cout << "[Test]: initializing the test node" << std::endl;

  ros::NodeHandle nh = ros::NodeHandle("~");

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

  mrs_lib::SubscribeHandler<std_msgs::Bool> sh_can_takeoff = mrs_lib::SubscribeHandler<std_msgs::Bool>(shopts, "/" + uav_name + "/automatic_start/can_takeoff");

  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag =
      mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "/" + uav_name + "/control_manager/diagnostics");

  std::cout << "[Test]: subscribers initialized" << std::endl;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<std_srvs::SetBool> sch_arming   = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh, "/" + uav_name + "/hw_api/arming");
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_offboard = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh, "/" + uav_name + "/hw_api/offboard");

  std::cout << "[Test]: service clients initialized" << std::endl;

  // | ---------------- wait for ready to takeoff --------------- |

  while (ros::ok()) {

    std::cout << "[Test]: waiting for 'ready to takeoff'" << std::endl;

    if (sh_can_takeoff.hasMsg() && sh_can_takeoff.getMsg()->data) {
      break;
    }

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  std::cout << "[Test]: ready for takeoff" << std::endl;

  // | ---------------------- arm the drone --------------------- |

  std::cout << "[Test]: arming the drone" << std::endl;

  std_srvs::SetBool arming;
  arming.request.data = true;

  sch_arming.call(arming);

  // | -------------------------- wait -------------------------- |

  ros::Duration(0.2).sleep();

  // | -------------------- trigger offboard -------------------- |

  std::cout << "[Test]: triggering offboard" << std::endl;

  std_srvs::Trigger offboard;

  sch_offboard.call(offboard);

  // | -------------- wait for the takeoff finished ------------- |

  while (ros::ok()) {

    std::cout << "[Test]: waiting for the takeoff to finish" << std::endl;

    if (sh_control_manager_diag.getMsg()->flying_normally) {
      result = 1;
      break;
    }

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  std::cout << "[Test]: finished" << std::endl;

  EXPECT_TRUE(result);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "Test");

  return RUN_ALL_TESTS();
}
