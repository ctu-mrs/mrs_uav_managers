#include <ros/ros.h>
#include <mrs_msgs/SetSafetyAreaSrv.h>
#include <mrs_msgs/SafetyAreaManagerDiagnostics.h>
#include <mrs_msgs/SetSafetyAreaSrvRequest.h>
#include <mrs_msgs/SetSafetyAreaResponse.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_mission_manager");

  ros::ServiceClient sc_set_safety_area_;

  ros::NodeHandle nh;

  sc_set_safety_area_ = nh.serviceClient<mrs_msgs::SetSafetyAreaSrv>("safety_area_manager/set_safety_area");

  ROS_INFO("Waiting for the 'svc/set_safety_area' service to become available...");
  sc_set_safety_area_.waitForExistence();
  ROS_INFO("Service is now available.");

  mrs_msgs::SetSafetyAreaSrv srv;
  // Populate the safety_area part of the request
  mrs_msgs::SafetyArea safety_area;

  // Set basic information
  safety_area.units = "LATLON";
  safety_area.origin_x = 47.397743;
  safety_area.origin_y = 8.545594;

  // Define the safety border
  mrs_msgs::SafetyBorder border;

  border.enabled = true;
  border.horizontal_frame = "world_origin";
  border.vertical_frame = "world_origin";
  border.max_z = 19.4;
  border.min_z = -0.000004;  // Corrected from border.max_z to border.min_z

  // Define the border points (4 points to form a polygon)
  std::vector<mrs_msgs::Point2D> border_points;

  mrs_msgs::Point2D point1;
  point1.x = -60.446158;
  point1.y = -55.604095;
  border_points.push_back(point1);

  mrs_msgs::Point2D point2;
  point2.x = -53.138152;
  point2.y = 44.709313;
  border_points.push_back(point2);

  mrs_msgs::Point2D point3;
  point3.x = 39.553842;
  point3.y = 44.395905;
  border_points.push_back(point3);

  mrs_msgs::Point2D point4;
  point4.x = 39.553842;
  point4.y = -55.604095;
  border_points.push_back(point4);

  border.points = border_points;
  
  ROS_INFO("Size of border points %d", static_cast<int>(border_points.size()));

  // Assign the border to the safety_area
  safety_area.border = border;

  // Define obstacles
  mrs_msgs::Obstacles obstacles;

  obstacles.present = true;

  // Populate obstacle data (each obstacle has x and y)
  std::vector<mrs_msgs::Point2D> obstacle_data;

  // List of obstacle points as per the YAML configuration
  std::vector<std::pair<double, double>> obstacle_points = {
    {-24.566868, 31.065876},
    {-25.059674, 13.487221},
    {-40.814034, 13.622604},
    {-41.041201, 30.935526},
    {-25.929022, -22.057648},
    {-25.293093, -41.950817},
    {-43.347534, -38.765572},
    {-43.665257, -20.787308},
    {19.488552, -37.909249},
    {19.488552, -42.909249},
    {7.187679, -45.003483},
    {7.036793, -29.641193}
  };

  for (const auto& op : obstacle_points) {
    mrs_msgs::Point2D obstacle_point;
    obstacle_point.x = op.first;
    obstacle_point.y = op.second;
    obstacle_data.push_back(obstacle_point);
  }

  /* obstacles.data = obstacle_data; */

  // Define rows, max_z, and min_z for obstacles
  obstacles.rows = std::vector<int>{4, 4, 4};
  obstacles.max_z = std::vector<double>{7.400008, 10.000000, 5.000000};
  obstacles.min_z = std::vector<double>{0.000001, 0.000000, 0.000000};

  // Assign obstacles to the safety_area
  safety_area.obstacles = obstacles;

  srv.request.safety_area = safety_area;

  // Call the service
  if (sc_set_safety_area_.call(srv))
  {
    ROS_INFO("Service call successful.");
  } else
  {
    ROS_ERROR("Failed to call service 'svc/set_safety_area'.");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
