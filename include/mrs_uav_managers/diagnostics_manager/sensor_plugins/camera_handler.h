#include <mrs_robot_diagnostics/sensor_handler.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <mrs_robot_diagnostics/SensorInfo.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


namespace mrs_robot_diagnostics
{

namespace sensor_handlers
{

namespace camera_handler
{

class CameraHandler : public mrs_robot_diagnostics::sensor_handlers::SensorHandler {
public:
  CameraHandler() : tf_listener_(tf_buffer_) {
  }
  bool initialize(ros::NodeHandle &nh, const std::string &name, const std::string &name_space, const std::string &topic) override;
  mrs_robot_diagnostics::SensorStatus updateStatus() override;


private:
  std::string _name_;
  std::string _topic_;
  bool is_initialized_ = false;
  bool is_active_      = false;
  std::string _camera_frame_;
  std::string _optical_frame_;
  std::string _fcu_frame_;


  mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> sh_camera_info_;
  mrs_lib::SubscribeHandler<sensor_msgs::Image> sh_image_;
  mrs_lib::SubscribeHandler<std_msgs::Float32MultiArray> sh_camera_orientation_;
  mrs_lib::PublisherHandler<mrs_robot_diagnostics::SensorInfo> ph_camera_details_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

};

} // namespace camera_handler
} // namespace sensor_handlers
} // namespace mrs_robot_diagnostics
