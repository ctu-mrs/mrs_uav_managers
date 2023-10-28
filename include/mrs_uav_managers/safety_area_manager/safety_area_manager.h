#include "ros/ros.h"

#include "mrs_msgs/ReferenceStamped.h"

#include "std_msgs/String.h"

#include "mrs_lib/mutex.h"
#include "mrs_lib/transformer.h"
#include "mrs_lib/safety_zone/safety_zone.h"

#include "nodelet/nodelet.h"

// TODO: viz .cpp file

namespace mrs_uav_managers
{

namespace safety_area_manager 
{

class SafetyAreaManager : public nodelet::Nodelet
{
private:
    std::shared_ptr<mrs_lib::Transformer> transformer_;

    std::mutex                           mutex_safety_area_min_z_;
    double                               safety_area_min_z_ = 0;
    std::string                          _safety_area_frame_;
    std::unique_ptr<mrs_lib::SafetyZone> safety_zone_;

    ros::NodeHandle nh;
    ros::Publisher publisher;

    /* data */
public:
    virtual void onInit();

    // SafetyAreaManager(/* args */);
    // ~SafetyAreaManager();

    bool isPointInSafetyArea3d(const mrs_msgs::ReferenceStamped point);

    bool isPointInSafetyArea2d(const mrs_msgs::ReferenceStamped point);

    bool isPathToPointInSafetyArea3d(const mrs_msgs::ReferenceStamped start, const mrs_msgs::ReferenceStamped end);

    bool isPathToPointInSafetyArea2d(const mrs_msgs::ReferenceStamped start, const mrs_msgs::ReferenceStamped end);

    double getMaxZ(const std::string& frame_id);

    double getMinZ(const std::string& frame_id);

}; // class SafetyAreaManager

} // namespace safety_area_manager 

} // namespace mrs_uav_managers