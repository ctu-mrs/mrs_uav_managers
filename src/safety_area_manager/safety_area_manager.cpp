/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/PathToPointInSafetyArea.h>
#include <mrs_msgs/HwApiCapabilities.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/GetPointStamped.h>
#include <mrs_msgs/GetBool.h>
#include <mrs_msgs/GetSafeZoneAtHeight.h>
#include <std_srvs/SetBool.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>

#include <mrs_lib/safety_zone/safety_zone.h>
#include <mrs_lib/safety_zone/static_edges_visualization.h>
#include <mrs_lib/safety_zone/int_edges_visualization.h>
#include <mrs_lib/safety_zone/vertex_control.h>
#include <mrs_lib/safety_zone/center_control.h>
#include <mrs_lib/safety_zone/bounds_control.h>
#include <mrs_lib/safety_zone/yaml_export_visitor.h>

#include <XmlRpcException.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <iostream>
#include <fstream>
#include <limits>
#include <memory>
#include <boost/geometry.hpp>


//}

namespace bg = boost::geometry;

namespace mrs_uav_managers
{

  namespace safety_area_manager
  {

    /* class SafetyAreaManager //{ */

    class SafetyAreaManager : public nodelet::Nodelet
    {
    private:
      std::shared_ptr<mrs_lib::Transformer> transformer_;
      std::unique_ptr<mrs_lib::SafetyZone> safety_zone_;
      ros::NodeHandle nh_;
      std::atomic<bool> is_initialized_ = false;

      // | ------------------- scope timer logger ------------------- |

      bool scope_timer_enabled_ = false;
      std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;


      // Wolrd config parameters
      std::string uav_name_;
      std::string safety_area_horizontal_frame_;
      std::string safety_area_vertical_frame_;
      std::string world_origin_units_;
      double origin_y_;
      double origin_x_;
      bool use_safety_area_;

      // Visualization objects
      std::vector<std::unique_ptr<mrs_lib::StaticEdgesVisualization>> static_edges_;
      std::vector<std::unique_ptr<mrs_lib::IntEdgesVisualization>> int_edges_;
      std::vector<std::unique_ptr<mrs_lib::VertexControl>> vertices_;
      std::vector<std::unique_ptr<mrs_lib::CenterControl>> centers_;
      std::vector<std::unique_ptr<mrs_lib::BoundsControl>> bounds_;

      // profiling
      mrs_lib::Profiler profiler_;
      bool profiler_enabled_ = false;
      int status_timer_rate_ = 0;


      // diagnostics publishing
      void publishDiagnostics(void);
      std::mutex mutex_diagnostics_;

      void preinitialize();
      void initialize();

      // | --------------------- service servers -------------------- |

      // safety area services
      ros::ServiceServer service_server_get_safety_zone_at_height_;
      ros::ServiceServer service_server_point_in_safety_area_3d_;
      ros::ServiceServer service_server_point_in_safety_area_2d_;
      ros::ServiceServer service_server_path_in_safety_area_3d_;
      ros::ServiceServer service_server_path_in_safety_area_2d_;
      ros::ServiceServer service_server_save_world_config_;
      ros::ServiceServer service_server_load_world_config_;
      ros::ServiceServer service_server_set_world_config_;
      ros::ServiceServer service_server_get_world_config_;
      ros::ServiceServer service_server_use_safety_area_;
      ros::ServiceServer service_server_add_obstacle_;
      ros::ServiceServer service_server_get_max_z_;
      ros::ServiceServer service_server_get_min_z_;
      ros::ServiceServer service_server_get_use_;

      // | ----------------------- subscribers ----------------------- |

      mrs_lib::SubscribeHandler<mrs_msgs::HwApiCapabilities> sh_hw_api_capabilities_;

      // | ----------------------- publishers ----------------------- |

      // | ----------------------- timers ----------------------- |

      // this timer will check till we already got the hardware api diagnostics
      // then it will trigger the initialization of the controllers and finish
      // the initialization of the SafetyAreaManager
      ros::Timer timer_hw_api_capabilities_;
      void timerHwApiCapabilities([[maybe_unused]] const ros::TimerEvent& event);

      // timer for regular status publishing
      ros::Timer timer_status_;
      void timerStatus(const ros::TimerEvent& event);

      // | ----------------------- callbacks ----------------------- |

      // services
      bool callbackValidatePoint3d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);
      bool callbackValidatePoint2d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);
      bool callbackValidatePathToPoint3d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res);
      bool callbackValidatePathToPoint2d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res);
      bool callbackGetSafeZoneAtHeight(mrs_msgs::GetSafeZoneAtHeight::Request& req, mrs_msgs::GetSafeZoneAtHeight::Response& res);
      bool callbackSaveWorldConfig(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);
      bool callbackGetWorldConfig([[maybe_unused]] mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);
      bool callbackLoadWorldConfig(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);
      bool callbackSetWorldConfig(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);
      bool callbackToggleSafetyArea(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
      bool callbackAddObstacle(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);
      bool callbackGetMaxZ([[maybe_unused]] mrs_msgs::GetPointStamped::Request& req, mrs_msgs::GetPointStamped::Response& res);
      bool callbackGetMinZ([[maybe_unused]] mrs_msgs::GetPointStamped::Request& req, mrs_msgs::GetPointStamped::Response& res);
      bool callbackGetUse([[maybe_unused]] mrs_msgs::GetBool::Request& req, mrs_msgs::GetBool::Response& res);

      // | ----------------------- routines ----------------------- |

      // Safety area building
      std::unique_ptr<mrs_lib::Prism> makePrism(const Eigen::MatrixXd matrix, const double max_z, const double min_z);
      double transformZ(const std::string& current_frame, const std::string& target_frame, const double z);
      bool initializationFromFile(mrs_lib::ParamLoader& param_loader, std::string filename);

      // Reference validation
      // those are passed to trackers using the common_handlers object  TODO
      bool isPointInSafetyArea2d(const mrs_msgs::ReferenceStamped& point);
      bool isPointInSafetyArea3d(const mrs_msgs::ReferenceStamped& point);
      bool isPathToPointInSafetyArea2d(const mrs_msgs::ReferenceStamped& from, const mrs_msgs::ReferenceStamped& to);
      bool isPathToPointInSafetyArea3d(const mrs_msgs::ReferenceStamped& from, const mrs_msgs::ReferenceStamped& to);
      double getMinZ(const std::string& frame_id);
      double getMaxZ(const std::string& frame_id);

    public:
      virtual void onInit();

      ~SafetyAreaManager() = default;

    };  // class SafetyAreaManager

    //}

    /* onInit() //{ */
    void SafetyAreaManager::onInit()
    {
      preinitialize();
    }
    //}

    /* preinitialize() //{ */

    void SafetyAreaManager::preinitialize()
    {
      nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

      ros::Time::waitForValid();

      mrs_lib::SubscribeHandlerOptions shopts;
      shopts.nh = nh_;
      shopts.node_name = "SafetyAreaManager";
      shopts.no_message_timeout = mrs_lib::no_timeout;
      shopts.threadsafe = true;
      shopts.autostart = true;
      shopts.queue_size = 10;
      shopts.transport_hints = ros::TransportHints().tcpNoDelay();

      sh_hw_api_capabilities_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiCapabilities>(shopts, "hw_api_capabilities_in");

      timer_hw_api_capabilities_ = nh_.createTimer(ros::Rate(1.0), &SafetyAreaManager::timerHwApiCapabilities, this);
    }


    //}

    /* initialize() //{ */

    void SafetyAreaManager::initialize()
    {
      ROS_INFO("[SafetyAreaManager]: initializing");

      // | --------------------- parameters ---------------------- |

      mrs_lib::ParamLoader param_loader(nh_, "SafetyAreaManager");

      std::string world_config;
      param_loader.loadParam("world_config", world_config);

      param_loader.addYamlFileFromParam("private_config");
      param_loader.addYamlFileFromParam("public_config");

      param_loader.loadParam("uav_name", uav_name_);
      param_loader.loadParam("enable_profiler", profiler_enabled_);

      const std::string yaml_prefix = "mrs_uav_managers/safety_area_manager/";
      param_loader.loadParam(yaml_prefix + "status_timer_rate", status_timer_rate_);

      // | ---------------------- transformer ----------------------- |

      transformer_ = std::make_shared<mrs_lib::Transformer>(nh_, "SafetyAreaManager");
      transformer_->setDefaultPrefix(uav_name_);

      // | ---------------------- safety zone ----------------------- |
      // Note: safety_zone is initialized even if the use_safety_area_ is false
      // The manager will just always return true untill it's turned on
      bool safety_zone_inited = false;
      try
      {
        safety_zone_inited = initializationFromFile(param_loader, world_config);
      }
      catch (std::invalid_argument& e)
      {
        ROS_ERROR("[SafetyAreaManager]: wrong configruation for the safety zone polygons. %s", e.what());
        ros::shutdown();
      }
      catch (XmlRpc::XmlRpcException& e)
      {
        ROS_ERROR("[SafetyAreaManager]: error during parsing parameters. Please make sure parameters are written correctly");
        ros::shutdown();
      }
      catch (...)
      {
        ROS_ERROR("[SafetyAreaManager]: unhandled exception!");
        ros::shutdown();
      }

      ROS_INFO("[SafetyAreaManager]: safety area initialized");

      // | ------------------------ services ------------------------ |

      service_server_get_safety_zone_at_height_ = nh_.advertiseService("get_safety_zone_at_height_in", &SafetyAreaManager::callbackGetSafeZoneAtHeight, this);
      service_server_point_in_safety_area_3d_ = nh_.advertiseService("point_in_safety_area_3d_in", &SafetyAreaManager::callbackValidatePoint3d, this);
      service_server_point_in_safety_area_2d_ = nh_.advertiseService("point_in_safety_area_2d_in", &SafetyAreaManager::callbackValidatePoint2d, this);
      service_server_path_in_safety_area_3d_ = nh_.advertiseService("path_in_safety_area_3d_in", &SafetyAreaManager::callbackValidatePathToPoint3d, this);
      service_server_path_in_safety_area_2d_ = nh_.advertiseService("path_in_safety_area_2d_in", &SafetyAreaManager::callbackValidatePathToPoint2d, this);
      service_server_save_world_config_ = nh_.advertiseService("save_world_config_in", &SafetyAreaManager::callbackSaveWorldConfig, this);
      service_server_load_world_config_ = nh_.advertiseService("load_world_config_in", &SafetyAreaManager::callbackLoadWorldConfig, this);
      service_server_set_world_config_ = nh_.advertiseService("set_world_config_in", &SafetyAreaManager::callbackSetWorldConfig, this);
      service_server_get_world_config_ = nh_.advertiseService("get_world_config_in", &SafetyAreaManager::callbackGetWorldConfig, this);
      service_server_use_safety_area_ = nh_.advertiseService("set_use_safety_area_in", &SafetyAreaManager::callbackToggleSafetyArea, this);
      service_server_add_obstacle_ = nh_.advertiseService("add_obstacle_in", &SafetyAreaManager::callbackAddObstacle, this);
      service_server_get_max_z_ = nh_.advertiseService("get_max_z_in", &SafetyAreaManager::callbackGetMaxZ, this);
      service_server_get_min_z_ = nh_.advertiseService("get_min_z_in", &SafetyAreaManager::callbackGetMinZ, this);
      service_server_get_use_ = nh_.advertiseService("get_use_in", &SafetyAreaManager::callbackGetUse, this);


      // | ------------------------- timers ------------------------- |

      timer_status_ = nh_.createTimer(ros::Rate(status_timer_rate_), &SafetyAreaManager::timerStatus, this);

      // | ------------------------ profiler ------------------------ |

      profiler_ = mrs_lib::Profiler(nh_, "SafetyAreaManager", profiler_enabled_);

      // | ------------------- scope timer logger ------------------- |

      param_loader.loadParam(yaml_prefix + "scope_timer/enabled", scope_timer_enabled_);
      const std::string scope_timer_log_filename = param_loader.loadParam2("scope_timer/log_filename", std::string(""));
      scope_timer_logger_ = std::make_shared<mrs_lib::ScopeTimerLogger>(scope_timer_log_filename, scope_timer_enabled_);

      // | ----------------------- finish init ---------------------- |

      if (!param_loader.loadedSuccessfully() || !safety_zone_inited)
      {
        ROS_ERROR("[SafetyAreaManager]: could not load all parameters!");
        ros::shutdown();
      }

      is_initialized_ = true;
      ROS_INFO("[SafetyAreaManager]: initialized");
    }

    //}

    // --------------------------------------------------------------
    // |                          timers                            |
    // --------------------------------------------------------------

    /* timerHwApiCapabilities() //{ */

    void SafetyAreaManager::timerHwApiCapabilities([[maybe_unused]] const ros::TimerEvent& event)
    {
      mrs_lib::Routine profiler_routine = profiler_.createRoutine("timerHwApiCapabilities", status_timer_rate_, 1.0, event);
      mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("SafetyAreaManager::timerHwApiCapabilities", scope_timer_logger_, scope_timer_enabled_);

      if (!sh_hw_api_capabilities_.hasMsg())
      {
        ROS_INFO_THROTTLE(1.0, "[SafetyAreaManager]: waiting for HW API capabilities");
        ROS_INFO("[SafetyAreaManager]: waiting for HW API capabilities");
        return;
      }

      // Note: all the hw_ap_capabilities seemed to be useless

      initialize();
      timer_hw_api_capabilities_.stop();
    }
    //}

    /* timerStatus //{ */

    void SafetyAreaManager::timerStatus(const ros::TimerEvent& event)
    {

      if (!is_initialized_)
      {
        return;
      }

      mrs_lib::Routine profiler_routine = profiler_.createRoutine("timerStatus", status_timer_rate_, 0.1, event);
      mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("ControlManager::timerStatus", scope_timer_logger_, scope_timer_enabled_);


      publishDiagnostics();
    }

    //}

    // --------------------------------------------------------------
    // |                          callbacks                          |
    // --------------------------------------------------------------

    // | -------------------- service callbacks ------------------- |

    /* callbackAddObstacle() //{ */

    bool SafetyAreaManager::callbackAddObstacle(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res)
    {

      if (!is_initialized_)
      {
        res.message = "not initialized";
        res.success = false;
        return true;
      }

      mrs_msgs::ReferenceStamped point;
      point.header = req.header;
      point.reference = req.reference;
      auto tfed_horizontal = transformer_->transformSingle(point, safety_area_horizontal_frame_);

      if (!tfed_horizontal)
      {
        ROS_WARN_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
        res.message = "Could not transform the point to the safety area horizontal frame";
        res.success = false;
        return true;
      }

      double offset_x = tfed_horizontal->reference.position.x;
      double offset_y = tfed_horizontal->reference.position.y;

      std::vector<mrs_lib::Point2d> points = {
          mrs_lib::Point2d{2.5 + offset_x, 2.5 + offset_y},
          mrs_lib::Point2d{2.5 + offset_x, -2.5 + offset_y},
          mrs_lib::Point2d{-2.5 + offset_x, -2.5 + offset_y},
          mrs_lib::Point2d{-2.5 + offset_x, 2.5 + offset_y},
      };

      int id = safety_zone_->addObstacle(std::make_unique<mrs_lib::Prism>(points, 5, 0));

      static_edges_.push_back(
          std::make_unique<mrs_lib::StaticEdgesVisualization>(safety_zone_.get(), id, uav_name_ + "/" + safety_area_horizontal_frame_, nh_, 2));
      int_edges_.push_back(std::make_unique<mrs_lib::IntEdgesVisualization>(safety_zone_.get(), id, uav_name_ + "/" + safety_area_horizontal_frame_, nh_));
      vertices_.push_back(std::make_unique<mrs_lib::VertexControl>(safety_zone_.get(), id, uav_name_ + "/" + safety_area_horizontal_frame_, nh_));
      centers_.push_back(std::make_unique<mrs_lib::CenterControl>(safety_zone_.get(), id, uav_name_ + "/" + safety_area_horizontal_frame_, nh_));
      bounds_.push_back(std::make_unique<mrs_lib::BoundsControl>(safety_zone_.get(), id, uav_name_ + "/" + safety_area_horizontal_frame_, nh_));

      return true;
    }

    //}

    /* callbackToggleSafetyArea() //{ */

    bool SafetyAreaManager::callbackToggleSafetyArea(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
    {
      if (!is_initialized_)
      {
        res.success = false;
        return true;
      }
      use_safety_area_ = req.data;
      res.success = true;
      ROS_INFO("[SafetyAreaManager]: safety area usage has been turned %s", (use_safety_area_ ? "on" : "off"));
      return true;
    }

    //}

    /* callbackLoadWorldConfig() //{ */

    bool SafetyAreaManager::callbackLoadWorldConfig(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res)
    {

      if (!is_initialized_)
      {
        res.message = "not initialized";
        res.success = false;
        return true;
      }
      // Backup
      auto old_safety_zone = std::move(safety_zone_);
      auto old_static_edges = std::move(static_edges_);
      auto old_int_edges = std::move(int_edges_);
      auto old_vertices = std::move(vertices_);
      auto old_centers = std::move(centers_);
      auto old_bounds = std::move(bounds_);
      auto old_world_origin_units = world_origin_units_;
      auto old_safety_area_horizontal_frame = safety_area_horizontal_frame_;
      auto old_safety_area_vertical_frame = safety_area_vertical_frame_;
      auto old_origin_y = origin_y_;
      auto old_origin_x = origin_x_;
      auto old_use_safety_area = use_safety_area_;

      mrs_lib::ParamLoader param_loader(nh_, "SafetyAreaManager");
      bool success = initializationFromFile(param_loader, req.value);

      res.success = true;
      if (!param_loader.loadedSuccessfully())
      {
        ROS_WARN("[SafetyAreaManager]: Could not read the file. Probably data format is not correct.");
        res.message = "Could not read the file. Probably data format is not correct.";
        res.success = false;
      }
      if (!success)
      {
        ROS_WARN("[SafetyAreaManager]: Could not load world config. Please, check the config file.");
        res.message = "Could not load world config. Please, check the config file.";
        res.success = false;
      }

      // If smth went wrong, restore from backup, if successful no need to delete the backups as they will get destroyed automatically
      if (!res.success)
      {
        if (safety_zone_)
        {
          safety_zone_ = std::move(old_safety_zone);
          static_edges_ = std::move(old_static_edges);
          int_edges_ = std::move(old_int_edges);
          vertices_ = std::move(old_vertices);
          centers_ = std::move(old_centers);
          bounds_ = std::move(old_bounds);
          world_origin_units_ = old_world_origin_units;
          safety_area_horizontal_frame_ = old_safety_area_horizontal_frame;
          safety_area_vertical_frame_ = old_safety_area_vertical_frame;
          origin_y_ = old_origin_y;
          origin_x_ = old_origin_x;
          use_safety_area_ = old_use_safety_area;
        }
      }
      return true;
    }

    //}

    /* callbackSetWorldConfig() //{ */

    bool SafetyAreaManager::callbackSetWorldConfig(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res)
    {

      if (!is_initialized_)
      {
        res.message = "not initialized";
        res.success = false;
        return true;
      }
      std::string filename = "/tmp/cur_world_config.yaml";
      std::ofstream ofs(filename, std::ofstream::out | std::ofstream::trunc);
      if (!ofs.is_open())
      {
        ROS_WARN("[SafetyAreaManager]: Could not open file %s", filename.c_str());
        res.success = false;
        res.message = "Could not open file " + filename;
        return true;
      }

      ofs << req.value;
      ofs.close();

      mrs_msgs::String load_config_srv;
      load_config_srv.request.value = filename;
      callbackLoadWorldConfig(load_config_srv.request, load_config_srv.response);
      res = load_config_srv.response;
      return true;
    }

    //}

    /* callbackSaveWorldConfig() //{ */

    bool SafetyAreaManager::callbackSaveWorldConfig(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res)
    {
      if (!is_initialized_)
      {
        res.message = "not initialized";
        res.success = false;
        return true;
      }

      mrs_lib::YamlExportVisitor visitor(uav_name_, safety_area_horizontal_frame_, safety_area_horizontal_frame_, safety_area_vertical_frame_,
                                         world_origin_units_, origin_x_, origin_y_);

      safety_zone_->accept(visitor);

      if (!visitor.isSuccessful())
      {
        res.message = "Something went wrong during exporting parameters";
        res.success = false;
        return true;
      }

      std::ofstream ofs(req.value, std::ofstream::out | std::ofstream::trunc);
      if (!ofs.is_open())
      {
        ROS_WARN("[SafetyAreaManager]: Could not open file %s", req.value.c_str());
        res.success = false;
        res.message = "Could not open file " + req.value;
        return true;
      }

      ofs << visitor.getResult();
      ofs.close();
      res.success = true;
      ROS_INFO("[SafetyAreaManager]: world config has been saved to %s", req.value.c_str());
      return true;
    }

    //}

    /* callbackGetWorldConfig() //{ */

    bool SafetyAreaManager::callbackGetWorldConfig([[maybe_unused]] mrs_msgs::String::Request& req, mrs_msgs::String::Response& res)
    {

      if (!is_initialized_)
      {
        res.message = "not initialized";
        res.success = false;
        return true;
      }

      mrs_lib::YamlExportVisitor visitor(uav_name_, safety_area_horizontal_frame_, safety_area_horizontal_frame_, safety_area_vertical_frame_,
                                         world_origin_units_, origin_x_, origin_y_);

      safety_zone_->accept(visitor);

      if (!visitor.isSuccessful())
      {
        res.message = "Something went wrong during exporting parameters";
        res.success = false;
        return true;
      }

      res.success = true;
      res.message = visitor.getResult();
      ROS_INFO("[SafetyAreaManager]: world config has been extracted");
      return true;
    }

    //}

    /* callbackValidatePoint3d() //{ */

    bool SafetyAreaManager::callbackValidatePoint3d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res)
    {

      if (!is_initialized_)
      {
        res.message = "not initialized";
        res.success = false;
        return true;
      }

      if (!use_safety_area_)
      {
        res.success = true;
        res.message = "Safety area is disabled";
        return true;
      }

      mrs_msgs::ReferenceStamped point;
      point.header = req.header;
      point.reference = req.reference;

      auto tfed_horizontal = transformer_->transformSingle(point, safety_area_horizontal_frame_);

      if (!tfed_horizontal)
      {
        ROS_WARN_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
        res.message = "Could not transform the point to the safety area horizontal frame";
        res.success = false;
        return true;
      }

      if (!safety_zone_->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y, tfed_horizontal->reference.position.z))
      {
        res.success = false;
        return true;
      }

      res.success = true;
      return true;
    }

    //}

    /* callbackValidatePoint2d() //{ */

    bool SafetyAreaManager::callbackValidatePoint2d(mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res)
    {
      if (!is_initialized_)
      {
        res.message = "not initialized";
        res.success = false;
        return true;
      }

      if (!use_safety_area_)
      {
        res.success = true;
        res.message = "Safety area is disabled";
        return true;
      }

      mrs_msgs::ReferenceStamped point;
      point.reference = req.reference;
      point.header = req.header;

      auto tfed_horizontal = transformer_->transformSingle(point, safety_area_horizontal_frame_);

      if (!tfed_horizontal)
      {
        ROS_WARN_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
        res.message = "Could not transform the point to the safety area horizontal frame";
        res.success = false;
        return true;
      }

      if (!safety_zone_->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y))
      {
        res.success = false;

        return true;
      }
      res.success = true;
      return true;
    }

    //}

    /* callbackValidatePathtoPoint3d() //{ */

    bool SafetyAreaManager::callbackValidatePathToPoint3d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res)
    {

      if (!is_initialized_)
      {
        res.message = "not initialized";
        res.success = false;
        return true;
      }

      if (!use_safety_area_)
      {
        res.success = true;
        res.message = "Safety area is disabled";
        return true;
      }

      geometry_msgs::PointStamped start = req.start;
      geometry_msgs::PointStamped end = req.end;

      // transform points
      geometry_msgs::PointStamped start_transformed, end_transformed;

      {
        auto resp = transformer_->transformSingle(start, safety_area_horizontal_frame_);

        if (!resp)
        {
          ROS_WARN_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
          res.message = "Could not transform the first point in the path";
          res.success = false;
          return true;
        }

        start_transformed = resp.value();
      }

      {
        auto resp = transformer_->transformSingle(end, safety_area_horizontal_frame_);

        if (!resp)
        {
          ROS_WARN_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
          res.message = "Could not transform the first point in the path";
          return true;
        }

        end_transformed = resp.value();
      }

      // verify the whole path
      mrs_lib::Point3d start_point, end_point;
      start_point.set<0>(start_transformed.point.x);
      start_point.set<1>(start_transformed.point.y);
      start_point.set<2>(start_transformed.point.z);
      end_point.set<0>(end_transformed.point.x);
      end_point.set<1>(end_transformed.point.y);
      end_point.set<2>(end_transformed.point.z);

      if (!safety_zone_->isPathValid(start_point, end_point))
      {
        res.message = "The path is not valid";
        res.success = false;
        return true;
      }

      res.success = true;
      return true;
    }

    //}

    /* callbackValidatePathtoPoint2d() //{ */

    bool SafetyAreaManager::callbackValidatePathToPoint2d(mrs_msgs::PathToPointInSafetyArea::Request& req, mrs_msgs::PathToPointInSafetyArea::Response& res)
    {

      if (!is_initialized_)
      {
        res.message = "not initialized";
        res.success = false;
        return true;
      }

      if (!use_safety_area_)
      {
        res.success = true;
        res.message = "Safety area is disabled";
        return true;
      }

      geometry_msgs::PointStamped start = req.start;
      geometry_msgs::PointStamped end = req.end;

      // transform points
      geometry_msgs::PointStamped start_transformed, end_transformed;

      {
        auto resp = transformer_->transformSingle(start, safety_area_horizontal_frame_);

        if (!resp)
        {
          ROS_WARN_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
          res.message = "Could not transform the first point in the path";
          res.success = false;
          return true;
        }

        start_transformed = resp.value();
      }

      {
        auto resp = transformer_->transformSingle(end, safety_area_horizontal_frame_);

        if (!resp)
        {
          ROS_WARN_THROTTLE(1.0, "[SafetyAreaManager]: Could not transform the point to the safety area horizontal frame");
          res.message = "Could not transform the first point in the path";
          return true;
        }

        end_transformed = resp.value();
      }

      // verify the whole path
      mrs_lib::Point2d start_point, end_point;
      start_point.set<0>(start_transformed.point.x);
      start_point.set<1>(start_transformed.point.y);
      end_point.set<0>(end_transformed.point.x);
      end_point.set<1>(end_transformed.point.y);

      if (!safety_zone_->isPathValid(start_point, end_point))
      {
        res.message = "The path is not valid";
        res.success = false;
        return true;
      }

      res.success = true;
      return true;
    }

    //}

    /* callbackGetSafeZoneAtHeight() //{ */

    bool SafetyAreaManager::callbackGetSafeZoneAtHeight(mrs_msgs::GetSafeZoneAtHeight::Request& req, mrs_msgs::GetSafeZoneAtHeight::Response& res)
    {

      if (!is_initialized_)
      {
        return true;
      }

      // Transform height to the current frame
      double height = transformZ(req.header.frame_id, safety_area_horizontal_frame_, req.height);

      // If main prism is on different height, safety zone is empty
      if (safety_zone_->getBorder()->getMaxZ() < height || height < safety_zone_->getBorder()->getMinZ())
      {
        return true;
      }

      // Set response header
      res.header.frame_id = uav_name_ + "/" + safety_area_horizontal_frame_;

      // Add polygon of main prism
      auto border = safety_zone_->getBorder()->getPolygon().outer();
      geometry_msgs::Polygon border_polygon;
      for (size_t i = 0; i < border.size(); i++)
      {
        geometry_msgs::Point32 p;
        p.x = bg::get<0>(border[i]);
        p.y = bg::get<1>(border[i]);
        border_polygon.points.push_back(p);
      }
      res.safety_zone.push_back(border_polygon);

      // Add polygons of required obstacles
      for (auto it = safety_zone_->getObstaclesBegin(); it != safety_zone_->getObstaclesEnd(); it++)
      {
        if (it->second->getMaxZ() < height || height < it->second->getMinZ())
        {
          continue;
        }

        geometry_msgs::Polygon polygon;
        auto outer_ring = it->second->getPolygon().outer();
        for (size_t i = 0; i < outer_ring.size(); i++)
        {
          geometry_msgs::Point32 p;
          p.x = bg::get<0>(outer_ring[i]);
          p.y = bg::get<1>(outer_ring[i]);
          polygon.points.push_back(p);
        }
        res.safety_zone.push_back(polygon);
      }

      return true;
    }

    //}

    /* callbackGetMaxZ() //{ */

    bool SafetyAreaManager::callbackGetMaxZ([[maybe_unused]] mrs_msgs::GetPointStamped::Request& req, mrs_msgs::GetPointStamped::Response& res)
    {

      if (!is_initialized_)
      {
        res.success = false;
        return true;
      }

      res.result.header.frame_id = safety_area_horizontal_frame_;
      res.result.point.x = 0;
      res.result.point.y = 0;
      if (use_safety_area_)
      {
        res.result.point.z = safety_zone_->getBorder()->getMaxZ();
      } else
      {
        res.result.point.z = std::numeric_limits<double>::max();
      }

      res.success = true;
      return true;
    }

    //}

    /* callbackGetMinZ() //{ */

    bool SafetyAreaManager::callbackGetMinZ([[maybe_unused]] mrs_msgs::GetPointStamped::Request& req, mrs_msgs::GetPointStamped::Response& res)
    {

      if (!is_initialized_)
      {
        res.success = false;
        return true;
      }

      res.result.header.frame_id = safety_area_horizontal_frame_;
      res.result.point.x = 0;
      res.result.point.y = 0;
      if (use_safety_area_)
      {
        res.result.point.z = safety_zone_->getBorder()->getMinZ();
      } else
      {
        res.result.point.z = std::numeric_limits<double>::lowest();
      }

      res.success = true;
      return true;
    }

    //}

    /* callbackGetUse() //{ */

    bool SafetyAreaManager::callbackGetUse([[maybe_unused]] mrs_msgs::GetBool::Request& req, mrs_msgs::GetBool::Response& res)
    {
      if (!is_initialized_)
      {
        return true;
      }

      res.result = use_safety_area_;
      return true;
    }

    //}

    // --------------------------------------------------------------
    // |                          routines                          |
    // --------------------------------------------------------------

    // | -------------------- service callbacks ------------------- |

    /* initializationFromFile() //{ */

    bool SafetyAreaManager::initializationFromFile(mrs_lib::ParamLoader& param_loader, std::string filename)
    {
      if (!param_loader.addYamlFile(filename))
      {
        return false;
      }

      param_loader.loadParam("world_origin/units", world_origin_units_);
      param_loader.loadParam("world_origin/origin_x", origin_x_);
      param_loader.loadParam("world_origin/origin_y", origin_y_);
      param_loader.loadParam("safety_area/enabled", use_safety_area_);
      param_loader.loadParam("safety_area/horizontal_frame", safety_area_horizontal_frame_);
      param_loader.loadParam("safety_area/vertical_frame", safety_area_vertical_frame_);

      // Make border prism
      const Eigen::MatrixXd border_points = param_loader.loadMatrixDynamic2("safety_area/border/points", -1, 2);
      const auto max_z = param_loader.loadParam2<double>("safety_area/border/max_z");
      const auto min_z = param_loader.loadParam2<double>("safety_area/border/min_z");
      const auto transformed_max_z = transformZ(safety_area_vertical_frame_, safety_area_horizontal_frame_, max_z);
      const auto transformed_min_z = transformZ(safety_area_vertical_frame_, safety_area_horizontal_frame_, min_z);

      auto border = makePrism(border_points, transformed_max_z, transformed_min_z);

      // Making obstacle prisms
      std::vector<std::unique_ptr<mrs_lib::Prism>> obstacles;

      bool obstacles_present;
      param_loader.loadParam("safety_area/obstacles/present", obstacles_present);

      // If any is present, fill obstacles
      if (obstacles_present)
      {
        // Read parameters for obstacles
        std::vector<Eigen::MatrixXd> obstacles_mat;
        Eigen::MatrixXd max_z_mat;
        Eigen::MatrixXd min_z_mat;
        param_loader.loadMatrixDynamic("safety_area/obstacles/max_z", max_z_mat, -1, 1);
        param_loader.loadMatrixDynamic("safety_area/obstacles/min_z", min_z_mat, -1, 1);

        Eigen::MatrixXd current_mat = param_loader.loadMatrixDynamic2("safety_area/obstacles/data", -1, 2);
        Eigen::MatrixXd rows = param_loader.loadMatrixDynamic2("safety_area/obstacles/rows", -1, 1);

        obstacles.reserve(current_mat.size());

        int start_row = 0;
        obstacles_mat.reserve(rows.rows());

        // Iterate over obstacles matrix and extract points based on rows matrix
        //"rows" matrix define the points for every obstacle
        for (int i = 0; i < rows.rows(); i++)
        {
          int row_num = static_cast<int>(rows(i, 0));

          if (row_num < 0 || start_row + row_num > current_mat.rows())
          {
            ROS_WARN("[SafetyAreaManager]: Invalid obstacle rows!, check your config file");
            return false;
          }

          Eigen::MatrixXd obstacle_mat = current_mat.block(start_row, 0, row_num, current_mat.cols());
          obstacles_mat.push_back(obstacle_mat);
          start_row += row_num;
        }

        if (start_row != current_mat.rows())
        {
          ROS_WARN("[SafetyAreaManager]: The number of obstacles is not consistent! No obstacle has been added");
          return false;
        }

        if (!(max_z_mat.rows() == min_z_mat.rows() && min_z_mat.rows() == static_cast<long int>(obstacles_mat.size())))
        {
          ROS_WARN("[SafetyAreaManager]: The number of obstacles is not consistent! No obstacle has been added");
          return false;
        }

        // Make obstacle prisms
        for (size_t i = 0; i < obstacles_mat.size(); i++)
        {
          const auto obs_max_z = max_z_mat(i, 0);
          const auto obs_min_z = min_z_mat(i, 0);
          const auto transformed_obs_max_z = transformZ(safety_area_vertical_frame_, safety_area_horizontal_frame_, obs_max_z);
          const auto transformed_obs_min_z = transformZ(safety_area_vertical_frame_, safety_area_horizontal_frame_, obs_min_z);
          auto prism = makePrism(obstacles_mat[i], transformed_obs_max_z, transformed_obs_min_z);
          if (prism)
          {
            obstacles.push_back(std::move(prism));
          } else
          {
            ROS_WARN("[SafetyAreaManager]: Failed to create obstacle prism!");
          }
        }
      }

      safety_zone_ = std::make_unique<mrs_lib::SafetyZone>(*border, std::move(obstacles));

      // Add visualizations

      // Safety area
      static_edges_.push_back(std::make_unique<mrs_lib::StaticEdgesVisualization>(safety_zone_.get(), uav_name_ + "/" + safety_area_horizontal_frame_, nh_, 2));
      int_edges_.push_back(std::make_unique<mrs_lib::IntEdgesVisualization>(safety_zone_.get(), uav_name_ + "/" + safety_area_horizontal_frame_, nh_));
      vertices_.push_back(std::make_unique<mrs_lib::VertexControl>(safety_zone_.get(), uav_name_ + "/" + safety_area_horizontal_frame_, nh_));
      centers_.push_back(std::make_unique<mrs_lib::CenterControl>(safety_zone_.get(), uav_name_ + "/" + safety_area_horizontal_frame_, nh_));
      bounds_.push_back(std::make_unique<mrs_lib::BoundsControl>(safety_zone_.get(), uav_name_ + "/" + safety_area_horizontal_frame_, nh_));

      // Obstacles, overloading for obstacles
      for (auto it = safety_zone_->getObstaclesBegin(); it != safety_zone_->getObstaclesEnd(); it++)
      {
        static_edges_.push_back(
            std::make_unique<mrs_lib::StaticEdgesVisualization>(safety_zone_.get(), it->first, uav_name_ + "/" + safety_area_horizontal_frame_, nh_, 2));
        int_edges_.push_back(
            std::make_unique<mrs_lib::IntEdgesVisualization>(safety_zone_.get(), it->first, uav_name_ + "/" + safety_area_horizontal_frame_, nh_));
        vertices_.push_back(std::make_unique<mrs_lib::VertexControl>(safety_zone_.get(), it->first, uav_name_ + "/" + safety_area_horizontal_frame_, nh_));
        centers_.push_back(std::make_unique<mrs_lib::CenterControl>(safety_zone_.get(), it->first, uav_name_ + "/" + safety_area_horizontal_frame_, nh_));
        bounds_.push_back(std::make_unique<mrs_lib::BoundsControl>(safety_zone_.get(), it->first, uav_name_ + "/" + safety_area_horizontal_frame_, nh_));
      }

      return param_loader.loadedSuccessfully();
    }

    //}

    /* makePrism() //{ */

    std::unique_ptr<mrs_lib::Prism> SafetyAreaManager::makePrism(const Eigen::MatrixXd matrix, const double max_z, const double min_z)
    {

      if (matrix.rows() < 3)
      {
        ROS_WARN_STREAM("[SafetyAreaManager]: Invalid polygon, must have at least 3 points. Provided:  " << std::to_string(matrix.rows()));
      }

      std::vector<mrs_lib::Point2d> points;
      points.reserve(matrix.rows());

      for (int i = 0; i < matrix.rows(); i++)
      {
        points.emplace_back(mrs_lib::Point2d{matrix(i, 0), matrix(i, 1)});
      }

      return std::make_unique<mrs_lib::Prism>(points, max_z, min_z);
    }

    //}

    /* transformZ //{ */

    double SafetyAreaManager::transformZ(const std::string& current_frame, const std::string& target_frame, const double z)
    {
      geometry_msgs::Point point;
      point.x = 0;
      point.y = 0;
      point.z = z;

      const auto res = transformer_->transformSingle(current_frame, point, target_frame);
      if (!res)
      {
        ROS_WARN("[SafetyAreaManager]: Could not transform point from %s to %s.", current_frame.c_str(), target_frame.c_str());
        return 0;
      }

      return res.value().z;
    }

    //}

    /* //{ isPointInSafetyArea2d() */

    bool SafetyAreaManager::isPointInSafetyArea2d(const mrs_msgs::ReferenceStamped& point)
    {

      if (!use_safety_area_)
      {
        return true;
      }

      auto tfed_horizontal = transformer_->transformSingle(point, safety_area_horizontal_frame_);

      if (!tfed_horizontal)
      {
        ROS_ERROR_THROTTLE(1.0, "[SafetyAreaManager]: SafetyArea: Could not transform the point to the safety area horizontal frame");
        return false;
      }

      if (!safety_zone_->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y))
      {
        return false;
      }

      return true;
    }

    //}

    /* //{ isPointInSafetyArea3d() */

    bool SafetyAreaManager::isPointInSafetyArea3d(const mrs_msgs::ReferenceStamped& point)
    {

      if (!use_safety_area_)
      {
        return true;
      }

      auto tfed_horizontal = transformer_->transformSingle(point, safety_area_horizontal_frame_);

      if (!tfed_horizontal)
      {
        ROS_ERROR_THROTTLE(1.0, "[SafetyAreaManager]: SafetyArea: Could not transform the point to the safety area horizontal frame");
        return false;
      }

      if (!safety_zone_->isPointValid(tfed_horizontal->reference.position.x, tfed_horizontal->reference.position.y, tfed_horizontal->reference.position.z))
      {
        return false;
      }

      return true;

      return true;
    }

    //}

    /* //{ isPathToPointInSafetyArea2d() */

    bool SafetyAreaManager::isPathToPointInSafetyArea2d(const mrs_msgs::ReferenceStamped& start, const mrs_msgs::ReferenceStamped& end)
    {

      if (!use_safety_area_)
      {
        return true;
      }

      mrs_msgs::ReferenceStamped start_transformed, end_transformed;

      if (!isPointInSafetyArea2d(start) || !isPointInSafetyArea2d(end))
      {
        return false;
      }

      {
        auto ret = transformer_->transformSingle(start, safety_area_horizontal_frame_);

        if (!ret)
        {

          ROS_ERROR("[ControlManager]: SafetyArea: Could not transform the first point in the path");

          return false;
        }

        start_transformed = ret.value();
      }

      {
        auto ret = transformer_->transformSingle(end, safety_area_horizontal_frame_);

        if (!ret)
        {

          ROS_ERROR("[ControlManager]: SafetyArea: Could not transform the first point in the path");

          return false;
        }

        end_transformed = ret.value();
      }

      // verify the whole path
      mrs_lib::Point2d start_point, end_point;
      start_point.set<0>(start_transformed.reference.position.x);
      start_point.set<1>(start_transformed.reference.position.y);
      end_point.set<0>(end_transformed.reference.position.x);
      end_point.set<1>(end_transformed.reference.position.y);

      return safety_zone_->isPathValid(start_point, end_point);
    }

    //}

    /* //{ isPathToPointInSafetyArea3d() */

    bool SafetyAreaManager::isPathToPointInSafetyArea3d(const mrs_msgs::ReferenceStamped& start, const mrs_msgs::ReferenceStamped& end)
    {

      if (!use_safety_area_)
      {
        return true;
      }

      if (!isPointInSafetyArea3d(start) || !isPointInSafetyArea3d(end))
      {
        return false;
      }

      mrs_msgs::ReferenceStamped start_transformed, end_transformed;

      {
        auto ret = transformer_->transformSingle(start, safety_area_horizontal_frame_);

        if (!ret)
        {

          ROS_ERROR("[SafetyAreaManager]: SafetyArea: Could not transform the first point in the path");

          return false;
        }

        start_transformed = ret.value();
      }

      {
        auto ret = transformer_->transformSingle(end, safety_area_horizontal_frame_);

        if (!ret)
        {

          ROS_ERROR("[ControlManager]: SafetyArea: Could not transform the first point in the path");

          return false;
        }

        end_transformed = ret.value();
      }

      // verify the whole path
      mrs_lib::Point3d start_point, end_point;
      start_point.set<0>(start_transformed.reference.position.x);
      start_point.set<1>(start_transformed.reference.position.y);
      start_point.set<2>(start_transformed.reference.position.z);
      end_point.set<0>(end_transformed.reference.position.x);
      end_point.set<1>(end_transformed.reference.position.y);
      end_point.set<2>(end_transformed.reference.position.z);

      return safety_zone_->isPathValid(start_point, end_point);
    }

    //}

    /* publishDiagnostics() //{ */

    void SafetyAreaManager::publishDiagnostics(void)
    {

      if (!is_initialized_)
      {
        return;
      }

      mrs_lib::Routine profiler_routine = profiler_.createRoutine("publishDiagnostics");
      mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("SafetyAreaManager::publishDiagnostics", scope_timer_logger_, scope_timer_enabled_);

      std::scoped_lock lock(mutex_diagnostics_);
    }

    //}
    
  }  // namespace safety_area_manager

}  // namespace mrs_uav_managers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_managers::safety_area_manager::SafetyAreaManager, nodelet::Nodelet)
