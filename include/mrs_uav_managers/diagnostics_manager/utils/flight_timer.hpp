#pragma once

/* includes //{ */

#include <atomic>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <fstream>
#include <stdexcept>

//}

namespace mrs_uav_managers::diagnostics_manager::utils
{

/**
 * @brief File-backed cumulative-flight-time counter.
 *
 * On construction, attempts to load any previously persisted second count from
 * @p persist_path. Each call to tick() advances the counter while the UAV is
 * actively flying (null_tracker == false) and persists the new total to disk.
 */
class FlightTimer {
public:
  /**
   * @param clock        Used to stamp the takeoff moment and compute elapsed seconds.
   * @param persist_path On-disk file persisting secs_flown across restarts. Default is /tmp/mrs_status_flight_time.txt.
   */
  FlightTimer(rclcpp::Clock::SharedPtr clock, std::string persist_path = "/tmp/mrs_status_flight_time.txt");

  /**
   * @brief Update flight state, accumulate elapsed time, persist on each whole-second tick.
   * @param null_tracker True while the control tracker is in NullTracker state.
   */
  void tick(bool null_tracker);

  /* secsFlown() //{ */

  /** @brief Current accumulated flight time. */
  uint32_t secsFlown() const {
    return secs_flown_.load();
  }

  //}

private:
  void loadFromDisk();
  void persistToDisk();

  rclcpp::Clock::SharedPtr clock_;
  std::string              persist_path_;
  std::atomic<uint32_t>    secs_flown_ = 0;
  std::atomic<bool>        is_flying_  = false;
  rclcpp::Time             last_flight_time_;
};

} // namespace mrs_uav_managers::diagnostics_manager::utils
