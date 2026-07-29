#pragma once

#include <filesystem>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace mrs_uav_managers::utils
{

/**
 * @brief File-backed watt-hour integrator.
 *
 * On each integrate() call (typically from a battery-state callback), accumulates
 * voltage * current * dt / 3600 into a running total and persists it to disk.
 * First call seeds the timestamp but does not accumulate (dt is unknown).
 */
class WhDrainedIntegrator {
public:
  /**
   * @param clock        Used to compute dt between samples.
   * @param persist_path On-disk file persisting wh_drained across restarts. Default
   *                     matches data_acquisition's historical location.
   */
  WhDrainedIntegrator(rclcpp::Clock::SharedPtr clock, std::string persist_path = "/tmp/mrs_status_wh_drained.txt");

  /**
   * @brief Integrate the latest battery sample.
   * @param voltage Volts.
   * @param current Amps.
   */
  void integrate(double voltage, double current);

  /** @brief Total energy drained, in watt-hours. */
  double whDrained() const;

private:
  void loadFromDisk();
  void persistToDisk();

  rclcpp::Clock::SharedPtr clock_;
  std::string              persist_path_;
  mutable std::mutex       mutex_;
  double                   wh_drained_ = 0.0;
  rclcpp::Time             last_sample_;
  bool                     have_sample_ = false;
};

} // namespace mrs_uav_managers::utils
