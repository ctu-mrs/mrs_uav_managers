#include <mrs_uav_managers/diagnostics_manager/utils/flight_timer.hpp>

namespace mrs_uav_managers::diagnostics_manager::utils
{

FlightTimer::FlightTimer(rclcpp::Clock::SharedPtr clock, std::string persist_path)
    : clock_(std::move(clock)), persist_path_(std::move(persist_path)), last_flight_time_(0, 0, clock_->get_clock_type()) {
  loadFromDisk();
}

void FlightTimer::tick(bool null_tracker) {

  if (null_tracker) {
    is_flying_ = false;
    return;
  }

  // If we weren't flying before and tracker just became active
  if (!is_flying_) {
    is_flying_        = true;
    last_flight_time_ = clock_->now();
    return;
  }

  const auto now         = clock_->now();
  const int  secs_passed = static_cast<int>((now - last_flight_time_).seconds());

  if (secs_passed > 0) {
    secs_flown_.fetch_add(static_cast<uint32_t>(secs_passed));
    last_flight_time_ = last_flight_time_ + rclcpp::Duration(secs_passed, 0);
    persistToDisk();
  }
}

void FlightTimer::loadFromDisk() {
  if (!std::filesystem::exists(persist_path_)) {
    return;
  }
  std::ifstream file(persist_path_);
  std::string   line;
  if (!std::getline(file, line)) {
    return;
  }
  try {
    secs_flown_ = static_cast<uint32_t>(std::stoul(line));
  }
  catch (const std::invalid_argument &) {
    secs_flown_ = 0;
  }
  catch (const std::out_of_range &) {
    secs_flown_ = 0;
  }
}

void FlightTimer::persistToDisk() {
  std::ofstream file(persist_path_);
  file << secs_flown_.load();
}

} // namespace mrs_uav_managers::diagnostics_manager::utils
