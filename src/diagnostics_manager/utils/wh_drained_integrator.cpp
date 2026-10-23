#include <mrs_uav_managers/diagnostics_manager/utils/wh_drained_integrator.hpp>

namespace mrs_uav_managers::diagnostics_manager::utils
{

WhDrainedIntegrator::WhDrainedIntegrator(rclcpp::Clock::SharedPtr clock, std::string persist_path)
    : clock_(std::move(clock)), persist_path_(std::move(persist_path)), last_sample_(0, 0, clock_->get_clock_type()) {
  loadFromDisk();
}

void WhDrainedIntegrator::integrate(double voltage, double current) {

  std::scoped_lock lock(mutex_);

  const auto now = clock_->now();

  if (!have_sample_) {
    have_sample_ = true;
    last_sample_ = now;
    return;
  }

  const double dt_seconds = (now - last_sample_).seconds();
  last_sample_            = now;

  // Ignore non-positive time deltas
  if (dt_seconds <= 0.0) {
    return;
  }

  // V * I * (dt / 3600) gives watt-hours
  wh_drained_ += voltage * current * (dt_seconds / 3600.0);
  persistToDisk();
}

double WhDrainedIntegrator::whDrained() const {
  std::scoped_lock lock(mutex_);
  return wh_drained_;
}

void WhDrainedIntegrator::loadFromDisk() {
  if (!std::filesystem::exists(persist_path_)) {
    return;
  }
  std::ifstream file(persist_path_);
  std::string   line;
  if (!std::getline(file, line)) {
    return;
  }
  try {
    wh_drained_ = std::stod(line);
  }
  catch (const std::invalid_argument &) {
    wh_drained_ = 0.0;
  }
  catch (const std::out_of_range &) {
    wh_drained_ = 0.0;
  }
}

void WhDrainedIntegrator::persistToDisk() {
  const std::string tmp_path = persist_path_ + ".tmp";
  {
    std::ofstream file(tmp_path, std::ios::trunc);
    file << wh_drained_;
  }
  std::filesystem::rename(tmp_path, persist_path_);
}

} // namespace mrs_uav_managers::diagnostics_manager::utils
