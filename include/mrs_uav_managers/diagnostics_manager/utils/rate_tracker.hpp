#pragma once

#include <cstddef>
#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

namespace mrs_uav_managers::utils
{

/**
 * @brief Sliding-window message-rate tracker.
 *
 * Records arrival timestamps and computes the average rate over the most recent
 * window. Internally synchronised — record() may be called from a subscriber
 * callback thread while rate() is read from a timer thread.
 */
class RateTracker {
public:
  explicit RateTracker(std::size_t window_size = 10) : window_size_(window_size) {
  }

  /** @brief Record a new message arrival at @p t. */
  void record(const rclcpp::Time &t) {
    std::scoped_lock lck(mtx_);
    timestamps_.push_back(t);
    if (timestamps_.size() > window_size_) {
      timestamps_.pop_front();
    }
  }

  /** @brief Average rate over the current window, or 0.0 if insufficient samples. */
  double rate() const {
    std::scoped_lock lck(mtx_);
    if (timestamps_.size() < 2) {
      return 0.0;
    }
    const double span = (timestamps_.back() - timestamps_.front()).seconds();
    if (span <= 0.0) {
      return 0.0;
    }
    return static_cast<double>(timestamps_.size() - 1) / span;
  }

  void clear() {
    std::scoped_lock lck(mtx_);
    timestamps_.clear();
  }

private:
  mutable std::mutex       mtx_;
  std::size_t              window_size_;
  std::deque<rclcpp::Time> timestamps_;
};

} // namespace mrs_uav_managers::utils
