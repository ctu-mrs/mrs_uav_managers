#pragma once
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace enum_helpers
{

// Holds one Enum_T value and logs every change, tagged with a human-readable name (e.g. "UAV state").
template <typename Enum_T>
struct enum_updater
{
public:
  /* enum_updater() //{ */

  // Initializes the value to Enum_T::UNKNOWN.
  enum_updater(const rclcpp::Logger logger, const std::string_view name) : m_name(name), m_enum(Enum_T::UNKNOWN), logger_(logger) {
  }

  // Initializes the value to init_value.
  enum_updater(const rclcpp::Logger logger, const std::string_view name, const Enum_T init_value) : m_name(name), m_enum(init_value), logger_(logger) {
  }

  //}

  /* value() //{ */

  // Returns the current value.
  Enum_T value() {
    return m_enum;
  }

  //}

  /* set() //{ */

  // Updates the value, logging the transition; a no-op (and no log) if new_value equals the current value.
  void set(const Enum_T new_value) {
    if (m_enum == new_value)
      return;

    // ROS_INFO_STREAM("Changing " << m_name << ": \"" << to_string(m_enum) << "\" => \"" << to_string(new_value) << "\"");
    RCLCPP_INFO_STREAM(logger_, "Changing " << m_name << ": \"" << to_string(m_enum) << "\" => \"" << to_string(new_value) << "\"");
    m_enum = new_value;
  }

  //}

  /* operator==() //{ */

  // True if the current value equals other.
  bool operator==(const Enum_T other) {
    return other == m_enum;
  }

  //}

  /* operator!=() //{ */

  // True if the current value does not equal other.
  bool operator!=(const Enum_T other) {
    return other != m_enum;
  }

  //}

private:
  std::string    m_name;
  Enum_T         m_enum;
  rclcpp::Logger logger_;
};

} // namespace enum_helpers
