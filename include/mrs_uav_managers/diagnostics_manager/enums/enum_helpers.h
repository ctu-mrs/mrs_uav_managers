#pragma once
#include <string>

namespace enum_helpers
{

  template <typename Enum_T>
  struct enum_updater
  {
    public:
      enum_updater(const std::string_view name) : m_name(name), m_enum(Enum_T::UNKNOWN) {}

      enum_updater(const std::string_view name, const Enum_T init_value) : m_name(name), m_enum(init_value) {}

      Enum_T value() {return m_enum;}

      void set(const Enum_T new_value)
      {
        if (m_enum == new_value)
          return;

        // ROS_INFO_STREAM("Changing " << m_name << ": \"" << to_string(m_enum) << "\" => \"" << to_string(new_value) << "\"");
        m_enum = new_value;
      }

      bool operator==(const Enum_T other)
      {
        return other == m_enum;
      }

      bool operator!=(const Enum_T other)
      {
        return other != m_enum;
      }

    private:
      std::string m_name;
      Enum_T m_enum;
  };

}
