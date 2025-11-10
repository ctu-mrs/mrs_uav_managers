#pragma once
#include <boost/preprocessor.hpp>

template <typename Enum_T, typename Ros_T>
inline constexpr Enum_T from_ros(Ros_T ros)
{
  return Enum_T::UNKNOWN;
}

// modified from https://stackoverflow.com/questions/5093460/how-to-convert-an-enum-type-variable-to-a-string
// a helper macro to generate a switch case in the format
// case enum_t::element_name: return "ELEMENT_NAME";
#define X_DEFINE_ENUM_STRING_CONVERSIONS_TOSTRING_CASE(r, enum_t, elem)       \
          case enum_t::elem: return BOOST_PP_STRINGIZE(elem);

// this macro defines an enumerator using the given name, base type and enumeration values
// together with a to_string() conversion function
#define DEFINE_ENUM_WITH_CONVERSIONS(enum_t, base_t, enumerators)             \
    enum class enum_t : base_t                                                \
    {                                                                         \
        BOOST_PP_SEQ_ENUM(enumerators),                                       \
        UNKNOWN                                                               \
    };                                                                        \
                                                                              \
    inline constexpr const char* to_string(enum_t v)                          \
    {                                                                         \
        switch (v)                                                            \
        {                                                                     \
            BOOST_PP_SEQ_FOR_EACH(                                            \
                X_DEFINE_ENUM_STRING_CONVERSIONS_TOSTRING_CASE,               \
                enum_t,                                                       \
                enumerators                                                   \
            )                                                                 \
            default: return "UNKNOWN";                                        \
        }                                                                     \
    }                                                                         \

// some helper macros for expanding and concatenating macro variables using black magic
#define X_PPCAT_NX(A, B) A ## B
#define X_PPCAT(A, B) X_PPCAT_NX(A, B)
#define X_TO_ROS_ENUM(element_prefix, value) X_PPCAT_NX(element_prefix, value)
// a helper macro to generate a switch case in the format
// case enum_t::element_name: return msg_t::PREFIX_ELEMENT_NAME;
#define X_DEFINE_ENUM_MSG_CONVERSIONS_TOMSG_CASE(r, data, elem)               \
          case BOOST_PP_TUPLE_ELEM(3, 0, data)::elem: return BOOST_PP_TUPLE_ELEM(3, 1, data)::X_TO_ROS_ENUM(BOOST_PP_TUPLE_ELEM(3, 2, data), elem);
#define X_DEFINE_ENUM_MSG_CONVERSIONS_FROMMSG_CASE(r, data, elem)             \
          if (ros_value == BOOST_PP_TUPLE_ELEM(3, 1, data)::X_TO_ROS_ENUM(BOOST_PP_TUPLE_ELEM(3, 2, data), elem)) return BOOST_PP_TUPLE_ELEM(3, 0, data)::elem;

// this macro defines a to_ros() enum to ROS message conversion function given the enumeration type,
// ROS message type, name of the message element, prefix of the message elements corresponding to
// the enumeration values, and a list of the enumeration values
#define DEFINE_ENUM_MSG_CONVERSIONS(enum_t, msg_t, elem_name, elem_pre, enumerators)    \
                                                                              \
    inline constexpr decltype(msg_t::elem_name) to_ros(enum_t v)              \
    {                                                                         \
        switch (v)                                                            \
        {                                                                     \
            BOOST_PP_SEQ_FOR_EACH(                                            \
                X_DEFINE_ENUM_MSG_CONVERSIONS_TOMSG_CASE,                     \
                (                                                             \
                 enum_t,                                                      \
                 msg_t,                                                       \
                 elem_pre                                                     \
                 ),                                                           \
                enumerators                                                   \
            )                                                                 \
          default: return msg_t::X_PPCAT(elem_pre, UNKNOWN);                  \
        }                                                                     \
    }                                                                         \
                                                                              \
    template <>                                                               \
    inline constexpr enum_t from_ros<enum_t, decltype(msg_t::elem_name)>(decltype(msg_t::elem_name) ros_value)     \
    {                                                                         \
      BOOST_PP_SEQ_FOR_EACH(                                                  \
          X_DEFINE_ENUM_MSG_CONVERSIONS_FROMMSG_CASE,                         \
          (                                                                   \
           enum_t,                                                            \
           msg_t,                                                             \
           elem_pre,                                                          \
          ),                                                                  \
          enumerators                                                         \
      )                                                                       \
      return enum_t::UNKNOWN;                                                 \
    }                                                                         
