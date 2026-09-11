// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from eufs_msgs:msg/CarForces.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CAR_FORCES__TRAITS_HPP_
#define EUFS_MSGS__MSG__DETAIL__CAR_FORCES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "eufs_msgs/msg/detail/car_forces__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace eufs_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CarForces & msg,
  std::ostream & out)
{
  out << "{";
  // member: front_left_lateral
  {
    out << "front_left_lateral: ";
    rosidl_generator_traits::value_to_yaml(msg.front_left_lateral, out);
    out << ", ";
  }

  // member: front_left_longitudinal
  {
    out << "front_left_longitudinal: ";
    rosidl_generator_traits::value_to_yaml(msg.front_left_longitudinal, out);
    out << ", ";
  }

  // member: front_left_vertical
  {
    out << "front_left_vertical: ";
    rosidl_generator_traits::value_to_yaml(msg.front_left_vertical, out);
    out << ", ";
  }

  // member: front_right_lateral
  {
    out << "front_right_lateral: ";
    rosidl_generator_traits::value_to_yaml(msg.front_right_lateral, out);
    out << ", ";
  }

  // member: front_right_longitudinal
  {
    out << "front_right_longitudinal: ";
    rosidl_generator_traits::value_to_yaml(msg.front_right_longitudinal, out);
    out << ", ";
  }

  // member: front_right_vertical
  {
    out << "front_right_vertical: ";
    rosidl_generator_traits::value_to_yaml(msg.front_right_vertical, out);
    out << ", ";
  }

  // member: rear_left_lateral
  {
    out << "rear_left_lateral: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_left_lateral, out);
    out << ", ";
  }

  // member: rear_left_longitudinal
  {
    out << "rear_left_longitudinal: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_left_longitudinal, out);
    out << ", ";
  }

  // member: rear_left_vertical
  {
    out << "rear_left_vertical: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_left_vertical, out);
    out << ", ";
  }

  // member: rear_right_lateral
  {
    out << "rear_right_lateral: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_right_lateral, out);
    out << ", ";
  }

  // member: rear_right_longitudinal
  {
    out << "rear_right_longitudinal: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_right_longitudinal, out);
    out << ", ";
  }

  // member: rear_right_vertical
  {
    out << "rear_right_vertical: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_right_vertical, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarForces & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: front_left_lateral
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front_left_lateral: ";
    rosidl_generator_traits::value_to_yaml(msg.front_left_lateral, out);
    out << "\n";
  }

  // member: front_left_longitudinal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front_left_longitudinal: ";
    rosidl_generator_traits::value_to_yaml(msg.front_left_longitudinal, out);
    out << "\n";
  }

  // member: front_left_vertical
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front_left_vertical: ";
    rosidl_generator_traits::value_to_yaml(msg.front_left_vertical, out);
    out << "\n";
  }

  // member: front_right_lateral
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front_right_lateral: ";
    rosidl_generator_traits::value_to_yaml(msg.front_right_lateral, out);
    out << "\n";
  }

  // member: front_right_longitudinal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front_right_longitudinal: ";
    rosidl_generator_traits::value_to_yaml(msg.front_right_longitudinal, out);
    out << "\n";
  }

  // member: front_right_vertical
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front_right_vertical: ";
    rosidl_generator_traits::value_to_yaml(msg.front_right_vertical, out);
    out << "\n";
  }

  // member: rear_left_lateral
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rear_left_lateral: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_left_lateral, out);
    out << "\n";
  }

  // member: rear_left_longitudinal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rear_left_longitudinal: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_left_longitudinal, out);
    out << "\n";
  }

  // member: rear_left_vertical
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rear_left_vertical: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_left_vertical, out);
    out << "\n";
  }

  // member: rear_right_lateral
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rear_right_lateral: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_right_lateral, out);
    out << "\n";
  }

  // member: rear_right_longitudinal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rear_right_longitudinal: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_right_longitudinal, out);
    out << "\n";
  }

  // member: rear_right_vertical
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rear_right_vertical: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_right_vertical, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarForces & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace eufs_msgs

namespace rosidl_generator_traits
{

[[deprecated("use eufs_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const eufs_msgs::msg::CarForces & msg,
  std::ostream & out, size_t indentation = 0)
{
  eufs_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use eufs_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const eufs_msgs::msg::CarForces & msg)
{
  return eufs_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<eufs_msgs::msg::CarForces>()
{
  return "eufs_msgs::msg::CarForces";
}

template<>
inline const char * name<eufs_msgs::msg::CarForces>()
{
  return "eufs_msgs/msg/CarForces";
}

template<>
struct has_fixed_size<eufs_msgs::msg::CarForces>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<eufs_msgs::msg::CarForces>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<eufs_msgs::msg::CarForces>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EUFS_MSGS__MSG__DETAIL__CAR_FORCES__TRAITS_HPP_
