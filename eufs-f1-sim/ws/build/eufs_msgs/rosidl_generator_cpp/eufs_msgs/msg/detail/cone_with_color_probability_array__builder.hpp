// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eufs_msgs:msg/ConeWithColorProbabilityArray.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CONE_WITH_COLOR_PROBABILITY_ARRAY__BUILDER_HPP_
#define EUFS_MSGS__MSG__DETAIL__CONE_WITH_COLOR_PROBABILITY_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "eufs_msgs/msg/detail/cone_with_color_probability_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace eufs_msgs
{

namespace msg
{

namespace builder
{

class Init_ConeWithColorProbabilityArray_cones
{
public:
  explicit Init_ConeWithColorProbabilityArray_cones(::eufs_msgs::msg::ConeWithColorProbabilityArray & msg)
  : msg_(msg)
  {}
  ::eufs_msgs::msg::ConeWithColorProbabilityArray cones(::eufs_msgs::msg::ConeWithColorProbabilityArray::_cones_type arg)
  {
    msg_.cones = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithColorProbabilityArray msg_;
};

class Init_ConeWithColorProbabilityArray_header
{
public:
  Init_ConeWithColorProbabilityArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConeWithColorProbabilityArray_cones header(::eufs_msgs::msg::ConeWithColorProbabilityArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ConeWithColorProbabilityArray_cones(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithColorProbabilityArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::eufs_msgs::msg::ConeWithColorProbabilityArray>()
{
  return eufs_msgs::msg::builder::Init_ConeWithColorProbabilityArray_header();
}

}  // namespace eufs_msgs

#endif  // EUFS_MSGS__MSG__DETAIL__CONE_WITH_COLOR_PROBABILITY_ARRAY__BUILDER_HPP_
