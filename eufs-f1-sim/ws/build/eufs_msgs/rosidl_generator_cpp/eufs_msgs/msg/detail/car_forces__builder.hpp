// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eufs_msgs:msg/CarForces.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CAR_FORCES__BUILDER_HPP_
#define EUFS_MSGS__MSG__DETAIL__CAR_FORCES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "eufs_msgs/msg/detail/car_forces__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace eufs_msgs
{

namespace msg
{

namespace builder
{

class Init_CarForces_rear_right_vertical
{
public:
  explicit Init_CarForces_rear_right_vertical(::eufs_msgs::msg::CarForces & msg)
  : msg_(msg)
  {}
  ::eufs_msgs::msg::CarForces rear_right_vertical(::eufs_msgs::msg::CarForces::_rear_right_vertical_type arg)
  {
    msg_.rear_right_vertical = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eufs_msgs::msg::CarForces msg_;
};

class Init_CarForces_rear_right_longitudinal
{
public:
  explicit Init_CarForces_rear_right_longitudinal(::eufs_msgs::msg::CarForces & msg)
  : msg_(msg)
  {}
  Init_CarForces_rear_right_vertical rear_right_longitudinal(::eufs_msgs::msg::CarForces::_rear_right_longitudinal_type arg)
  {
    msg_.rear_right_longitudinal = std::move(arg);
    return Init_CarForces_rear_right_vertical(msg_);
  }

private:
  ::eufs_msgs::msg::CarForces msg_;
};

class Init_CarForces_rear_right_lateral
{
public:
  explicit Init_CarForces_rear_right_lateral(::eufs_msgs::msg::CarForces & msg)
  : msg_(msg)
  {}
  Init_CarForces_rear_right_longitudinal rear_right_lateral(::eufs_msgs::msg::CarForces::_rear_right_lateral_type arg)
  {
    msg_.rear_right_lateral = std::move(arg);
    return Init_CarForces_rear_right_longitudinal(msg_);
  }

private:
  ::eufs_msgs::msg::CarForces msg_;
};

class Init_CarForces_rear_left_vertical
{
public:
  explicit Init_CarForces_rear_left_vertical(::eufs_msgs::msg::CarForces & msg)
  : msg_(msg)
  {}
  Init_CarForces_rear_right_lateral rear_left_vertical(::eufs_msgs::msg::CarForces::_rear_left_vertical_type arg)
  {
    msg_.rear_left_vertical = std::move(arg);
    return Init_CarForces_rear_right_lateral(msg_);
  }

private:
  ::eufs_msgs::msg::CarForces msg_;
};

class Init_CarForces_rear_left_longitudinal
{
public:
  explicit Init_CarForces_rear_left_longitudinal(::eufs_msgs::msg::CarForces & msg)
  : msg_(msg)
  {}
  Init_CarForces_rear_left_vertical rear_left_longitudinal(::eufs_msgs::msg::CarForces::_rear_left_longitudinal_type arg)
  {
    msg_.rear_left_longitudinal = std::move(arg);
    return Init_CarForces_rear_left_vertical(msg_);
  }

private:
  ::eufs_msgs::msg::CarForces msg_;
};

class Init_CarForces_rear_left_lateral
{
public:
  explicit Init_CarForces_rear_left_lateral(::eufs_msgs::msg::CarForces & msg)
  : msg_(msg)
  {}
  Init_CarForces_rear_left_longitudinal rear_left_lateral(::eufs_msgs::msg::CarForces::_rear_left_lateral_type arg)
  {
    msg_.rear_left_lateral = std::move(arg);
    return Init_CarForces_rear_left_longitudinal(msg_);
  }

private:
  ::eufs_msgs::msg::CarForces msg_;
};

class Init_CarForces_front_right_vertical
{
public:
  explicit Init_CarForces_front_right_vertical(::eufs_msgs::msg::CarForces & msg)
  : msg_(msg)
  {}
  Init_CarForces_rear_left_lateral front_right_vertical(::eufs_msgs::msg::CarForces::_front_right_vertical_type arg)
  {
    msg_.front_right_vertical = std::move(arg);
    return Init_CarForces_rear_left_lateral(msg_);
  }

private:
  ::eufs_msgs::msg::CarForces msg_;
};

class Init_CarForces_front_right_longitudinal
{
public:
  explicit Init_CarForces_front_right_longitudinal(::eufs_msgs::msg::CarForces & msg)
  : msg_(msg)
  {}
  Init_CarForces_front_right_vertical front_right_longitudinal(::eufs_msgs::msg::CarForces::_front_right_longitudinal_type arg)
  {
    msg_.front_right_longitudinal = std::move(arg);
    return Init_CarForces_front_right_vertical(msg_);
  }

private:
  ::eufs_msgs::msg::CarForces msg_;
};

class Init_CarForces_front_right_lateral
{
public:
  explicit Init_CarForces_front_right_lateral(::eufs_msgs::msg::CarForces & msg)
  : msg_(msg)
  {}
  Init_CarForces_front_right_longitudinal front_right_lateral(::eufs_msgs::msg::CarForces::_front_right_lateral_type arg)
  {
    msg_.front_right_lateral = std::move(arg);
    return Init_CarForces_front_right_longitudinal(msg_);
  }

private:
  ::eufs_msgs::msg::CarForces msg_;
};

class Init_CarForces_front_left_vertical
{
public:
  explicit Init_CarForces_front_left_vertical(::eufs_msgs::msg::CarForces & msg)
  : msg_(msg)
  {}
  Init_CarForces_front_right_lateral front_left_vertical(::eufs_msgs::msg::CarForces::_front_left_vertical_type arg)
  {
    msg_.front_left_vertical = std::move(arg);
    return Init_CarForces_front_right_lateral(msg_);
  }

private:
  ::eufs_msgs::msg::CarForces msg_;
};

class Init_CarForces_front_left_longitudinal
{
public:
  explicit Init_CarForces_front_left_longitudinal(::eufs_msgs::msg::CarForces & msg)
  : msg_(msg)
  {}
  Init_CarForces_front_left_vertical front_left_longitudinal(::eufs_msgs::msg::CarForces::_front_left_longitudinal_type arg)
  {
    msg_.front_left_longitudinal = std::move(arg);
    return Init_CarForces_front_left_vertical(msg_);
  }

private:
  ::eufs_msgs::msg::CarForces msg_;
};

class Init_CarForces_front_left_lateral
{
public:
  Init_CarForces_front_left_lateral()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CarForces_front_left_longitudinal front_left_lateral(::eufs_msgs::msg::CarForces::_front_left_lateral_type arg)
  {
    msg_.front_left_lateral = std::move(arg);
    return Init_CarForces_front_left_longitudinal(msg_);
  }

private:
  ::eufs_msgs::msg::CarForces msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::eufs_msgs::msg::CarForces>()
{
  return eufs_msgs::msg::builder::Init_CarForces_front_left_lateral();
}

}  // namespace eufs_msgs

#endif  // EUFS_MSGS__MSG__DETAIL__CAR_FORCES__BUILDER_HPP_
