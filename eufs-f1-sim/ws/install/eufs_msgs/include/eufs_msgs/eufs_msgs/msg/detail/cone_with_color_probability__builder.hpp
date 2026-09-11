// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eufs_msgs:msg/ConeWithColorProbability.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CONE_WITH_COLOR_PROBABILITY__BUILDER_HPP_
#define EUFS_MSGS__MSG__DETAIL__CONE_WITH_COLOR_PROBABILITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "eufs_msgs/msg/detail/cone_with_color_probability__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace eufs_msgs
{

namespace msg
{

namespace builder
{

class Init_ConeWithColorProbability_covariance
{
public:
  explicit Init_ConeWithColorProbability_covariance(::eufs_msgs::msg::ConeWithColorProbability & msg)
  : msg_(msg)
  {}
  ::eufs_msgs::msg::ConeWithColorProbability covariance(::eufs_msgs::msg::ConeWithColorProbability::_covariance_type arg)
  {
    msg_.covariance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithColorProbability msg_;
};

class Init_ConeWithColorProbability_point
{
public:
  explicit Init_ConeWithColorProbability_point(::eufs_msgs::msg::ConeWithColorProbability & msg)
  : msg_(msg)
  {}
  Init_ConeWithColorProbability_covariance point(::eufs_msgs::msg::ConeWithColorProbability::_point_type arg)
  {
    msg_.point = std::move(arg);
    return Init_ConeWithColorProbability_covariance(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithColorProbability msg_;
};

class Init_ConeWithColorProbability_unknown_prob
{
public:
  explicit Init_ConeWithColorProbability_unknown_prob(::eufs_msgs::msg::ConeWithColorProbability & msg)
  : msg_(msg)
  {}
  Init_ConeWithColorProbability_point unknown_prob(::eufs_msgs::msg::ConeWithColorProbability::_unknown_prob_type arg)
  {
    msg_.unknown_prob = std::move(arg);
    return Init_ConeWithColorProbability_point(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithColorProbability msg_;
};

class Init_ConeWithColorProbability_big_orange_prob
{
public:
  explicit Init_ConeWithColorProbability_big_orange_prob(::eufs_msgs::msg::ConeWithColorProbability & msg)
  : msg_(msg)
  {}
  Init_ConeWithColorProbability_unknown_prob big_orange_prob(::eufs_msgs::msg::ConeWithColorProbability::_big_orange_prob_type arg)
  {
    msg_.big_orange_prob = std::move(arg);
    return Init_ConeWithColorProbability_unknown_prob(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithColorProbability msg_;
};

class Init_ConeWithColorProbability_orange_prob
{
public:
  explicit Init_ConeWithColorProbability_orange_prob(::eufs_msgs::msg::ConeWithColorProbability & msg)
  : msg_(msg)
  {}
  Init_ConeWithColorProbability_big_orange_prob orange_prob(::eufs_msgs::msg::ConeWithColorProbability::_orange_prob_type arg)
  {
    msg_.orange_prob = std::move(arg);
    return Init_ConeWithColorProbability_big_orange_prob(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithColorProbability msg_;
};

class Init_ConeWithColorProbability_yellow_prob
{
public:
  explicit Init_ConeWithColorProbability_yellow_prob(::eufs_msgs::msg::ConeWithColorProbability & msg)
  : msg_(msg)
  {}
  Init_ConeWithColorProbability_orange_prob yellow_prob(::eufs_msgs::msg::ConeWithColorProbability::_yellow_prob_type arg)
  {
    msg_.yellow_prob = std::move(arg);
    return Init_ConeWithColorProbability_orange_prob(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithColorProbability msg_;
};

class Init_ConeWithColorProbability_blue_prob
{
public:
  explicit Init_ConeWithColorProbability_blue_prob(::eufs_msgs::msg::ConeWithColorProbability & msg)
  : msg_(msg)
  {}
  Init_ConeWithColorProbability_yellow_prob blue_prob(::eufs_msgs::msg::ConeWithColorProbability::_blue_prob_type arg)
  {
    msg_.blue_prob = std::move(arg);
    return Init_ConeWithColorProbability_yellow_prob(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithColorProbability msg_;
};

class Init_ConeWithColorProbability_id
{
public:
  Init_ConeWithColorProbability_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConeWithColorProbability_blue_prob id(::eufs_msgs::msg::ConeWithColorProbability::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_ConeWithColorProbability_blue_prob(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithColorProbability msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::eufs_msgs::msg::ConeWithColorProbability>()
{
  return eufs_msgs::msg::builder::Init_ConeWithColorProbability_id();
}

}  // namespace eufs_msgs

#endif  // EUFS_MSGS__MSG__DETAIL__CONE_WITH_COLOR_PROBABILITY__BUILDER_HPP_
