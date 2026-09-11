// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from eufs_msgs:msg/CarForces.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CAR_FORCES__STRUCT_HPP_
#define EUFS_MSGS__MSG__DETAIL__CAR_FORCES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__eufs_msgs__msg__CarForces __attribute__((deprecated))
#else
# define DEPRECATED__eufs_msgs__msg__CarForces __declspec(deprecated)
#endif

namespace eufs_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CarForces_
{
  using Type = CarForces_<ContainerAllocator>;

  explicit CarForces_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->front_left_lateral = 0.0;
      this->front_left_longitudinal = 0.0;
      this->front_left_vertical = 0.0;
      this->front_right_lateral = 0.0;
      this->front_right_longitudinal = 0.0;
      this->front_right_vertical = 0.0;
      this->rear_left_lateral = 0.0;
      this->rear_left_longitudinal = 0.0;
      this->rear_left_vertical = 0.0;
      this->rear_right_lateral = 0.0;
      this->rear_right_longitudinal = 0.0;
      this->rear_right_vertical = 0.0;
    }
  }

  explicit CarForces_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->front_left_lateral = 0.0;
      this->front_left_longitudinal = 0.0;
      this->front_left_vertical = 0.0;
      this->front_right_lateral = 0.0;
      this->front_right_longitudinal = 0.0;
      this->front_right_vertical = 0.0;
      this->rear_left_lateral = 0.0;
      this->rear_left_longitudinal = 0.0;
      this->rear_left_vertical = 0.0;
      this->rear_right_lateral = 0.0;
      this->rear_right_longitudinal = 0.0;
      this->rear_right_vertical = 0.0;
    }
  }

  // field types and members
  using _front_left_lateral_type =
    double;
  _front_left_lateral_type front_left_lateral;
  using _front_left_longitudinal_type =
    double;
  _front_left_longitudinal_type front_left_longitudinal;
  using _front_left_vertical_type =
    double;
  _front_left_vertical_type front_left_vertical;
  using _front_right_lateral_type =
    double;
  _front_right_lateral_type front_right_lateral;
  using _front_right_longitudinal_type =
    double;
  _front_right_longitudinal_type front_right_longitudinal;
  using _front_right_vertical_type =
    double;
  _front_right_vertical_type front_right_vertical;
  using _rear_left_lateral_type =
    double;
  _rear_left_lateral_type rear_left_lateral;
  using _rear_left_longitudinal_type =
    double;
  _rear_left_longitudinal_type rear_left_longitudinal;
  using _rear_left_vertical_type =
    double;
  _rear_left_vertical_type rear_left_vertical;
  using _rear_right_lateral_type =
    double;
  _rear_right_lateral_type rear_right_lateral;
  using _rear_right_longitudinal_type =
    double;
  _rear_right_longitudinal_type rear_right_longitudinal;
  using _rear_right_vertical_type =
    double;
  _rear_right_vertical_type rear_right_vertical;

  // setters for named parameter idiom
  Type & set__front_left_lateral(
    const double & _arg)
  {
    this->front_left_lateral = _arg;
    return *this;
  }
  Type & set__front_left_longitudinal(
    const double & _arg)
  {
    this->front_left_longitudinal = _arg;
    return *this;
  }
  Type & set__front_left_vertical(
    const double & _arg)
  {
    this->front_left_vertical = _arg;
    return *this;
  }
  Type & set__front_right_lateral(
    const double & _arg)
  {
    this->front_right_lateral = _arg;
    return *this;
  }
  Type & set__front_right_longitudinal(
    const double & _arg)
  {
    this->front_right_longitudinal = _arg;
    return *this;
  }
  Type & set__front_right_vertical(
    const double & _arg)
  {
    this->front_right_vertical = _arg;
    return *this;
  }
  Type & set__rear_left_lateral(
    const double & _arg)
  {
    this->rear_left_lateral = _arg;
    return *this;
  }
  Type & set__rear_left_longitudinal(
    const double & _arg)
  {
    this->rear_left_longitudinal = _arg;
    return *this;
  }
  Type & set__rear_left_vertical(
    const double & _arg)
  {
    this->rear_left_vertical = _arg;
    return *this;
  }
  Type & set__rear_right_lateral(
    const double & _arg)
  {
    this->rear_right_lateral = _arg;
    return *this;
  }
  Type & set__rear_right_longitudinal(
    const double & _arg)
  {
    this->rear_right_longitudinal = _arg;
    return *this;
  }
  Type & set__rear_right_vertical(
    const double & _arg)
  {
    this->rear_right_vertical = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    eufs_msgs::msg::CarForces_<ContainerAllocator> *;
  using ConstRawPtr =
    const eufs_msgs::msg::CarForces_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<eufs_msgs::msg::CarForces_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<eufs_msgs::msg::CarForces_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      eufs_msgs::msg::CarForces_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<eufs_msgs::msg::CarForces_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      eufs_msgs::msg::CarForces_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<eufs_msgs::msg::CarForces_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<eufs_msgs::msg::CarForces_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<eufs_msgs::msg::CarForces_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__eufs_msgs__msg__CarForces
    std::shared_ptr<eufs_msgs::msg::CarForces_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__eufs_msgs__msg__CarForces
    std::shared_ptr<eufs_msgs::msg::CarForces_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CarForces_ & other) const
  {
    if (this->front_left_lateral != other.front_left_lateral) {
      return false;
    }
    if (this->front_left_longitudinal != other.front_left_longitudinal) {
      return false;
    }
    if (this->front_left_vertical != other.front_left_vertical) {
      return false;
    }
    if (this->front_right_lateral != other.front_right_lateral) {
      return false;
    }
    if (this->front_right_longitudinal != other.front_right_longitudinal) {
      return false;
    }
    if (this->front_right_vertical != other.front_right_vertical) {
      return false;
    }
    if (this->rear_left_lateral != other.rear_left_lateral) {
      return false;
    }
    if (this->rear_left_longitudinal != other.rear_left_longitudinal) {
      return false;
    }
    if (this->rear_left_vertical != other.rear_left_vertical) {
      return false;
    }
    if (this->rear_right_lateral != other.rear_right_lateral) {
      return false;
    }
    if (this->rear_right_longitudinal != other.rear_right_longitudinal) {
      return false;
    }
    if (this->rear_right_vertical != other.rear_right_vertical) {
      return false;
    }
    return true;
  }
  bool operator!=(const CarForces_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CarForces_

// alias to use template instance with default allocator
using CarForces =
  eufs_msgs::msg::CarForces_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace eufs_msgs

#endif  // EUFS_MSGS__MSG__DETAIL__CAR_FORCES__STRUCT_HPP_
