// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from eufs_msgs:msg/ConeWithColorProbabilityArray.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CONE_WITH_COLOR_PROBABILITY_ARRAY__STRUCT_H_
#define EUFS_MSGS__MSG__DETAIL__CONE_WITH_COLOR_PROBABILITY_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'cones'
#include "eufs_msgs/msg/detail/cone_with_color_probability__struct.h"

/// Struct defined in msg/ConeWithColorProbabilityArray in the package eufs_msgs.
/**
  * Array of ConeWithColourProbability.
 */
typedef struct eufs_msgs__msg__ConeWithColorProbabilityArray
{
  std_msgs__msg__Header header;
  eufs_msgs__msg__ConeWithColorProbability__Sequence cones;
} eufs_msgs__msg__ConeWithColorProbabilityArray;

// Struct for a sequence of eufs_msgs__msg__ConeWithColorProbabilityArray.
typedef struct eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence
{
  eufs_msgs__msg__ConeWithColorProbabilityArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EUFS_MSGS__MSG__DETAIL__CONE_WITH_COLOR_PROBABILITY_ARRAY__STRUCT_H_
