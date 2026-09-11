// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from eufs_msgs:msg/CarForces.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CAR_FORCES__STRUCT_H_
#define EUFS_MSGS__MSG__DETAIL__CAR_FORCES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CarForces in the package eufs_msgs.
/**
  * A message containing various forces on the car
 */
typedef struct eufs_msgs__msg__CarForces
{
  /// Car tyre forces
  double front_left_lateral;
  double front_left_longitudinal;
  double front_left_vertical;
  double front_right_lateral;
  double front_right_longitudinal;
  double front_right_vertical;
  double rear_left_lateral;
  double rear_left_longitudinal;
  double rear_left_vertical;
  double rear_right_lateral;
  double rear_right_longitudinal;
  double rear_right_vertical;
} eufs_msgs__msg__CarForces;

// Struct for a sequence of eufs_msgs__msg__CarForces.
typedef struct eufs_msgs__msg__CarForces__Sequence
{
  eufs_msgs__msg__CarForces * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eufs_msgs__msg__CarForces__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EUFS_MSGS__MSG__DETAIL__CAR_FORCES__STRUCT_H_
