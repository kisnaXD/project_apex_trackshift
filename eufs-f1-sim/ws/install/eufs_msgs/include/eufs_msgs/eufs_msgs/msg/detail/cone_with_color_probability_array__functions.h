// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from eufs_msgs:msg/ConeWithColorProbabilityArray.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CONE_WITH_COLOR_PROBABILITY_ARRAY__FUNCTIONS_H_
#define EUFS_MSGS__MSG__DETAIL__CONE_WITH_COLOR_PROBABILITY_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "eufs_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "eufs_msgs/msg/detail/cone_with_color_probability_array__struct.h"

/// Initialize msg/ConeWithColorProbabilityArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * eufs_msgs__msg__ConeWithColorProbabilityArray
 * )) before or use
 * eufs_msgs__msg__ConeWithColorProbabilityArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
bool
eufs_msgs__msg__ConeWithColorProbabilityArray__init(eufs_msgs__msg__ConeWithColorProbabilityArray * msg);

/// Finalize msg/ConeWithColorProbabilityArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
void
eufs_msgs__msg__ConeWithColorProbabilityArray__fini(eufs_msgs__msg__ConeWithColorProbabilityArray * msg);

/// Create msg/ConeWithColorProbabilityArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * eufs_msgs__msg__ConeWithColorProbabilityArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
eufs_msgs__msg__ConeWithColorProbabilityArray *
eufs_msgs__msg__ConeWithColorProbabilityArray__create();

/// Destroy msg/ConeWithColorProbabilityArray message.
/**
 * It calls
 * eufs_msgs__msg__ConeWithColorProbabilityArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
void
eufs_msgs__msg__ConeWithColorProbabilityArray__destroy(eufs_msgs__msg__ConeWithColorProbabilityArray * msg);

/// Check for msg/ConeWithColorProbabilityArray message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
bool
eufs_msgs__msg__ConeWithColorProbabilityArray__are_equal(const eufs_msgs__msg__ConeWithColorProbabilityArray * lhs, const eufs_msgs__msg__ConeWithColorProbabilityArray * rhs);

/// Copy a msg/ConeWithColorProbabilityArray message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
bool
eufs_msgs__msg__ConeWithColorProbabilityArray__copy(
  const eufs_msgs__msg__ConeWithColorProbabilityArray * input,
  eufs_msgs__msg__ConeWithColorProbabilityArray * output);

/// Initialize array of msg/ConeWithColorProbabilityArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * eufs_msgs__msg__ConeWithColorProbabilityArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
bool
eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence__init(eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence * array, size_t size);

/// Finalize array of msg/ConeWithColorProbabilityArray messages.
/**
 * It calls
 * eufs_msgs__msg__ConeWithColorProbabilityArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
void
eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence__fini(eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence * array);

/// Create array of msg/ConeWithColorProbabilityArray messages.
/**
 * It allocates the memory for the array and calls
 * eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence *
eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence__create(size_t size);

/// Destroy array of msg/ConeWithColorProbabilityArray messages.
/**
 * It calls
 * eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
void
eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence__destroy(eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence * array);

/// Check for msg/ConeWithColorProbabilityArray message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
bool
eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence__are_equal(const eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence * lhs, const eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence * rhs);

/// Copy an array of msg/ConeWithColorProbabilityArray messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_eufs_msgs
bool
eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence__copy(
  const eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence * input,
  eufs_msgs__msg__ConeWithColorProbabilityArray__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EUFS_MSGS__MSG__DETAIL__CONE_WITH_COLOR_PROBABILITY_ARRAY__FUNCTIONS_H_
