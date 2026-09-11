// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from eufs_msgs:msg/CarForces.idl
// generated code does not contain a copyright notice
#include "eufs_msgs/msg/detail/car_forces__rosidl_typesupport_fastrtps_cpp.hpp"
#include "eufs_msgs/msg/detail/car_forces__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace eufs_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_eufs_msgs
cdr_serialize(
  const eufs_msgs::msg::CarForces & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: front_left_lateral
  cdr << ros_message.front_left_lateral;
  // Member: front_left_longitudinal
  cdr << ros_message.front_left_longitudinal;
  // Member: front_left_vertical
  cdr << ros_message.front_left_vertical;
  // Member: front_right_lateral
  cdr << ros_message.front_right_lateral;
  // Member: front_right_longitudinal
  cdr << ros_message.front_right_longitudinal;
  // Member: front_right_vertical
  cdr << ros_message.front_right_vertical;
  // Member: rear_left_lateral
  cdr << ros_message.rear_left_lateral;
  // Member: rear_left_longitudinal
  cdr << ros_message.rear_left_longitudinal;
  // Member: rear_left_vertical
  cdr << ros_message.rear_left_vertical;
  // Member: rear_right_lateral
  cdr << ros_message.rear_right_lateral;
  // Member: rear_right_longitudinal
  cdr << ros_message.rear_right_longitudinal;
  // Member: rear_right_vertical
  cdr << ros_message.rear_right_vertical;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_eufs_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  eufs_msgs::msg::CarForces & ros_message)
{
  // Member: front_left_lateral
  cdr >> ros_message.front_left_lateral;

  // Member: front_left_longitudinal
  cdr >> ros_message.front_left_longitudinal;

  // Member: front_left_vertical
  cdr >> ros_message.front_left_vertical;

  // Member: front_right_lateral
  cdr >> ros_message.front_right_lateral;

  // Member: front_right_longitudinal
  cdr >> ros_message.front_right_longitudinal;

  // Member: front_right_vertical
  cdr >> ros_message.front_right_vertical;

  // Member: rear_left_lateral
  cdr >> ros_message.rear_left_lateral;

  // Member: rear_left_longitudinal
  cdr >> ros_message.rear_left_longitudinal;

  // Member: rear_left_vertical
  cdr >> ros_message.rear_left_vertical;

  // Member: rear_right_lateral
  cdr >> ros_message.rear_right_lateral;

  // Member: rear_right_longitudinal
  cdr >> ros_message.rear_right_longitudinal;

  // Member: rear_right_vertical
  cdr >> ros_message.rear_right_vertical;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_eufs_msgs
get_serialized_size(
  const eufs_msgs::msg::CarForces & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: front_left_lateral
  {
    size_t item_size = sizeof(ros_message.front_left_lateral);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: front_left_longitudinal
  {
    size_t item_size = sizeof(ros_message.front_left_longitudinal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: front_left_vertical
  {
    size_t item_size = sizeof(ros_message.front_left_vertical);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: front_right_lateral
  {
    size_t item_size = sizeof(ros_message.front_right_lateral);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: front_right_longitudinal
  {
    size_t item_size = sizeof(ros_message.front_right_longitudinal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: front_right_vertical
  {
    size_t item_size = sizeof(ros_message.front_right_vertical);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rear_left_lateral
  {
    size_t item_size = sizeof(ros_message.rear_left_lateral);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rear_left_longitudinal
  {
    size_t item_size = sizeof(ros_message.rear_left_longitudinal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rear_left_vertical
  {
    size_t item_size = sizeof(ros_message.rear_left_vertical);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rear_right_lateral
  {
    size_t item_size = sizeof(ros_message.rear_right_lateral);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rear_right_longitudinal
  {
    size_t item_size = sizeof(ros_message.rear_right_longitudinal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rear_right_vertical
  {
    size_t item_size = sizeof(ros_message.rear_right_vertical);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_eufs_msgs
max_serialized_size_CarForces(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: front_left_lateral
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: front_left_longitudinal
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: front_left_vertical
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: front_right_lateral
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: front_right_longitudinal
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: front_right_vertical
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: rear_left_lateral
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: rear_left_longitudinal
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: rear_left_vertical
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: rear_right_lateral
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: rear_right_longitudinal
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: rear_right_vertical
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = eufs_msgs::msg::CarForces;
    is_plain =
      (
      offsetof(DataType, rear_right_vertical) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _CarForces__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const eufs_msgs::msg::CarForces *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _CarForces__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<eufs_msgs::msg::CarForces *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _CarForces__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const eufs_msgs::msg::CarForces *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _CarForces__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_CarForces(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _CarForces__callbacks = {
  "eufs_msgs::msg",
  "CarForces",
  _CarForces__cdr_serialize,
  _CarForces__cdr_deserialize,
  _CarForces__get_serialized_size,
  _CarForces__max_serialized_size
};

static rosidl_message_type_support_t _CarForces__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_CarForces__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace eufs_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_eufs_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<eufs_msgs::msg::CarForces>()
{
  return &eufs_msgs::msg::typesupport_fastrtps_cpp::_CarForces__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, eufs_msgs, msg, CarForces)() {
  return &eufs_msgs::msg::typesupport_fastrtps_cpp::_CarForces__handle;
}

#ifdef __cplusplus
}
#endif
