// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from astra_interfaces:msg/CmdOutcome.idl
// generated code does not contain a copyright notice
#include "astra_interfaces/msg/detail/cmd_outcome__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "astra_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "astra_interfaces/msg/detail/cmd_outcome__struct.h"
#include "astra_interfaces/msg/detail/cmd_outcome__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _CmdOutcome__ros_msg_type = astra_interfaces__msg__CmdOutcome;

static bool _CmdOutcome__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CmdOutcome__ros_msg_type * ros_message = static_cast<const _CmdOutcome__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: code
  {
    cdr << ros_message->code;
  }

  return true;
}

static bool _CmdOutcome__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CmdOutcome__ros_msg_type * ros_message = static_cast<_CmdOutcome__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: code
  {
    cdr >> ros_message->code;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_interfaces
size_t get_serialized_size_astra_interfaces__msg__CmdOutcome(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CmdOutcome__ros_msg_type * ros_message = static_cast<const _CmdOutcome__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name code
  {
    size_t item_size = sizeof(ros_message->code);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _CmdOutcome__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_astra_interfaces__msg__CmdOutcome(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_interfaces
size_t max_serialized_size_astra_interfaces__msg__CmdOutcome(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: code
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _CmdOutcome__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_astra_interfaces__msg__CmdOutcome(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_CmdOutcome = {
  "astra_interfaces::msg",
  "CmdOutcome",
  _CmdOutcome__cdr_serialize,
  _CmdOutcome__cdr_deserialize,
  _CmdOutcome__get_serialized_size,
  _CmdOutcome__max_serialized_size
};

static rosidl_message_type_support_t _CmdOutcome__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CmdOutcome,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, astra_interfaces, msg, CmdOutcome)() {
  return &_CmdOutcome__type_support;
}

#if defined(__cplusplus)
}
#endif
