// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from astra_interfaces:msg/CmdOutcome.idl
// generated code does not contain a copyright notice
#include "astra_interfaces/msg/detail/cmd_outcome__rosidl_typesupport_fastrtps_cpp.hpp"
#include "astra_interfaces/msg/detail/cmd_outcome__struct.hpp"

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

namespace astra_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_astra_interfaces
cdr_serialize(
  const astra_interfaces::msg::CmdOutcome & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: id
  cdr << ros_message.id;
  // Member: code
  cdr << ros_message.code;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_astra_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  astra_interfaces::msg::CmdOutcome & ros_message)
{
  // Member: id
  cdr >> ros_message.id;

  // Member: code
  cdr >> ros_message.code;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_astra_interfaces
get_serialized_size(
  const astra_interfaces::msg::CmdOutcome & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: id
  {
    size_t item_size = sizeof(ros_message.id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: code
  {
    size_t item_size = sizeof(ros_message.code);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_astra_interfaces
max_serialized_size_CmdOutcome(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: code
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _CmdOutcome__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const astra_interfaces::msg::CmdOutcome *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _CmdOutcome__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<astra_interfaces::msg::CmdOutcome *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _CmdOutcome__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const astra_interfaces::msg::CmdOutcome *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _CmdOutcome__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_CmdOutcome(full_bounded, 0);
}

static message_type_support_callbacks_t _CmdOutcome__callbacks = {
  "astra_interfaces::msg",
  "CmdOutcome",
  _CmdOutcome__cdr_serialize,
  _CmdOutcome__cdr_deserialize,
  _CmdOutcome__get_serialized_size,
  _CmdOutcome__max_serialized_size
};

static rosidl_message_type_support_t _CmdOutcome__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_CmdOutcome__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace astra_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_astra_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<astra_interfaces::msg::CmdOutcome>()
{
  return &astra_interfaces::msg::typesupport_fastrtps_cpp::_CmdOutcome__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, astra_interfaces, msg, CmdOutcome)() {
  return &astra_interfaces::msg::typesupport_fastrtps_cpp::_CmdOutcome__handle;
}

#ifdef __cplusplus
}
#endif
