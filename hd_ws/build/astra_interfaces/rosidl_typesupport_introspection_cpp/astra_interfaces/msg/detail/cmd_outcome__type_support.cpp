// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from astra_interfaces:msg/CmdOutcome.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "astra_interfaces/msg/detail/cmd_outcome__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace astra_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void CmdOutcome_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) astra_interfaces::msg::CmdOutcome(_init);
}

void CmdOutcome_fini_function(void * message_memory)
{
  auto typed_message = static_cast<astra_interfaces::msg::CmdOutcome *>(message_memory);
  typed_message->~CmdOutcome();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CmdOutcome_message_member_array[2] = {
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astra_interfaces::msg::CmdOutcome, id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "code",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astra_interfaces::msg::CmdOutcome, code),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CmdOutcome_message_members = {
  "astra_interfaces::msg",  // message namespace
  "CmdOutcome",  // message name
  2,  // number of fields
  sizeof(astra_interfaces::msg::CmdOutcome),
  CmdOutcome_message_member_array,  // message members
  CmdOutcome_init_function,  // function to initialize message memory (memory has to be allocated)
  CmdOutcome_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CmdOutcome_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CmdOutcome_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace astra_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<astra_interfaces::msg::CmdOutcome>()
{
  return &::astra_interfaces::msg::rosidl_typesupport_introspection_cpp::CmdOutcome_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, astra_interfaces, msg, CmdOutcome)() {
  return &::astra_interfaces::msg::rosidl_typesupport_introspection_cpp::CmdOutcome_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
