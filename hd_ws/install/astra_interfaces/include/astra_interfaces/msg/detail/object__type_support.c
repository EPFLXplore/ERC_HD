// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from astra_interfaces:msg/Object.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "astra_interfaces/msg/detail/object__rosidl_typesupport_introspection_c.h"
#include "astra_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "astra_interfaces/msg/detail/object__functions.h"
#include "astra_interfaces/msg/detail/object__struct.h"


// Include directives for member types
// Member `type`
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `dims`
#include "std_msgs/msg/float32_multi_array.h"
// Member `dims`
#include "std_msgs/msg/detail/float32_multi_array__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Object__rosidl_typesupport_introspection_c__Object_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astra_interfaces__msg__Object__init(message_memory);
}

void Object__rosidl_typesupport_introspection_c__Object_fini_function(void * message_memory)
{
  astra_interfaces__msg__Object__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Object__rosidl_typesupport_introspection_c__Object_message_member_array[4] = {
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astra_interfaces__msg__Object, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astra_interfaces__msg__Object, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astra_interfaces__msg__Object, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dims",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astra_interfaces__msg__Object, dims),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Object__rosidl_typesupport_introspection_c__Object_message_members = {
  "astra_interfaces__msg",  // message namespace
  "Object",  // message name
  4,  // number of fields
  sizeof(astra_interfaces__msg__Object),
  Object__rosidl_typesupport_introspection_c__Object_message_member_array,  // message members
  Object__rosidl_typesupport_introspection_c__Object_init_function,  // function to initialize message memory (memory has to be allocated)
  Object__rosidl_typesupport_introspection_c__Object_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Object__rosidl_typesupport_introspection_c__Object_message_type_support_handle = {
  0,
  &Object__rosidl_typesupport_introspection_c__Object_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astra_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astra_interfaces, msg, Object)() {
  Object__rosidl_typesupport_introspection_c__Object_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  Object__rosidl_typesupport_introspection_c__Object_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Float32MultiArray)();
  if (!Object__rosidl_typesupport_introspection_c__Object_message_type_support_handle.typesupport_identifier) {
    Object__rosidl_typesupport_introspection_c__Object_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Object__rosidl_typesupport_introspection_c__Object_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
