// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astra_interfaces:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef ASTRA_INTERFACES__MSG__DETAIL__TASK__STRUCT_H_
#define ASTRA_INTERFACES__MSG__DETAIL__TASK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'description'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

// Struct defined in msg/Task in the package astra_interfaces.
typedef struct astra_interfaces__msg__Task
{
  rosidl_runtime_c__String description;
  int8_t id;
  geometry_msgs__msg__Pose pose;
} astra_interfaces__msg__Task;

// Struct for a sequence of astra_interfaces__msg__Task.
typedef struct astra_interfaces__msg__Task__Sequence
{
  astra_interfaces__msg__Task * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astra_interfaces__msg__Task__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRA_INTERFACES__MSG__DETAIL__TASK__STRUCT_H_
