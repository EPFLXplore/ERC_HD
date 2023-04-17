// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astra_interfaces:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef ASTRA_INTERFACES__MSG__DETAIL__OBJECT__STRUCT_H_
#define ASTRA_INTERFACES__MSG__DETAIL__OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'type'
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'dims'
#include "std_msgs/msg/detail/float32_multi_array__struct.h"

// Struct defined in msg/Object in the package astra_interfaces.
typedef struct astra_interfaces__msg__Object
{
  rosidl_runtime_c__String type;
  rosidl_runtime_c__String name;
  geometry_msgs__msg__Pose pose;
  std_msgs__msg__Float32MultiArray dims;
} astra_interfaces__msg__Object;

// Struct for a sequence of astra_interfaces__msg__Object.
typedef struct astra_interfaces__msg__Object__Sequence
{
  astra_interfaces__msg__Object * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astra_interfaces__msg__Object__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRA_INTERFACES__MSG__DETAIL__OBJECT__STRUCT_H_
