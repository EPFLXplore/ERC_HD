// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/PanelObject.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__PANEL_OBJECT__STRUCT_H_
#define INTERFACES__MSG__DETAIL__PANEL_OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

// Struct defined in msg/PanelObject in the package interfaces.
typedef struct interfaces__msg__PanelObject
{
  int64_t id;
  geometry_msgs__msg__Pose pose;
} interfaces__msg__PanelObject;

// Struct for a sequence of interfaces__msg__PanelObject.
typedef struct interfaces__msg__PanelObject__Sequence
{
  interfaces__msg__PanelObject * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__PanelObject__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__PANEL_OBJECT__STRUCT_H_
