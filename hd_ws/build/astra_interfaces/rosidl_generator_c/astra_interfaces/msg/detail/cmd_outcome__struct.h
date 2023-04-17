// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astra_interfaces:msg/CmdOutcome.idl
// generated code does not contain a copyright notice

#ifndef ASTRA_INTERFACES__MSG__DETAIL__CMD_OUTCOME__STRUCT_H_
#define ASTRA_INTERFACES__MSG__DETAIL__CMD_OUTCOME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/CmdOutcome in the package astra_interfaces.
typedef struct astra_interfaces__msg__CmdOutcome
{
  uint16_t id;
  uint8_t code;
} astra_interfaces__msg__CmdOutcome;

// Struct for a sequence of astra_interfaces__msg__CmdOutcome.
typedef struct astra_interfaces__msg__CmdOutcome__Sequence
{
  astra_interfaces__msg__CmdOutcome * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astra_interfaces__msg__CmdOutcome__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRA_INTERFACES__MSG__DETAIL__CMD_OUTCOME__STRUCT_H_
