// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from interfaces:msg/PanelObject.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__PANEL_OBJECT__FUNCTIONS_H_
#define INTERFACES__MSG__DETAIL__PANEL_OBJECT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "interfaces/msg/detail/panel_object__struct.h"

/// Initialize msg/PanelObject message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * interfaces__msg__PanelObject
 * )) before or use
 * interfaces__msg__PanelObject__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_interfaces
bool
interfaces__msg__PanelObject__init(interfaces__msg__PanelObject * msg);

/// Finalize msg/PanelObject message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interfaces
void
interfaces__msg__PanelObject__fini(interfaces__msg__PanelObject * msg);

/// Create msg/PanelObject message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * interfaces__msg__PanelObject__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interfaces
interfaces__msg__PanelObject *
interfaces__msg__PanelObject__create();

/// Destroy msg/PanelObject message.
/**
 * It calls
 * interfaces__msg__PanelObject__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interfaces
void
interfaces__msg__PanelObject__destroy(interfaces__msg__PanelObject * msg);

/// Check for msg/PanelObject message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interfaces
bool
interfaces__msg__PanelObject__are_equal(const interfaces__msg__PanelObject * lhs, const interfaces__msg__PanelObject * rhs);

/// Copy a msg/PanelObject message.
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
ROSIDL_GENERATOR_C_PUBLIC_interfaces
bool
interfaces__msg__PanelObject__copy(
  const interfaces__msg__PanelObject * input,
  interfaces__msg__PanelObject * output);

/// Initialize array of msg/PanelObject messages.
/**
 * It allocates the memory for the number of elements and calls
 * interfaces__msg__PanelObject__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_interfaces
bool
interfaces__msg__PanelObject__Sequence__init(interfaces__msg__PanelObject__Sequence * array, size_t size);

/// Finalize array of msg/PanelObject messages.
/**
 * It calls
 * interfaces__msg__PanelObject__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interfaces
void
interfaces__msg__PanelObject__Sequence__fini(interfaces__msg__PanelObject__Sequence * array);

/// Create array of msg/PanelObject messages.
/**
 * It allocates the memory for the array and calls
 * interfaces__msg__PanelObject__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interfaces
interfaces__msg__PanelObject__Sequence *
interfaces__msg__PanelObject__Sequence__create(size_t size);

/// Destroy array of msg/PanelObject messages.
/**
 * It calls
 * interfaces__msg__PanelObject__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interfaces
void
interfaces__msg__PanelObject__Sequence__destroy(interfaces__msg__PanelObject__Sequence * array);

/// Check for msg/PanelObject message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interfaces
bool
interfaces__msg__PanelObject__Sequence__are_equal(const interfaces__msg__PanelObject__Sequence * lhs, const interfaces__msg__PanelObject__Sequence * rhs);

/// Copy an array of msg/PanelObject messages.
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
ROSIDL_GENERATOR_C_PUBLIC_interfaces
bool
interfaces__msg__PanelObject__Sequence__copy(
  const interfaces__msg__PanelObject__Sequence * input,
  interfaces__msg__PanelObject__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__PANEL_OBJECT__FUNCTIONS_H_
