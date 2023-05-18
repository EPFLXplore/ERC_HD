// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces:msg/PanelObject.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/panel_object__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
interfaces__msg__PanelObject__init(interfaces__msg__PanelObject * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    interfaces__msg__PanelObject__fini(msg);
    return false;
  }
  return true;
}

void
interfaces__msg__PanelObject__fini(interfaces__msg__PanelObject * msg)
{
  if (!msg) {
    return;
  }
  // id
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
}

bool
interfaces__msg__PanelObject__are_equal(const interfaces__msg__PanelObject * lhs, const interfaces__msg__PanelObject * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  return true;
}

bool
interfaces__msg__PanelObject__copy(
  const interfaces__msg__PanelObject * input,
  interfaces__msg__PanelObject * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  return true;
}

interfaces__msg__PanelObject *
interfaces__msg__PanelObject__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__PanelObject * msg = (interfaces__msg__PanelObject *)allocator.allocate(sizeof(interfaces__msg__PanelObject), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__msg__PanelObject));
  bool success = interfaces__msg__PanelObject__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interfaces__msg__PanelObject__destroy(interfaces__msg__PanelObject * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interfaces__msg__PanelObject__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interfaces__msg__PanelObject__Sequence__init(interfaces__msg__PanelObject__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__PanelObject * data = NULL;

  if (size) {
    data = (interfaces__msg__PanelObject *)allocator.zero_allocate(size, sizeof(interfaces__msg__PanelObject), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__msg__PanelObject__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__msg__PanelObject__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
interfaces__msg__PanelObject__Sequence__fini(interfaces__msg__PanelObject__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interfaces__msg__PanelObject__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

interfaces__msg__PanelObject__Sequence *
interfaces__msg__PanelObject__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__PanelObject__Sequence * array = (interfaces__msg__PanelObject__Sequence *)allocator.allocate(sizeof(interfaces__msg__PanelObject__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interfaces__msg__PanelObject__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interfaces__msg__PanelObject__Sequence__destroy(interfaces__msg__PanelObject__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interfaces__msg__PanelObject__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interfaces__msg__PanelObject__Sequence__are_equal(const interfaces__msg__PanelObject__Sequence * lhs, const interfaces__msg__PanelObject__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interfaces__msg__PanelObject__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interfaces__msg__PanelObject__Sequence__copy(
  const interfaces__msg__PanelObject__Sequence * input,
  interfaces__msg__PanelObject__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interfaces__msg__PanelObject);
    interfaces__msg__PanelObject * data =
      (interfaces__msg__PanelObject *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interfaces__msg__PanelObject__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interfaces__msg__PanelObject__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interfaces__msg__PanelObject__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
