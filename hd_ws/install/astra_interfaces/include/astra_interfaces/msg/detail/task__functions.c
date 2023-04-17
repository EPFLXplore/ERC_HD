// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astra_interfaces:msg/Task.idl
// generated code does not contain a copyright notice
#include "astra_interfaces/msg/detail/task__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `description`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
astra_interfaces__msg__Task__init(astra_interfaces__msg__Task * msg)
{
  if (!msg) {
    return false;
  }
  // description
  if (!rosidl_runtime_c__String__init(&msg->description)) {
    astra_interfaces__msg__Task__fini(msg);
    return false;
  }
  // id
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    astra_interfaces__msg__Task__fini(msg);
    return false;
  }
  return true;
}

void
astra_interfaces__msg__Task__fini(astra_interfaces__msg__Task * msg)
{
  if (!msg) {
    return;
  }
  // description
  rosidl_runtime_c__String__fini(&msg->description);
  // id
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
}

bool
astra_interfaces__msg__Task__are_equal(const astra_interfaces__msg__Task * lhs, const astra_interfaces__msg__Task * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // description
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->description), &(rhs->description)))
  {
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
astra_interfaces__msg__Task__copy(
  const astra_interfaces__msg__Task * input,
  astra_interfaces__msg__Task * output)
{
  if (!input || !output) {
    return false;
  }
  // description
  if (!rosidl_runtime_c__String__copy(
      &(input->description), &(output->description)))
  {
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

astra_interfaces__msg__Task *
astra_interfaces__msg__Task__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astra_interfaces__msg__Task * msg = (astra_interfaces__msg__Task *)allocator.allocate(sizeof(astra_interfaces__msg__Task), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astra_interfaces__msg__Task));
  bool success = astra_interfaces__msg__Task__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astra_interfaces__msg__Task__destroy(astra_interfaces__msg__Task * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astra_interfaces__msg__Task__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astra_interfaces__msg__Task__Sequence__init(astra_interfaces__msg__Task__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astra_interfaces__msg__Task * data = NULL;

  if (size) {
    data = (astra_interfaces__msg__Task *)allocator.zero_allocate(size, sizeof(astra_interfaces__msg__Task), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astra_interfaces__msg__Task__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astra_interfaces__msg__Task__fini(&data[i - 1]);
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
astra_interfaces__msg__Task__Sequence__fini(astra_interfaces__msg__Task__Sequence * array)
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
      astra_interfaces__msg__Task__fini(&array->data[i]);
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

astra_interfaces__msg__Task__Sequence *
astra_interfaces__msg__Task__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astra_interfaces__msg__Task__Sequence * array = (astra_interfaces__msg__Task__Sequence *)allocator.allocate(sizeof(astra_interfaces__msg__Task__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astra_interfaces__msg__Task__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astra_interfaces__msg__Task__Sequence__destroy(astra_interfaces__msg__Task__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astra_interfaces__msg__Task__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astra_interfaces__msg__Task__Sequence__are_equal(const astra_interfaces__msg__Task__Sequence * lhs, const astra_interfaces__msg__Task__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astra_interfaces__msg__Task__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astra_interfaces__msg__Task__Sequence__copy(
  const astra_interfaces__msg__Task__Sequence * input,
  astra_interfaces__msg__Task__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astra_interfaces__msg__Task);
    astra_interfaces__msg__Task * data =
      (astra_interfaces__msg__Task *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astra_interfaces__msg__Task__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          astra_interfaces__msg__Task__fini(&data[i]);
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
    if (!astra_interfaces__msg__Task__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
