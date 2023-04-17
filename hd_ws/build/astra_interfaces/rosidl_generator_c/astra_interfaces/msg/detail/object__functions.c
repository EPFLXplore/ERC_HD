// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astra_interfaces:msg/Object.idl
// generated code does not contain a copyright notice
#include "astra_interfaces/msg/detail/object__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `type`
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `dims`
#include "std_msgs/msg/detail/float32_multi_array__functions.h"

bool
astra_interfaces__msg__Object__init(astra_interfaces__msg__Object * msg)
{
  if (!msg) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__init(&msg->type)) {
    astra_interfaces__msg__Object__fini(msg);
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    astra_interfaces__msg__Object__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    astra_interfaces__msg__Object__fini(msg);
    return false;
  }
  // dims
  if (!std_msgs__msg__Float32MultiArray__init(&msg->dims)) {
    astra_interfaces__msg__Object__fini(msg);
    return false;
  }
  return true;
}

void
astra_interfaces__msg__Object__fini(astra_interfaces__msg__Object * msg)
{
  if (!msg) {
    return;
  }
  // type
  rosidl_runtime_c__String__fini(&msg->type);
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // dims
  std_msgs__msg__Float32MultiArray__fini(&msg->dims);
}

bool
astra_interfaces__msg__Object__are_equal(const astra_interfaces__msg__Object * lhs, const astra_interfaces__msg__Object * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->type), &(rhs->type)))
  {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // dims
  if (!std_msgs__msg__Float32MultiArray__are_equal(
      &(lhs->dims), &(rhs->dims)))
  {
    return false;
  }
  return true;
}

bool
astra_interfaces__msg__Object__copy(
  const astra_interfaces__msg__Object * input,
  astra_interfaces__msg__Object * output)
{
  if (!input || !output) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__copy(
      &(input->type), &(output->type)))
  {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // dims
  if (!std_msgs__msg__Float32MultiArray__copy(
      &(input->dims), &(output->dims)))
  {
    return false;
  }
  return true;
}

astra_interfaces__msg__Object *
astra_interfaces__msg__Object__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astra_interfaces__msg__Object * msg = (astra_interfaces__msg__Object *)allocator.allocate(sizeof(astra_interfaces__msg__Object), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astra_interfaces__msg__Object));
  bool success = astra_interfaces__msg__Object__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astra_interfaces__msg__Object__destroy(astra_interfaces__msg__Object * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astra_interfaces__msg__Object__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astra_interfaces__msg__Object__Sequence__init(astra_interfaces__msg__Object__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astra_interfaces__msg__Object * data = NULL;

  if (size) {
    data = (astra_interfaces__msg__Object *)allocator.zero_allocate(size, sizeof(astra_interfaces__msg__Object), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astra_interfaces__msg__Object__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astra_interfaces__msg__Object__fini(&data[i - 1]);
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
astra_interfaces__msg__Object__Sequence__fini(astra_interfaces__msg__Object__Sequence * array)
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
      astra_interfaces__msg__Object__fini(&array->data[i]);
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

astra_interfaces__msg__Object__Sequence *
astra_interfaces__msg__Object__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astra_interfaces__msg__Object__Sequence * array = (astra_interfaces__msg__Object__Sequence *)allocator.allocate(sizeof(astra_interfaces__msg__Object__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astra_interfaces__msg__Object__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astra_interfaces__msg__Object__Sequence__destroy(astra_interfaces__msg__Object__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astra_interfaces__msg__Object__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astra_interfaces__msg__Object__Sequence__are_equal(const astra_interfaces__msg__Object__Sequence * lhs, const astra_interfaces__msg__Object__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astra_interfaces__msg__Object__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astra_interfaces__msg__Object__Sequence__copy(
  const astra_interfaces__msg__Object__Sequence * input,
  astra_interfaces__msg__Object__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astra_interfaces__msg__Object);
    astra_interfaces__msg__Object * data =
      (astra_interfaces__msg__Object *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astra_interfaces__msg__Object__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          astra_interfaces__msg__Object__fini(&data[i]);
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
    if (!astra_interfaces__msg__Object__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
