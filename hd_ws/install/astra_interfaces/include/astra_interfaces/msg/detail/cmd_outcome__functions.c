// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astra_interfaces:msg/CmdOutcome.idl
// generated code does not contain a copyright notice
#include "astra_interfaces/msg/detail/cmd_outcome__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
astra_interfaces__msg__CmdOutcome__init(astra_interfaces__msg__CmdOutcome * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // code
  return true;
}

void
astra_interfaces__msg__CmdOutcome__fini(astra_interfaces__msg__CmdOutcome * msg)
{
  if (!msg) {
    return;
  }
  // id
  // code
}

bool
astra_interfaces__msg__CmdOutcome__are_equal(const astra_interfaces__msg__CmdOutcome * lhs, const astra_interfaces__msg__CmdOutcome * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // code
  if (lhs->code != rhs->code) {
    return false;
  }
  return true;
}

bool
astra_interfaces__msg__CmdOutcome__copy(
  const astra_interfaces__msg__CmdOutcome * input,
  astra_interfaces__msg__CmdOutcome * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // code
  output->code = input->code;
  return true;
}

astra_interfaces__msg__CmdOutcome *
astra_interfaces__msg__CmdOutcome__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astra_interfaces__msg__CmdOutcome * msg = (astra_interfaces__msg__CmdOutcome *)allocator.allocate(sizeof(astra_interfaces__msg__CmdOutcome), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astra_interfaces__msg__CmdOutcome));
  bool success = astra_interfaces__msg__CmdOutcome__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astra_interfaces__msg__CmdOutcome__destroy(astra_interfaces__msg__CmdOutcome * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astra_interfaces__msg__CmdOutcome__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astra_interfaces__msg__CmdOutcome__Sequence__init(astra_interfaces__msg__CmdOutcome__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astra_interfaces__msg__CmdOutcome * data = NULL;

  if (size) {
    data = (astra_interfaces__msg__CmdOutcome *)allocator.zero_allocate(size, sizeof(astra_interfaces__msg__CmdOutcome), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astra_interfaces__msg__CmdOutcome__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astra_interfaces__msg__CmdOutcome__fini(&data[i - 1]);
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
astra_interfaces__msg__CmdOutcome__Sequence__fini(astra_interfaces__msg__CmdOutcome__Sequence * array)
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
      astra_interfaces__msg__CmdOutcome__fini(&array->data[i]);
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

astra_interfaces__msg__CmdOutcome__Sequence *
astra_interfaces__msg__CmdOutcome__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astra_interfaces__msg__CmdOutcome__Sequence * array = (astra_interfaces__msg__CmdOutcome__Sequence *)allocator.allocate(sizeof(astra_interfaces__msg__CmdOutcome__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astra_interfaces__msg__CmdOutcome__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astra_interfaces__msg__CmdOutcome__Sequence__destroy(astra_interfaces__msg__CmdOutcome__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astra_interfaces__msg__CmdOutcome__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astra_interfaces__msg__CmdOutcome__Sequence__are_equal(const astra_interfaces__msg__CmdOutcome__Sequence * lhs, const astra_interfaces__msg__CmdOutcome__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astra_interfaces__msg__CmdOutcome__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astra_interfaces__msg__CmdOutcome__Sequence__copy(
  const astra_interfaces__msg__CmdOutcome__Sequence * input,
  astra_interfaces__msg__CmdOutcome__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astra_interfaces__msg__CmdOutcome);
    astra_interfaces__msg__CmdOutcome * data =
      (astra_interfaces__msg__CmdOutcome *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astra_interfaces__msg__CmdOutcome__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          astra_interfaces__msg__CmdOutcome__fini(&data[i]);
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
    if (!astra_interfaces__msg__CmdOutcome__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
