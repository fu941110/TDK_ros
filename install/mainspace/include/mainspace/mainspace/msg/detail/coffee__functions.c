// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mainspace:msg/Coffee.idl
// generated code does not contain a copyright notice
#include "mainspace/msg/detail/coffee__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `type`
#include "rosidl_runtime_c/string_functions.h"

bool
mainspace__msg__Coffee__init(mainspace__msg__Coffee * msg)
{
  if (!msg) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__init(&msg->type)) {
    mainspace__msg__Coffee__fini(msg);
    return false;
  }
  // number
  return true;
}

void
mainspace__msg__Coffee__fini(mainspace__msg__Coffee * msg)
{
  if (!msg) {
    return;
  }
  // type
  rosidl_runtime_c__String__fini(&msg->type);
  // number
}

bool
mainspace__msg__Coffee__are_equal(const mainspace__msg__Coffee * lhs, const mainspace__msg__Coffee * rhs)
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
  // number
  if (lhs->number != rhs->number) {
    return false;
  }
  return true;
}

bool
mainspace__msg__Coffee__copy(
  const mainspace__msg__Coffee * input,
  mainspace__msg__Coffee * output)
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
  // number
  output->number = input->number;
  return true;
}

mainspace__msg__Coffee *
mainspace__msg__Coffee__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mainspace__msg__Coffee * msg = (mainspace__msg__Coffee *)allocator.allocate(sizeof(mainspace__msg__Coffee), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mainspace__msg__Coffee));
  bool success = mainspace__msg__Coffee__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mainspace__msg__Coffee__destroy(mainspace__msg__Coffee * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mainspace__msg__Coffee__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mainspace__msg__Coffee__Sequence__init(mainspace__msg__Coffee__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mainspace__msg__Coffee * data = NULL;

  if (size) {
    data = (mainspace__msg__Coffee *)allocator.zero_allocate(size, sizeof(mainspace__msg__Coffee), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mainspace__msg__Coffee__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mainspace__msg__Coffee__fini(&data[i - 1]);
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
mainspace__msg__Coffee__Sequence__fini(mainspace__msg__Coffee__Sequence * array)
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
      mainspace__msg__Coffee__fini(&array->data[i]);
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

mainspace__msg__Coffee__Sequence *
mainspace__msg__Coffee__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mainspace__msg__Coffee__Sequence * array = (mainspace__msg__Coffee__Sequence *)allocator.allocate(sizeof(mainspace__msg__Coffee__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mainspace__msg__Coffee__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mainspace__msg__Coffee__Sequence__destroy(mainspace__msg__Coffee__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mainspace__msg__Coffee__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mainspace__msg__Coffee__Sequence__are_equal(const mainspace__msg__Coffee__Sequence * lhs, const mainspace__msg__Coffee__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mainspace__msg__Coffee__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mainspace__msg__Coffee__Sequence__copy(
  const mainspace__msg__Coffee__Sequence * input,
  mainspace__msg__Coffee__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mainspace__msg__Coffee);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mainspace__msg__Coffee * data =
      (mainspace__msg__Coffee *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mainspace__msg__Coffee__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mainspace__msg__Coffee__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mainspace__msg__Coffee__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
