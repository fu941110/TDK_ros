// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mainspace:msg/EncoderSpeed.idl
// generated code does not contain a copyright notice
#include "mainspace/msg/detail/encoder_speed__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
mainspace__msg__EncoderSpeed__init(mainspace__msg__EncoderSpeed * msg)
{
  if (!msg) {
    return false;
  }
  // fl
  // fr
  // rl
  // rr
  return true;
}

void
mainspace__msg__EncoderSpeed__fini(mainspace__msg__EncoderSpeed * msg)
{
  if (!msg) {
    return;
  }
  // fl
  // fr
  // rl
  // rr
}

bool
mainspace__msg__EncoderSpeed__are_equal(const mainspace__msg__EncoderSpeed * lhs, const mainspace__msg__EncoderSpeed * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // fl
  if (lhs->fl != rhs->fl) {
    return false;
  }
  // fr
  if (lhs->fr != rhs->fr) {
    return false;
  }
  // rl
  if (lhs->rl != rhs->rl) {
    return false;
  }
  // rr
  if (lhs->rr != rhs->rr) {
    return false;
  }
  return true;
}

bool
mainspace__msg__EncoderSpeed__copy(
  const mainspace__msg__EncoderSpeed * input,
  mainspace__msg__EncoderSpeed * output)
{
  if (!input || !output) {
    return false;
  }
  // fl
  output->fl = input->fl;
  // fr
  output->fr = input->fr;
  // rl
  output->rl = input->rl;
  // rr
  output->rr = input->rr;
  return true;
}

mainspace__msg__EncoderSpeed *
mainspace__msg__EncoderSpeed__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mainspace__msg__EncoderSpeed * msg = (mainspace__msg__EncoderSpeed *)allocator.allocate(sizeof(mainspace__msg__EncoderSpeed), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mainspace__msg__EncoderSpeed));
  bool success = mainspace__msg__EncoderSpeed__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mainspace__msg__EncoderSpeed__destroy(mainspace__msg__EncoderSpeed * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mainspace__msg__EncoderSpeed__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mainspace__msg__EncoderSpeed__Sequence__init(mainspace__msg__EncoderSpeed__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mainspace__msg__EncoderSpeed * data = NULL;

  if (size) {
    data = (mainspace__msg__EncoderSpeed *)allocator.zero_allocate(size, sizeof(mainspace__msg__EncoderSpeed), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mainspace__msg__EncoderSpeed__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mainspace__msg__EncoderSpeed__fini(&data[i - 1]);
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
mainspace__msg__EncoderSpeed__Sequence__fini(mainspace__msg__EncoderSpeed__Sequence * array)
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
      mainspace__msg__EncoderSpeed__fini(&array->data[i]);
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

mainspace__msg__EncoderSpeed__Sequence *
mainspace__msg__EncoderSpeed__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mainspace__msg__EncoderSpeed__Sequence * array = (mainspace__msg__EncoderSpeed__Sequence *)allocator.allocate(sizeof(mainspace__msg__EncoderSpeed__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mainspace__msg__EncoderSpeed__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mainspace__msg__EncoderSpeed__Sequence__destroy(mainspace__msg__EncoderSpeed__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mainspace__msg__EncoderSpeed__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mainspace__msg__EncoderSpeed__Sequence__are_equal(const mainspace__msg__EncoderSpeed__Sequence * lhs, const mainspace__msg__EncoderSpeed__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mainspace__msg__EncoderSpeed__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mainspace__msg__EncoderSpeed__Sequence__copy(
  const mainspace__msg__EncoderSpeed__Sequence * input,
  mainspace__msg__EncoderSpeed__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mainspace__msg__EncoderSpeed);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mainspace__msg__EncoderSpeed * data =
      (mainspace__msg__EncoderSpeed *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mainspace__msg__EncoderSpeed__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mainspace__msg__EncoderSpeed__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mainspace__msg__EncoderSpeed__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
