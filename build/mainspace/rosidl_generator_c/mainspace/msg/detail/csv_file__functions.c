// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mainspace:msg/CsvFile.idl
// generated code does not contain a copyright notice
#include "mainspace/msg/detail/csv_file__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `file`
#include "rosidl_runtime_c/string_functions.h"

bool
mainspace__msg__CsvFile__init(mainspace__msg__CsvFile * msg)
{
  if (!msg) {
    return false;
  }
  // file
  if (!rosidl_runtime_c__String__init(&msg->file)) {
    mainspace__msg__CsvFile__fini(msg);
    return false;
  }
  return true;
}

void
mainspace__msg__CsvFile__fini(mainspace__msg__CsvFile * msg)
{
  if (!msg) {
    return;
  }
  // file
  rosidl_runtime_c__String__fini(&msg->file);
}

bool
mainspace__msg__CsvFile__are_equal(const mainspace__msg__CsvFile * lhs, const mainspace__msg__CsvFile * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // file
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->file), &(rhs->file)))
  {
    return false;
  }
  return true;
}

bool
mainspace__msg__CsvFile__copy(
  const mainspace__msg__CsvFile * input,
  mainspace__msg__CsvFile * output)
{
  if (!input || !output) {
    return false;
  }
  // file
  if (!rosidl_runtime_c__String__copy(
      &(input->file), &(output->file)))
  {
    return false;
  }
  return true;
}

mainspace__msg__CsvFile *
mainspace__msg__CsvFile__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mainspace__msg__CsvFile * msg = (mainspace__msg__CsvFile *)allocator.allocate(sizeof(mainspace__msg__CsvFile), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mainspace__msg__CsvFile));
  bool success = mainspace__msg__CsvFile__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mainspace__msg__CsvFile__destroy(mainspace__msg__CsvFile * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mainspace__msg__CsvFile__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mainspace__msg__CsvFile__Sequence__init(mainspace__msg__CsvFile__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mainspace__msg__CsvFile * data = NULL;

  if (size) {
    data = (mainspace__msg__CsvFile *)allocator.zero_allocate(size, sizeof(mainspace__msg__CsvFile), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mainspace__msg__CsvFile__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mainspace__msg__CsvFile__fini(&data[i - 1]);
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
mainspace__msg__CsvFile__Sequence__fini(mainspace__msg__CsvFile__Sequence * array)
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
      mainspace__msg__CsvFile__fini(&array->data[i]);
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

mainspace__msg__CsvFile__Sequence *
mainspace__msg__CsvFile__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mainspace__msg__CsvFile__Sequence * array = (mainspace__msg__CsvFile__Sequence *)allocator.allocate(sizeof(mainspace__msg__CsvFile__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mainspace__msg__CsvFile__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mainspace__msg__CsvFile__Sequence__destroy(mainspace__msg__CsvFile__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mainspace__msg__CsvFile__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mainspace__msg__CsvFile__Sequence__are_equal(const mainspace__msg__CsvFile__Sequence * lhs, const mainspace__msg__CsvFile__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mainspace__msg__CsvFile__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mainspace__msg__CsvFile__Sequence__copy(
  const mainspace__msg__CsvFile__Sequence * input,
  mainspace__msg__CsvFile__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mainspace__msg__CsvFile);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mainspace__msg__CsvFile * data =
      (mainspace__msg__CsvFile *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mainspace__msg__CsvFile__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mainspace__msg__CsvFile__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mainspace__msg__CsvFile__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
