// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mainspace:msg/Coffee.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__COFFEE__STRUCT_H_
#define MAINSPACE__MSG__DETAIL__COFFEE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Coffee in the package mainspace.
typedef struct mainspace__msg__Coffee
{
  rosidl_runtime_c__String type;
  int32_t number;
} mainspace__msg__Coffee;

// Struct for a sequence of mainspace__msg__Coffee.
typedef struct mainspace__msg__Coffee__Sequence
{
  mainspace__msg__Coffee * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mainspace__msg__Coffee__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAINSPACE__MSG__DETAIL__COFFEE__STRUCT_H_
