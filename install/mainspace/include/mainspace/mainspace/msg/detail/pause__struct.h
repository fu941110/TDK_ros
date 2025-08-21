// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mainspace:msg/Pause.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__PAUSE__STRUCT_H_
#define MAINSPACE__MSG__DETAIL__PAUSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Pause in the package mainspace.
typedef struct mainspace__msg__Pause
{
  bool pause;
} mainspace__msg__Pause;

// Struct for a sequence of mainspace__msg__Pause.
typedef struct mainspace__msg__Pause__Sequence
{
  mainspace__msg__Pause * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mainspace__msg__Pause__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAINSPACE__MSG__DETAIL__PAUSE__STRUCT_H_
