// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mainspace:msg/Desk.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__DESK__STRUCT_H_
#define MAINSPACE__MSG__DETAIL__DESK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Desk in the package mainspace.
typedef struct mainspace__msg__Desk
{
  double x;
  double y;
} mainspace__msg__Desk;

// Struct for a sequence of mainspace__msg__Desk.
typedef struct mainspace__msg__Desk__Sequence
{
  mainspace__msg__Desk * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mainspace__msg__Desk__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAINSPACE__MSG__DETAIL__DESK__STRUCT_H_
