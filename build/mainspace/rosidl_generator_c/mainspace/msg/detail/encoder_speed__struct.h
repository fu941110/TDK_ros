// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mainspace:msg/EncoderSpeed.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__ENCODER_SPEED__STRUCT_H_
#define MAINSPACE__MSG__DETAIL__ENCODER_SPEED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/EncoderSpeed in the package mainspace.
typedef struct mainspace__msg__EncoderSpeed
{
  int32_t fl;
  int32_t fr;
  int32_t rl;
  int32_t rr;
} mainspace__msg__EncoderSpeed;

// Struct for a sequence of mainspace__msg__EncoderSpeed.
typedef struct mainspace__msg__EncoderSpeed__Sequence
{
  mainspace__msg__EncoderSpeed * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mainspace__msg__EncoderSpeed__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAINSPACE__MSG__DETAIL__ENCODER_SPEED__STRUCT_H_
