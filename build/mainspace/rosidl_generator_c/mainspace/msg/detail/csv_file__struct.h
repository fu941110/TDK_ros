// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mainspace:msg/CsvFile.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__CSV_FILE__STRUCT_H_
#define MAINSPACE__MSG__DETAIL__CSV_FILE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'file'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/CsvFile in the package mainspace.
typedef struct mainspace__msg__CsvFile
{
  rosidl_runtime_c__String file;
} mainspace__msg__CsvFile;

// Struct for a sequence of mainspace__msg__CsvFile.
typedef struct mainspace__msg__CsvFile__Sequence
{
  mainspace__msg__CsvFile * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mainspace__msg__CsvFile__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAINSPACE__MSG__DETAIL__CSV_FILE__STRUCT_H_
