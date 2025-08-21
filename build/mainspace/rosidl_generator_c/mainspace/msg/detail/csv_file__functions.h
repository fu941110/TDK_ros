// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mainspace:msg/CsvFile.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__CSV_FILE__FUNCTIONS_H_
#define MAINSPACE__MSG__DETAIL__CSV_FILE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "mainspace/msg/rosidl_generator_c__visibility_control.h"

#include "mainspace/msg/detail/csv_file__struct.h"

/// Initialize msg/CsvFile message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mainspace__msg__CsvFile
 * )) before or use
 * mainspace__msg__CsvFile__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mainspace
bool
mainspace__msg__CsvFile__init(mainspace__msg__CsvFile * msg);

/// Finalize msg/CsvFile message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mainspace
void
mainspace__msg__CsvFile__fini(mainspace__msg__CsvFile * msg);

/// Create msg/CsvFile message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mainspace__msg__CsvFile__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mainspace
mainspace__msg__CsvFile *
mainspace__msg__CsvFile__create();

/// Destroy msg/CsvFile message.
/**
 * It calls
 * mainspace__msg__CsvFile__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mainspace
void
mainspace__msg__CsvFile__destroy(mainspace__msg__CsvFile * msg);

/// Check for msg/CsvFile message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mainspace
bool
mainspace__msg__CsvFile__are_equal(const mainspace__msg__CsvFile * lhs, const mainspace__msg__CsvFile * rhs);

/// Copy a msg/CsvFile message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mainspace
bool
mainspace__msg__CsvFile__copy(
  const mainspace__msg__CsvFile * input,
  mainspace__msg__CsvFile * output);

/// Initialize array of msg/CsvFile messages.
/**
 * It allocates the memory for the number of elements and calls
 * mainspace__msg__CsvFile__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mainspace
bool
mainspace__msg__CsvFile__Sequence__init(mainspace__msg__CsvFile__Sequence * array, size_t size);

/// Finalize array of msg/CsvFile messages.
/**
 * It calls
 * mainspace__msg__CsvFile__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mainspace
void
mainspace__msg__CsvFile__Sequence__fini(mainspace__msg__CsvFile__Sequence * array);

/// Create array of msg/CsvFile messages.
/**
 * It allocates the memory for the array and calls
 * mainspace__msg__CsvFile__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mainspace
mainspace__msg__CsvFile__Sequence *
mainspace__msg__CsvFile__Sequence__create(size_t size);

/// Destroy array of msg/CsvFile messages.
/**
 * It calls
 * mainspace__msg__CsvFile__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mainspace
void
mainspace__msg__CsvFile__Sequence__destroy(mainspace__msg__CsvFile__Sequence * array);

/// Check for msg/CsvFile message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mainspace
bool
mainspace__msg__CsvFile__Sequence__are_equal(const mainspace__msg__CsvFile__Sequence * lhs, const mainspace__msg__CsvFile__Sequence * rhs);

/// Copy an array of msg/CsvFile messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mainspace
bool
mainspace__msg__CsvFile__Sequence__copy(
  const mainspace__msg__CsvFile__Sequence * input,
  mainspace__msg__CsvFile__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MAINSPACE__MSG__DETAIL__CSV_FILE__FUNCTIONS_H_
