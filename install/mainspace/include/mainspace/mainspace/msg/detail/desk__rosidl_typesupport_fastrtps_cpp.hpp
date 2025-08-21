// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from mainspace:msg/Desk.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__DESK__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define MAINSPACE__MSG__DETAIL__DESK__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "mainspace/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "mainspace/msg/detail/desk__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace mainspace
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mainspace
cdr_serialize(
  const mainspace::msg::Desk & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mainspace
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  mainspace::msg::Desk & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mainspace
get_serialized_size(
  const mainspace::msg::Desk & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mainspace
max_serialized_size_Desk(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace mainspace

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mainspace
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mainspace, msg, Desk)();

#ifdef __cplusplus
}
#endif

#endif  // MAINSPACE__MSG__DETAIL__DESK__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
