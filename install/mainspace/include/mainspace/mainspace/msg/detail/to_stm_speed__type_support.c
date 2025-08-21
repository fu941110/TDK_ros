// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mainspace:msg/ToStmSpeed.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mainspace/msg/detail/to_stm_speed__rosidl_typesupport_introspection_c.h"
#include "mainspace/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mainspace/msg/detail/to_stm_speed__functions.h"
#include "mainspace/msg/detail/to_stm_speed__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void mainspace__msg__ToStmSpeed__rosidl_typesupport_introspection_c__ToStmSpeed_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mainspace__msg__ToStmSpeed__init(message_memory);
}

void mainspace__msg__ToStmSpeed__rosidl_typesupport_introspection_c__ToStmSpeed_fini_function(void * message_memory)
{
  mainspace__msg__ToStmSpeed__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mainspace__msg__ToStmSpeed__rosidl_typesupport_introspection_c__ToStmSpeed_message_member_array[3] = {
  {
    "vx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mainspace__msg__ToStmSpeed, vx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mainspace__msg__ToStmSpeed, vy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "w",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mainspace__msg__ToStmSpeed, w),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mainspace__msg__ToStmSpeed__rosidl_typesupport_introspection_c__ToStmSpeed_message_members = {
  "mainspace__msg",  // message namespace
  "ToStmSpeed",  // message name
  3,  // number of fields
  sizeof(mainspace__msg__ToStmSpeed),
  mainspace__msg__ToStmSpeed__rosidl_typesupport_introspection_c__ToStmSpeed_message_member_array,  // message members
  mainspace__msg__ToStmSpeed__rosidl_typesupport_introspection_c__ToStmSpeed_init_function,  // function to initialize message memory (memory has to be allocated)
  mainspace__msg__ToStmSpeed__rosidl_typesupport_introspection_c__ToStmSpeed_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mainspace__msg__ToStmSpeed__rosidl_typesupport_introspection_c__ToStmSpeed_message_type_support_handle = {
  0,
  &mainspace__msg__ToStmSpeed__rosidl_typesupport_introspection_c__ToStmSpeed_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mainspace
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mainspace, msg, ToStmSpeed)() {
  if (!mainspace__msg__ToStmSpeed__rosidl_typesupport_introspection_c__ToStmSpeed_message_type_support_handle.typesupport_identifier) {
    mainspace__msg__ToStmSpeed__rosidl_typesupport_introspection_c__ToStmSpeed_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mainspace__msg__ToStmSpeed__rosidl_typesupport_introspection_c__ToStmSpeed_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
