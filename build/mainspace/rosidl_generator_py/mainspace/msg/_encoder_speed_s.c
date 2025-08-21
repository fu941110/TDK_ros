// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mainspace:msg/EncoderSpeed.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "mainspace/msg/detail/encoder_speed__struct.h"
#include "mainspace/msg/detail/encoder_speed__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool mainspace__msg__encoder_speed__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[42];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("mainspace.msg._encoder_speed.EncoderSpeed", full_classname_dest, 41) == 0);
  }
  mainspace__msg__EncoderSpeed * ros_message = _ros_message;
  {  // fl
    PyObject * field = PyObject_GetAttrString(_pymsg, "fl");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->fl = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // fr
    PyObject * field = PyObject_GetAttrString(_pymsg, "fr");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->fr = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // rl
    PyObject * field = PyObject_GetAttrString(_pymsg, "rl");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rl = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // rr
    PyObject * field = PyObject_GetAttrString(_pymsg, "rr");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rr = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mainspace__msg__encoder_speed__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of EncoderSpeed */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mainspace.msg._encoder_speed");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "EncoderSpeed");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mainspace__msg__EncoderSpeed * ros_message = (mainspace__msg__EncoderSpeed *)raw_ros_message;
  {  // fl
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->fl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fr
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->fr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rl
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->rl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rr
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->rr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
