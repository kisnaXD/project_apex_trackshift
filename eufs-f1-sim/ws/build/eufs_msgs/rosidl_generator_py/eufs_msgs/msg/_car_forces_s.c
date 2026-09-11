// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from eufs_msgs:msg/CarForces.idl
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
#include "eufs_msgs/msg/detail/car_forces__struct.h"
#include "eufs_msgs/msg/detail/car_forces__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool eufs_msgs__msg__car_forces__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[36];
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
    assert(strncmp("eufs_msgs.msg._car_forces.CarForces", full_classname_dest, 35) == 0);
  }
  eufs_msgs__msg__CarForces * ros_message = _ros_message;
  {  // front_left_lateral
    PyObject * field = PyObject_GetAttrString(_pymsg, "front_left_lateral");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->front_left_lateral = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // front_left_longitudinal
    PyObject * field = PyObject_GetAttrString(_pymsg, "front_left_longitudinal");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->front_left_longitudinal = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // front_left_vertical
    PyObject * field = PyObject_GetAttrString(_pymsg, "front_left_vertical");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->front_left_vertical = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // front_right_lateral
    PyObject * field = PyObject_GetAttrString(_pymsg, "front_right_lateral");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->front_right_lateral = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // front_right_longitudinal
    PyObject * field = PyObject_GetAttrString(_pymsg, "front_right_longitudinal");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->front_right_longitudinal = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // front_right_vertical
    PyObject * field = PyObject_GetAttrString(_pymsg, "front_right_vertical");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->front_right_vertical = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rear_left_lateral
    PyObject * field = PyObject_GetAttrString(_pymsg, "rear_left_lateral");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rear_left_lateral = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rear_left_longitudinal
    PyObject * field = PyObject_GetAttrString(_pymsg, "rear_left_longitudinal");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rear_left_longitudinal = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rear_left_vertical
    PyObject * field = PyObject_GetAttrString(_pymsg, "rear_left_vertical");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rear_left_vertical = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rear_right_lateral
    PyObject * field = PyObject_GetAttrString(_pymsg, "rear_right_lateral");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rear_right_lateral = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rear_right_longitudinal
    PyObject * field = PyObject_GetAttrString(_pymsg, "rear_right_longitudinal");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rear_right_longitudinal = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rear_right_vertical
    PyObject * field = PyObject_GetAttrString(_pymsg, "rear_right_vertical");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rear_right_vertical = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * eufs_msgs__msg__car_forces__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CarForces */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("eufs_msgs.msg._car_forces");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CarForces");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  eufs_msgs__msg__CarForces * ros_message = (eufs_msgs__msg__CarForces *)raw_ros_message;
  {  // front_left_lateral
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->front_left_lateral);
    {
      int rc = PyObject_SetAttrString(_pymessage, "front_left_lateral", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // front_left_longitudinal
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->front_left_longitudinal);
    {
      int rc = PyObject_SetAttrString(_pymessage, "front_left_longitudinal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // front_left_vertical
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->front_left_vertical);
    {
      int rc = PyObject_SetAttrString(_pymessage, "front_left_vertical", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // front_right_lateral
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->front_right_lateral);
    {
      int rc = PyObject_SetAttrString(_pymessage, "front_right_lateral", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // front_right_longitudinal
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->front_right_longitudinal);
    {
      int rc = PyObject_SetAttrString(_pymessage, "front_right_longitudinal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // front_right_vertical
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->front_right_vertical);
    {
      int rc = PyObject_SetAttrString(_pymessage, "front_right_vertical", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rear_left_lateral
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rear_left_lateral);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rear_left_lateral", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rear_left_longitudinal
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rear_left_longitudinal);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rear_left_longitudinal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rear_left_vertical
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rear_left_vertical);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rear_left_vertical", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rear_right_lateral
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rear_right_lateral);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rear_right_lateral", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rear_right_longitudinal
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rear_right_longitudinal);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rear_right_longitudinal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rear_right_vertical
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rear_right_vertical);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rear_right_vertical", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
