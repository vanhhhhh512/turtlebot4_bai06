// Persistent Gazebo transport client; rendering never shares the control node.
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <google/protobuf/text_format.h>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/marker_v.pb.h>
#include <ignition/msgs/boolean.pb.h>

static PyObject *Send(PyObject *, PyObject *args) {
  const char *text;
  unsigned int timeout = 60;
  if (!PyArg_ParseTuple(args, "s|I", &text, &timeout)) return nullptr;
  ignition::msgs::Marker_V request;
  if (!google::protobuf::TextFormat::ParseFromString(text, &request)) {
    PyErr_SetString(PyExc_ValueError, "Invalid Gazebo Marker_V message");
    return nullptr;
  }
  static ignition::transport::Node node;
  ignition::msgs::Boolean reply;
  bool result = false, executed = false;
  Py_BEGIN_ALLOW_THREADS
  executed = node.Request("/marker_array", request, timeout, reply, result);
  Py_END_ALLOW_THREADS
  return PyBool_FromLong(executed && result && reply.data());
}

static PyMethodDef methods[] = {
  {"send", Send, METH_VARARGS, "Send a bounded Marker_V request to Gazebo GUI."},
  {nullptr, nullptr, 0, nullptr}
};
static PyModuleDef module = {PyModuleDef_HEAD_INIT, "_ign_markers", nullptr, -1, methods};
PyMODINIT_FUNC PyInit__ign_markers() { return PyModule_Create(&module); }
