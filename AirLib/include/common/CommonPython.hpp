#ifndef COMMON_PYTHON_HPP
#define COMMON_PYTHON_HPP

#include <Python.h>
#include <string>

void initializePython();
int finalizePython();

PyObject * getNoiseFunc(std::string noise_name);
PyObject * getDepthNoiseFunc();

void python_lock();
void python_unlock();

#endif
