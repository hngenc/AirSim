#include "common/CommonPython.hpp"
#include <Python.h>
#include <string>
#include <mutex>

static const std::string module_name = "sensor_noise";
static PyObject * pModule = nullptr;

static PyObject * pDepthNoiseFunc = nullptr;
static bool depthNoiseFuncInitialized = false;

// Globals to lock and unlock python
static std::mutex python_mutex;

// Globals to keep track of when to finalize and initialize
static const int sensors_num = 3;
static int sensors_exited = 0;

void initializePython() {
	python_lock();

	static bool first_initialized = false;

	if (!first_initialized) {
		Py_Initialize();
		PyObject *pName = PyUnicode_FromString(module_name.c_str());
		pModule = PyImport_Import(pName);
		Py_DECREF(pName);

		if (pModule == nullptr) {
			throw "python module could not be imported";
		}

		first_initialized = true;
	}
	
	sensors_exited = 0;
	pModule = PyImport_ReloadModule(pModule);

	if (pModule == nullptr) {
		throw "python module could not be imported";
	}

	python_unlock();
}

int finalizePython() {
	python_lock();

	sensors_exited++;

	if (sensors_exited == sensors_num) {
		// Py_DECREF(pModule);
		Py_XDECREF(pDepthNoiseFunc);
		// Py_Finalize();

		// pModule = nullptr;
		pDepthNoiseFunc = nullptr;
		depthNoiseFuncInitialized = false;
	}

	python_unlock();

	return sensors_num - sensors_exited;
}

PyObject * getNoiseFunc(std::string noise_name) {
	python_lock();

	PyObject * pNoiseFunc;

	if (pModule != NULL) {
		pNoiseFunc = PyObject_GetAttrString(pModule, noise_name.c_str());
		if (!(pNoiseFunc && PyCallable_Check(pNoiseFunc))) {
			//can't do it
			pNoiseFunc = nullptr;
			PyErr_Print();
		}
	}
	else {
		//can't do it
		pNoiseFunc = nullptr;
		PyErr_Print();
	}

	python_unlock();
	return pNoiseFunc;
}

PyObject * getDepthNoiseFunc() {
	if (!depthNoiseFuncInitialized) {
		pDepthNoiseFunc = getNoiseFunc("depth_noise");
		depthNoiseFuncInitialized = true;
	}

	return pDepthNoiseFunc;
}

void python_lock()
{
	python_mutex.lock();
}

void python_unlock()
{
	python_mutex.unlock();
}
