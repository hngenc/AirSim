// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.


#ifndef msr_airlib_SimpleImu_hpp
#define msr_airlib_SimpleImu_hpp

#include "common/Common.hpp"
#include "ImuSimpleParams.hpp"
#include "ImuBase.hpp"
#include <Python.h>
#include "controllers/Settings.hpp"
#include <chrono>

static bool file_exists_imu(const char * name) {
	FILE * file = fopen(name, "r");
	if (file) {
		fclose(file);
		return true;
	}
	return false;
}

namespace msr { namespace airlib {

class ImuSimple : public ImuBase {
public:
    //constructors
    ImuSimple(const ImuSimpleParams& params = ImuSimpleParams())
        : params_(params)
    {
        gyro_bias_stability_norm = params_.gyro.bias_stability / sqrt(params_.gyro.tau);
        accel_bias_stability_norm = params_.accel.bias_stability / sqrt(params_.accel.tau);

		auto& settings = Settings::singleton();
		settings.initializePython();
		SetupPythonNoise();
    }

    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        ImuBase::reset();

        last_time_ = clock()->nowNanos();

        state_.gyroscope_bias = params_.gyro.turn_on_bias;
        state_.accelerometer_bias = params_.accel.turn_on_bias;
        gauss_dist.reset();
        updateOutput();
    }

    virtual void update() override
    {
        ImuBase::update();

        updateOutput();
    }
    //*** End: UpdatableState implementation ***//

    // virtual ~ImuSimple() = default;

	virtual ~ImuSimple() {
		if (python_works) {
			Py_XDECREF(pNoiseFunc);
			Py_DECREF(pModule);
		}
	}

private: //methods
    void updateOutput()
    {
		// auto start = std::chrono::system_clock::now();

        Output output;
        const GroundTruth& ground_truth = getGroundTruth();

		output.time_stamp = ClockFactory::get()->nowNanos();
        output.angular_velocity = ground_truth.kinematics->twist.angular;
        output.linear_acceleration = ground_truth.kinematics->accelerations.linear - ground_truth.environment->getState().gravity;
        output.orientation = ground_truth.kinematics->pose.orientation;

        //acceleration is in world frame so transform to body frame
        output.linear_acceleration = VectorMath::transformToBodyFrame(output.linear_acceleration, 
            ground_truth.kinematics->pose.orientation, true);
        //add noise

		// auto start_noise = std::chrono::system_clock::now();
		/*if (!python_works)
			addNoise(output.linear_acceleration, output.angular_velocity);
		else
			AddPythonNoise(output);*/
		// auto end_noise = std::chrono::system_clock::now();
        // TODO: Add noise in orientation?

		if (!file_exists_imu("C:\\Users\\root\\Documents\\AirSim\\killimu"))
			setOutput(output);

		auto end = std::chrono::system_clock::now();

		// volatile auto diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
		// volatile auto noise_diff = std::chrono::duration_cast<std::chrono::microseconds>(end_noise - start_noise).count();

		// diff++;
		// noise_diff++;
    }

	void SetupPythonNoise() {
		PyObject *pName;
		pName = PyUnicode_FromString(module_name.c_str());
		// Error checking of pName left out

		pModule = PyImport_Import(pName);

		Py_DECREF(pName);
		if (pModule != NULL) {
			pNoiseFunc = PyObject_GetAttrString(pModule, "imu_noise");
			if (!(pNoiseFunc && PyCallable_Check(pNoiseFunc))) {
				//can't do it
				python_works = false;
			}
		}
		else {
			//can't do it
			python_works = false;
			PyErr_Print();
		}
	}

	void AddPythonNoise(Output& output) {
		PyObject * pArgs = PyTuple_New(3); // adding noise to x, y, z acceleration

		PyObject *pValue0 = PyFloat_FromDouble(output.linear_acceleration.x());
		PyTuple_SetItem(pArgs, 0, pValue0);
		PyObject *pValue2 = PyFloat_FromDouble(output.linear_acceleration.y());
		PyTuple_SetItem(pArgs, 1, pValue2);
		PyObject *pValue3 = PyFloat_FromDouble(output.linear_acceleration.z());
		PyTuple_SetItem(pArgs, 2, pValue3);

		// Returns tuple of three ordered values
		PyObject * pValue = PyObject_CallObject(pNoiseFunc, pArgs);
		if (pValue == NULL) {
			//function call failed; return orig output
			return;
		}

		output.linear_acceleration.x() = PyFloat_AsDouble(PyTuple_GetItem(pValue, 0));
		output.linear_acceleration.y() = PyFloat_AsDouble(PyTuple_GetItem(pValue, 1));
		output.linear_acceleration.z() = PyFloat_AsDouble(PyTuple_GetItem(pValue, 2));

		Py_DECREF(pArgs);
		Py_DECREF(pValue);
		Py_DECREF(pValue0);
		Py_DECREF(pValue2);
		Py_DECREF(pValue3);
	}

    void addNoise(Vector3r& linear_acceleration, Vector3r& angular_velocity)
    {
        TTimeDelta dt = clock()->updateSince(last_time_);

        //ref: An introduction to inertial navigation, Oliver J. Woodman, Sec 3.2, pp 10-12
        //https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf

        real_T sqrt_dt = static_cast<real_T>(sqrt(std::max<TTimeDelta>(dt, params_.min_sample_time)));

        // Gyrosocpe
        //convert arw to stddev
        real_T gyro_sigma_arw = params_.gyro.arw / sqrt_dt;
        angular_velocity += gauss_dist.next() * gyro_sigma_arw + state_.gyroscope_bias;
        //update bias random walk
        real_T gyro_sigma_bias = gyro_bias_stability_norm * sqrt_dt;
        state_.gyroscope_bias += gauss_dist.next() * gyro_sigma_bias;

        //accelerometer
        //convert vrw to stddev
        real_T accel_sigma_vrw = params_.accel.vrw / sqrt_dt;
        linear_acceleration += gauss_dist.next() * accel_sigma_vrw + state_.accelerometer_bias;
        //update bias random walk
        real_T accel_sigma_bias = accel_bias_stability_norm * sqrt_dt;
        state_.accelerometer_bias += gauss_dist.next() * accel_sigma_bias;
    }

private: //fields
    ImuSimpleParams params_;
    RandomVectorGaussianR gauss_dist = RandomVectorGaussianR(0, 1);

    //cached calculated values
    real_T gyro_bias_stability_norm, accel_bias_stability_norm;

    struct State {
        Vector3r gyroscope_bias;
        Vector3r accelerometer_bias;
    } state_;

    TTimePoint last_time_;

	bool python_works = true;
	PyObject *pNoiseFunc, *pModule;
	const std::string module_name = "sensor_noise";
};


class ImuSimple2 : public ImuBase {
public:
    //constructors
    ImuSimple2(const ImuSimpleParams& params = ImuSimpleParams())
        : params_(params)
    {
        gyro_bias_stability_norm = params_.gyro.bias_stability / sqrt(params_.gyro.tau);
        accel_bias_stability_norm = params_.accel.bias_stability / sqrt(params_.accel.tau);
    }

    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        ImuBase::reset();

        last_time_ = clock()->nowNanos();

        state_.gyroscope_bias = params_.gyro.turn_on_bias;
        state_.accelerometer_bias = params_.accel.turn_on_bias;
        gauss_dist.reset();
        updateOutput();
    }

    virtual void update() override
    {
        ImuBase::update();

        updateOutput();
    }
    //*** End: UpdatableState implementation ***//

    virtual ~ImuSimple2() = default;

private: //methods
    void updateOutput()
    {
        Output output;
        const GroundTruth& ground_truth = getGroundTruth();

        output.angular_velocity = ground_truth.kinematics->twist.angular;
        output.linear_acceleration = ground_truth.kinematics->accelerations.linear - ground_truth.environment->getState().gravity;
        output.orientation = ground_truth.kinematics->pose.orientation;

        //acceleration is in world frame so transform to body frame
        output.linear_acceleration = VectorMath::transformToBodyFrame(output.linear_acceleration, 
            ground_truth.kinematics->pose.orientation, true);
		output.time_stamp = ClockFactory::get()->nowNanos();
        //add noise
        // addNoise(output.linear_acceleration, output.angular_velocity);
        // TODO: Add noise in orientation?

        setOutput(output);
    }

    void addNoise(Vector3r& linear_acceleration, Vector3r& angular_velocity)
    {
        TTimeDelta dt = clock()->updateSince(last_time_);

        //ref: An introduction to inertial navigation, Oliver J. Woodman, Sec 3.2, pp 10-12
        //https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf

        real_T sqrt_dt = static_cast<real_T>(sqrt(std::max<TTimeDelta>(dt, params_.min_sample_time)));

        // Gyrosocpe
        //convert arw to stddev
        real_T gyro_sigma_arw = params_.gyro.arw / sqrt_dt;
        angular_velocity += gauss_dist.next() * gyro_sigma_arw + state_.gyroscope_bias;
        //update bias random walk
        real_T gyro_sigma_bias = gyro_bias_stability_norm * sqrt_dt;
        state_.gyroscope_bias += gauss_dist.next() * gyro_sigma_bias;

        //accelerometer
        //convert vrw to stddev
        real_T accel_sigma_vrw = params_.accel.vrw / sqrt_dt;
        linear_acceleration += gauss_dist.next() * accel_sigma_vrw + state_.accelerometer_bias;
        //update bias random walk
        real_T accel_sigma_bias = accel_bias_stability_norm * sqrt_dt;
        state_.accelerometer_bias += gauss_dist.next() * accel_sigma_bias;
    }



private: //fields
    ImuSimpleParams params_;
    RandomVectorGaussianR gauss_dist = RandomVectorGaussianR(0, 1);

    //cached calculated values
    real_T gyro_bias_stability_norm, accel_bias_stability_norm;

    struct State {
        Vector3r gyroscope_bias;
        Vector3r accelerometer_bias;
    } state_;

    TTimePoint last_time_;
};


}} //namespace
#endif
