// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_Gps_hpp
#define msr_airlib_Gps_hpp

#include <random>
#include "common/Common.hpp"
#include "GpsSimpleParams.hpp"
#include "GpsBase.hpp"
#include "common/FirstOrderFilter.hpp"
#include "common/FrequencyLimiter.hpp"
#include "common/DelayLine.hpp"
#include "common/CommonPython.hpp"

static bool file_exists_gps(const char * name) {
	FILE * file = fopen(name, "r");
	if (file) {
		fclose(file);
		return true;
	}
	return false;
}

namespace msr { namespace airlib {

class GpsSimple : public GpsBase {
public: //methods
    GpsSimple(const GpsSimpleParams& params = GpsSimpleParams())
        : params_(params)
    {
        //initialize frequency limiter
        freq_limiter_.initialize(params_.update_frequency, params_.startup_delay);
        delay_line_.initialize(params_.update_latency);

        //initialize filters
        eph_filter.initialize(params_.eph_time_constant, params_.eph_final, params_.eph_initial); //starting dilution set to 100 which we will reduce over time to targeted 0.3f, with 45% accuracy within 100 updates, each update occurring at 0.2s interval
        epv_filter.initialize(params_.epv_time_constant, params_.epv_final, params_.epv_initial);

		// Py_Initialize();
		initializePython();
		SetupPythonNoise();
    }

    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        GpsBase::reset();

        freq_limiter_.reset();
        delay_line_.reset();

        eph_filter.reset();
        epv_filter.reset();

        addOutputToDelayLine(eph_filter.getOutput(), epv_filter.getOutput());
    }

    virtual void update() override
    {
        GpsBase::update();

        freq_limiter_.update();
        eph_filter.update();
        epv_filter.update();

        if (freq_limiter_.isWaitComplete()) {   //update output
            addOutputToDelayLine(eph_filter.getOutput(), epv_filter.getOutput());
        }

        delay_line_.update();

		if (file_exists_gps("C:\\Users\\root\\Documents\\AirSim\\killgps"))
			return;

        if (freq_limiter_.isWaitComplete())
            setOutput(delay_line_.getOutput());
    }

    //*** End: UpdatableState implementation ***//

    // virtual ~GpsSimple() = default;

	virtual ~GpsSimple() {
		if (python_works) {
			Py_XDECREF(pNoiseFunc);
		}
		// Py_Finalize();
		debug = finalizePython();
		std::cout << debug << std::endl;
	}

private:
    void addOutputToDelayLine(real_T eph, real_T epv)
    {
        Output output;
        const GroundTruth& ground_truth = getGroundTruth();

        //GNSS
        output.gnss.time_utc = static_cast<uint64_t>(clock()->nowNanos() / 1.0E3);
        output.gnss.geo_point = ground_truth.environment->getState().geo_point;
        output.gnss.eph = eph;
        output.gnss.epv = epv;
        output.gnss.velocity = ground_truth.kinematics->twist.linear;
        output.is_valid = true;

        output.gnss.fix_type =
            output.gnss.eph <= params_.eph_min_3d ? GnssFixType::GNSS_FIX_3D_FIX
            : output.gnss.eph <= params_.eph_min_2d ? GnssFixType::GNSS_FIX_2D_FIX
            : GnssFixType::GNSS_FIX_NO_FIX;

		output.time_stamp = ClockFactory::get()->nowNanos();

		if (python_works) {
			AddPythonNoise(output);
		}

        delay_line_.push_back(output);
    }

	void SetupPythonNoise() {
		pNoiseFunc = getNoiseFunc("gps_noise");
		python_works = (pNoiseFunc != nullptr);
	}

	void AddPythonNoise(Output& output) {
		python_lock();

		PyObject * pArgs = PyTuple_New(3); // adding noise to x, y, z acceleration

		PyObject *pValue0 = PyFloat_FromDouble(output.gnss.geo_point.latitude);
		PyTuple_SetItem(pArgs, 0, pValue0);
		PyObject *pValue1 = PyFloat_FromDouble(output.gnss.geo_point.longitude);
		PyTuple_SetItem(pArgs, 1, pValue1);
		PyObject *pValue2 = PyFloat_FromDouble(output.gnss.geo_point.altitude);
		PyTuple_SetItem(pArgs, 2, pValue2);

		// Returns tuple of three ordered values
		PyObject * pValue = PyObject_CallObject(pNoiseFunc, pArgs);
		if (pValue != NULL) {
			output.gnss.geo_point.latitude = PyFloat_AsDouble(PyTuple_GetItem(pValue, 0));
			output.gnss.geo_point.longitude = PyFloat_AsDouble(PyTuple_GetItem(pValue, 1));
			output.gnss.geo_point.altitude = PyFloat_AsDouble(PyTuple_GetItem(pValue, 2));
		}

		Py_DECREF(pArgs);
		Py_XDECREF(pValue);
		Py_DECREF(pValue0);
		Py_DECREF(pValue1);
		Py_DECREF(pValue2);

		python_unlock();
	}

private:
    typedef std::normal_distribution<> NormalDistribution;

    GpsSimpleParams params_;

    FirstOrderFilter<real_T> eph_filter, epv_filter;
    FrequencyLimiter freq_limiter_;
    DelayLine<Output> delay_line_;

	bool python_works = true;
	PyObject *pNoiseFunc = nullptr;
	const std::string module_name = "sensor_noise";
	int debug = -1;
};

}} //namespace
#endif
