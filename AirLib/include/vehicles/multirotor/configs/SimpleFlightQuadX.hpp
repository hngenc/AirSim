// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_SimpleFlightQuadX_hpp
#define msr_airlib_vehicles_SimpleFlightQuadX_hpp

#include "vehicles/multirotor/firmwares/simple_flight/SimpleFlightDroneController.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"

// Setup the drone's parameters based on several pre-defined models
// #define F450
// #define _3DR_SOLO
#define MATRICE_100


namespace msr { namespace airlib {

class SimpleFlightQuadX : public MultiRotorParams {
public:
    SimpleFlightQuadX(const AirSimSettings::VehicleSettings& vehicle_settings, std::shared_ptr<const SensorFactory> sensor_factory)
        : vehicle_settings_(vehicle_settings), sensor_factory_(sensor_factory)
    {
    }

    virtual ~SimpleFlightQuadX() = default;

protected:
    virtual void setupParams() override
    {
        auto& params = getParams();

        /******* Below is same config as PX4 generic model ********/
#ifdef F450
        /* TODO: calculate and un-comment max_rpm and propeller_diameter */
        params.rotor_count = 4;
        params.mass = 1.0f;
        real_T motor_assembly_weight = 0.055f;
        real_T motor_arm_length = 0.2275f;
        params.rotor_params.C_P = 0.040164f; // the torque co-efficient
        params.rotor_params.C_T = 0.109919f; // the thrust co-efficient
        // params.rotor_params.max_rpm = 13024 * .6; // RPM can be obtained by KV * Voltage
        // params.rotor_params.propeller_diameter = 0.254f; // in meters

        params.body_box.x() = 0.180f; params.body_box.y() = 0.11f; params.body_box.z() = 0.040f;
        real_T rotor_z = 2.5f / 100;
#elif defined(_3DR_SOLO)
        /* TODO: make a new rotor_params and update it with the correct coefficents, rpm, and prop diameter */
        params.rotor_count = 4;
        params.mass = 1.5f;
        real_T motor_assembly_weight = 0.0680389f;
        real_T motor_arm_length = .130175f;
        params.rotor_params.C_P = 0.040164f; // the torque co-efficient
        params.rotor_params.C_T = 0.109919f; // the thrust co-efficient
        params.rotor_params.max_rpm = 13024 * .6; //RPM can be obtained by KV * Voltage
        params.rotor_params.propeller_diameter = 0.254f; //in meters

        params.body_box.x() = .2428875f; params.body_box.y() = .10795; params.body_box.z() = .066675;
        real_T rotor_z = 0.9525f / 100;
#elif defined(MATRICE_100)
        /* TODO: recalculate thrust coefficent to account for propeller efficency */
        params.rotor_count = 4;
        params.mass = 2.431f;
        real_T motor_assembly_weight = 0.106f;
        real_T motor_arm_length = .325f;
        params.rotor_params.C_P = .033526f; // the torque co-efficient
        params.rotor_params.C_T = .027122f; // the thrust co-efficient
        params.rotor_params.max_rpm = 8750; // RPM can be obtained by KV * Voltage (or from docs)
        params.rotor_params.propeller_diameter = 0.3302f; //in meters

        params.body_box.x() = .252f; params.body_box.y() = .190f; params.body_box.z() = .131f;
        real_T rotor_z = .0569f;
#endif

        //set up arm lengths
        //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
        std::vector<real_T> arm_lengths(params.rotor_count, motor_arm_length);

        //set up mass
        real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

        // using rotor_param default, but if you want to change any of the rotor_params, call calculateMaxThrust() to recompute the max_thrust
        // given new thrust coefficients, motor max_rpm and propeller diameter.
        params.rotor_params.calculateMaxThrust();

        //computer rotor poses
        initializeRotorQuadX(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);

        //compute inertia matrix
        computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);

        //leave everything else to defaults
    }

    virtual std::unique_ptr<SensorBase> createSensor(SensorBase::SensorType sensor_type) override
    {
        return sensor_factory_->createSensor(sensor_type);
    }

    virtual std::unique_ptr<DroneControllerBase> createController() override
    {
        return std::unique_ptr<DroneControllerBase>(new SimpleFlightDroneController(this, vehicle_settings_));
    }


private:
    vector<unique_ptr<SensorBase>> sensor_storage_;
    AirSimSettings::VehicleSettings vehicle_settings_;
    std::shared_ptr<const SensorFactory> sensor_factory_;
};

}} //namespace
#endif