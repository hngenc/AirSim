// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_CommonStructs_hpp
#define msr_airlib_CommonStructs_hpp

#include "common/Common.hpp"
#include <ostream>



namespace msr { namespace airlib {

class EnergyRotorSpecs{
    public:
        
        EnergyRotorSpecs() 
        {}
        /* 
        EnergyRotorSpecs(float mass, float mass_coeff, float vxy_coeff, 
                float axy_coeff, float vxy_axy_coeffs,
                float vz_coeff, float az_coeff, float vz_az_coeff, float one_coeff, ){
            mass_ = mass; 
            mass_coeff_ = mass_coeff; 
            one_coeff_ = one_coeff; 
            vxy_wxy_coeff_ = vyx_wxy_coeff; 
            vxy_coeff_ = vxy_coeff; 
            axy_coeff_ = axy_coeff; 
            vxy_axy_coeffs_ = vxy_axy_coeffs;
            vz_coeff_ = vz_coeff; 
            az_coeff_ = az_coeff;
            vz_az_coeff_ = vz_az_coeff;
        }
        */
        float get_mass(){
            return mass_;
        }
       
        float get_mass_coeff(){
            return mass_;
        }
        
        float get_one_coeff(){
            return one_coeff_;
        }
        
        float get_vxy_wxy_coeff(){
            return vxy_wxy_coeff_;
        }

        float get_vxy_coeff(){
            return vxy_coeff_; 
        }
        
        float get_vxy_axy_coeff(){
            return vxy_coeff_; 
        }

        float get_vz_coeff(){
            return vz_coeff_; 
        }

        float get_axy_coeff(){
            return axy_coeff_; 
        }

        float get_az_coeff(){
            return az_coeff_; 
        }
        float get_vz_az_coeff(){
            return vz_az_coeff_; 
        }
    
        void set_mass(float mass){
            mass_ = mass;
        }
       
        void set_mass_coeff(float mass_coeff){
            mass_coeff_ = mass_coeff;
        }
        
        void set_one_coeff(float one_coeff){
            one_coeff_ = one_coeff;
        }
        
        void set_vxy_wxy_coeff(float vxy_wxy_coeff){
            vxy_wxy_coeff_ = vxy_wxy_coeff;
        }

        void set_vxy_coeff(float vxy_coeff){
            vxy_coeff_ = vxy_coeff; 
        }
        
        void set_vxy_axy_coeff(float vxy_axy_coeff){
            vxy_axy_coeff_ = vxy_axy_coeff; 
        }

        void set_vz_coeff(float vz_coeff){
            vz_coeff_ = vz_coeff; 
        }

        void set_axy_coeff(float axy_coeff){
            axy_coeff_ = axy_coeff; 
        }

        void set_az_coeff(float az_coeff){
            az_coeff_ = az_coeff; 
        }
        void set_vz_az_coeff(float vz_az_coeff){
            vz_az_coeff_ = vz_az_coeff; 
        }

    private:
        float mass_, 
         mass_coeff_,
         vxy_coeff_, 
         axy_coeff_, 
         vxy_axy_coeff_,
         vz_coeff_, 
         az_coeff_, 
         vz_az_coeff_,
         one_coeff_,
         vxy_wxy_coeff_;
};


//velocity
struct Twist {
    Vector3r linear, angular;

    Twist()
    {}

    Twist(const Vector3r& linear_val, const Vector3r& angular_val)
        : linear(linear_val), angular(angular_val)
    {
    }

    static const Twist zero()
    {
        static const Twist zero_twist(Vector3r::Zero(), Vector3r::Zero());
        return zero_twist;
    }
};

//force & torque
struct Wrench {
    Vector3r force, torque;

    Wrench()
    {}

    Wrench(const Vector3r& force_val, const Vector3r& torque_val)
        : force(force_val), torque(torque_val)
    {
    }

    //support basic arithmatic
    Wrench operator+(const Wrench& other) const
    {
        Wrench result;
        result.force = this->force + other.force;
        result.torque = this->torque + other.torque;
        return result;
    }
    Wrench operator+=(const Wrench& other)
    {
        force += other.force;
        torque += other.torque;
        return *this;
    }
    Wrench operator-(const Wrench& other) const
    {
        Wrench result;
        result.force = this->force - other.force;
        result.torque = this->torque - other.torque;
        return result;
    }
    Wrench operator-=(const Wrench& other)
    {
        force -= other.force;
        torque -= other.torque;
        return *this;
    }

    static const Wrench zero()
    {
        static const Wrench zero_wrench(Vector3r::Zero(), Vector3r::Zero());
        return zero_wrench;
    }
};

struct Momentums {
    Vector3r linear;
    Vector3r angular;

    Momentums()
    {}

    Momentums(const Vector3r& linear_val, const Vector3r& angular_val)
        : linear(linear_val), angular(angular_val)
    {
    }

    static const Momentums zero()
    {
        static const Momentums zero_val(Vector3r::Zero(), Vector3r::Zero());
        return zero_val;
    }
};

struct Accelerations {
    Vector3r linear;
    Vector3r angular;

    Accelerations()
    {}

    Accelerations(const Vector3r& linear_val, const Vector3r& angular_val)
        : linear(linear_val), angular(angular_val)
    {
    }

    static const Accelerations zero()
    {
        static const Accelerations zero_val(Vector3r::Zero(), Vector3r::Zero());
        return zero_val;
    }
};

struct PoseWithCovariance {
    VectorMath::Pose pose;
    vector<real_T> covariance;	//36 elements, 6x6 matrix

    PoseWithCovariance()
        : covariance(36, 0)
    {}
};

struct PowerSupply {
    vector<real_T> voltage, current;
};

struct TwistWithCovariance {
    Twist twist;
    vector<real_T> covariance;	//36 elements, 6x6 matrix

    TwistWithCovariance()
        : covariance(36, 0)
    {}
};

struct Joystick {
    vector<float> axes;
    vector<int> buttons;
};

struct Odometry {
    PoseWithCovariance pose;
    TwistWithCovariance twist;
};

struct GeoPoint {
    double latitude = 0, longitude = 0;
    float altitude = 0;

    GeoPoint()
    {}

    GeoPoint(double latitude_val, double longitude_val, float altitude_val)
    {
        set(latitude_val, longitude_val, altitude_val);
    }

    void set(double latitude_val, double longitude_val, float altitude_val)
    {
        latitude = latitude_val, longitude = longitude_val; altitude = altitude_val;
    }

    friend std::ostream& operator<<(std::ostream &os, GeoPoint const &g) {
        return os << "[" << g.latitude << ", " << g.longitude << ", " << g.altitude << "]";
    }

    std::string to_string()
    {
        return std::to_string(latitude) + string(", ") + std::to_string(longitude) + string(", ") + std::to_string(altitude);
    }
};

struct CollisionInfo {
    bool has_collided = false;
    Vector3r normal = Vector3r::Zero();
    Vector3r impact_point = Vector3r::Zero();
    Vector3r position = Vector3r::Zero();
    real_T penetration_depth = 0;
    TTimePoint time_stamp = 0;
    unsigned int collision_count = 0;
    std::string object_name;
    int object_id = -1;
    CollisionInfo()
    {}

    CollisionInfo(bool has_collided_val, const Vector3r& normal_val, 
        const Vector3r& impact_point_val, const Vector3r& position_val, 
        real_T penetration_depth_val, TTimePoint time_stamp_val,
        const std::string& object_name_val, int object_id_val)
        : has_collided(has_collided_val), normal(normal_val),
        impact_point(impact_point_val), position(position_val),
        penetration_depth(penetration_depth_val), time_stamp(time_stamp_val),
        object_name(object_name_val), object_id(object_id_val)
    {
    }
};

struct CollisionResponseInfo {
    unsigned int collision_count_raw = 0;
    unsigned int collision_count_non_resting = 0;
    TTimePoint collision_time_stamp = 0;
};

struct GeoPose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Quaternionr orientation;
    GeoPoint position;
};

// TODO(wcui): Change this class if you want to provide more information to
// RPC Client
struct FlightStats{
    float state_of_charge;
    float voltage;
    float energy_consumed;
    float flight_time;
    float distance_traveled;
    int collision_count;  
    
    FlightStats()
    {}
    
    FlightStats(float state_of_charge_val,
                float voltage_val,
                float energy_consumed_val,
                float flight_time_val,
                float distance_traveled_val,
                int collision_count_val):
        state_of_charge(state_of_charge_val), 
        voltage(voltage_val), 
        energy_consumed(energy_consumed_val), 
        flight_time(flight_time_val), 
        distance_traveled(distance_traveled_val),
        collision_count(collision_count_val)
    {
    }

};


struct IMUStats{
    Quaternionr orientation;
    Vector3r angular_velocity;
    Vector3r linear_acceleration;
    uint64_t time_stamp;

    
    /* 
    float state_of_charge;
    float voltage;
    float energy_consumed;
    float flight_time;
    float distance_traveled;
    int collision_count;  
    */
    IMUStats()
    {}
    
    IMUStats(Quaternionr orientation_val, Vector3r angular_velocity_val, 
            Vector3r linear_acceleration_val, uint64_t time_stamp_val): orientation(orientation_val),
                                      angular_velocity(angular_velocity_val),
                                      linear_acceleration(linear_acceleration_val),
                                      time_stamp(time_stamp_val)
    {}
    /*
    FlightStats(float state_of_charge_val,
                float voltage_val,
                float energy_consumed_val,
                float flight_time_val,
                float distance_traveled_val,
                int collision_count_val):
        state_of_charge(state_of_charge_val), 
        voltage(voltage_val), 
        energy_consumed(energy_consumed_val), 
        flight_time(flight_time_val), 
        distance_traveled(distance_traveled_val),
        collision_count(collision_count_val)
    {
    }
    */
};

struct GPSStats {
	double latitude;
	double longitude;
	double altitude;
	uint64_t time_stamp;

	GPSStats()
	{}

	GPSStats(double latitude_val, double longitude_val,
		double altitude_val, uint64_t time_stamp_val) : latitude(latitude_val),
		longitude(longitude_val),
		altitude(altitude_val),
		time_stamp(time_stamp_val)
	{}
};


}} //namespace
#endif

