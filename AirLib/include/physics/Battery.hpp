#ifndef __BATTERY_HPP
#define __BATTERY_HPP

#include "common/Common.hpp"
#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include "common/CommonStructs.hpp"
#include "common/SteppableClock.hpp"
#include <cinttypes>

namespace msr { namespace airlib {

namespace powerlib {

#define cap2coulomb(c)  ((c) * 3600.0f)

class BatteryFuelGauge {
 public:
  BatteryFuelGauge(): coulombs_(0.0f) {}

  void update(TTimeDelta dt, float vol, float power) {
    coulombs_ += (power / vol) * dt;
  }

  float Coulombs() const {
    return coulombs_;
  }

 private:
  float coulombs_;
};

class PowerModel {
 public:

};

class Battery {
 public:
  Battery(float voltage, float capacity):
    voltage_(voltage), nominal_voltage_(voltage),
    capacity_(capacity), state_of_charge_(100.0f) {
    voltage_ = RealtimeVolatge();
  }

  float Voltage() const { return voltage_; }

  float NominalVoltage() const { return nominal_voltage_; }

  float Capacity() const { return capacity_; }

  float StateOfCharge() const { return state_of_charge_; }

  void update(TTimeDelta dt, float power) {
    fuel_gauge_.update(dt, voltage_, power);
    auto total_coulombs = cap2coulomb(capacity_);
    state_of_charge_ = 1.0f -
                       fuel_gauge_.Coulombs() / total_coulombs;
    state_of_charge_ *= 100.0f;
    if (state_of_charge_ < 0) {
      state_of_charge_ = -1.0;
    }
    voltage_ = RealtimeVolatge();
  }

  void reset() {
    state_of_charge_ = 100.0f;
    voltage_ = RealtimeVolatge();
  }

 protected:
  virtual float RealtimeVolatge() {
    // TODO(wcui) Now we simulate a simple constant voltage battery, but in real
    // world, it could be a function of state_of_charge_
    return nominal_voltage_;
  }

 private:
  float voltage_;
  float nominal_voltage_;
  float capacity_;
  float state_of_charge_;
  BatteryFuelGauge fuel_gauge_;
};

}  // namespace powerlib

}  }  // namespace msr::airlib

#endif
