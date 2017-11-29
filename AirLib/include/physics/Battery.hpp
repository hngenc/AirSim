#ifndef __BATTERY_HPP
#define __BATTERY_HPP

#include "Kinematics.hpp"
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

struct cvec2f {
  cvec2f(float f1, float f2): x(f1), y(f2) {}

  bool operator<(const cvec2f &rhs) const { return x < rhs.x; }

  float x, y;
};

class CurveEvaluator {
 public:
  CurveEvaluator() : size_(0) {}

  void setData(std::vector<float> &Xs, std::vector<float> &Ys) {
    data_.clear();
    size_ = std::min(Xs.size(), Ys.size());
    for (int i = 0; i < size_; i++) {
      cvec2f p(Xs[i], Ys[i]);
      data_.push_back(p);
    }

    std::sort(data_.begin(), data_.end());
  }

  virtual float eval(float x, bool extrapolate = true) {
    int i = 0;
    if (x > data_[size_ - 2].x) {
      i = size_ - 2;
    } else {
      while (x > data_[i + 1].x)
        i++;
    }
    float xL = data_[i].x, yL = data_[i].y,
          xR = data_[i + 1].x, yR = data_[i + 1].y;
    if (!extrapolate) {
      if (x < xL) yR = yL;
      if (x > xR) yL = yR;
    }

    float k = (yR - yL) / (xR - xL);
    return yL + k * (x - xL);
  }

 private:
  int size_;
  std::vector<cvec2f> data_;
};

class PowerEstimator {
 public:
  PowerEstimator(float acc_thresh = 1.0f, float deacc_thresh = -1.0f):
    acc_thresh_(acc_thresh), deacc_thresh_(deacc_thresh) {
    // make deaccelration threshold a negative number
    if (deacc_thresh_ > 0) deacc_thresh_ = -deacc_thresh_;

    std::vector<float> x1 = {0.0f, 4.0f, 8.0f, 11.0f, 12.5f, 13.0f, 13.1f};
    std::vector<float> y1 = {205.0f, 215.0f, 225.0f, 240.0f, 255.0f, 275.0f, 285.0f};
    accelerate_.setData(x1, y1);
    std::vector<float> x2 = {15.0f, 14.0f, 12.5f, 10.0f, 7.5f, 5.0f, 2.5f, 0.0f};
    std::vector<float> y2 = {305.0f, 230.0f, 210.0f, 200.0f, 200.0f, 210.0f, 220.0f, 215.0f};
    deaccelerate_.setData(x2, y2);
    std::vector<float> x3 = {0.0f, 2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f, 14.0f, 15.0f};
    std::vector<float> y3 = {220.0f, 220.0f, 215.0f, 210.0f, 207.0f, 215.0f, 232.0f, 280.0f, 330.0f};
    steady_.setData(x3, y3);
  }

  float estimate(const Kinematics::State& current,
                 const Kinematics::State& next) {
    auto v1 = current.twist.linear.norm(),
         v2 = next.twist.linear.norm();
    auto diff = v2 - v1;
    if (diff >= acc_thresh_) {
      return accelerate_.eval(v1);
    } else if (diff <= deacc_thresh_) {
      return deaccelerate_.eval(v1);
    } else {
      return steady_.eval(v1);
    }
  }

 private:
  float acc_thresh_, deacc_thresh_;
  CurveEvaluator accelerate_, deaccelerate_, steady_;
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
