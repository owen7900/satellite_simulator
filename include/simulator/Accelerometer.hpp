#ifndef ACCELEROMETER_HPP_
#define ACCELEROMETER_HPP_

#include "simulator/CommonStructs.hpp"
#include "simulator/Sensor.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>

class Accelerometer : public Sensor {
public:
  Accelerometer(const std::string &name_) : Sensor(name_){};
  Eigen::Vector3d GetValue() override { return value; }

private:
  void tick(double /* timestep */, const SatelliteState &state) override {
    value = state.rpyAccel;
  };

private:
  Eigen::Vector3d value;
};

#endif
