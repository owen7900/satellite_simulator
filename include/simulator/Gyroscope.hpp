#ifndef GYROSCOPE_HPP_
#define GYROSCOPE_HPP_

#include "simulator/CommonStructs.hpp"
#include "simulator/Sensor.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>

class Gyroscope : public Sensor {
public:
  Gyroscope(const std::string &name_) : Sensor(name_){};
  Eigen::Vector3d GetValue() override { return value; }

private:
  void tick(double /* timestep */, const SatelliteState &state) override {
    value = state.rpy;
  };

private:
  Eigen::Vector3d value;
};

#endif
