#ifndef SENSOR_HPP_
#define SENSOR_HPP_

#include "ADCSDevice.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <string>

class Sensor : public ADSCDevice {
public:
  Sensor(const std::string &name_) : ADSCDevice(name_){};
  virtual ~Sensor(){};
  virtual Eigen::Vector3d GetValue() = 0;
};

#endif
