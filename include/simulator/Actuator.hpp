
#ifndef ACTUATOR_HPP_
#define ACTUATOR_HPP_

#include "ADCSDevice.hpp"
#include <string>

class Actuator : public ADSCDevice {
public:
  Actuator(const std::string &name_) : ADSCDevice(name_){};
  virtual ~Actuator(){};
  virtual SatelliteState GetChangeInState() const = 0;
  virtual void SetCommandValue(double val) = 0;
};

#endif
