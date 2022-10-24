
#ifndef ADCSDEVICE_HPP
#define ADCSDEVICE_HPP

#include "CommonStructs.hpp"

#include <chrono>
#include <string>
#include <yaml-cpp/yaml.h>

class ADSCDevice {
public:
  ADSCDevice(const std::string &name_) : name(name_){};
  virtual ~ADSCDevice(){};
  inline void Tick(double timestep, const SatelliteState &state) {
    const auto startTime = std::chrono::steady_clock::now();
    tick(timestep, state);
    const auto endTime = std::chrono::steady_clock::now();
    lastTickDuration =
        std::chrono::duration<double>(endTime - startTime).count();
  };
  std::string GetName() { return name; };
  double GetLastTickDuration() { return lastTickDuration; };

private:
  virtual void tick(double timestep, const SatelliteState &state) = 0;

private:
  std::string name;
  double lastTickDuration;
};

#endif
