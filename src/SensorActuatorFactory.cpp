#include "simulator/SensorActuatorFactory.hpp"

#include "simulator/Accelerometer.hpp"
#include "simulator/ConfigurationSingleton.hpp"
#include "simulator/Gyroscope.hpp"
#include "simulator/ReactionWheel.hpp"

std::unique_ptr<Sensor>
SensorActuatorFactory::GetSensor(const std::string &name) {
  auto &config = Configuration::GetInstance();
  const auto type = config.GetSensorConfig(name)->type;
  switch (type) {
  case SensorType::Accelerometer:
    return std::make_unique<Accelerometer>(name);
  case SensorType::Gyroscope:
    return std::make_unique<Gyroscope>(name);
  }
  return {};
}

std::unique_ptr<Actuator>
SensorActuatorFactory::GetActuator(const std::string &name) {
  auto &config = Configuration::GetInstance();
  const auto type = config.GetActuatorConfig(name)->type;
  switch (type) {
  case ActuatorType::ReactionWheel:
    return std::make_unique<ReactionWheel>(name);
  }
  return {};
}
