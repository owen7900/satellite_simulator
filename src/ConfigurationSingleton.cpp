
#include "simulator/ConfigurationSingleton.hpp"
#include <iostream>

GyroConfig::GyroConfig(const YAML::Node & /* node */)
    : BaseSensorConfig(SensorType::Gyroscope) {}

AccelerometerConfig::AccelerometerConfig(const YAML::Node & /* node */)
    : BaseSensorConfig(SensorType::Accelerometer) {}

ReactionWheelConfig::ReactionWheelConfig(const YAML::Node &node)
    : BaseActuatorConfig(ActuatorType::ReactionWheel) {
  int i = 0, j = 0;
  for (const auto &n : node["Moment"]) {
    j = 0;
    for (const auto &a : n) {
      momentOfInertia(i, j++) = a.as<double>();
    }
    ++i;
  }

  i = 0;
  for (const auto &n : node["UnitVector"]) {
    unitVector(i++) = n.as<double>();
  }
  unitVector.normalize();

  momentOfInertiaInv = momentOfInertia.inverse();
  maxAngVel = node["MaxAngVel"].as<double>();
  maxAngAccel = node["MaxAngAccel"].as<double>();
}

bool Configuration::Load(const std::string &configFile) {
  try {
    top = YAML::LoadFile(configFile);
  } catch (YAML::Exception &e) {
    std::cout << "YAML File Load failure for file: " << configFile << " : "
              << e.what() << std::endl;
    return false;
  }
  try {
    YAML::Node sensors = top["Sensors"];
    for (const auto &n : sensors) {
      const std::string type = n.second["type"].as<std::string>();
      if (type == "Gyroscope") {
        sensorConfigs[n.first.as<std::string>()] =
            std::make_shared<GyroConfig>(n.second);
      } else if (type == "Accelerometer") {
        sensorConfigs[n.first.as<std::string>()] =
            std::make_shared<AccelerometerConfig>(n.second);
      } else {
        std::cout << "Error unknown sensor type: " << type << std::endl;
      }
    }
  } catch (YAML::Exception &e) {
    std::cout << "YAML ERROR ON ACTUATORS: " << e.what() << std::endl;
  }

  try {
    YAML::Node actuatorsYaml = top["Actuators"];

    for (const auto &n : actuatorsYaml) {
      const std::string type = n.second["type"].as<std::string>();
      if (type == "ReactionWheel") {
        actuatorConfigs[n.first.as<std::string>()] =
            std::make_shared<ReactionWheelConfig>(n.second);
      } else {
        std::cout << "Unkown actuator type: " << type << std::endl;
      }
    }
  } catch (YAML::Exception &e) {
    std::cout << "YAML ERROR ON ACTUATORS: " << e.what() << std::endl;
  }

  try {
    timeStep = top["TimeStep"].as<double>();
  } catch (YAML::Exception &e) {
    std::cout << "TimeStep not found" << std::endl;
    return false;
  }

  try {
    tickPeriod = top["TickPeriod"].as<double>();
  } catch (YAML::Exception &e) {
    std::cout << "TickPeriod not found" << std::endl;
    return false;
  }

  try {
    printStats = top["PrintStats"].as<bool>();
  } catch (YAML::Exception &e) {
    std::cout << "PrintStats not found" << std::endl;
    return false;
  }

  try {
    int i = 0, j = 0;
    for (const auto &n : top["Moment"]) {
      j = 0;
      for (const auto &a : n) {
        momentOfInertia(i, j++) = a.as<double>();
      }
      ++i;
    }
  } catch (YAML::Exception &e) {
    std::cout << "Moment not found " << e.what() << std::endl;
  }
  momentOfInertiaInv = momentOfInertia.inverse();

  return true;
}
