#ifndef CONFIGURATIONSINGLETON_HPP_
#define CONFIGURATIONSINGLETON_HPP_

#include "simulator/CommonStructs.hpp"
#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

struct BaseSensorConfig {
  BaseSensorConfig(SensorType t) : type(t){};
  SensorType type;
};

struct BaseActuatorConfig {
  BaseActuatorConfig(ActuatorType t) : type(t){};
  ActuatorType type;
};

struct GyroConfig : public BaseSensorConfig {
  GyroConfig(const YAML::Node &node);
};

struct AccelerometerConfig : public BaseSensorConfig {
  AccelerometerConfig(const YAML::Node &node);
};

struct ReactionWheelConfig : public BaseActuatorConfig {
  ReactionWheelConfig(const YAML::Node &node);

  Eigen::Matrix3d momentOfInertia;
  Eigen::Matrix3d momentOfInertiaInv;
  Eigen::Vector3d unitVector;
  double maxAngVel;
  double maxAngAccel;
};

class Configuration {
public:
  bool Load(const std::string &fileName);

  Configuration(const Configuration &) = delete;
  void operator=(Configuration &) = delete;
  static Configuration &GetInstance() {
    static Configuration instance;
    return instance;
  };

  inline const std::shared_ptr<BaseSensorConfig> &
  GetSensorConfig(const std::string &name) {
    return sensorConfigs[name];
  }

  inline const std::shared_ptr<BaseActuatorConfig> &
  GetActuatorConfig(const std::string &name) {
    return actuatorConfigs[name];
  }

  inline const std::unordered_map<std::string,
                                  std::shared_ptr<BaseSensorConfig>> &
  GetSensorConfigs() {
    return sensorConfigs;
  };

  inline const std::unordered_map<std::string,
                                  std::shared_ptr<BaseActuatorConfig>> &
  GetActuatorConfigs() {
    return actuatorConfigs;
  };
  inline double GetTimeStep() { return timeStep; };
  inline double GetTickPeriod() { return tickPeriod; };

  inline bool IsPrintStats() { return printStats; };

  inline const Eigen::Matrix3d &GetMomentOfIntertia() const {
    return momentOfInertia;
  };
  inline const Eigen::Matrix3d &GetInvMomentOfInertia() const {
    return momentOfInertiaInv;
  };

private:
  Configuration(){};

private:
  YAML::Node top;
  std::unordered_map<std::string, std::shared_ptr<BaseSensorConfig>>
      sensorConfigs;
  std::unordered_map<std::string, std::shared_ptr<BaseActuatorConfig>>
      actuatorConfigs;

  double timeStep;
  double tickPeriod;
  bool printStats;
  Eigen::Matrix3d momentOfInertia;
  Eigen::Matrix3d momentOfInertiaInv;
};

#endif
