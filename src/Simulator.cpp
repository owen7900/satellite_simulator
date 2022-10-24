#include "simulator/Simulator.hpp"
#include "simulator/ConfigurationSingleton.hpp"
#include "simulator/SensorActuatorFactory.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <rclcpp/logging.hpp>
#include <thread>

using namespace std::chrono_literals;
Simulator::Simulator(const std::string &configFile) : Node("simulator") {

  auto &config = Configuration::GetInstance();
  if (!config.Load(configFile)) {
    RCLCPP_ERROR(get_logger(), "Configuration Failed to load");
  }

  for (const auto &accel : config.GetSensorConfigs()) {
    create_sensor(accel.first);
  }

  for (const auto &reac : config.GetActuatorConfigs()) {
    create_actuator(reac.first);
  }

  timestep = config.GetTimeStep();

  timePub = this->create_publisher<std_msgs::msg::Float64>("time", 10);

  this->loopTimer = this->create_wall_timer(
      std::chrono::duration<double>(config.GetTickPeriod()),
      std::bind(&Simulator::timer_callback, this));
}

void Simulator::create_sensor(const std::string &name) {
  if (name.empty()) {
    RCLCPP_ERROR_STREAM(get_logger(),
                        "Device name must be populated. Got " << name);
    return;
  }
  auto sensorPtr = SensorActuatorFactory::GetSensor(name);
  if (!sensorPtr) {
    RCLCPP_ERROR_STREAM(get_logger(), "Unknown sensor type: " << name);
    return;
  }

  sensors[name] = std::move(sensorPtr);
  dataPubs[name] =
      this->create_publisher<geometry_msgs::msg::Vector3>(name, 10);
}

void Simulator::create_actuator(const std::string &name) {
  if (name.empty()) {
    RCLCPP_ERROR_STREAM(get_logger(),
                        "Device name must be populated. Got " << name);
    return;
  }
  auto actPtr = SensorActuatorFactory::GetActuator(name);
  if (!actPtr) {
    RCLCPP_ERROR_STREAM(get_logger(), "Unknown actuator type: " << name);
    return;
  }

  actuators[name] = std::move(actPtr);
  commandSubs[name] = this->create_subscription<std_msgs::msg::Float64>(
      name, 10, [this, name](std_msgs::msg::Float64 data) {
        this->actuators[name]->SetCommandValue(data.data);
      });
}

void Simulator::timer_callback() {
  std::vector<std::thread> threads;

  for (auto &sens : sensors) {
    threads.emplace_back(&Sensor::Tick,
                         static_cast<ADSCDevice *>(sens.second.get()), timestep,
                         state);
  }

  for (auto &act : actuators) {
    threads.emplace_back(&Actuator::Tick,
                         static_cast<ADSCDevice *>(act.second.get()), timestep,
                         state);
  }

  for (auto &t : threads) {
    t.join();
  }

  state.rpyAccel = state.rpyAccel.Zero();
  for (const auto &act : actuators) {
    state += act.second->GetChangeInState();
  }

  for (const auto &sens : sensors) {
    geometry_msgs::msg::Vector3 m;
    const auto data = sens.second->GetValue();
    m.x = data.x();
    m.y = data.y();
    m.z = data.z();
    dataPubs[sens.first]->publish(m);
  }

  propogate_physics();
  if (Configuration::GetInstance().IsPrintStats()) {
    print_stats();
  }
  currentTime += timestep;
  std_msgs::msg::Float64 m;
  m.data = currentTime;
  timePub->publish(m);
}

void Simulator::propogate_physics() {
  auto &config = Configuration::GetInstance();
  state.rpyAccel += (config.GetInvMomentOfInertia() * state.rpyVel)
                        .cross(config.GetMomentOfIntertia() * state.rpyVel);

  state.rpy += (state.rpyVel * timestep);
  state.rpyVel += (state.rpyAccel * timestep);
}

void Simulator::print_stats() {
  RCLCPP_INFO_STREAM(get_logger(), "NAME: \t\tTIME");
  for (const auto &act : actuators) {
    RCLCPP_INFO_STREAM(get_logger(), act.first
                                         << ": \t\t"
                                         << act.second->GetLastTickDuration());
  }

  for (const auto &sens : sensors) {
    RCLCPP_INFO_STREAM(get_logger(), sens.first
                                         << ": \t\t"
                                         << sens.second->GetLastTickDuration());
  }
}
