#ifndef SIMULATOR_HPP_
#define SIMULATOR_HPP_

#include "Actuator.hpp"
#include "Sensor.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>

class Simulator : public rclcpp::Node {
public:
  Simulator(const std::string &configFile);

private:
  void timer_callback();
  void propogate_physics();
  void print_stats();
  void create_actuator(const std::string &name);
  void create_sensor(const std::string &name);

private:
  std::unordered_map<std::string, std::unique_ptr<Sensor>> sensors;
  std::unordered_map<std::string, std::unique_ptr<Actuator>> actuators;
  std::unordered_map<std::string,
                     rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr>
      commandSubs;
  std::unordered_map<std::string,
                     rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr>
      dataPubs;
  double timestep;
  SatelliteState state;
  double currentTime;
  rclcpp::TimerBase::SharedPtr loopTimer;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr timePub;
};

#endif
