#include <cstdio>

#include "simulator/Simulator.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simulator>("simulator.yaml"));
  rclcpp::shutdown();
  return 0;
}
