#ifndef COMMONSTRUCTS_HPP_
#define COMMONSTRUCTS_HPP_

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

enum class SensorType { Accelerometer, Gyroscope };

enum class ActuatorType { ReactionWheel };

struct SatelliteState {
  Eigen::Vector3d rpy;      // rad
  Eigen::Vector3d rpyVel;   // rad/s
  Eigen::Vector3d rpyAccel; // rad/s^s

  SatelliteState &operator+=(const SatelliteState &s) {
    rpy += s.rpy;
    rpyVel += s.rpyVel;
    rpyAccel += rpyAccel;
    return *this;
  }
};

#endif
