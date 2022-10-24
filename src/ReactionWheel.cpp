#include "simulator/ReactionWheel.hpp"
#include "simulator/ConfigurationSingleton.hpp"

void ReactionWheel::tick(double timestep, const SatelliteState &state) {
  deltaState.rpy = deltaState.rpy.Zero();
  deltaState.rpyVel = deltaState.rpyVel.Zero();
  auto config = std::static_pointer_cast<ReactionWheelConfig>(
      Configuration::GetInstance().GetActuatorConfig(this->GetName()));

  double angAccel = std::min((command - angVel) / timestep, config->maxAngVel);
  angVel += angAccel * timestep;

  const auto angAccelVec = angAccel * config->unitVector;
  const auto angVelVec = angVel * config->unitVector;

  deltaState.rpyAccel =
      -Configuration::GetInstance().GetInvMomentOfInertia() *
      (config->momentOfInertia * angAccelVec +
       state.rpyVel.cross(config->momentOfInertia * angVelVec));
}
