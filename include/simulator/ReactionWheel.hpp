#ifndef REACTIONWHEEL_HPP_
#define REACTIONWHEEL_HPP_

#include "simulator/Actuator.hpp"

class ReactionWheel : public Actuator {
public:
  ReactionWheel(const std::string &name) : Actuator(name){};
  SatelliteState GetChangeInState() const override { return deltaState; }
  void SetCommandValue(double val) override { command = val; };

private:
  void tick(double timestep, const SatelliteState &state) override;

private:
  double command;
  SatelliteState deltaState;
  double angVel;
};

#endif
