#include "../include/s2Engine.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Run a complete physics step
  void Engine::runStep() const
  {
    // Compute the net forces
    forceRegister_->computeForces();

    // Compute the new positions, velocities and accelerations
    integrator_->compute(environment_);
  }
}
