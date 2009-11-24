#include "../include/s2Engine.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Run a complete physics step
  void Engine::runStep()
  {
    // Compute the net forces
    forceRegister_->computeForces();

    // Update positions, velocities and accelerations
    environment_->integrateAllBody();


    // Increment the step counter
    stepCounter_++;
  }
}
