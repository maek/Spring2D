#include "../include/s2Engine.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Run a complete physics step
  void Engine::runStep()
  {
    // Compute the net forces
    environment_->computeForces();
    // Compute the net torques
    environment_->computeTorques();

    // Update positions & velocities
    environment_->integrateBodies();


    // Find collision (broad phase)
    collisionDetector_.findCollisions(environment_->getBodyList());

    // Increment the step counter
    stepCounter_++;
  }
}
