#include "../include/s2Engine.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Run a complete physics step
  void Engine::runStep()
  {
    // Compute the net forces & net torques
    environment_->handleDynamics();

    // Update positions & velocities
    environment_->integrateBodies();


    // Find collision
    collisionDetector_.findCollisions(environment_->getBodyList());

    // Solve collision
    collisionSolver_.solveCollisions(
        collisionDetector_.getFrontContacts(),
        collisionDetector_.getBackContacts());


    // Increment the step counter
    stepCounter_++;
  }
}
