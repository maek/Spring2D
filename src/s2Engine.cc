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


    // Find violated constraints
    collisionDetector_.checkConstraints(environment_->getConstraints());

    // Solve violated constraints
    collisionSolver_.solveCollisions(
        collisionDetector_.getConstraintsContacts(), true);

    // Find collisions
    collisionDetector_.findCollisions(environment_->getBodyList());

    // Solve collisions
    collisionSolver_.solveCollisions(collisionDetector_.getContacts());


    // Increment the step counter
    stepCounter_++;
  }
}
