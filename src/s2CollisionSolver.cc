#include "../include/s2CollisionSolver.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Solve collisions
  void CollisionSolver::solveCollisions (ContactSet* contacts)
  {
    // Solve interpenetration
    interpenetrationSolver_.solveInterpenetration(contacts);

    // Solve velocities
    velocitySolver_.solveVelocity(contacts);
  }


}
