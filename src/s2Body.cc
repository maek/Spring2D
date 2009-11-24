#include "../include/s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Compute the new position & update velocity and acceleration
  // (VELOCIY VERLET)
  void Body::integrate (const Real& TIME_STEP)
  {
    // (1) Update the position
    position_ += (velocity_ * TIME_STEP) +
      (0.5 * acceleration_ * TIME_STEP * TIME_STEP);

    // (2) Update the velocity I
    velocity_ += 0.5 * acceleration_ * TIME_STEP;

    // (3) Update the acceleration
    acceleration_ = netForce_ * (1.0 / mass_);

    // (4) Update the velocity II
    velocity_ += 0.5 * acceleration_ * TIME_STEP;


    // Clear the net force
    netForce_ = Vector::ZERO;

  }


}
