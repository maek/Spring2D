#include "../include/s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Compute the new position & update velocity
  // (EULER)
  void Body::integrate (const Real& TIME_STEP)
  {
    if (static_)
      return;

    // (1) Update the position
    position_ += (velocity_ * TIME_STEP);

    // (2) Update the velocity
    velocity_ += netForce_ * (1.0 / mass_) * TIME_STEP;


    // Clear the net force
    netForce_ = Vector2::ZERO;

  }


}
