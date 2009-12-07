#include "../include/s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Compute the new position & update velocity
  // (EULER)
  void Body::integrate (const Real TIME_STEP)
  {
    if (static_)
      return;

    // (1a) Update the velocity
    velocity_ += netForce_  * (1.0 / mass_) * TIME_STEP;

    // (1b) Update the rotation
    rotation_ += netTorque_ * (1.0 / momentOfInertia_) * TIME_STEP;

    // (2a) Update the position
    position_ += (velocity_ * TIME_STEP);

    // (2b) Update the orientation
    orientation_.rotate(rotation_ * TIME_STEP);


    // Normalize the orientation
    orientation_.normalize();

    // Clear the accumulators
    netForce_   = Vector2::ZERO;
    netTorque_  = 0;

  }


}
