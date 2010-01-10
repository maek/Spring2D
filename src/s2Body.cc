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

    // Update the acceleration
    acceleration_ = netForce_  * (1.0 / mass_);
    velocityFromAcceleration_ = acceleration_ * TIME_STEP;

    // Update the velocity
    velocity_ += velocityFromAcceleration_;

    // Update the rotation
    rotation_ += netTorque_ * (1.0 / momentOfInertia_) * TIME_STEP;

    // Update the position
    position_ += (velocity_ * TIME_STEP);

    // Update the orientation
    orientation_.rotate(rotation_ * TIME_STEP);


    // Update the AABB
    shape_->updateAABB();

    // Normalize the orientation
    orientation_.normalize();

    // Clear the accumulators
    netForce_   = Vector2::ZERO;
    netTorque_  = 0;
  }


}
