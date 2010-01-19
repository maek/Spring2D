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

    // Calculate the velocity from acceleration
    velocityFromAcceleration_ =
      (acceleration_ + netForce_ * shape_->iMass_) * TIME_STEP;

    // Update the velocity
    velocity_ += velocityFromAcceleration_;

    // Update the position
    position_ += (velocity_ * TIME_STEP);


    // Update the rotation
    rotation_ += netTorque_ * shape_->iMomentOfInertia_ * TIME_STEP;

    // Update the orientation
    orientation_.rotate(rotation_ * TIME_STEP);


    // Update the AABB
    shape_->updateAABB();

    // Normalize the orientation
    // TODO: only 1 time every second
    orientation_.normalize();

    // Clear the accumulators
    netForce_   = Vector2::ZERO;
    netTorque_  = 0;
  }


}
