#include "../include/s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Compute the new position & update velocity
  // (EULER)
  void Body::integrate ()
  {
    if (static_)
      return;

    // Calculate the velocity from acceleration
    velocityFromAcceleration_ =
      (acceleration_ + netForce_ * shape_->iMass_) * timestep_;

    // Update the position
    position_ += (velocity_ * timestep_);

    // Update the velocity
    velocity_ = iDrag_ * velocity_ + velocityFromAcceleration_;


    // Update the orientation
    orientation_.rotate(rotation_ * timestep_);

    // Update the rotation
    rotation_ = iDrag_ * rotation_ +
      netTorque_ * shape_->iMomentOfInertia_ * timestep_;


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
