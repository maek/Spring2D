/*
 * Copyright (C) 2010   Marco Dalla Via (maek@paranoici.org)
 *
 *  This file is part of Spring2D.
 *
 *  Spring2D is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Spring2D is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Lesser Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser Public License
 *  along with Spring2D. If not, see <http://www.gnu.org/licenses/>.
 */

#include "../include/s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Compute the new position & update velocity
  // (EULER)
  void Body::integrate ()
  {
    if (static_ || sleeping_)
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


    // Calculate the current motion
    Real currentMotion =
      (velocity_ - velocityFromAcceleration_).getSquaredMagnitude() +
      (rotation_ * rotation_);

    motion_ = MOTION_BIAS * motion_ + (1 - MOTION_BIAS) * currentMotion;

    // Let the body sleep if its level of motion is under a certain threshold
    if (motion_ <= MOTION_THRESHOLD)
    {
      sleeping_ = true;
    }

    // Normalize the orientation
    // TODO: only 1 time every second
    orientation_.normalize();

    // Clear the accumulators
    netForce_   = Vector2::ZERO;
    netTorque_  = 0;
  }


}
