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

#ifndef __CIRCLE_SHAPE_H__
#define __CIRCLE_SHAPE_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2Shape.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The circle shape
  class CircleShape : public Shape
  {
    public:

      // Constructor
      CircleShape (const Real RADIUS, const Real DENSITY = 1)
        : radius_(RADIUS)
      {
        if (RADIUS <= 0)
        {
          valid_ = false;
          return;
        }

        if (DENSITY <= 0)
        {
          valid_ = false;
          return;
        }

        area_     = M_PI * RADIUS * RADIUS;
        density_  = DENSITY;
        iMass_    = 1. / (area_ * density_);
        updateInverseMomentOfInertia();

        valid_ = true;
      }


      // Get the radius
      Real getRadius () const
      {
        return radius_;
      }


      // Get the shape type
      ShapeType getType () const
      {
        return Shape::CIRCLE;
      }


      // Update the associated AABR
      void updateAABR ()
      {
        Vector2 extent(radius_, radius_);
        aabr_.min = body_->getPosition() - extent;
        aabr_.max = body_->getPosition() + extent;
      }


      // Calculate the moment of inertia
      // m * r^2 / 2
      void updateInverseMomentOfInertia ()
      {
        iMomentOfInertia_ = 2. /
          (area_ * density_ * radius_ * radius_);
      }


      // Return the center of the circle (see GJK optimization)
      Vector2 getSupportPoint0 () const
      {
        return body_->getPosition();
      }

      // Return the center of the circle (see GJK optimization)
      Vector2 getSupportPoint (const Vector2& DIRECTION) const
      {
        return body_->getPosition();
      }


    private:

      Real  radius_;

  };


}


#endif // __CIRCLE_SHAPE_H__
