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

#ifndef __RECT_SHAPE_H__
#define __RECT_SHAPE_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2Shape.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The rectangle shape
  class RectShape : public Shape
  {
    public:

      // Constructor
      RectShape (const Vector2& EXTENT, const Real DENSITY = 1)
        : extent_(EXTENT)
      {
        if (EXTENT.x <= 0 || EXTENT.y <= 0)
        {
          valid_ = false;
          return;
        }

        if (DENSITY <= 0)
        {
          valid_ = false;
          return;
        }

        area_     = 4 * EXTENT.x * EXTENT.y;
        density_  = DENSITY;
        iMass_    = 1. / (area_ * density_);
        updateInverseMomentOfInertia();

        valid_ = true;
      }


      // Get the extent
      Vector2 getExtent () const
      {
        return extent_;
      }


      // Get the shape type
      ShapeType getType () const
      {
        return Shape::RECT;
      }


      void updateAABR ();


      // Calculate the moment of inertia
      // (m * (w^2 + h^2)) / 12
      void updateInverseMomentOfInertia ()
      {
        iMomentOfInertia_ = 3. /
          (area_ * density_ * (s2sqr(extent_.x) + s2sqr(extent_.y)));
      }


      // Return any vertex as the initial support point
      Vector2 getSupportPoint0 () const
      {
        return body_->getOrientationMatrix() * extent_ + body_->getPosition();
      }


      Vector2 getSupportPoint (const Vector2&) const;


    private:

      Vector2 extent_;

  };


}


#endif // __RECT_SHAPE_H__
