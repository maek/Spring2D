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

#include "../include/s2RectShape.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Update the associated AABR
  void RectShape::updateAABR ()
  {
    Vector2 point0 = body_->transformWorld(-extent_);
    Vector2 point1 = body_->transformWorld(Vector2( extent_.x, -extent_.y));
    Vector2 point2 = body_->transformWorld(extent_);
    Vector2 point3 = body_->transformWorld(Vector2(-extent_.x,  extent_.y));

    aabr_.min = point0;
    aabr_.max = point0;

    if (point1.x < aabr_.min.x) aabr_.min.x = point1.x;
    if (point1.x > aabr_.max.x) aabr_.max.x = point1.x;
    if (point1.y < aabr_.min.y) aabr_.min.y = point1.y;
    if (point1.y > aabr_.max.y) aabr_.max.y = point1.y;

    if (point2.x < aabr_.min.x) aabr_.min.x = point2.x;
    if (point2.x > aabr_.max.x) aabr_.max.x = point2.x;
    if (point2.y < aabr_.min.y) aabr_.min.y = point2.y;
    if (point2.y > aabr_.max.y) aabr_.max.y = point2.y;

    if (point3.x < aabr_.min.x) aabr_.min.x = point3.x;
    if (point3.x > aabr_.max.x) aabr_.max.x = point3.x;
    if (point3.y < aabr_.min.y) aabr_.min.y = point3.y;
    if (point3.y > aabr_.max.y) aabr_.max.y = point3.y;

  }



  // ---------------------------------------------------------------------------
  // Return the furthest point along the given direction
  Vector2 RectShape::getSupportPoint (const Vector2& DIRECTION) const
  {
    // Transform the direction in the local coordinates
    Vector2 direction = body_->getOrientationMatrix().getInverse() *
      DIRECTION;

    Vector2 points[4];
    points[0] = Vector2(-extent_.x, -extent_.y);
    points[1] = Vector2( extent_.x, -extent_.y);
    points[2] = Vector2( extent_.x,  extent_.y);
    points[3] = Vector2(-extent_.x,  extent_.y);

    int index = 0;
    Real projection   = dot(points[0], direction);

    Real tprojection  = dot(points[1], direction);
    if (projection < tprojection)
    {
      index = 1;
      projection = tprojection;
    }

    tprojection  = dot(points[2], direction);
    if (projection < tprojection)
    {
      index = 2;
      projection = tprojection;
    }

    tprojection  = dot(points[3], direction);
    if (projection < tprojection)
    {
      index = 3;
      projection = tprojection;
    }


    return body_->transformWorld(points[index]);
  }


}
