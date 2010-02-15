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

#include "../include/s2PolygonShape.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Update the associated AABR
  void PolygonShape::updateAABR ()
  {
    Vector2 min;
    Vector2 max;
    Real    projection;

    Vector2 directionX =
      body_->getOrientationMatrix().getInverse() * Vector2::X;
    Vector2 directionY =
      body_->getOrientationMatrix().getInverse() * Vector2::Y;

    min.x = max.x = dot(vertices_[0], directionX);
    min.y = max.y = dot(vertices_[0], directionY);

    for (int i = 1; i < nVertices_; ++i)
    {
      projection = dot(vertices_[i], directionX);

      if (projection < min.x)
      {
        min.x = projection;
      }
      if (projection > max.x)
      {
        max.x = projection;
      }

      projection = dot(vertices_[i], directionY);

      if (projection < min.y)
      {
        min.y = projection;
      }
      if (projection > max.y)
      {
        max.y = projection;
      }
    }

    aabr_.min = body_->getPosition() + min;
    aabr_.max = body_->getPosition() + max;

  }



  // ---------------------------------------------------------------------------
  // Calculate the moment of inertia
  // D / 12 * sum [ ||Pn+1 x Pn|| * (Pn+1^2 + Pn+1 . Pn + Pn^2) ]
  void PolygonShape::updateInverseMomentOfInertia ()
  {
    // Calculate the moment of inertia about the centroid
    Real momentOfInertia =
      s2fabs(cross(vertices_[0], vertices_[nVertices_ - 1])) * (
          dot(vertices_[0],              vertices_[0]) +
          dot(vertices_[0],              vertices_[nVertices_ - 1]) +
          dot(vertices_[nVertices_ - 1], vertices_[nVertices_ - 1])
          );
    for (int i = 0; i < nVertices_ - 1; ++i)
    {
      momentOfInertia +=
        s2fabs(cross(vertices_[i + 1], vertices_[i])) * (
            dot(vertices_[i + 1], vertices_[i + 1]) +
            dot(vertices_[i + 1], vertices_[i]) +
            dot(vertices_[i],     vertices_[i])
            );
    }

    momentOfInertia *= density_ / 12.;
    iMomentOfInertia_ = 1. / momentOfInertia;
  }



  // ---------------------------------------------------------------------------
  // Return the furthest point along the given direction
  Vector2 PolygonShape::getSupportPoint (const Vector2& DIRECTION) const
  {
    // Transform the direction in the local coordinates
    Vector2 direction =
      body_->getOrientationMatrix().getInverse() * DIRECTION;

    Vector2 pointCW   = vertices_[0];
    Vector2 pointCCW  = vertices_[0];

    Real projection;
    Real tprojection;

    // Counter-clockwise
    projection = dot(pointCCW, direction);
    for (int i = 1; i < nVertices_; ++i)
    {
      tprojection = dot(vertices_[i], direction);
      if (projection <= tprojection)
      {
        pointCCW = vertices_[i];
        projection = tprojection;
      }
      else
      {
        break;
      }
    }

    // Clockwise
    projection = dot(pointCW, direction);
    for (int i = nVertices_ - 1; i > 0; --i)
    {
      tprojection = dot(vertices_[i], direction);
      if (projection <= tprojection)
      {
        pointCW = vertices_[i];
        projection = tprojection;
      }
      else
      {
        break;
      }
    }

    // Pick the furthest
    if (dot(pointCCW, direction) > projection)
    {
      // Transform the point in the world coordinates
      return body_->transformWorld(pointCCW);
    }
    else
    {
      // Transform the point in the world coordinates
      return body_->transformWorld(pointCW);
    }
  }



  // ---------------------------------------------------------------------------
  // Check the given polygon for convexity
  bool PolygonShape::convexityTest (
      const int N_VERTICES, const Vector2* VERTICES) const
  {
    int counterX  = 0;
    int counterY  = 0;

    // First edge
    Vector2 previousEdge  = VERTICES[0] - VERTICES[N_VERTICES - 1];
    Vector2 currentEdge   = VERTICES[1] - VERTICES[0];

    if (cross(previousEdge, currentEdge) < 0)
    {
      return false;
    }

    if ((previousEdge.x >= 0 && currentEdge.x <  0) ||
        (previousEdge.x <  0 && currentEdge.x >= 0))
      ++counterX;
    if ((previousEdge.y >= 0 && currentEdge.y <  0) ||
        (previousEdge.y <  0 && currentEdge.y >= 0))
      ++counterY;


    // Central edges
    for (int i = 1; i < N_VERTICES - 1; ++i)
    {
      previousEdge = currentEdge;
      currentEdge = VERTICES[i + 1] - VERTICES[i];

      if (cross(previousEdge, currentEdge) < 0)
      {
        return false;
      }

      if ((previousEdge.x >= 0 && currentEdge.x <  0) ||
          (previousEdge.x <  0 && currentEdge.x >= 0))
      {
        ++counterX;
      }
      if ((previousEdge.y >= 0 && currentEdge.y <  0) ||
          (previousEdge.y <  0 && currentEdge.y >= 0))
      {
        ++counterY;
      }

      if (counterX > 2 || counterY > 2)
      {
        return false;
      }
    }


    // Last edge
    previousEdge = currentEdge;
    currentEdge = VERTICES[0] - VERTICES[N_VERTICES - 1];

    if (cross(previousEdge, currentEdge) < 0)
    {
      return false;
    }

    if ((previousEdge.x >= 0 && currentEdge.x <  0) ||
        (previousEdge.x <  0 && currentEdge.x >= 0))
    {
      ++counterX;
    }
    if ((previousEdge.y >= 0 && currentEdge.y <  0) ||
        (previousEdge.y <  0 && currentEdge.y >= 0))
    {
      ++counterY;
    }

    if (counterX > 2 || counterY > 2)
    {
      return false;
    }


    return true;
  }


}
