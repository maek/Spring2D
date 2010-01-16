#include "../include/s2PolygonShape.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Build the associated AABB
  // TODO: check for correctness
  void PolygonShape::buildAABB (Vector2* CENTER)
  {
    aabb_.center_ = CENTER;
    updateAABB();
  }



  // ---------------------------------------------------------------------------
  // Update the associated AABB
  void PolygonShape::updateAABB ()
  {
    Real tx;
    Real ty;
    // TODO: OPTIMIZATION -> check if is necessary to re-build the AABB
    //                       (only if rotating)
    // TODO: check if directionX & directionY are normalized
    Vector2 directionX = body_->getOrientationMatrix() * Vector2::X;
    Vector2 directionY = body_->getOrientationMatrix() * Vector2::Y;

    aabb_.halfSize_.x = s2fabs(dot(vertices_[0], directionX));
    aabb_.halfSize_.y = s2fabs(dot(vertices_[0], directionY));

    for (int i = 1; i < nVertices_; ++i)
    {
      tx = s2fabs(dot(vertices_[i], directionX));
      ty = s2fabs(dot(vertices_[i], directionY));

      if (tx > aabb_.halfSize_.x)
      {
        aabb_.halfSize_.x = tx;
      }
      if (ty > aabb_.halfSize_.y)
      {
        aabb_.halfSize_.y = ty;
      }

    }

  }



  // ---------------------------------------------------------------------------
  // Calculate the moment of inertia
  // D / 12 * sum [ ||Pn+1 x Pn|| * (Pn+1^2 + Pn+1 . Pn + Pn^2) ]
  Real PolygonShape::calculateMomentOfInertia () const
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

    momentOfInertia *= density_ / 12;

    return momentOfInertia;
  }



  // ---------------------------------------------------------------------------
  // Return the furthest point along the given direction
  Vector2 PolygonShape::getSupportPoint (const Vector2& DIRECTION) const
  {
    // Transform the direction in the local coordinates
    Vector2 direction = body_->getOrientationMatrix().getInverse() *
      DIRECTION;

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
      body_->transformWorld(&pointCCW);
      return pointCCW;
    }
    else
    {
      // Transform the point in the world coordinates
      body_->transformWorld(&pointCW);
      return pointCW;
    }
  }



  // ---------------------------------------------------------------------------
  // Return the furthest point along the given direction
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
