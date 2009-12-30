#include "../include/s2RectShape.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Build the associated AABB
  // TODO: check for correctness
  void RectShape::buildAABB (Vector2* CENTER)
  {
    aabb_.center_ = CENTER;
    updateAABB();
  }



  // ---------------------------------------------------------------------------
  // Update the associated AABB
  void RectShape::updateAABB ()
  {
    // TODO: OPTIMIZATION -> check if is necessary to re-build the AABB
    //                       (only if rotating)
    // TODO: make not inline
    // TODO: check if directionX & directionY are normalized
    Vector2 directionX = body_->getOrientationMatrix() * Vector2::X;
    Vector2 directionY = body_->getOrientationMatrix() * Vector2::Y;
    Vector2 point = -halfSize_;
    aabb_.halfSize_.x = s2fabs(dot(point, directionX));
    aabb_.halfSize_.y = s2fabs(dot(point, directionY));

    point.x = -point.x;
    Real tx = s2fabs(dot(point, directionX));
    Real ty = s2fabs(dot(point, directionY));
    if (tx > aabb_.halfSize_.x)
    {
      aabb_.halfSize_.x = tx;
    }
    if (ty > aabb_.halfSize_.y)
    {
      aabb_.halfSize_.y = ty;
    }

    point.y = -point.y;
    tx = s2fabs(dot(point, directionX));
    ty = s2fabs(dot(point, directionY));
    if (tx > aabb_.halfSize_.x)
    {
      aabb_.halfSize_.x = tx;
    }
    if (ty > aabb_.halfSize_.y)
    {
      aabb_.halfSize_.y = ty;
    }

    point.x = -point.x;
    tx = s2fabs(dot(point, directionX));
    ty = s2fabs(dot(point, directionY));
    if (tx > aabb_.halfSize_.x)
    {
      aabb_.halfSize_.x = tx;
    }
    if (ty > aabb_.halfSize_.y)
    {
      aabb_.halfSize_.y = ty;
    }

  }



  // ---------------------------------------------------------------------------
  // Calculate the moment of inertia
  // (m * (w^2 + h^2)) / 12
  // TODO: make inline
  Real RectShape::calculateMomentOfInertia () const
  {
    return area_ * density_ *
      (s2sqr(2 * halfSize_.x) + s2sqr(2 * halfSize_.y)) / 12;
  }



  // ---------------------------------------------------------------------------
  // Return any vertex as the initial support point
  // TODO: make inline
  Vector2 RectShape::getSupportPoint0 () const
  {
    return body_->getPosition() +
      body_->getOrientationMatrix() * Vector2(-halfSize_.x, -halfSize_.y);
  }



  // ---------------------------------------------------------------------------
  // Return the furthest point along the given direction
  Vector2 RectShape::getSupportPoint (const Vector2& DIRECTION) const
  {
    // Transform the direction in the local coordinates
    Vector2 direction = body_->getOrientationMatrix().getInverse() *
      DIRECTION;

    Vector2 points[4];
    points[0] = Vector2(-halfSize_.x, -halfSize_.y);
    points[1] = Vector2( halfSize_.x, -halfSize_.y);
    points[2] = Vector2( halfSize_.x,  halfSize_.y);
    points[3] = Vector2(-halfSize_.x,  halfSize_.y);

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


    body_->transformWorld(&points[index]);
    return points[index];
  }


}
