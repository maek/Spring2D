#include "../include/s2CircleShape.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Build the associated AABB
  void CircleShape::buildAABB (Vector2* CENTER)
  {
    aabb_.center_   = CENTER;
    aabb_.halfSize_ = Vector2(radius_, radius_);
  }



  // ---------------------------------------------------------------------------
  // Update the associated AABB
  // TODO: make inline
  void CircleShape::updateAABB () { }



  // ---------------------------------------------------------------------------
  // Calculate the moment of inertia
  // m * r^2 / 2
  // TODO: make inline
  Real CircleShape::calculateMomentOfInertia () const
  {
    return area_ * density_ * radius_ * radius_ / 2;
  }



  // ---------------------------------------------------------------------------
  // TODO: comment this
  // TODO: make inline
  Vector2 CircleShape::getSupportPoint0 () const
  {
    return body_->getPosition();
  }



  // ---------------------------------------------------------------------------
  // TODO: comment this
  // TODO: make inline
  Vector2 CircleShape::getSupportPoint (const Vector2& DIRECTION) const
  {
    return body_->getPosition();
  }


}
