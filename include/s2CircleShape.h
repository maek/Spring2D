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
        iMass_    = 1 / (area_ * density_);
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


      // Build the associated AABB
      void buildAABB (Vector2* CENTER)
      {
        aabb_.center_   = CENTER;
        aabb_.halfSize_ = Vector2(radius_, radius_);
      }

      // Update the associated AABB
      void updateAABB () { }


      // Calculate the moment of inertia
      // m * r^2 / 2
      void updateInverseMomentOfInertia ()
      {
        iMomentOfInertia_ = 2 /
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
