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
      CircleShape (const Real RADIUS)
        : radius_(RADIUS)
      {
        assert(RADIUS > 0);
      }


      // Set the radius
      void setRadius (const Real RADIUS)
      {
        assert(RADIUS > 0);
        radius_ = RADIUS;
      }

      // Get the radius
      Real getRadius () const
      {
        return radius_;
      }


      // Build the associated AABB
      void buildAABB (Vector2* CENTER)
      {
        aabb_.center_   = CENTER;
        aabb_.halfSize_ = Vector2(radius_, radius_);
      }

      // Update the associated AABB
      void updateAABB () { }


    private:

      Real  radius_;

  };


}


#endif // __CIRCLE_SHAPE_H__
