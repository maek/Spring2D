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
        assert(RADIUS > 0);
        assert(DENSITY > 0);

        density_ = DENSITY;
        area_ = M_PI * RADIUS * RADIUS;
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


      // Get the shape type
      ShapeType getType () const
      {
        return Shape::CIRCLE;
      }


      void buildAABB (Vector2*);

      void updateAABB ();

      Real calculateMomentOfInertia () const;

      Vector2 getSupportPoint0 () const;

      Vector2 getSupportPoint (const Vector2&) const;


    private:

      Real  radius_;

  };


}


#endif // __CIRCLE_SHAPE_H__
