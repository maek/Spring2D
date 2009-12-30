#ifndef __SHAPE_H__
#define __SHAPE_H__

#include "s2Settings.h"
#include "s2Body.h"
#include "s2AABB.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The shape of the body
  class Shape
  {
    public:

      friend class Body;

      enum ShapeType {CIRCLE, POLYGON, RECT};


    public:

      // Destructor
      virtual ~Shape () { }


      // Return a pointer to the body
      Body* getBody () const
      {
        return body_;
      }

      // Return the halfSize of the AABB
      AABB* getAABB ()
      {
        return &aabb_;
      }

      // Return the density of the shape
      Real getDensity ()
      {
        return density_;
      }

      // Return the area of the shape
      Real getArea ()
      {
        return area_;
      }

      // Calculate the mass of the shape
      Real calculateMass () const
      {
        return area_ * density_;
      }


      virtual ShapeType getType () const = 0;

      virtual void buildAABB (Vector2*) = 0;

      virtual void updateAABB () = 0;

      virtual Real calculateMomentOfInertia () const = 0;

      virtual Vector2 getSupportPoint0 () const = 0;

      virtual Vector2 getSupportPoint (const Vector2&) const = 0;


    protected:

      Body*     body_;

      AABB      aabb_;

      Real      density_;

      Real      area_;

  };


}


#endif // __SHAPE_H__
