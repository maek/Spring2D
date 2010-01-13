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

      // TODO: does it need ???
      friend class Body;

      enum ShapeType {CIRCLE, RECT, POLYGON};


    public:

      // Destructor
      virtual ~Shape () { }


      virtual ShapeType getType () const = 0;

      // Return a pointer to the body
      Body* getBody () const
      {
        return body_;
      }


      // Return a pointer to the AABB
      const AABB* getAABB () const
      {
        return &aabb_;
      }

      virtual void buildAABB (Vector2*) = 0;

      virtual void updateAABB () = 0;


      // Set the density of the shape
      bool setDensity (const Real DENSITY)
      {
        if (DENSITY <= 0)
        {
          return false;
        }

        density_ = DENSITY;
        return true;
      }

      // Return the density of the shape
      Real getDensity () const
      {
        return density_;
      }


      // Return the area of the shape
      Real getArea () const
      {
        return area_;
      }


      // Calculate the mass of the shape
      Real calculateMass () const
      {
        return area_ * density_;
      }

      virtual Real calculateMomentOfInertia () const = 0;


      virtual Vector2 getSupportPoint0 () const = 0;

      virtual Vector2 getSupportPoint (const Vector2&) const = 0;


      // Check if the shape is valid
      bool isValid () const
      {
        return valid_;
      }


    protected:

      Body*   body_;

      AABB    aabb_;

      Real    area_;

      Real    density_;

      bool    valid_;

  };


}


#endif // __SHAPE_H__
