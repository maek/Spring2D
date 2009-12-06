#ifndef __CIRCLE_H__
#define __CIRCLE_H__

#include "s2Settings.h"
#include "s2Body.h"
#include "s2CollisionPrimitive.h"
#include "s2Vector2.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The circle primitive
  class Circle : public CollisionPrimitive
  {
    public:

      // Constructor
      Circle (const Real RADIUS = 1)
        : CollisionPrimitive(), radius_(RADIUS)
      {
        if (radius_ <= 0)
        {
          radius_ = 1;
        }
      }


      // Set the primitive radius
      void setRadius (const Real RADIUS)
      {
        if (RADIUS <= 0)
        {
          return;
        }

        radius_ = RADIUS;
      }

      // Get the primitive radius
      Real getRadius () const
      {
        return radius_;
      }


    private:

      Real      radius_;

  };


}


#endif // __CIRCLE_H__
