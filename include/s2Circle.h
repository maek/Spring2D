#ifndef __CIRCLE_H__
#define __CIRCLE_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2CollisionPrimitive.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The circle primitive
  class Circle : public CollisionPrimitive
  {
    public:

      // Constructor
      Circle (const Real RADIUS)
        : CollisionPrimitive()
      {
        if (RADIUS <= 0)
        {
          // TODO: Exception ???
        }

        radius_ = RADIUS;
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

  };


}


#endif // __CIRCLE_H__
