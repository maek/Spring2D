#ifndef __CIRCLE_SHAPE_H__
#define __CIRCLE_SHAPE_H__

#include "s2Settings.h"
#include "s2Shape.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The circle shape
  class CircleShape : public Shape
  {
    public:

      Real radius;


    public:

      // Constructor
      CircleShape (const Real& RADIUS = 1.0) : Shape(), radius(RADIUS) { }

  };


}


#endif // __CIRCLE_SHAPE_H__
