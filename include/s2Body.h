#ifndef __BODY_H__
#define __BODY_H__

#include "s2Settings.h"
#include "s2Vector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The body entity
  class Body
  {
    public:

      Vector position;

      Vector velocity;

      Vector acceleration;


    public:

      // Constructor
      Body (const Vector& POSITION = Vector::ZERO,
            const Vector& VELOCITY = Vector::ZERO)
        : position(POSITION), velocity(VELOCITY), acceleration(Vector::ZERO)
      { }

  };


}


#endif // __BODY_H__
