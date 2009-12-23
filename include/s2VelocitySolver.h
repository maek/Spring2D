#ifndef __VELOCITY_SOLVER_H__
#define __VELOCITY_SOLVER_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Contact.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The velocity solver
  class VelocitySolver
  {
    public:

      void solveVelocity (ContactSet*);

  };


}


#endif // __VELOCITY_SOLVER_H__
