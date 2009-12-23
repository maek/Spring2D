#ifndef __COLLISION_SOLVER_H__
#define __COLLISION_SOLVER_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Contact.h"
#include "s2Body.h"
#include "s2InterpenetrationSolver.h"
#include "s2VelocitySolver.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The collisions solver
  class CollisionSolver
  {
    public:

      void solveCollisions (ContactSet*);


    private:

      InterpenetrationSolver  interpenetrationSolver_;

      VelocitySolver          velocitySolver_;

  };


}


#endif // __COLLISION_SOLVER_H__
