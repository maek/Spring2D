#ifndef __INTERPENETRATION_SOLVER_H__
#define __INTERPENETRATION_SOLVER_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Contact.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The interpenetration solver
  class InterpenetrationSolver
  {
    public:

      void solveInterpenetration (ContactSet*);

  };


}


#endif // __INTERPENETRATION_SOLVER_H__

