#ifndef __VERLET_INTEGRATOR_H__
#define __VERLET_INTEGRATOR_H__

#include "s2Settings.h"
#include "s2Integrator.h"
#include "s2Environment.h"
#include "s2Body.h"
#include "s2Vector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The Verle integrator
  class VerletIntegrator : public Integrator
  {
    public:

      // Constructor
      VerletIntegrator (const Real& TIME_STEP) : Integrator(TIME_STEP) { }


      void compute (Environment*) const;

  };


}


#endif // __VERLET_INTEGRATOR_H__
