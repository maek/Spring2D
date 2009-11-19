#ifndef __EULER_INTEGRATOR_H__
#define __EULER_INTEGRATOR_H__

#include "s2Settings.h"
#include "s2Integrator.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The Euler integrator
  class EulerIntegrator : public Integrator
  {
    public:

      void compute (Environment*);
  };


}


#endif // __EULER_INTEGRATOR_H__
