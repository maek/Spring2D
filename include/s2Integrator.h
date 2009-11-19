#ifndef __INTEGRATOR_H__
#define __INTEGRATOR_H__

#include "s2Settings.h"
#include "s2Environment.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The interface for the integrators
  class Integrator
  {
    public:

      virtual void compute (Environment*) = 0;
  };


}


#endif // __INTEGRATOR_H__
