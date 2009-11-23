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

      // Virtual Destructor
      virtual ~Integrator() { }


      virtual void compute (Environment*) const = 0;


    protected:

      // Constructor
      Integrator (const Real& TIME_STEP) : timeStep_(TIME_STEP) { }


    protected:

      Real timeStep_;

  };


}


#endif // __INTEGRATOR_H__