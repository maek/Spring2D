#ifndef __ENGINE_H__
#define __ENGINE_H__

#include "s2Settings.h"
#include "s2EulerIntegrator.h"
#include "s2Integrator.h"
#include "s2Environment.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The core class
  class Engine
  {
    public:

      // Constructor
      Engine (const Real& TIME_STEP)
      {
        environment_  = new Environment();
        integrator_   = new EulerIntegrator(TIME_STEP);
      }

      // Destructor
      ~Engine ()
      {
        delete integrator_;
        delete environment_;
      }


      Environment* getEnvironment () const;

      void runStep () const;


    private:

      Environment *environment_;

      Integrator  *integrator_;

  };


}


#endif // __ENGINE_H__
