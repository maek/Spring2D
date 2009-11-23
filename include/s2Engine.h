#ifndef __ENGINE_H__
#define __ENGINE_H__

#include "s2Settings.h"
#include "s2EulerIntegrator.h"
#include "s2Integrator.h"
#include "s2Environment.h"
#include "s2ForceRegister.h"


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
        environment_    = new Environment();
        integrator_     = new EulerIntegrator(TIME_STEP);
        forceRegister_  = new ForceRegister();
      }

      // Destructor
      ~Engine ()
      {
        delete forceRegister_;
        delete integrator_;
        delete environment_;
      }


      Environment* getEnvironment () const
      {
        return environment_;
      }

      ForceRegister* getForceRegister () const
      {
        return forceRegister_;
      }


      void runStep () const;


    private:

      Environment *environment_;

      Integrator  *integrator_;

      ForceRegister *forceRegister_;
  };


}


#endif // __ENGINE_H__
