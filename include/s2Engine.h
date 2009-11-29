#ifndef __ENGINE_H__
#define __ENGINE_H__

#include "s2Settings.h"
#include "s2Environment.h"
#include "s2ForceRegister.h"
#include "s2TorqueRegister.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The core class
  class Engine
  {
    public:

      // Constructor
      Engine (const Real TIME_STEP) : timeStep_(TIME_STEP), stepCounter_(0)
      {
        environment_    = new Environment(TIME_STEP);
        forceRegister_  = new ForceRegister();
        torqueRegister_  = new TorqueRegister();
      }

      // Destructor
      ~Engine ()
      {
        delete torqueRegister_;
        delete forceRegister_;
        delete environment_;
      }


      // Get a pointer to the environment
      Environment* getEnvironment () const
      {
        return environment_;
      }

      // Get a pointer to the force register
      ForceRegister* getForceRegister () const
      {
        return forceRegister_;
      }
      // Get a pointer to the torque register
      TorqueRegister* getTorqueRegister () const
      {
        return torqueRegister_;
      }


      // Return the current physical time
      Real getCurrentTime ()
      {
        return stepCounter_ * timeStep_;
      }


      void runStep ();


    private:

      Real timeStep_;

      int stepCounter_;

      Environment *environment_;

      ForceRegister *forceRegister_;

      TorqueRegister *torqueRegister_;
  };


}


#endif // __ENGINE_H__
