#ifndef __ENGINE_H__
#define __ENGINE_H__

#include "s2Settings.h"
#include "s2Environment.h"
#include "s2CollisionDetector.h"
#include "s2CollisionSolver.h"


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
      }

      // Destructor
      ~Engine ()
      {
        delete environment_;
      }


      // Get a pointer to the environment
      Environment* getEnvironment () const
      {
        return environment_;
      }

      // Return the current physical time
      Real getCurrentTime ()
      {
        return stepCounter_ * timeStep_;
      }


      void runStep ();


      // TODO: TESTING
      ContactList* getContacts ()
      {
        return collisionDetector_.getFrontContacts();
      }


    private:

      Real                timeStep_;

      int                 stepCounter_;

      Environment*        environment_;

      CollisionDetector   collisionDetector_;

      CollisionSolver     collisionSolver_;

  };


}


#endif // __ENGINE_H__
