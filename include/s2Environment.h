#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2ForceRegister.h"
#include "s2TorqueRegister.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The physics environment
  class Environment
  {
    public:

      // Constructor
      Environment (const Real TIME_STEP) : timeStep_(TIME_STEP)
      {
        forceRegister_  = new ForceRegister();
        torqueRegister_ = new TorqueRegister();
      }

      // Destructor
      ~Environment ()
      {
        delete forceRegister_;
        delete torqueRegister_;
      }

      Body* createBody (
          const Vector2& POSITION = Vector2::ZERO,
          const Vector2& VELOCITY = Vector2::ZERO,
          const Real ORIENTATION = 0);

      void destroyBody (Body*);


      // Return a pointer to the body list
      const BodyList& getBodyList ()
      {
        return bodyList_;
      }


      // Compute the net force
      void computeForces ()
      {
        forceRegister_->computeForces();
      }

      // Compute the net torque
      void computeTorques ()
      {
        torqueRegister_->computeTorques();
      }


      void integrateBodies ();


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


    private:

      Real              timeStep_;


      BodyList          bodyList_;

      ForceRegister*    forceRegister_;

      TorqueRegister*   torqueRegister_;
  };


}


#endif // __ENVIRONMENT_H__
