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
          Shape* SHAPE,
          const bool STATIC,
          const Vector2& POSITION,
          const Vector2& VELOCITY     = Vector2::ZERO,
          const Vector2& ACCELERATION = Vector2::ZERO,
          const Complex& ORIENTATION  = Complex::ZERO,
          const Real ROTATION         = 0);

      void destroyBody (Body*);


      // Return a pointer to the body list
      const BodyList& getBodyList ()
      {
        return bodyList_;
      }


      // Register the given force
      bool registerForce (Force* FORCE) const
      {
        return forceRegister_->registerForce(FORCE);
      }

      // Unregister the given force
      bool unregisterForce (Force* FORCE) const
      {
        return forceRegister_->unregisterForce(FORCE);
      }

      // Compute the net force
      void handleForces () const
      {
        forceRegister_->calculateNetForces();
      }


      // Apply the given torque to the given body
      bool registerTorqueToBody (Torque* TORQUE, Body* BODY) const
      {
        return torqueRegister_->applyTorqueToBody(TORQUE, BODY);
      }

      // Remove the given torque from the application to the given body
      bool unregisterTorqueFromBody (Torque* TORQUE, Body* BODY) const
      {
        return torqueRegister_->removeTorqueFromBody(TORQUE, BODY);
      }

      // Remove a torque from the register
      bool unregisterTorque (Torque* TORQUE) const
      {
        return torqueRegister_->removeTorque(TORQUE);
      }

      // Compute the net torque
      void handleTorques () const
      {
        torqueRegister_->calculateNetTorques();
      }


      void integrateBodies ();




    private:

      Real              timeStep_;


      BodyList          bodyList_;

      ForceRegister*    forceRegister_;

      TorqueRegister*   torqueRegister_;
  };


}


#endif // __ENVIRONMENT_H__
