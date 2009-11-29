#ifndef __TORQUE_MOTOR_H__
#define __TORQUE_MOTOR_H__

#include "s2Settings.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The torque motor
  class TorqueMotor : public Torque
  {
    public:

      // Constructor
      TorqueMotor (Body* BODY, const Real TORQUE = 0)
        : body_(BODY), torque_(TORQUE)
      { }


      // Apply the torque
      void apply ()
      {
        if (body_->isStatic())
        {
          return;
        }

        body_->applyTorque(torque_);
      }


    private:

      Body* body_;
      Real torque_;

  };


}


#endif // __TORQUE_MOTOR_H__
