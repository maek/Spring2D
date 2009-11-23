#ifndef __GRAVITATIONAL_FORCE_H__
#define __GRAVITATIONAL_FORCE_H__

#include "s2Settings.h"
#include "s2Body.h"
#include "s2Vector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The gravitational force
  class GravitationalForce : public Force
  {
    public:

      // Constructor
      GravitationalForce (Body* BODY,
                          const Vector& GRAVITY_ACCELERATION = Vector(0, G))
        : body_(BODY), gravityAcceleration_(GRAVITY_ACCELERATION)
      { }


      // Apply the force
      void apply ()
      {
        if (body_->isStatic())
        {
          return;
        }

        // TODO: it is faster to change directly the acceleration
        body_->applyForce(gravityAcceleration_ * body_->getMass());
      }


    private:

      Body* body_;
      Vector gravityAcceleration_;

  };


}


#endif // __GRAVITATIONAL_FORCE_H__
