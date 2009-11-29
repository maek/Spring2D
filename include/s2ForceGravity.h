#ifndef __FORCE_GRAVITY_H__
#define __FORCE_GRAVITY_H__

#include "s2Settings.h"
#include "s2Body.h"
#include "s2Vector2.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The gravitational force
  class ForceGravity : public Force
  {
    public:

      // Constructor
      ForceGravity (Body* BODY,
                          const Vector2& GRAVITY_ACCELERATION = Vector2(0, G))
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
      Vector2 gravityAcceleration_;

  };


}


#endif // __FORCE_GRAVITY_H__
