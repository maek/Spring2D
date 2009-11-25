#ifndef __BODY_H__
#define __BODY_H__

#include "s2Settings.h"
#include "s2Vector2.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The body entity
  class Body
  {
    public:

      // Constructor
      Body (const Vector2& POSITION = Vector2::ZERO,
            const Vector2& VELOCITY = Vector2::ZERO, const Real& MASS = 1.0,
            const bool STATIC = false)
        : position_(POSITION), velocity_(VELOCITY), acceleration_(Vector2::ZERO),
          mass_(MASS), static_(STATIC)
      { }


      // Set the body position
      void setPosition (const Vector2& POSITION)
      {
        position_ = POSITION;
      }
      void setPosition (Real X, Real Y)
      {
        position_.x = X;
        position_.y = Y;
      }

      // Get the body position
      Vector2 getPosition () const
      {
        return position_;
      }

      // Set the body velocity
      void setVelocity (const Vector2& VELOCITY)
      {
        velocity_ = VELOCITY;
      }
      void setVelocity (Real X, Real Y)
      {
        velocity_.x = X;
        velocity_.y = Y;
      }

      // Get the body velocity
      Vector2 getVelocity () const
      {
        return velocity_;
      }

      // Get the body acceleration
      Vector2 getAcceleration () const
      {
        return acceleration_;
      }


      // Set the body mass
      void setMass (const Real& MASS)
      {
        if (MASS <= 0)
        {
          return;
        }

        mass_ = MASS;
      }

      // Get the body mass
      Real getMass () const
      {
        return mass_;
      }


      // Make the body static
      void makeStatic ()
      {
        static_ = true;
      }

      // Is the body static ?
      bool isDynamic () const
      {
        return !static_;
      }


      // Make the body dynamic
      void makeDynamic ()
      {
        static_ = false;
      }

      // Is the body dynamic ?
      bool isStatic () const
      {
        return static_;
      }


      // Add a force to the net force
      void applyForce (const Vector2& FORCE)
      {
        netForce_ += FORCE;
      }


      void integrate (const Real&);


    private:

      Vector2  position_;

      Vector2  velocity_;

      Vector2  acceleration_;

      Real    mass_;


      bool static_;

      Vector2 netForce_;

  };


}


#endif // __BODY_H__
