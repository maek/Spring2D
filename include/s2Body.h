#ifndef __BODY_H__
#define __BODY_H__

#include "s2Settings.h"
#include "s2Complex.h"
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
            const Vector2& VELOCITY = Vector2::ZERO,
            const Complex& ORIENTATION = Complex(0, 0),
            const Vector2& ROTATION = Vector2::ZERO,
            const Real MASS = 1.0,
            const bool STATIC = false)
        : position_(POSITION), velocity_(VELOCITY),
          orientation_(ORIENTATION), rotation_(ROTATION),
          mass_(MASS), static_(STATIC)
      { }


      // Set the body position
      void setPosition (const Vector2& POSITION)
      {
        position_ = POSITION;
      }
      void setPosition (const Real X, const Real Y)
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
      void setVelocity (const Real X, const Real Y)
      {
        velocity_.x = X;
        velocity_.y = Y;
      }

      // Get the body velocity
      Vector2 getVelocity () const
      {
        return velocity_;
      }


      // Set the body orientation
      void setOrientation (const Complex& ORIENTATION)
      {
        orientation_ = ORIENTATION;
      }
      void setOrientation (const Real ANGLE)
      {
        orientation_.r = s2cos(ANGLE);
        orientation_.i = s2sin(ANGLE);
      }

      // Get the body orientation (in radians)
      Real getOrientation () const
      {
        return s2atan2(orientation_.i, orientation_.r);
      }

      // Set the body rotation
      void setRotation (const Vector2& ROTATION)
      {
        rotation_ = ROTATION;
      }
      void setRotation (const Real X, const Real Y)
      {
        rotation_.x = X;
        rotation_.y = Y;
      }

      // Get the body rotation
      Vector2 getRotation () const
      {
        return rotation_;
      }


      // Set the body mass
      void setMass (const Real MASS)
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


      void integrate (const Real);


    private:

      Vector2   position_;

      Vector2   velocity_;

      Complex   orientation_;

      Vector2   rotation_;


      Real      mass_;


      bool static_;

      Vector2   netForce_;

  };


}


#endif // __BODY_H__
