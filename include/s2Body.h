#ifndef __BODY_H__
#define __BODY_H__

#include "s2Settings.h"
#include "s2Vector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The body entity
  class Body
  {
    public:

      // Constructor
      Body (const Vector& POSITION = Vector::ZERO,
            const Vector& VELOCITY = Vector::ZERO, const Real& MASS = 1.0,
            const bool STATIC = false)
        : position_(POSITION), velocity_(VELOCITY), acceleration_(Vector::ZERO),
          mass_(MASS), static_(STATIC)
      { }


      // Set the body position
      void setPosition (const Vector& POSITION)
      {
        position_ = POSITION;
      }
      void setPosition (Real X, Real Y)
      {
        position_.x = X;
        position_.y = Y;
      }

      // Get the body position
      Vector getPosition () const
      {
        return position_;
      }

      // Set the body velocity
      void setVelocity (const Vector& VELOCITY)
      {
        velocity_ = VELOCITY;
      }
      void setVelocity (Real X, Real Y)
      {
        velocity_.x = X;
        velocity_.y = Y;
      }

      // Get the body velocity
      Vector getVelocity () const
      {
        return velocity_;
      }

      // Get the body acceleration
      Vector getAcceleration () const
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
      Real getMass ()
      {
        return mass_;
      }


      // Make the body static
      void makeStatic ()
      {
        static_ = true;
      }

      // Is the body static ?
      bool isDynamic ()
      {
        return !static_;
      }


      // Make the body dynamic
      void makeDynamic ()
      {
        static_ = false;
      }

      // Is the body dynamic ?
      bool isStatic ()
      {
        return static_;
      }


    private:

      // TODO: find another way
      friend class EulerIntegrator;


    private:

      Vector position_;

      Vector velocity_;

      Vector acceleration_;

      Real mass_;

      bool static_;
  };


}


#endif // __BODY_H__
