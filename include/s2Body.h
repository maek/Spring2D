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
          const Vector& VELOCITY = Vector::ZERO)
        : position_(POSITION), velocity_(VELOCITY), acceleration_(Vector::ZERO)
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

      // Get the body position
      Vector getPosition () const
      {
        return position_;
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


    private:

      Vector position_;
      Vector velocity_;
      Vector acceleration_;

  };


}


#endif // __BODY_H__
