#ifndef __BODY_H__
#define __BODY_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2CollisionPrimitive.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The body entity
  class Body
  {
    public:

      // Constructor
      // TODO: get the moment of intertia from the shape (otherwise 0)
      //  CIRCLE  = (m * r^2) / 2
      //  RECT    = (m * (b^2 + h^2)) / 12
      Body (
          const Vector2& POSITION = Vector2::ZERO,
          const Vector2& VELOCITY = Vector2::ZERO,
          const Complex& ORIENTATION = Complex(1, 0),
          const Real ROTATION = 0,
          const Real MASS = 1,
          const Real MOMENT_OF_INERTIA = 1,
          const bool STATIC = false)
        : position_(POSITION), velocity_(VELOCITY),
        orientation_(ORIENTATION), rotation_(ROTATION),
        mass_(MASS), momentOfInertia_(MOMENT_OF_INERTIA), static_(STATIC)
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

      // Get the body orientation (in radians)
      Matrix2x2 getOrientationMatrix () const
      {
        return Matrix2x2(orientation_);
      }


      // Set the body rotation
      void setRotation (const Real ROTATION)
      {
        rotation_ = ROTATION;
      }

      // Get the body rotation
      Real getRotation () const
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


      // Set the collision primitive
      void setCollisionPrimitive (CollisionPrimitive* collisionPrimitive)
      {
        collisionPrimitive_ = collisionPrimitive;
        collisionPrimitive_->body_ = this;
      }

      // Get the collision primitive
      CollisionPrimitive* getCollisionPrimitive () const
      {
        return collisionPrimitive_;
      }


      // Make the body static
      void makeStatic ()
      {
        static_ = true;
      }

      // Is the body static ?
      bool isStatic () const
      {
        return static_;
      }


      // Make the body dynamic
      void makeDynamic ()
      {
        static_ = false;
      }

      // Is the body dynamic ?
      bool isDynamic () const
      {
        return !static_;
      }


      // Apply a force
      void applyForce (const Vector2& FORCE)
      {
        netForce_ += FORCE;
      }

      // Apply a force (WORLD) to a point (BODY)
      void applyForceAtPoint (const Vector2& FORCE, const Vector2& POINT)
      {
        // TODO: check for external point (NULL force)

        // Apply the force to the netForce
        netForce_ += FORCE;

        // Rotate the point
        Vector2 worldPoint = Matrix2x2(orientation_) * POINT;
        // Apply the torque as point CROSS force
        netTorque_ += crossProduct(worldPoint, FORCE);
      }


      // Add a torque
      void applyTorque (const Real TORQUE)
      {
        netTorque_ += TORQUE;
      }


      void integrate (const Real);


    private:

      Vector2               position_;

      Vector2               velocity_;

      Complex               orientation_;

      Real                  rotation_;


      Real                  mass_;

      Real                  momentOfInertia_;


      Vector2               netForce_;

      Real                  netTorque_;


      CollisionPrimitive*   collisionPrimitive_;


      bool                  static_;

  };


}


#endif // __BODY_H__
