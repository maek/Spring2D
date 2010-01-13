#ifndef __BODY_H__
#define __BODY_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Shape.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The body entity
  class Body
  {
    public:

      // Constructor
      Body (
          Shape* SHAPE,
          const bool STATIC = false,
          const Vector2& POSITION = Vector2::ZERO,
          const Vector2& VELOCITY = Vector2::ZERO,
          const Vector2& ACCELERATION = Vector2::ZERO,
          const Complex& ORIENTATION = Complex::ZERO,
          const Real ROTATION = 0)
        : static_(STATIC),
        position_(POSITION), velocity_(VELOCITY), acceleration_(ACCELERATION),
        orientation_(ORIENTATION), rotation_(ROTATION),
        drag_(0), elasticity_(1), friction_(0)
      {
        // Check if the shape is valid
        assert(SHAPE->isValid());

        shape_ = SHAPE;
        shape_->body_ = this;
        shape_->buildAABB(&position_);
        iMass_            = (1 / shape_->calculateMass());
        iMomentOfInertia_ = (1 / shape_->calculateMomentOfInertia());
      }


      // Get the shape
      Shape* getShape () const
      {
        return shape_;
      }


      // Is the body static ?
      bool isStatic () const
      {
        return static_;
      }


      // Set the body position
      bool setPosition (const Vector2& POSITION)
      {
        position_ = POSITION;
        return true;
      }

      // Get the body position
      Vector2 getPosition () const
      {
        return position_;
      }


      // Set the body velocity
      bool setVelocity (const Vector2& VELOCITY)
      {
        velocity_ = VELOCITY;
        return true;
      }

      // Get the body velocity
      Vector2 getVelocity () const
      {
        return velocity_;
      }


      // Set the body acceleration
      bool setAcceleration (const Vector2& ACCELERATION)
      {
        acceleration_ = ACCELERATION;
        return true;
      }

      // Get the body acceleration
      Vector2 getAcceleration () const
      {
        return acceleration_;
      }


      // Get the body velocity caused only from acceleration
      Vector2 getVelocityFromAcceleration () const
      {
        return velocityFromAcceleration_;
      }


      // Set the body orientation
      bool setOrientation (const Complex& ORIENTATION)
      {
        orientation_ = ORIENTATION;
        return true;
      }

      // Get the body orientation
      Complex getOrientation () const
      {
        return orientation_;
      }

      // Get the body orientation (the matrix)
      Matrix2x2 getOrientationMatrix () const
      {
        return Matrix2x2(orientation_);
      }


      // Set the body rotation
      bool setRotation (const Real ROTATION)
      {
        rotation_ = ROTATION;
        return true;
      }

      // Get the body rotation
      Real getRotation () const
      {
        return rotation_;
      }


      // Set the body density
      bool setDensity (const Real DENSITY)
      {
        if (shape_->setDensity(DENSITY) == false)
        {
          return false;
        }

        iMass_            = 1 / shape_->calculateMass();
        iMomentOfInertia_ = 1 / shape_->calculateMomentOfInertia();
        return true;
      }

      // Get the body density
      Real getDensity () const
      {
        return shape_->getDensity();
      }


      // Get the body mass
      Real getMass () const
      {
        return (1 / iMass_);
      }

      // Get the body inverse mass
      Real getInverseMass () const
      {
        return iMass_;
      }

      // Get the body moment of inertia
      Real getMomentOfInertia () const
      {
        return (1 / iMomentOfInertia_);
      }

      // Get the body inverse moment of inertia
      Real getInverseMomentOfInertia () const
      {
        return iMomentOfInertia_;
      }


      // Set the body drag
      bool setDrag (const Real DRAG)
      {
        drag_ = DRAG;
        return true;
      }

      // Get the body drag
      Real getDrag () const
      {
        return drag_;
      }


      // Set the body elasticity
      bool setElasticity (const Real ELASTICITY)
      {
        elasticity_ = ELASTICITY;
        return true;
      }

      // Get the body elasticity
      Real getElasticity () const
      {
        return elasticity_;
      }


      // Set the body friction
      bool setFriction (const Real FRICTION)
      {
        friction_ = FRICTION;
        return true;
      }

      // Get the body friction
      Real getFriction () const
      {
        return friction_;
      }


      // Transform the point in local coordinates
      void transformLocal (Vector2* point) const
      {
        *point = getOrientationMatrix().getInverse() * ((*point) - position_);
      }

      // Transform the point from body to world coordinates
      void transformWorld (Vector2* point)
      {
        *point = getOrientationMatrix() * (*point) + position_;
      }


      // Add the given force to the net force
      void applyForce (const Vector2& FORCE)
      {
        netForce_ += FORCE;
      }

      // Apply a force (WORLD) to a point (LOCAL)
      void applyForceAtPoint (const Vector2& FORCE, const Vector2& POINT)
      {
        // TODO: check for external point (NULL force)

        // Add the given force to the net force
        netForce_ += FORCE;

        // Add the calculate torque to the net torque
        netTorque_ += cross(Matrix2x2(orientation_) * POINT, FORCE);
      }


      // Add the given torque to the net torque
      void applyTorque (const Real TORQUE)
      {
        netTorque_ += TORQUE;
      }


      void integrate (const Real);


    private:

      Shape*      shape_;


      bool        static_;


      Vector2     position_;

      Vector2     velocity_;

      Vector2     acceleration_;

      Vector2     velocityFromAcceleration_;


      Complex     orientation_;

      Real        rotation_;


      Real        iMass_;

      Real        iMomentOfInertia_;


      Real        drag_;

      Real        elasticity_;

      Real        friction_;


      Vector2     netForce_;

      Real        netTorque_;

  };



  // ---------------------------------------------------------------------------
  // The body list
  typedef std::list<Body*> BodyList;


}


#endif // __BODY_H__
