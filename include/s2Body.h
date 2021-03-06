/*
 * Copyright (C) 2010   Marco Dalla Via (maek@paranoici.org)
 *
 *  This file is part of Spring2D.
 *
 *  Spring2D is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Spring2D is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Lesser Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser Public License
 *  along with Spring2D. If not, see <http://www.gnu.org/licenses/>.
 */

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
          const Real TIMESTEP,
          Shape* SHAPE,
          const bool STATIC = false,
          const Vector2& POSITION = Vector2::ZERO,
          const Vector2& VELOCITY = Vector2::ZERO,
          const Vector2& ACCELERATION = Vector2::ZERO,
          const Complex& ORIENTATION = Complex::ZERO,
          const Real ROTATION = 0)
        : timestep_(TIMESTEP), static_(STATIC),
        position_(POSITION), velocity_(VELOCITY), acceleration_(ACCELERATION),
        orientation_(ORIENTATION), rotation_(ROTATION),
        iDrag_(1), elasticity_(1), friction_(0),
        sleeping_(false), motion_(2 * MOTION_THRESHOLD)
      {
        shape_ = SHAPE;
        shape_->body_ = this;
        shape_->updateAABR();

        if (STATIC)
        {
          sleeping_ = true;
        }
      }

      // Destructor
      ~Body ()
      {
        delete shape_;
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
        if (ORIENTATION.getMagnitude() != 1)
        {
          return false;
        }

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
        return shape_->getMass();
      }

      // Get the body inverse mass
      Real getInverseMass () const
      {
        return shape_->getInverseMass();
      }

      // Get the body moment of inertia
      Real getMomentOfInertia () const
      {
        return shape_->getMomentOfInertia();
      }

      // Get the body inverse moment of inertia
      Real getInverseMomentOfInertia () const
      {
        return shape_->getInverseMomentOfInertia();
      }


      // Set the body drag
      bool setDrag (const Real DRAG)
      {
        if (DRAG < 0 || 1 < DRAG)
        {
          return false;
        }

        iDrag_ = 1 - DRAG;
        return true;
      }

      // Get the body drag
      Real getDrag () const
      {
        return (1 - iDrag_);
      }


      // Set the body elasticity
      bool setElasticity (const Real ELASTICITY)
      {
        if (ELASTICITY < 0 || 1 < ELASTICITY)
        {
          return false;
        }

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
        if (FRICTION < 0)
        {
          return false;
        }

        friction_ = FRICTION;
        return true;
      }

      // Get the body friction
      Real getFriction () const
      {
        return friction_;
      }


      // Is the body sleeping ?
      bool isSleeping () const
      {
        return sleeping_;
      }

      // Awake the body
      void awake ()
      {
        sleeping_ = false;
        motion_ = 2 * MOTION_THRESHOLD;
      }


      // Transform the point in local coordinates
      void transformLocal (Vector2* point) const
      {
        *point = getOrientationMatrix().getInverse() * ((*point) - position_);
      }
      Vector2 transformLocal (Vector2 point) const
      {
        return getOrientationMatrix().getInverse() * (point - position_);
      }

      // Transform the point from body to world coordinates
      void transformWorld (Vector2* point)
      {
        *point = getOrientationMatrix() * (*point) + position_;
      }
      Vector2 transformWorld (Vector2 point)
      {
        return getOrientationMatrix() * point + position_;
      }


      // Add the given force to the net force
      void addForce (const Vector2& FORCE)
      {
        netForce_ += FORCE;
      }

      // Apply a force (WORLD) to a point (LOCAL)
      void addForceAtPoint (const Vector2& FORCE, const Vector2& POINT)
      {
        // TODO: check for external point (NULL force)

        // Add the given force to the net force
        netForce_ += FORCE;

        // Add the calculate torque to the net torque
        netTorque_ += cross(Matrix2x2(orientation_) * POINT, FORCE);
      }


      // Add the given torque to the net torque
      void addTorque (const Real TORQUE)
      {
        netTorque_ += TORQUE;
      }


      void integrate ();


    private:

      Real        timestep_;


      Shape*      shape_;


      bool        static_;


      Vector2     position_;

      Vector2     velocity_;

      Vector2     acceleration_;

      Vector2     velocityFromAcceleration_;


      Complex     orientation_;

      Real        rotation_;


      Real        iDrag_;

      Real        elasticity_;

      Real        friction_;


      bool        sleeping_;

      Real        motion_;


      Vector2     netForce_;

      Real        netTorque_;

  };



  // ---------------------------------------------------------------------------
  // The body list type
  typedef std::list<Body*> BodyList;



  // ---------------------------------------------------------------------------
  // The const body list type
  typedef std::list<const Body*> ConstBodyList;



  // ---------------------------------------------------------------------------
  // The body vector type
  typedef std::vector<Body*> BodyVector;



  // ---------------------------------------------------------------------------
  // The const body vector type
  typedef std::vector<const Body*> ConstBodyVector;


}


#endif // __BODY_H__
