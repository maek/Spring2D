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

#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2DynamicsRegister.h"
#include "s2ConstraintsRegister.h"
#include "s2SpringForce.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The physics environment
  class Environment
  {
    public:

      // Constructor
      Environment (const Real TIMESTEP) : timestep_(TIMESTEP) { }


      Body* createBody (
          Shape* SHAPE,
          const bool STATIC,
          const Vector2& POSITION,
          const Vector2& VELOCITY     = Vector2::ZERO,
          const Vector2& ACCELERATION = Vector2::ZERO,
          const Complex& ORIENTATION  = Complex::ZERO,
          const Real ROTATION         = 0);

      void destroyBody (Body*);


      // Return a pointer to the body list
      const BodyList& getBodyList ()
      {
        return bodyList_;
      }

      // Return a pointer to the constraint list
      ConstraintsList* getConstraints ()
      {
        return constraintsRegister_.getConstraints();
      }


      // Register the given dynamic
      bool registerDynamic (DynamicEntry* DYNAMIC)
      {
        return dynamicsRegister_.registerDynamic(DYNAMIC);
      }

      // Unregister the given dynamic
      bool unregisterDynamic (DynamicEntry* DYNAMIC)
      {
        return dynamicsRegister_.unregisterDynamic(DYNAMIC);
      }

      // Compute the net dynamics
      void handleDynamics () const
      {
        dynamicsRegister_.calculateNetDynamics();
      }


      // Create a spring
      SpringForce* createSpring (
          Body* BODY1,
          const Vector2& POINT1,
          Body* BODY2,
          const Vector2& POINT2,
          const Real REST_LENGTH,
          const Real STIFFNESS = 1,
          const Real DAMP = 0,
          const bool BUNGEE = false)
      {
        SpringForce* springForce = new SpringForce(
            BODY1, POINT1, BODY2, POINT2, REST_LENGTH, STIFFNESS, DAMP, BUNGEE);
        dynamicsRegister_.registerDynamic(
            static_cast<DynamicEntry*>(springForce));
        return springForce;
      }

      // Destroy the given spring
      bool destroySpring (SpringForce* SPRING_FORCE)
      {
        return dynamicsRegister_.unregisterDynamic(
            static_cast<DynamicEntry*>(SPRING_FORCE));
      }


      // Create a constraint
      Constraint* createConstraint (
          Body* BODY1,
          const Vector2& POINT1,
          Body* BODY2,
          const Vector2& POINT2,
          const Real LENGTH,
          const bool CABLE = false)
      {
        Constraint* constraint = new Constraint(
            BODY1, POINT1, BODY2, POINT2, LENGTH, CABLE);
        constraintsRegister_.registerConstraint(constraint);
        return constraint;
      }

      // Destroy the given constraint
      bool destroyConstraint (Constraint* CONSTRAINT)
      {
        return constraintsRegister_.unregisterConstraint(CONSTRAINT);
      }


      void integrateBodies ();


    private:

      Real                  timestep_;


      BodyList              bodyList_;

      DynamicsRegister      dynamicsRegister_;

      ConstraintsRegister   constraintsRegister_;
  };


}


#endif // __ENVIRONMENT_H__
