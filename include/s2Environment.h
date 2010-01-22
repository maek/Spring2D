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
      Environment (const Real TIME_STEP) : timeStep_(TIME_STEP) { }


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

      Real                  timeStep_;


      BodyList              bodyList_;

      DynamicsRegister      dynamicsRegister_;

      ConstraintsRegister   constraintsRegister_;
  };


}


#endif // __ENVIRONMENT_H__
