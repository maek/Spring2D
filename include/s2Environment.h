#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2DynamicsRegister.h"
#include "s2SpringForce.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The physics environment
  class Environment
  {
    public:

      // Constructor
      Environment (const Real TIME_STEP) : timeStep_(TIME_STEP)
      {
        dynamicsRegister_  = new DynamicsRegister ();
      }

      // Destructor
      ~Environment ()
      {
        delete dynamicsRegister_;
      }


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


      // Register the given dynamic
      bool registerDynamic (DynamicEntry* DYNAMIC) const
      {
        return dynamicsRegister_->registerDynamic(DYNAMIC);
      }

      // Unregister the given dynamic
      bool unregisterDynamic (DynamicEntry* DYNAMIC) const
      {
        return dynamicsRegister_->unregisterDynamic(DYNAMIC);
      }

      // Compute the net dynamics
      void handleDynamics () const
      {
        dynamicsRegister_->calculateNetDynamics();
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
          const bool BUNGEE = false) const
      {
        SpringForce* springForce = new SpringForce(
            BODY1, POINT1, BODY2, POINT2, REST_LENGTH, STIFFNESS, DAMP, BUNGEE);
        dynamicsRegister_->registerDynamic(
            static_cast<DynamicEntry*>(springForce));
        return springForce;
      }

      // Destroy the given spring
      bool destroySpring (SpringForce* SPRING_FORCE) const
      {
        return dynamicsRegister_->unregisterDynamic(
            static_cast<DynamicEntry*>(SPRING_FORCE));
      }



      void integrateBodies ();




    private:

      Real                timeStep_;


      BodyList            bodyList_;

      DynamicsRegister*   dynamicsRegister_;
  };


}


#endif // __ENVIRONMENT_H__
