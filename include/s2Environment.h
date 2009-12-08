#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2UGrid.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The physics environment
  class Environment
  {
    public:

      typedef std::list<Body*> BodyList;


    public:

      // Constructor
      Environment (const Real TIME_STEP) : timeStep_(TIME_STEP)
      {
        // TODO: let be parametric
        grid_ = new UGrid (100, 20, 20);
      }

      // Destructor
      ~Environment ()
      {
        delete grid_;
      }


      Body* createBody (
          const Vector2& POSITION = Vector2::ZERO,
          const Vector2& VELOCITY = Vector2::ZERO,
          const Real ORIENTATION = 0);

      void destroyBody (Body*);


      void integrateAllBody ();


      void findCollisionBroad ();


    private:

      UGrid      *grid_;

      BodyList    bodyList_;

      Real        timeStep_;

  };


}


#endif // __ENVIRONMENT_H__
