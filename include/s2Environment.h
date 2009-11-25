#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include "s2Settings.h"
#include "s2Body.h"
#include "s2Vector2.h"


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
      Environment (const Real& TIME_STEP) : timeStep_(TIME_STEP) {}


      Body* createBody (const Vector2& POSITION = Vector2::ZERO,
                        const Vector2& VELOCITY = Vector2::ZERO);

      void destroyBody (Body*);


      void integrateAllBody ();


    private:

      BodyList bodyList_;
      Real timeStep_;

  };


}


#endif // __ENVIRONMENT_H__
