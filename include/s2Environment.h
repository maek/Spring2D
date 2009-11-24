#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include "s2Settings.h"
#include "s2Body.h"
#include "s2Vector.h"


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


      Body* createBody (const Vector& POSITION = Vector::ZERO,
                        const Vector& VELOCITY = Vector::ZERO);

      void destroyBody (Body*);


      void integrateAllBody ();


    private:

      BodyList bodyList_;
      Real timeStep_;

  };


}


#endif // __ENVIRONMENT_H__
