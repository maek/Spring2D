#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include "s2Settings.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The physics environment
  class Environment
  {
    public:

      typedef std::list<Body*> BodyList;


      Body* createBody (const Vector& POS = Vector::ZERO);

      void destroyBody (Body*);


    private:

      BodyList bodyList_;

  };


}


#endif // __ENVIRONMENT_H__