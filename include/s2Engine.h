#ifndef __ENGINE_H__
#define __ENGINE_H__

#include "s2Settings.h"
#include "s2Environment.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The core class
  class Engine
  {
    public:

      Environment* createEnvironment ();


    private:

      Environment *environment_;

  };


}


#endif // __ENGINE_H__
