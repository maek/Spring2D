#ifndef __ENGINE_H__
#define __ENGINE_H__

#include "s2Settings.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The core class
  class Engine
  {
    public:


    public:

      bool start ();

      bool runStep ();

      bool turnOff ();


    private:

      bool running_;


    private:

  };


}


#endif // __ENGINE_H__
