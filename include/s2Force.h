#ifndef __FORCE_H__
#define __FORCE_H__

#include "s2Settings.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The Force interface
  class Force
  {
    public:

      virtual void apply () = 0;

  };


}


#endif // __FORCE_H__
