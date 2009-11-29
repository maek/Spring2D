#ifndef __TORQUE_H__
#define __TORQUE_H__

#include "s2Settings.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The Torque interface
  class Torque
  {
    public:

      virtual void apply () = 0;

  };


}


#endif // __TORQUE_H__
