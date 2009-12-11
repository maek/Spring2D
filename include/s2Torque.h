#ifndef __TORQUE_H__
#define __TORQUE_H__

#include "s2Settings.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The torque interface
  class Torque
  {
    public:

      virtual void apply () = 0;

  };



  // ---------------------------------------------------------------------------
  // The torque list
  typedef std::list<Torque*> TorqueList;


}


#endif // __TORQUE_H__
