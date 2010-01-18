#ifndef __GENERIC_TORQUE_H__
#define __GENERIC_TORQUE_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // A generic torque
  class GenericTorque : public Torque
  {
    public:

      // Constructor
      GenericTorque (const Real TORQUE) : torque_(TORQUE) { }


      // Apply the torque
      void apply (Body* BODY) const
      {
        if (BODY->isStatic())
        {
          return;
        }

        BODY->applyTorque(torque_);
      }


    private:

      Real torque_;

  };


}


#endif // __GENERIC_TORQUE_H__
