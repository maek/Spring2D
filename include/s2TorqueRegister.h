#ifndef __TORQUE_REGISTER_H__
#define __TORQUE_REGISTER_H__

#include "s2Settings.h"
#include "s2Torque.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The register for the torques
  class TorqueRegister
  {
    public:

      // Register a new torque
      void addTorque (Torque* TORQUE)
      {
        torqueList_.push_back(TORQUE);
      }

      // Unregister a torque
      void removeTorque (Torque* TORQUE)
      {
        torqueList_.remove(TORQUE);
      }


      // Compute the net torque
      void computeTorques ()
      {
        for (TorqueList::iterator torqueListI = torqueList_.begin();
            torqueListI != torqueList_.end(); ++torqueListI)
        {
          (*torqueListI)->apply();
        }
      }


    private:

      TorqueList torqueList_;

  };


}


#endif // __TORQUE_REGISTER_H__
