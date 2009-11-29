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

      typedef std::list<Torque*> TorqueList;


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


      // Compute the net torques
      void computeTorques ()
      {
        std::for_each (torqueList_.begin(), torqueList_.end(),
            std::mem_fun(&Torque::apply));
      }


    private:

      TorqueList torqueList_;

  };


}


#endif // __TORQUE_REGISTER_H__
