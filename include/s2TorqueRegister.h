#ifndef __TORQUE_REGISTER_H__
#define __TORQUE_REGISTER_H__

#include "s2Settings.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The Torque interface
  class Torque
  {
    public:

      virtual ~Torque () { }


      virtual void apply () const = 0;


      virtual bool addBody (Body*) { return false; }

      virtual bool removeBody (Body*) { return false; }

  };



  // ---------------------------------------------------------------------------
  // The torque list
  typedef std::list<Torque*> TorqueList;





  // ---------------------------------------------------------------------------
  // The register for the torques
  class TorqueRegister
  {
    public:

      // Destructor
      ~TorqueRegister ()
      {
        torqueList_.clear();
      }


      // Register the given torque
      bool registerTorque (Torque* TORQUE)
      {
        for (TorqueList::iterator torqueListI = torqueList_.begin();
            torqueListI != torqueList_.end(); ++torqueListI)
        {
          // If the torque is already in the register
          if ((*torqueListI) == TORQUE)
          {
            return false;
          }
        }

        // If the torque is a new one
        torqueList_.push_back(TORQUE);
        return true;
      }


      // Unregister the given torque
      bool unregisterTorque (Torque* TORQUE)
      {
        for (TorqueList::iterator torqueListI = torqueList_.begin();
            torqueListI != torqueList_.end(); ++torqueListI)
        {
          // If the torque is in the register
          if ((*torqueListI) == TORQUE)
          {
            torqueList_.erase(torqueListI);
            return true;
          }
        }

        // If the torque is not in the register
        return false;
      }


      // Compute the net torque for all body
      void calculateNetTorques () const
      {
        for (TorqueList::const_iterator torqueListI = torqueList_.begin();
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
