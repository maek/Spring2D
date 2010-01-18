#ifndef __TORQUE_REGISTER_H__
#define __TORQUE_REGISTER_H__

#include "s2Settings.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The Torque interface
  class Torque
  {
    public:

      virtual void apply (Body*) const = 0;

  };





  // ---------------------------------------------------------------------------
  // The entry for the torque register
  class TorqueEntry
  {
    public:

      // Constructor
      TorqueEntry (Torque* TORQUE) : torque_(TORQUE) { }

      // Destructor
      ~TorqueEntry()
      {
        bodyList_.clear();
        delete torque_;
      }


      // Check if this entry has the given torque
      bool hasTorque (Torque* TORQUE) const
      {
        if (torque_ == TORQUE)
        {
          return true;
        }

        return false;
      }


      // Add a body to the list of body to apply the torque
      bool addBody (Body* BODY)
      {
        bodyList_.push_back(BODY);
        return true;
      }

      // Remove a body from the list of body to apply the torque
      bool removeBody (Body* BODY)
      {
        for (BodyList::iterator bodyListI = bodyList_.begin();
            bodyListI != bodyList_.end(); ++bodyListI)
        {
          if ((*bodyListI) == BODY)
          {
            bodyList_.erase(bodyListI);
            return true;
          }
        }

        return false;
      }


      // Apply the force to all register body
      void applyTorque ()
      {
        for (BodyList::iterator bodyListI = bodyList_.begin();
            bodyListI != bodyList_.end(); ++bodyListI)
        {
          torque_->apply((*bodyListI));
        }
      }


    private:

      Torque* torque_;

      BodyList bodyList_;

  };



  // ---------------------------------------------------------------------------
  // The force entry list
  typedef std::list<TorqueEntry*> TorqueEntryList;





  // ---------------------------------------------------------------------------
  // The register for the torques
  class TorqueRegister
  {
    public:

      // Apply the given torque to the given body
      bool applyTorqueToBody (Torque* TORQUE, Body* BODY)
      {
        // Check if the torque exists
        for (TorqueEntryList::iterator torqueEntryListI = torqueEntryList_.begin();
            torqueEntryListI != torqueEntryList_.end(); ++torqueEntryListI)
        {
          // If the torque is already in the register
          if ((*torqueEntryListI)->hasTorque(TORQUE))
          {
            (*torqueEntryListI)->addBody(BODY);
            return true;
          }
        }

        // If the torque is a new torque
        torqueEntryList_.push_back(new TorqueEntry(TORQUE));
        torqueEntryList_.back()->addBody(BODY);
        return true;
      }

      // Remove the given torque from the application to the given body
      bool removeTorqueFromBody (Torque* TORQUE, Body* BODY)
      {
        // Check if the torque exists
        for (TorqueEntryList::iterator torqueEntryListI = torqueEntryList_.begin();
            torqueEntryListI != torqueEntryList_.end(); ++torqueEntryListI)
        {
          if ((*torqueEntryListI)->hasTorque(TORQUE))
          {
            return (*torqueEntryListI)->removeBody(BODY);
          }
        }

        return false;
      }


      // Remove a torque from the register
      bool removeTorque (Torque* TORQUE)
      {
        // Check if the torque exists
        for (TorqueEntryList::iterator torqueEntryListI = torqueEntryList_.begin();
            torqueEntryListI != torqueEntryList_.end(); ++torqueEntryListI)
        {
          if ((*torqueEntryListI)->hasTorque(TORQUE))
          {
            torqueEntryList_.erase(torqueEntryListI);
            return true;
          }
        }

        return false;
      }


      // Compute the net torque for all body
      void calculateNetTorques ()
      {
        for (TorqueEntryList::iterator torqueEntryListI = torqueEntryList_.begin();
            torqueEntryListI != torqueEntryList_.end(); ++torqueEntryListI)
        {
          (*torqueEntryListI)->applyTorque();
        }
      }


    private:

      TorqueEntryList torqueEntryList_;

  };


}


#endif // __TORQUE_REGISTER_H__
