#ifndef __FORCE_REGISTER_H__
#define __FORCE_REGISTER_H__

#include "s2Settings.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The Force interface
  class Force
  {
    public:

      virtual void apply (Body*) const = 0;

  };





  // ---------------------------------------------------------------------------
  // The entry for the force register
  class ForceEntry
  {
    public:

      // Constructor
      ForceEntry (Force* FORCE) : force_(FORCE) { }

      // Destructor
      ~ForceEntry ()
      {
        bodyList_.clear();
        delete force_;
      }


      // Check if this entry has the given force
      bool hasForce (Force* FORCE) const
      {
        if (force_ == FORCE)
        {
          return true;
        }

        return false;
      }


      // Add a body to the list of body to apply the force
      bool addBody (Body* BODY)
      {
        bodyList_.push_back(BODY);
        return true;
      }

      // Remove a body from the list of body to apply the force
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
      void applyForce ()
      {
        for (BodyList::iterator bodyListI = bodyList_.begin();
            bodyListI != bodyList_.end(); ++bodyListI)
        {
          force_->apply((*bodyListI));
        }
      }


    private:

      Force* force_;

      BodyList bodyList_;

  };



  // ---------------------------------------------------------------------------
  // The force entry list
  typedef std::list<ForceEntry*> ForceEntryList;





  // ---------------------------------------------------------------------------
  // The register for the forces
  class ForceRegister
  {
    public:

      // Apply the given force to the given body
      bool applyForceToBody (Force* FORCE, Body* BODY)
      {
        // Check if the force exists
        for (ForceEntryList::iterator forceEntryListI = forceEntryList_.begin();
            forceEntryListI != forceEntryList_.end(); ++forceEntryListI)
        {
          // If the force is already in the register
          if ((*forceEntryListI)->hasForce(FORCE))
          {
            (*forceEntryListI)->addBody(BODY);
            return true;
          }
        }

        // If the force is a new force
        forceEntryList_.push_back(new ForceEntry(FORCE));
        forceEntryList_.back()->addBody(BODY);
        return true;
      }

      // Remove the given force from the application to the given body
      bool removeForceFromBody (Force* FORCE, Body* BODY)
      {
        // Check if the force exists
        for (ForceEntryList::iterator forceEntryListI = forceEntryList_.begin();
            forceEntryListI != forceEntryList_.end(); ++forceEntryListI)
        {
          if ((*forceEntryListI)->hasForce(FORCE))
          {
            return (*forceEntryListI)->removeBody(BODY);
          }
        }

        return false;
      }


      // Remove a force from the register
      bool removeForce (Force* FORCE)
      {
        // Check if the force exists
        for (ForceEntryList::iterator forceEntryListI = forceEntryList_.begin();
            forceEntryListI != forceEntryList_.end(); ++forceEntryListI)
        {
          if ((*forceEntryListI)->hasForce(FORCE))
          {
            forceEntryList_.erase(forceEntryListI);
            return true;
          }
        }

        return false;
      }


      // Compute the net force for all body
      void calculateNetForces ()
      {
        for (ForceEntryList::iterator forceEntryListI = forceEntryList_.begin();
            forceEntryListI != forceEntryList_.end(); ++forceEntryListI)
        {
          (*forceEntryListI)->applyForce();
        }
      }


    private:

      ForceEntryList forceEntryList_;

  };


}


#endif // __FORCE_REGISTER_H__
