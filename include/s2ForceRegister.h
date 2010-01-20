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

      virtual ~Force () { }

      virtual void apply () const = 0;

  };



  // ---------------------------------------------------------------------------
  // The force list
  typedef std::list<Force*> ForceList;





  // ---------------------------------------------------------------------------
  // The register for the forces
  class ForceRegister
  {
    public:

      // Destructor
      ~ForceRegister ()
      {
        forceList_.clear();
      }


      // Register the given force
      bool registerForce (Force* FORCE)
      {
        for (ForceList::iterator forceListI = forceList_.begin();
            forceListI != forceList_.end(); ++forceListI)
        {
          // If the force is already in the register
          if ((*forceListI) == FORCE)
          {
            return false;
          }
        }

        // If the force is a new one
        forceList_.push_back(FORCE);
        return true;
      }


      // Unregister the given force
      bool unregisterForce (Force* FORCE)
      {
        for (ForceList::iterator forceListI = forceList_.begin();
            forceListI != forceList_.end(); ++forceListI)
        {
          // If the force is in the register
          if ((*forceListI) == FORCE)
          {
            forceList_.erase(forceListI);
            return true;
          }
        }

        // If the force is not in the register
        return false;
      }


      // Compute the net force for all body
      void calculateNetForces () const
      {
        for (ForceList::const_iterator forceListI = forceList_.begin();
            forceListI != forceList_.end(); ++forceListI)
        {
          (*forceListI)->apply();
        }
      }


    private:

      ForceList forceList_;

  };


}


#endif // __FORCE_REGISTER_H__
