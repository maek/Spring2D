#ifndef __FORCE_REGISTER_H__
#define __FORCE_REGISTER_H__

#include "s2Settings.h"
#include "s2Force.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The register for the forces
  class ForceRegister
  {
    public:

      // Register a new force
      void addForce (Force* FORCE)
      {
        forceList_.push_back(FORCE);
      }

      // Unregister a force
      void removeForce (Force* FORCE)
      {
        forceList_.remove(FORCE);
      }


      // Compute the net force
      void computeForces ()
      {
        std::for_each (forceList_.begin(), forceList_.end(),
            std::mem_fun(&Force::apply));
      }


    private:

      ForceList forceList_;

  };


}


#endif // __FORCE_REGISTER_H__
