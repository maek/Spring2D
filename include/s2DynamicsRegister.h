#ifndef __DYNAMICS_REGISTER_H__
#define __DYNAMICS_REGISTER_H__

#include "s2Settings.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The entry interface for the force & torque register
  class DynamicEntry
  {
    public:

      virtual ~DynamicEntry () { }

      virtual void apply () const = 0;

  };



  // ---------------------------------------------------------------------------
  // The force & torque list
  typedef std::list<DynamicEntry*> DynamicsList;





  // ---------------------------------------------------------------------------
  // The force & torque register
  class DynamicsRegister
  {
    public:

      // Destructor
      ~DynamicsRegister ()
      {
        dynamicsList_.clear();
      }


      // Register the given force/torque
      bool registerDynamic (DynamicEntry* DYNAMIC)
      {
        dynamicsList_.push_back(DYNAMIC);
        return true;
      }


      // Unregister the given force/torque
      bool unregisterDynamic (DynamicEntry* DYNAMIC)
      {
        for (DynamicsList::iterator dynamicsListI = dynamicsList_.begin();
            dynamicsListI != dynamicsList_.end(); ++dynamicsListI)
        {
          // If the dynamic is in the register
          if ((*dynamicsListI) == DYNAMIC)
          {
            dynamicsList_.erase(dynamicsListI);
            return true;
          }
        }

        // If the dynamic is not in the register
        return false;
      }


      // Calculate net forces & net torques for all body
      void calculateNetDynamics () const
      {
        for (DynamicsList::const_iterator dynamicsListI = dynamicsList_.begin();
            dynamicsListI != dynamicsList_.end(); ++dynamicsListI)
        {
          (*dynamicsListI)->apply();
        }
      }


    private:

      DynamicsList dynamicsList_;

  };


}


#endif // __DYNAMICS_REGISTER_H__
