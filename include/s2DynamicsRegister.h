/*
 * Copyright (C) 2010   Marco Dalla Via (maek@paranoici.org)
 *
 *  This file is part of Spring2D.
 *
 *  Spring2D is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Spring2D is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Lesser Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser Public License
 *  along with Spring2D. If not, see <http://www.gnu.org/licenses/>.
 */

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
