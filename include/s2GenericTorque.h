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

#ifndef __GENERIC_TORQUE_H__
#define __GENERIC_TORQUE_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // A generic torque
  class GenericTorque : public DynamicEntry
  {
    public:

      // Constructor
      GenericTorque (const Real TORQUE) : torque_(TORQUE) { }

      // Destructor
      ~GenericTorque ()
      {
        bodyList_.clear();
      }


      // Add the given body
      bool addBody (Body* BODY)
      {
        for (BodyList::iterator bodyListI = bodyList_.begin();
            bodyListI != bodyList_.end(); ++bodyListI)
        {
          // If the is already in the body list
          if ((*bodyListI) == BODY)
          {
            return false;
          }
        }

        // If it is a new body
        bodyList_.push_back(BODY);
        return true;
      }


      // Remove the given body
      bool removeBody (Body* BODY)
      {
        for (BodyList::iterator bodyListI = bodyList_.begin();
            bodyListI != bodyList_.end(); ++bodyListI)
        {
          // If the body is in the body list
          if ((*bodyListI) == BODY)
          {
            bodyList_.erase(bodyListI);
            return true;
          }
        }

        // If the body is not in the body list
        return false;
      }


      // Apply the torque
      void apply () const
      {
        for (BodyList::const_iterator bodyListI = bodyList_.begin();
            bodyListI != bodyList_.end(); ++bodyListI)
        {
          // Skip static body
          if ((*bodyListI)->isStatic())
          {
            continue;
          }

          (*bodyListI)->addTorque(torque_);
        }
      }


    private:

      Real torque_;

      BodyList bodyList_;

  };


}


#endif // __GENERIC_TORQUE_H__
