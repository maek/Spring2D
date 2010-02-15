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

#ifndef __ENGINE_H__
#define __ENGINE_H__

#include "s2Settings.h"
#include "s2Environment.h"
#include "s2CollisionDetector.h"
#include "s2CollisionSolver.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The core class
  class Engine
  {
    public:

      // Constructor
      Engine (const Real TIMESTEP) : timestep_(TIMESTEP), stepCounter_(0)
      {
        environment_    = new Environment(TIMESTEP);
      }

      // Destructor
      ~Engine ()
      {
        delete environment_;
      }


      // Get a pointer to the environment
      Environment* getEnvironment () const
      {
        return environment_;
      }

      // Return the current physical time
      Real getCurrentTime ()
      {
        return stepCounter_ * timestep_;
      }


      void runStep ();


      // TODO: TESTING
      ContactList* getContacts ()
      {
        return collisionDetector_.getContacts();
      }

      // TODO: TESTING
      Grid* getGrid ()
      {
        return collisionDetector_.getGrid();
      }


    private:

      Real                timestep_;

      int                 stepCounter_;

      Environment*        environment_;

      CollisionDetector   collisionDetector_;

      CollisionSolver     collisionSolver_;
  };


}


#endif // __ENGINE_H__
