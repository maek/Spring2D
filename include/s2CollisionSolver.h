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

#ifndef __COLLISION_SOLVER_H__
#define __COLLISION_SOLVER_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Contact.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The collisions solver
  class CollisionSolver
  {
    public:

      void solveCollisions (ContactList*, bool constraint = false);


    private:

      void preprocessContacts (ContactList*);

      void preprocessConstraintsContacts (ContactList*);


      void solveInterpenetration (Contact*);

      void solveVelocity (Contact*);

  };


}


#endif // __COLLISION_SOLVER_H__
