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

#include "../include/s2Environment.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Create a new body in the environment
  Body* Environment::createBody (
      Shape* SHAPE,
      const bool STATIC,
      const Vector2& POSITION,
      const Vector2& VELOCITY,
      const Vector2& ACCELERATION,
      const Complex& ORIENTATION,
      const Real ROTATION)
  {
    // Check if the shape is valid
    if (SHAPE->isValid() == false)
    {
      return 0;
    }

    Body *body = new Body(
        timestep_,
        SHAPE,
        STATIC,
        POSITION,
        VELOCITY,
        ACCELERATION,
        ORIENTATION,
        ROTATION);

    bodyList_.push_back(body);

    return body;
  }


  // ---------------------------------------------------------------------------
  // Destroy the specified body
  void Environment::destroyBody (Body* body)
  {
    bodyList_.remove(body);
    delete body;
  }


  // ---------------------------------------------------------------------------
  // Update positions & velocities for each dynamic body
  void Environment::integrateBodies ()
  {
    for (BodyList::iterator bodyI = bodyList_.begin(); bodyI != bodyList_.end();
        ++bodyI)
    {
      // Integrate
      (*bodyI)->integrate();
    }
  }


}
