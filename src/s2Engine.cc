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

#include "../include/s2Engine.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Run a complete physics step
  void Engine::runStep()
  {
    // Compute the net forces & net torques
    environment_->handleDynamics();

    // Update positions & velocities
    environment_->integrateBodies();


    // Find violated constraints
    collisionDetector_.checkConstraints(environment_->getConstraints());

    // Solve violated constraints
    collisionSolver_.solveCollisions(
        collisionDetector_.getConstraintsContacts(), true);

    // Find collisions
    collisionDetector_.findCollisions(environment_->getBodyList());

    // Solve collisions
    collisionSolver_.solveCollisions(collisionDetector_.getContacts());


    // Increment the step counter
    stepCounter_++;
  }
}
