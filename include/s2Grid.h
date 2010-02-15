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

#ifndef __GRID_H__
#define __GRID_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2Shape.h"
#include "s2AABR.h"
#include "s2BroadPhaseDetector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // A uniform grid for collision detection
  class Grid : public BroadPhaseDetector
  {
    public:

      // Constructor
      Grid (
          const int CELL_SIZE,
          const int X_EXTENT,
          const int Y_EXTENT,
          const Vector2& ORIGIN = Vector2::ZERO)
        : cellSize_(CELL_SIZE), xExtent_(X_EXTENT), yExtent_(Y_EXTENT),
          size_(X_EXTENT * Y_EXTENT), origin_(ORIGIN)
      {
        cellList_ = new BodyList[size_];
      }

      // Destructor
      ~Grid ()
      {
        delete[] cellList_;
      }


      void findCollisions (const BodyList&, ContactList*);


      // TODO: fix
    private:
    public:

      // TODO: const body vector
      BodyList*   cellList_;


      int         cellSize_;

      int         xExtent_;

      int         yExtent_;

      int         size_;

      Vector2     origin_;

  };


}


#endif // __GRID_H__
