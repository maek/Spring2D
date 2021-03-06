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

#include "../include/s2Grid.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Test for coarse collisions using a uniform grid
  void Grid::findCollisions (const BodyList& bodyList, ContactList* contactList)
  {
    BodyList      collisionBodies;
    Shape*        shape;
    const AABR*   aabr;
    Vector2       min;
    Vector2       max;
    int           index;


    // Clear the grid
    // TODO: OPTIMIZATION -> dense bit array for engaged cells
    for (int i = 0; i < size_; ++i)
    {
      cellList_[i].clear();
    }


    // Insert each body & test it on the fly
    for (BodyList::const_iterator bodyI = bodyList.begin();
        bodyI != bodyList.end(); ++bodyI)
    {
      collisionBodies.clear();

      // Get the shape & AABR
      shape = (*bodyI)->getShape();
      aabr  = shape->getAABR();

      // Update the AABR
      shape->updateAABR();

      min = (aabr->min - origin_) * (1. / cellSize_);
      max = (aabr->max - origin_) * (1. / cellSize_);

      for (int x = s2floor(min.x); x <= s2floor(max.x); ++x)
      {
        for (int y = s2floor(min.y); y <= s2floor(max.y); ++y)
        {
          // Check for valid grid positions
          if ((0 <= x && x < xExtent_) &&
              (0 <= y && y < yExtent_))
          {
            index = (x + xExtent_ * y);

            // Check for collisions
            for (BodyList::reverse_iterator bodyJ = cellList_[index].rbegin();
                bodyJ != cellList_[index].rend(); ++bodyJ)
            {
              // Check the AABR for collisions
              if (testAABRAABR(aabr, (*bodyJ)->getShape()->getAABR()) == true)
              {
                // Check already existing contact
                BodyList::iterator bodyH;
                for (bodyH  = collisionBodies.begin();
                     bodyH != collisionBodies.end(); ++bodyH)
                {
                  if (*bodyJ == *bodyH)
                  {
                    break;
                  }
                }

                // If there aren't previous contact between these two bodies
                if (bodyH == collisionBodies.end())
                {
                  // Save this contact information
                  collisionBodies.push_back(*bodyJ);

                  // Generate the collision
                  contactList->push_back(new Contact((*bodyI), (*bodyJ)));
                }
              }
            }

            // Insert the body
            cellList_[index].push_back(*bodyI);
          }
        }
      }

    }

  }


}
