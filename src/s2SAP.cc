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

#include "../include/s2SAP.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Test for coarse collisions using the Sweep And Prune method
  void SAP::findCollisions (const BodyList& bodyList, ContactList* contactList)
  {
    // Update all AABR
    for (BodyList::const_iterator bodyI = bodyList.begin();
        bodyI != bodyList.end(); ++bodyI)
    {
      (*bodyI)->getShape()->updateAABR();
    }


    // Check for new/delete body
    // TODO: better test
    if (xAxis_.size() != bodyList.size())
    {
      const AABR*   aabr;
      Element*      element;

      // Clear the x axis list
      xAxis_.clear();

      // Free the memory
      for (ElementVector::iterator elementI = yAxis_.begin();
          elementI != yAxis_.end(); ++elementI)
      {
        delete (*elementI);
      }
      yAxis_.clear();


      // Rebuild the axes lists
      for (BodyList::const_iterator bodyI = bodyList.begin();
          bodyI != bodyList.end(); ++bodyI)
      {
        aabr = (*bodyI)->getShape()->getAABR();
        element = new Element(*bodyI,
            &aabr->min.x, &aabr->max.x, &aabr->min.y, &aabr->max.y);

        xAxis_.push_back(element);
        yAxis_.push_back(element);
      }


      // Sort the x axis (quicksort)
      std::sort(xAxis_.begin(), xAxis_.end(), xElementCompare);

      // Sort the y axis (quicksort)
      std::sort(yAxis_.begin(), yAxis_.end(), yElementCompare);
    }
    else
    {
      // Sort the x axis (insertion sort)
      insertionSort(xAxis_, 0);

      // Sort the y axis (insertion sort)
      insertionSort(yAxis_, 1);
    }


    // Find overlapping intervals on the x axis
    for (int i = 0, nElement = xAxis_.size(); i < nElement - 1; ++i)
    {
      for (int j = i + 1; j < nElement; ++j)
      {
        if (*xAxis_[i]->max[0] >= *xAxis_[j]->min[0])
        {
          xAxis_[i]->overlappingList.push_back(xAxis_[j]->body);
          xAxis_[j]->overlappingList.push_back(xAxis_[i]->body);
        }
        else
        {
          break;
        }
      }

    }

    // Find overlapping intervals on the y axis & generate collision data
    for (int i = 0, nElement = yAxis_.size(); i < nElement - 1; ++i)
    {
      for (int j = i + 1; j < nElement; ++j)
      {
        if (*yAxis_[i]->max[1] >= *yAxis_[j]->min[1])
        {
          for (ConstBodyVector::const_iterator bodyJ = yAxis_[i]->overlappingList.begin();
              bodyJ != yAxis_[i]->overlappingList.end(); ++bodyJ)
          {
            if (*bodyJ == yAxis_[j]->body)
            {
              // Generate the collision
              contactList->push_back(new Contact(
                    const_cast<Body*>(yAxis_[i]->body),
                    const_cast<Body*>(*bodyJ)));
              // TODO: remove the overlapping body from list ???
              break;
            }
          }
        }
        else
        {
          break;
        }
      }

      yAxis_[i]->overlappingList.clear();

    }

  }



  // ---------------------------------------------------------------------------
  // The compare function for the element in the x axis lists
  bool xElementCompare (Element* ELEMENT1, Element* ELEMENT2)
  {
    return (ELEMENT1->min[0] < ELEMENT2->min[0]);
  }



  // ---------------------------------------------------------------------------
  // The compare function for the element in the y axis lists
  bool yElementCompare (Element* ELEMENT1, Element* ELEMENT2)
  {
    return (ELEMENT1->min[1] < ELEMENT2->min[1]);
  }



  // ---------------------------------------------------------------------------
  // Sort the given element vector with insertion sort
  void insertionSort (ElementVector& axis, const int index)
  {
    int nElement = axis.size();

    for (int i = 1; i < nElement; ++i)
    {
      for (int j = i - 1; j >= 0; --j)
      {
        if (*axis[i]->min[index] < *axis[j]->min[index])
        {
          std::swap(axis[i], axis[j]);
        }
      }
    }

  }


}
