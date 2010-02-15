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

#ifndef __SAP_H__
#define __SAP_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2Shape.h"
#include "s2AABR.h"
#include "s2BroadPhaseDetector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The element for the 2 axes lists
  // TODO: make it static (only for this file)
  class Element
  {
    public:

      const Body*       body;

      ConstBodyVector   overlappingList;


      const Real*       min[2];

      const Real*       max[2];


    public:

      // Constructor
      Element (const Body* BODY,
          const Real* MIN1, const Real* MAX1,
          const Real* MIN2, const Real* MAX2)
        : body(BODY)
      {
        min[0] = MIN1;
        min[1] = MIN2;
        max[0] = MAX1;
        max[1] = MAX2;
      }

  };



  bool xElementCompare (Element*, Element*);

  bool yElementCompare (Element*, Element*);



  // ---------------------------------------------------------------------------
  // The element list
  typedef std::vector<Element*> ElementVector;



  void insertionSort (ElementVector&, const int);





  // ---------------------------------------------------------------------------
  // The Sweep And Prune collision detection
  class SAP : public BroadPhaseDetector
  {
    public:

      void findCollisions (const BodyList&, ContactList*);


    private:

      ElementVector xAxis_;

      ElementVector yAxis_;

  };


}


#endif // __SAP_H__
