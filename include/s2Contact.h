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

#ifndef __CONTACT_H__
#define __CONTACT_H__

#include "s2Settings.h"
#include "s2Body.h"
#include "s2Math.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The contact for collision
  class Contact
  {
    public:

      Body*     body[2];


      Vector2   point[2];

      Real      penetrationDepth;

      Vector2   normal;

      Vector2   tangent;

      Real      restitution;

      Real      friction;

      Vector2   relativeContactPoint[2];

      Real      closingVelocity;

      Real      slidingVelocity;

      Real      linearInertia[2];

      Real      angularInertia[2];


    public:

      // Constructor
      Contact (Body* BODY1, Body* BODY2)
      {
        body[0] = BODY1;
        body[1] = BODY2;
      }


      // Swap the data
      void swap ()
      {
        // Body
        Body* tbody = body[0];
        body[0]     = body[1];
        body[1]     = tbody;

        // Contact point
        Vector2 tpoint = point[0];
        point[0]       = point[1];
        point[1]       = tpoint;

        // The penetrationDepth is the same
      }

  };





  // ---------------------------------------------------------------------------
  // The contact list
  typedef std::list<Contact*> ContactList;


}


#endif // __CONTACT_H__
