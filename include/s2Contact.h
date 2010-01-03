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


      // Swap the body if the first is static
      void swap ()
      {
        Body* tbody = body[0];
        body[0] = body[1];
        body[1] = tbody;

        Vector2 tpoint = point[0];
        point[0] = point[1];
        point[1] = tpoint;
      }

  };



  // ---------------------------------------------------------------------------
  // The contact list
  typedef std::list<Contact*> ContactList;


}


#endif // __CONTACT_H__
