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

      Contact (Body* BODY1, Body* BODY2)
      {
        body[0] = BODY1;
        body[1] = BODY2;
      }


    public:

      Body     *body[2];


      Vector2   point[2];

      Real      penetrationDepth;

      Real      restitution;


      Vector2   normal;

      Vector2   relativeContactPoint[2];

      Real      closingVelocity;

      Real      linearInertia[2];

      Real      angularInertia[2];

  };



  // ---------------------------------------------------------------------------
  // The contact list
  typedef std::list<Contact*> ContactList;


}


#endif // __CONTACT_H__
