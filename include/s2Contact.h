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
        : point(Vector2::ZERO), normal(Vector2::ZERO), penetrationDepth(0)
      {
        body[0] = BODY1;
        body[1] = BODY2;
      }


    public:

      Body     *body[2];


      Vector2   point;

      Vector2   normal;

      Real      penetrationDepth;

  };



  // ---------------------------------------------------------------------------
  // The compare function for the set of Contact
  class ContactCompare
  {
    public:

      bool operator() (Contact* CONTACT1, Contact* CONTACT2) const
      {
        return (CONTACT1->penetrationDepth < CONTACT2->penetrationDepth);
      }

  };



  // ---------------------------------------------------------------------------
  // The contact set
  typedef std::multiset<Contact*, ContactCompare> ContactSet;


}


#endif // __CONTACT_H__
