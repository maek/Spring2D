#ifndef __COLLISION_NARROW_H__
#define __COLLISION_NARROW_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2CircleShape.h"
#include "s2RectShape.h"
#include "s2Contact.h"


namespace Spring2D
{
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
  // The collision detector for the narrow phase
  class CollisionNarrow
  {
    public:

      //typedef std::set<Contact*, ContactCompare> ContactSet;


    public:

      bool testCircleCircle (CircleShape*, CircleShape*);

      bool testCircleRect (CircleShape*, RectShape*);

      bool testRectRect (RectShape*, RectShape*);


    private:

      std::set<Contact*, ContactCompare> contactSet_;
      //ContactSet  contactSet_;

  };


}


#endif // __COLLISION_NARROW_H__
