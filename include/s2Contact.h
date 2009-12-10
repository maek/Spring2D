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

      Body     *body[2];


      Vector2   point;

      Vector2   normal;

      Real      penetrationDepth;

  };


}


#endif // __CONTACT_H__
