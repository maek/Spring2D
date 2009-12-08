#ifndef __COLLISION_PRIMITIVE_H__
#define __COLLISION_PRIMITIVE_H__

#include "s2Settings.h"
#include "s2Body.h"

namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The collision geometry
  class CollisionPrimitive
  {
    public:

      friend class Body;


    public:

      // Destructor
      virtual ~CollisionPrimitive () { }

      Body* getBody () const
      {
        return body_;
      }

      Real getRadius () const
      {
        return radius_;
      }


    protected:

      Body*     body_;

      Real      radius_;

  };


}


#endif // __COLLISION_PRIMITIVE_H__
