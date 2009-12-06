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


      // Get the body pointer
      Body* getBody () const
      {
        return body_;
      }

      // Destructor
      virtual ~CollisionPrimitive () { }

    protected:



    protected:

      Body*     body_;

  };


}


#endif // __COLLISION_PRIMITIVE_H__
