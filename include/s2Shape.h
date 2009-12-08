#ifndef __SHAPE_H__
#define __SHAPE_H__

#include "s2Settings.h"
#include "s2Body.h"
#include "s2AABB.h"

namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The shape of the body
  class Shape
  {
    public:

      friend class Body;


    public:

      // Destructor
      virtual ~Shape () { }


      // Return a pointer to the body
      Body* getBody () const
      {
        return body_;
      }

      // Return the halfSize of the AABB
      Vector2 getAABB () const
      {
        return aabb_.halfSize_;
      }


      virtual void buildAABB () = 0;

      virtual void updateAABB () = 0;


    protected:

      Body*     body_;

      AABB      aabb_;

  };


}


#endif // __SHAPE_H__
