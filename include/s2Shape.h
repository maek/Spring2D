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

      enum ShapeType {CIRCLE, RECT};


    public:

      // Destructor
      virtual ~Shape () { }


      // Return a pointer to the body
      Body* getBody () const
      {
        return body_;
      }

      // Return the halfSize of the AABB
      AABB* getAABB ()
      {
        return &aabb_;
      }


      virtual ShapeType getType () const = 0;

      virtual void buildAABB (Vector2*) = 0;

      virtual void updateAABB () = 0;


    protected:

      Body*     body_;

      AABB      aabb_;

  };


}


#endif // __SHAPE_H__
