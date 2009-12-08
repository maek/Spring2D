#ifndef __RECT_H__
#define __RECT_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2CollisionPrimitive.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The circle primitive
  class Rect : public CollisionPrimitive
  {
    public:

      // Constructor
      Rect (const Vector2& HALF_SIZE = Vector2::XY)
        : CollisionPrimitive(), halfSize_(HALF_SIZE)
      {
        // TODO: Exception ???
        if (halfSize_.x <= 0)
        {
          halfSize_.x = 1;
        }
        if (halfSize_.y <= 0)
        {
          halfSize_.y = 1;
        }

        radius_ = halfSize_.magnitude();
      }


      // Set the primitive half size
      void setHalfSize (const Vector2& HALF_SIZE)
      {
        if (HALF_SIZE.x <= 0 || HALF_SIZE.y <= 0)
        {
          return;
        }

        halfSize_ = HALF_SIZE;
      }

      // Get the primitive half size
      Vector2 getHalfSize () const
      {
        return halfSize_;
      }


    private:

      Vector2   halfSize_;

  };


}


#endif // __RECT_H__
