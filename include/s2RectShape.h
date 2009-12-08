#ifndef __RECT_SHAPE_H__
#define __RECT_SHAPE_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2Shape.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The rectangle shape
  class RectShape : public Shape
  {
    public:

      // Constructor
      RectShape (const Vector2& HALF_SIZE)
        : halfSize_(HALF_SIZE)
      {
        // TODO: Exception ???
        assert(HALF_SIZE.x > 0 && HALF_SIZE.y > 0);
      }


      // Set the half size
      void setHalfSize (const Vector2& HALF_SIZE)
      {
        // TODO: Exception ???
        assert(HALF_SIZE.x > 0 && HALF_SIZE.y > 0);
        halfSize_ = HALF_SIZE;
      }

      // Get the half size
      Vector2 getHalfSize () const
      {
        return halfSize_;
      }


    private:

      Vector2   halfSize_;

  };


}


#endif // __RECT_SHAPE_H__
