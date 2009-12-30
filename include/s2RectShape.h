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
      RectShape (const Vector2& HALF_SIZE, const Real DENSITY = 1)
        : halfSize_(HALF_SIZE)
      {
        assert(HALF_SIZE.x > 0 && HALF_SIZE.y > 0);
        assert(DENSITY > 0);

        density_ = DENSITY;
        area_ = 4 * HALF_SIZE.x * HALF_SIZE.y;
      }


      // Set the half size
      void setHalfSize (const Vector2& HALF_SIZE)
      {
        assert(HALF_SIZE.x > 0 && HALF_SIZE.y > 0);
        halfSize_ = HALF_SIZE;
      }

      // Get the half size
      Vector2 getHalfSize () const
      {
        return halfSize_;
      }


      // Get the shape type
      ShapeType getType () const
      {
        return RECT;
      }


      void buildAABB (Vector2*);

      void updateAABB ();

      Real calculateMomentOfInertia () const;

      Vector2 getSupportPoint0 () const;

      Vector2 getSupportPoint (const Vector2&) const;


    private:

      // TODO: change name -> extent
      Vector2   halfSize_;

  };


}


#endif // __RECT_SHAPE_H__
