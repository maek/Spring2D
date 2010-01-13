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
      RectShape (const Vector2& EXTENT, const Real DENSITY = 1)
        : extent_(EXTENT)
      {
        if (EXTENT.x <= 0 || EXTENT.y <= 0)
        {
          valid_ = false;
          return;
        }

        if (DENSITY <= 0)
        {
          valid_ = false;
          return;
        }

        density_  = DENSITY;
        area_     = 4 * EXTENT.x * EXTENT.y;

        valid_ = true;
      }


      // Get the extent
      Vector2 getExtent () const
      {
        return extent_;
      }


      // Get the shape type
      ShapeType getType () const
      {
        return Shape::RECT;
      }


      void buildAABB (Vector2*);

      void updateAABB ();


      // Calculate the moment of inertia
      // (m * (w^2 + h^2)) / 12
      Real calculateMomentOfInertia () const
      {
        return area_ * density_ *
          (s2sqr(2 * extent_.x) + s2sqr(2 * extent_.y)) / 12;
      }


      // Return any vertex as the initial support point
      Vector2 getSupportPoint0 () const
      {
        return body_->getOrientationMatrix() * extent_ + body_->getPosition();
      }


      Vector2 getSupportPoint (const Vector2&) const;


    private:

      Vector2 extent_;

  };


}


#endif // __RECT_SHAPE_H__
