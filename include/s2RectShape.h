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
        assert(HALF_SIZE.x > 0 && HALF_SIZE.y > 0);
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


      // Build the associated AABB
      // TODO: check for correctness
      void buildAABB (Vector2* CENTER)
      {
        aabb_.center_ = CENTER;
        updateAABB();
      }

      // Update the associated AABB
      void updateAABB ()
      {
        // TODO: OPTIMIZATION -> check if is necessary to re-build the AABB
        //                       (only if rotating)
        // TODO: make not inline
        // TODO: check if directionX & directionY are normalized
        Vector2 directionX = body_->getOrientationMatrix() * Vector2::X;
        Vector2 directionY = body_->getOrientationMatrix() * Vector2::Y;
        Vector2 point = -halfSize_;
        aabb_.halfSize_.x = s2fabs(dotProduct(point, directionX));
        aabb_.halfSize_.y = s2fabs(dotProduct(point, directionY));

        point.x = -point.x;
        Real tx = s2fabs(dotProduct(point, directionX));
        Real ty = s2fabs(dotProduct(point, directionY));
        if (tx > aabb_.halfSize_.x)
        {
          aabb_.halfSize_.x = tx;
        }
        if (ty > aabb_.halfSize_.y)
        {
          aabb_.halfSize_.y = ty;
        }

        point.y = -point.y;
        tx = s2fabs(dotProduct(point, directionX));
        ty = s2fabs(dotProduct(point, directionY));
        if (tx > aabb_.halfSize_.x)
        {
          aabb_.halfSize_.x = tx;
        }
        if (ty > aabb_.halfSize_.y)
        {
          aabb_.halfSize_.y = ty;
        }

        point.x = -point.x;
        tx = s2fabs(dotProduct(point, directionX));
        ty = s2fabs(dotProduct(point, directionY));
        if (tx > aabb_.halfSize_.x)
        {
          aabb_.halfSize_.x = tx;
        }
        if (ty > aabb_.halfSize_.y)
        {
          aabb_.halfSize_.y = ty;
        }
      }


      // Return the furthest point along the given direction
      Vector2 getSupportPoint (const Vector2& DIRECTION)
      {
        // TODO: implement
        return Vector2::ZERO;
      }


    private:

      // TODO: change name -> extent
      Vector2   halfSize_;

  };


}


#endif // __RECT_SHAPE_H__
