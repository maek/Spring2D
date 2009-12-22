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
        aabb_.halfSize_.x = s2fabs(dot(point, directionX));
        aabb_.halfSize_.y = s2fabs(dot(point, directionY));

        point.x = -point.x;
        Real tx = s2fabs(dot(point, directionX));
        Real ty = s2fabs(dot(point, directionY));
        if (tx > aabb_.halfSize_.x)
        {
          aabb_.halfSize_.x = tx;
        }
        if (ty > aabb_.halfSize_.y)
        {
          aabb_.halfSize_.y = ty;
        }

        point.y = -point.y;
        tx = s2fabs(dot(point, directionX));
        ty = s2fabs(dot(point, directionY));
        if (tx > aabb_.halfSize_.x)
        {
          aabb_.halfSize_.x = tx;
        }
        if (ty > aabb_.halfSize_.y)
        {
          aabb_.halfSize_.y = ty;
        }

        point.x = -point.x;
        tx = s2fabs(dot(point, directionX));
        ty = s2fabs(dot(point, directionY));
        if (tx > aabb_.halfSize_.x)
        {
          aabb_.halfSize_.x = tx;
        }
        if (ty > aabb_.halfSize_.y)
        {
          aabb_.halfSize_.y = ty;
        }
      }


      // Return any vertex as the initial support point
      Vector2 getSupportPoint0 () const
      {
        return body_->getPosition() +
          body_->getOrientationMatrix() * Vector2(-halfSize_.x, -halfSize_.y);
      }


      // Return the furthest point along the given direction
      Vector2 getSupportPoint (const Vector2& DIRECTION) const
      {
        // Transform the direction in the local coordinates
        Vector2 direction = body_->getOrientationMatrix().getInverse() *
          DIRECTION;

        Vector2 points[4];
        points[0] = Vector2(-halfSize_.x, -halfSize_.y);
        points[1] = Vector2( halfSize_.x, -halfSize_.y);
        points[2] = Vector2( halfSize_.x,  halfSize_.y);
        points[3] = Vector2(-halfSize_.x,  halfSize_.y);

        int index = 0;
        Real projection   = dot(points[0], direction);

        Real tprojection  = dot(points[1], direction);
        if (projection < tprojection)
        {
          index = 1;
          projection = tprojection;
        }

        tprojection  = dot(points[2], direction);
        if (projection < tprojection)
        {
          index = 2;
          projection = tprojection;
        }

        tprojection  = dot(points[3], direction);
        if (projection < tprojection)
        {
          index = 3;
          projection = tprojection;
        }


        body_->transformWorld(&points[index]);
        return points[index];
      }


    private:

      // TODO: change name -> extent
      Vector2   halfSize_;

  };


}


#endif // __RECT_SHAPE_H__
