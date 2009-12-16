#ifndef __POLYGON_SHAPE_H__
#define __POLYGON_SHAPE_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2Shape.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The polygon shape
  class PolygonShape : public Shape
  {
    public:

      // Constructor
      PolygonShape (const unsigned N_VERTICES, Vector2* VERTICES)
        : nVertices_(N_VERTICES)
      {
        assert(N_VERTICES >= 3);

        // TODO: add the convexity test

        vertices_ = new Vector2[N_VERTICES];
        for (unsigned i = 0; i < nVertices_; ++i)
        {
          vertices_[i] = VERTICES[i];
        }
      }


      // Get the number of vertices
      unsigned getNVertices() const
      {
        return nVertices_;
      }

      // Get the vertices
      Vector2* getVertices() const
      {
        return vertices_;
      }


      // Get the shape type
      ShapeType getType () const
      {
        return POLYGON;
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
        Real tx;
        Real ty;
        // TODO: OPTIMIZATION -> check if is necessary to re-build the AABB
        //                       (only if rotating)
        // TODO: check if directionX & directionY are normalized
        Vector2 directionX = body_->getOrientationMatrix() * Vector2::X;
        Vector2 directionY = body_->getOrientationMatrix() * Vector2::Y;

        aabb_.halfSize_.x = s2fabs(dotProduct(vertices_[0], directionX));
        aabb_.halfSize_.y = s2fabs(dotProduct(vertices_[0], directionY));

        for (unsigned i = 1; i < nVertices_; ++i)
        {
          tx = s2fabs(dotProduct(vertices_[i], directionX));
          ty = s2fabs(dotProduct(vertices_[i], directionY));

          if (tx > aabb_.halfSize_.x)
          {
            aabb_.halfSize_.x = tx;
          }
          if (ty > aabb_.halfSize_.y)
          {
            aabb_.halfSize_.y = ty;
          }

        }

      }


    private:

      unsigned  nVertices_;

      Vector2*  vertices_;

  };


}


#endif // __POLYGON_SHAPE_H__
