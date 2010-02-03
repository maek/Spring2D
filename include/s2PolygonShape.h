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
      PolygonShape (const int N_VERTICES, const Vector2* VERTICES,
          const Real DENSITY = 1)
        : nVertices_(N_VERTICES)
      {
        if (N_VERTICES < 3)
        {
          valid_ = false;
          return;
        }

        if (DENSITY <= 0)
        {
          valid_ = false;
          return;
        }

        // Convexity test
        if (convexityTest(N_VERTICES, VERTICES) == false)
        {
          valid_ = false;
          return;
        }


        vertices_ = new Vector2[N_VERTICES];
        for (int i = 0; i < N_VERTICES; ++i)
        {
          vertices_[i] = VERTICES[i];
        }

        // Calculate the area
        area_ =
          VERTICES[N_VERTICES - 1].x * VERTICES[0].y -
          VERTICES[0].x * VERTICES[N_VERTICES - 1].y;
        for (int i = 0; i < N_VERTICES - 1; ++i)
        {
          area_ +=
            VERTICES[i].x * VERTICES[i + 1].y -
            VERTICES[i + 1].x * VERTICES[i].y;
        }
        area_ /= 2.;

        // Counter-clockwise test
        if (area_ <= 0)
        {
          valid_ = false;
          return;
        }

        density_  = DENSITY;
        iMass_    = 1. / (area_ * density_);
        updateInverseMomentOfInertia();

        valid_ = true;
      }


      // Desstructor
      ~PolygonShape ()
      {
        delete vertices_;
      }


      // Get the number of vertices
      int getNVertices() const
      {
        return nVertices_;
      }

      // Get the vertices
      const Vector2* getVertices() const
      {
        return vertices_;
      }


      // Get the shape type
      ShapeType getType () const
      {
        return POLYGON;
      }


      void updateAABR ();


      void updateInverseMomentOfInertia ();


      // Return any vertex as the initial support point
      Vector2 getSupportPoint0 () const
      {
        return body_->getOrientationMatrix() * vertices_[0] +
          body_->getPosition();
      }

      Vector2 getSupportPoint (const Vector2&) const;


    private:

      int       nVertices_;

      Vector2*  vertices_;


    private:

      bool convexityTest (const int, const Vector2*) const;

  };


}


#endif // __POLYGON_SHAPE_H__
