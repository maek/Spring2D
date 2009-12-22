#ifndef __NARROW_PHASE_DETECTOR_H__
#define __NARROW_PHASE_DETECTOR_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2Shape.h"
#include "s2CircleShape.h"
#include "s2PolygonShape.h"
#include "s2RectShape.h"
#include "s2Contact.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The collision detector for the narrow phase
  class NarrowPhaseDetector
  {
    public:

      void findCollisions (const BodyList&, ContactSet*);


    private:

      bool testCircleCircle (CircleShape*, CircleShape*, Contact*);

      bool testCirclePolygon (CircleShape*, PolygonShape*, Contact*);

      bool testCircleRect (CircleShape*, RectShape*, Contact*);

      bool testPolygonPolygon (Shape*, Shape*, Contact*);


      Vector2 GJK (Simplex&, const Shape*, const Shape*) const;

      void EPA (const Simplex&, const Shape*, const Shape*, Contact*) const;

  };





  // ---------------------------------------------------------------------------
  // A n-dimensional simplex (n = [0~2])
  // TODO: use pointer
  class Simplex
  {
    public:

      Vector2 vertices[3];

      Vector2 supportPointsA[3];

      Vector2 supportPointsB[3];

      Real    bCoordinates[3];


      int size;

      bool includeOrigin;


    public:

      // Constructor
      Simplex () : size(0), includeOrigin(false) { }


      // Assignment
      Simplex& operator= (const Simplex& S)
      {
        size = S.size;
        switch (size)
        {
          case 3:
            vertices[3] = S.vertices[3];
          case 2:
            vertices[2] = S.vertices[2];
          case 1:
            vertices[1] = S.vertices[1];
          default:
            break;
        }

        return *this;
      }


      // Add a vertex to the simplex
      void addVertex (const Vector2& VERTEX,
          const Vector2& SUPPORT_POINT_A, const Vector2& SUPPORT_POINT_B)
      {
        assert(size < 3);
        vertices[size]       = VERTEX;
        supportPointsA[size] = SUPPORT_POINT_A;
        supportPointsB[size] = SUPPORT_POINT_B;
        ++size;
      }

      // Test if the given vertex is in the simplex
      bool hasVertex (const Vector2& VERTEX)
      {
        for (int i = 0; i < size; ++i)
        {
          if (VERTEX == vertices[i])
            return true;
        }

        return false;
      }


      // Return the point in A associated to the closest point to origin
      Vector2 getPointA () const
      {
        Vector2 pointA = Vector2::ZERO;
        for (int i = 0; i < size; ++i)
        {
          pointA += supportPointsA[i] * bCoordinates[i];
        }
        return pointA;
      }

      // Return the point in B associated to the closest point to origin
      Vector2 getPointB () const
      {
        Vector2 pointB = Vector2::ZERO;
        for (int i = 0; i < size; ++i)
        {
          pointB += supportPointsB[i] * bCoordinates[i];
        }
        return pointB;
      }


      Vector2 calculateClosestPointToOrigin ();

  };





  // ---------------------------------------------------------------------------
  // The edge for the EPA
  class Edge
  {
    public:

      Vector2 endpoints[2];

      Vector2 supportPointsA[2];

      Vector2 supportPointsB[2];


      Vector2 v;

      Real key;

      Real t;


    public:

      bool construct (
          const Vector2&, const Vector2&, const Vector2&,
          const Vector2&, const Vector2&, const Vector2&);

  };





  // ---------------------------------------------------------------------------
  // The compare function for the priority-queue of Edges
  class EdgeCompare
  {
    public:

      bool operator() (const Edge* EDGE1, const Edge* EDGE2) const
      {
        // Keep a decreasing sorting (best = minimum)
        return (EDGE1->key > EDGE2->key);
      }

  };


}


#endif // __NARROW_PHASE_DETECTOR_H__
