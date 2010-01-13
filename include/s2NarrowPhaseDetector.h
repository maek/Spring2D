#ifndef __NARROW_PHASE_DETECTOR_H__
#define __NARROW_PHASE_DETECTOR_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2Shape.h"
#include "s2CircleShape.h"
#include "s2RectShape.h"
#include "s2PolygonShape.h"
#include "s2Contact.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The collision detector for the narrow phase
  class NarrowPhaseDetector
  {
    public:

      void findCollisions (const BodyList&, ContactList*);


    private:

      bool testCircleCircle (CircleShape*, CircleShape*, Contact*);

      bool testCircleRect (CircleShape*, RectShape*, Contact*);

      bool testCirclePolygon (CircleShape*, PolygonShape*, Contact*);

      bool testPolygonPolygon (Shape*, Shape*, Contact*);


      Vector2 GJK (Simplex&, const Shape*, const Shape*) const;

      void EPA (const Simplex&, const Shape*, const Shape*, Contact*) const;
  };





  // ---------------------------------------------------------------------------
  // A n-dimensional simplex (n = [0-2])
  // TODO: use pointer
  class Simplex
  {
    public:

      Vector2 vertices[3];

      Vector2 supportPointsA[3];

      Vector2 supportPointsB[3];

      Real    bCoordinates[3];


      int size;


    public:

      // Constructor
      Simplex () : size(0) { }


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
  // The priority-queue of Edges
  class EdgeQueue
  {
    public:

      // Return the number of elements in the queue
      int size () const
      {
        return edgeList_.size();
      }


      // Insert element
      void push (Edge* EDGE)
      {
        std::list<Edge*>::iterator edgeI;
        for (edgeI = edgeList_.begin(); edgeI != edgeList_.end(); ++edgeI)
        {
          // Keep a decreasing sorting (best (minor) = back)
          if (EDGE->key > (*edgeI)->key)
          {
            edgeList_.insert(edgeI, EDGE);
            return;
          }
        }

        edgeList_.push_back(EDGE);
      }

      // Access top element
      Edge* top () const
      {
        return edgeList_.back();
      }

      // Remove top element
      void pop ()
      {
        edgeList_.pop_back();
      }


      // Test if the given Edge is in the queue
      bool hasEdge (Edge* EDGE)
      {
        std::list<Edge*>::iterator edgeI;
        for (edgeI = edgeList_.begin(); edgeI != edgeList_.end(); ++edgeI)
        {
          if (
              ((*edgeI)->endpoints[0] == EDGE->endpoints[0] &&
               (*edgeI)->endpoints[1] == EDGE->endpoints[1]) ||
              ((*edgeI)->endpoints[0] == EDGE->endpoints[1] &&
               (*edgeI)->endpoints[1] == EDGE->endpoints[0])
             )
          {
            return true;
          }
        }

        return false;
      }


    private:

      std::list<Edge*> edgeList_;

  };


}


#endif // __NARROW_PHASE_DETECTOR_H__
