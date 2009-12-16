#ifndef __NARROW_PHASE_DETECTOR_H__
#define __NARROW_PHASE_DETECTOR_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2Shape.h"
#include "s2Simplex.h"
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

      void findCollisions (const BodyList&, ContactSet*);


    private:

      bool testCircleCircle (CircleShape*, CircleShape*, Contact*);

      bool testCircleRect (CircleShape*, RectShape*, Contact*);

      bool testCirclePolygon (CircleShape*, PolygonShape*, Contact*);

      bool testPolygonPolygon (PolygonShape*, PolygonShape*, Contact*);


      Vector2 supportMapping (const CircleShape*, Vector2) const;

      Vector2 supportMapping (const PolygonShape*, Vector2) const;

      Vector2 supportMappingMinkowsky (
          const PolygonShape*, const PolygonShape*, Vector2) const;


      void EPA (const Simplex&, PolygonShape*, PolygonShape*, Contact*) const;

  };



  // ---------------------------------------------------------------------------
  class Entry
  {
    public:

      // The endpoints of the edge
      Vector2 y[2];

      // The point on the edge closest to the origin
      Vector2 v;

      // The distance of v from the origin
      Real key;


    public:

      bool build (const Vector2& A, const Vector2& B)
      {
        Vector2 O(0 ,0);
        Vector2 AB = B - A;

        // Project O onto AB, but deferring the division
        Real numerator = dotProduct(O - A, AB);
        // If O is outside segment & on the A side
        if (numerator < 0) return false;
        Real denominator = dotProduct(AB, AB);
        // If O is outside segment & on the B side
        if (numerator > denominator) return false;

        // O is on the segment
        y[0] = A;
        y[1] = B;
        v = (A + (numerator / denominator) * AB);
        key = v.getMagnitude();
        return true;
      }

  };



  // ---------------------------------------------------------------------------
  class PQComparison
  {
    public:

      bool operator() (const Entry* ENTRY1, const Entry* ENTRY2) const
      {
        // TODO: comment minimum
        return (ENTRY1->key > ENTRY2->key);
      }

  };


}


#endif // __NARROW_PHASE_DETECTOR_H__
