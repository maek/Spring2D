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

  };


}


#endif // __NARROW_PHASE_DETECTOR_H__
