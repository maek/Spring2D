#ifndef __NARROW_PHASE_DETECTOR_H__
#define __NARROW_PHASE_DETECTOR_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2CircleShape.h"
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

      bool testCircleCircle (CircleShape*, CircleShape*);

      bool testCircleRect (CircleShape*, RectShape*);

      bool testRectRect (RectShape*, RectShape*);

  };


}


#endif // __NARROW_PHASE_DETECTOR_H__
