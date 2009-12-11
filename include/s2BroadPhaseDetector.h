#ifndef __BROAD_PHASE_DETECTOR_H__
#define __BROAD_PHASE_DETECTOR_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2Contact.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The collision detector for the broad phase
  class BroadPhaseDetector
  {
    public:

      void findCollisions (const BodyList&, ContactSet*);

  };


}


#endif // __BROAD_PHASE_DETECTOR_H__
