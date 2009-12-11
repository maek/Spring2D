#ifndef __COLLISION_DETECTOR_H__
#define __COLLISION_DETECTOR_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2BroadPhaseDetector.h"
#include "s2NarrowPhaseDetector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The collision detector
  class CollisionDetector
  {
    public:

      // Constructor
      CollisionDetector ()
      {
        broadPhaseDetector_   = new BroadPhaseDetector();
        narrowPhaseDetector_  = new NarrowPhaseDetector();
      }

      // Destructor
      ~CollisionDetector ()
      {
        delete broadPhaseDetector_;
        delete narrowPhaseDetector_;
      }


      void findCollisions (const BodyList&);


    private:

      BroadPhaseDetector   *broadPhaseDetector_;

      NarrowPhaseDetector  *narrowPhaseDetector_;


      ContactSet            contactSet_;

  };


}


#endif // __COLLISION_DETECTOR_H__
