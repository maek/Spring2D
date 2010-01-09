#ifndef __COLLISION_DETECTOR_H__
#define __COLLISION_DETECTOR_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Contact.h"
#include "s2Body.h"
#include "s2BroadPhaseDetector.h"
#include "s2NarrowPhaseDetector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The collisions detector
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


      // Return a pointer to the front contact list
      ContactList* getFrontContacts ()
      {
        return &contactListFront_;
      }

      // Return a pointer to the back contact list
      ContactList* getBackContacts()
      {
        return &contactListBack_;
      }


    private:

      BroadPhaseDetector   *broadPhaseDetector_;

      NarrowPhaseDetector  *narrowPhaseDetector_;


      ContactList           contactListFront_;

      ContactList           contactListBack_;
  };


}


#endif // __COLLISION_DETECTOR_H__
