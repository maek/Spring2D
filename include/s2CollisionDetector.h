#ifndef __COLLISION_DETECTOR_H__
#define __COLLISION_DETECTOR_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Contact.h"
#include "s2Body.h"
#include "s2BroadPhaseDetector.h"
#include "s2Grid.h"
#include "s2NarrowPhaseDetector.h"
#include "s2ConstraintsRegister.h"


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
        //broadPhaseDetector_   = new BroadPhaseDetector();
        broadPhaseDetector_   = new Grid(100, 10, 10);
        narrowPhaseDetector_  = new NarrowPhaseDetector();
      }

      // Destructor
      ~CollisionDetector ()
      {
        delete broadPhaseDetector_;
        delete narrowPhaseDetector_;
      }


      void findCollisions (const BodyList&);

      void checkConstraints (ConstraintsList*);


      // Return a pointer to the contact list
      ContactList* getContacts ()
      {
        return &contactList_;
      }

      // Return a pointer to the constraints contacts list
      ContactList* getConstraintsContacts ()
      {
        return &constraintsContactsList_;
      }

      // TODO: TESTING
      Grid* getGrid ()
      {
        return static_cast<Grid*>(broadPhaseDetector_);
      }


    private:

      BroadPhaseDetector   *broadPhaseDetector_;

      NarrowPhaseDetector  *narrowPhaseDetector_;


      ContactList           contactList_;

      ContactList           constraintsContactsList_;

  };


}


#endif // __COLLISION_DETECTOR_H__
