#include "../include/s2CollisionDetector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Find collisions
  void CollisionDetector::findCollisions (const BodyList& bodyList)
  {
    // Clear the contact list
    for (ContactList::iterator contactI = contactList_.begin();
        contactI != contactList_.end(); ++contactI)
    {
      // Delete
      delete (*contactI);
    }
    contactList_.clear();

    // Find the collision (BROAD phase)
    broadPhaseDetector_->findCollisions(bodyList, &contactList_);

    // Find the collision (NARROW phase)
    narrowPhaseDetector_->findCollisions(bodyList, &contactList_);
  }


}
