#include "../include/s2CollisionDetector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Find collisions
  void CollisionDetector::findCollisions (const BodyList& bodyList)
  {
    // Clear the back contact list
    for (ContactList::iterator contactI = contactListBack_.begin();
        contactI != contactListBack_.end(); ++contactI)
    {
      // Delete
      delete (*contactI);
    }
    contactListBack_ = contactListFront_;
    contactListFront_.clear();

    // Find the collisions (BROAD phase)
    broadPhaseDetector_->findCollisions(bodyList, &contactListFront_);

    // Find the collisions (NARROW phase)
    narrowPhaseDetector_->findCollisions(bodyList, &contactListFront_);
  }


}
