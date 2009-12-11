#include "../include/s2CollisionDetector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Find collisions
  void CollisionDetector::findCollisions (const BodyList& bodyList)
  {
    // Clear the contact set
    for (ContactSet::iterator contactI = contactSet_.begin();
        contactI != contactSet_.end(); ++contactI)
    {
      // Delete
      delete (*contactI);
    }
    contactSet_.clear();

    // Find the collision (BROAD phase)
    broadPhaseDetector_->findCollisions(bodyList, &contactSet_);

    // Find the collision (NARROW phase)
    narrowPhaseDetector_->findCollisions(bodyList, &contactSet_);
  }


}
