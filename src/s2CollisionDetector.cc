#include "../include/s2CollisionDetector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Find collisions
  void CollisionDetector::findCollisions (const BodyList& bodyList)
  {
    // Clear the contact list
    if (contactList_.size() > 0)
    {
      for (ContactList::iterator contactI = contactList_.begin();
          contactI != contactList_.end(); ++contactI)
      {
        delete (*contactI);
      }
      contactList_.clear();
    }

    // Find the collisions (BROAD phase)
    broadPhaseDetector_->findCollisions(bodyList, &contactList_);

    // Find the collisions (NARROW phase)
    narrowPhaseDetector_->findCollisions(bodyList, &contactList_);
  }



  // ---------------------------------------------------------------------------
  // Find violated constraints
  void CollisionDetector::checkConstraints (ConstraintsList* constraintsList)
  {
    Vector2 point[2];
    Real    distance;

    // Clear the contact list
    if (constraintsContactsList_.size() > 0)
    {
      for (ContactList::iterator contactI = constraintsContactsList_.begin();
          contactI != constraintsContactsList_.end(); ++contactI)
      {
        delete (*contactI);
      }
      constraintsContactsList_.clear();
    }

    // Check all constraints
    for (ConstraintsList::iterator constraintI = constraintsList->begin();
        constraintI != constraintsList->end(); ++constraintI)
    {
      point[0] = (*constraintI)->body[0]->transformWorld(
          (*constraintI)->point[0]);
      point[1] = (*constraintI)->body[1]->transformWorld(
          (*constraintI)->point[1]);
      distance = (point[1] - point[0]).getMagnitude();

      // Extension
      if (distance > (*constraintI)->length)
      {
        Contact* contact =
          new Contact((*constraintI)->body[0], (*constraintI)->body[1]);
        contact->point[0] = point[0];
        contact->point[1] = point[1];
        contact->penetrationDepth = distance - (*constraintI)->length;

        constraintsContactsList_.push_back(contact);
      }
      // Compression
      else if (distance < (*constraintI)->length &&
               (*constraintI)->cable == false)
      {
        Contact* contact =
          new Contact((*constraintI)->body[0], (*constraintI)->body[1]);
        contact->point[0] = point[1];
        contact->point[1] = point[0];
        contact->penetrationDepth = (*constraintI)->length - distance;

        constraintsContactsList_.push_back(contact);
      }

    }

  }

}
