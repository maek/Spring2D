#include "../include/s2BroadPhaseDetector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Compute a conservative approximate list of collisions [DEFAULT = all pairs]
  void BroadPhaseDetector::findCollisions (
      const BodyList& bodyList, ContactList* contactList)
  {
    for (BodyList::const_iterator bodyI = bodyList.begin();
        bodyI != bodyList.end(); ++bodyI)
    {
      BodyList::const_iterator otherBodyI = bodyI;
      ++otherBodyI;
      for (; otherBodyI != bodyList.end(); ++otherBodyI)
      {
        // Insert all pairs
        contactList->push_back(new Contact((*bodyI), (*otherBodyI)));
      }
    }
  }


}
