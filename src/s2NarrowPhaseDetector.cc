#include "../include/s2NarrowPhaseDetector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Compute the exact list of collisions
  void NarrowPhaseDetector::findCollisions (
      const BodyList& bodyList, ContactSet* contactSet)
  {
    // TODO: retrieve shape information
    for (ContactSet::iterator contactI = contactSet->begin();
        contactI != contactSet->end(); ++contactI)
    {
      // Do an exact test
      if (testCircleCircle(
            static_cast<CircleShape*>((*contactI)->body[0]->getShape()),
            static_cast<CircleShape*>((*contactI)->body[1]->getShape())))
      {
        std::cout << "collision [NARROW]\n";
      }
    }

  }



  // ---------------------------------------------------------------------------
  // Check a circle against another one
  bool NarrowPhaseDetector::testCircleCircle (
      CircleShape* CIRCLE1, CircleShape* CIRCLE2)
  {
    Vector2 distance  = CIRCLE1->getBody()->getPosition() -
                        CIRCLE2->getBody()->getPosition();
    Real radiusSum    = CIRCLE1->getRadius() +
                        CIRCLE2->getRadius();

    // Comparing the squared distance avoid the computation of the radix
    if (distance.getSquaredMagnitude() < (radiusSum * radiusSum))
    {
      std::cout << "collision [CIRCLE - CIRCLE]\n";
      return true;
    }

    return false;
  }


  // ---------------------------------------------------------------------------
  // Check a circle against a rect
  bool NarrowPhaseDetector::testCircleRect (
      CircleShape* CIRCLE, RectShape* RECT)
  {
    return true;
  }


  // ---------------------------------------------------------------------------
  // Check a rect against another one
  bool NarrowPhaseDetector::testRectRect (
      RectShape* RECT1, RectShape* RECT2)
  {
    return true;
  }


}
