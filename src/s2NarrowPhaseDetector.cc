#include "../include/s2NarrowPhaseDetector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Compute the exact list of collisions
  void NarrowPhaseDetector::findCollisions (
      const BodyList& bodyList, ContactSet* contactSet)
  {
    bool collision;
    ContactSet::iterator contactI = contactSet->begin();
    ContactSet::iterator tcontactI;

    while (contactI != contactSet->end())
    {
      // Do an exact test
      switch ((*contactI)->body[0]->getShape()->getType())
      {

        case Shape::CIRCLE:
          switch ((*contactI)->body[1]->getShape()->getType())
          {
            case Shape::CIRCLE:  // CIRCLE - CIRCLE
              collision = testCircleCircle(
                  static_cast<CircleShape*>((*contactI)->body[0]->getShape()),
                  static_cast<CircleShape*>((*contactI)->body[1]->getShape()),
                  (*contactI));
              break;

            case Shape::RECT:    // CIRCLE - RECT
              collision = testCircleRect(
                  static_cast<CircleShape*>((*contactI)->body[0]->getShape()),
                  static_cast<RectShape*>((*contactI)->body[1]->getShape()),
                  (*contactI));
              break;
          }
          break;


        case Shape::RECT:
          switch ((*contactI)->body[1]->getShape()->getType())
          {
            case Shape::CIRCLE:  // CIRCLE - RECT
              collision = testCircleRect(
                  static_cast<CircleShape*>((*contactI)->body[1]->getShape()),
                  static_cast<RectShape*>((*contactI)->body[0]->getShape()),
                  (*contactI));
              break;

            case Shape::RECT:    // RECT - RECT
              collision = testRectRect(
                  static_cast<RectShape*>((*contactI)->body[0]->getShape()),
                  static_cast<RectShape*>((*contactI)->body[1]->getShape()),
                  (*contactI));
              break;
          }
          break;
      }


      if (collision == false)
      {
        // Delete the contact data
        delete *contactI;

        // Remove the contact
        tcontactI = contactI;
        ++contactI;
        contactSet->erase(tcontactI);
      }
      else
      {
        ++contactI;
      }

    }

  }



  // ---------------------------------------------------------------------------
  // Check a circle against another one
  bool NarrowPhaseDetector::testCircleCircle (
      CircleShape* CIRCLE1, CircleShape* CIRCLE2, Contact* contact)
  {
    Vector2 circle1Center = CIRCLE1->getBody()->getPosition();
    Vector2 circle2Center = CIRCLE2->getBody()->getPosition();

    Vector2 midLine = circle2Center - circle1Center;
    Real squaredDistance = midLine.getSquaredMagnitude();
    Real radiusSum = CIRCLE1->getRadius() + CIRCLE2->getRadius();

    // Comparing the squared distance avoid the computation of the radix
    // TODO: check for correctness (squaredDistance == 0)
    if (squaredDistance <= (radiusSum * radiusSum))
    {
      // TODO: point = circle1Center ???
      contact->point = circle1Center + midLine * 0.5;
      contact->normal = midLine.normalize();
      contact->penetrationDepth = radiusSum - s2sqrt(squaredDistance);
      std::cout << "CIRCLE - CIRCLE\n";

      return true;
    }

    return false;
  }


  // ---------------------------------------------------------------------------
  // Check a circle against a rect
  bool NarrowPhaseDetector::testCircleRect (
      CircleShape* CIRCLE, RectShape* RECT, Contact* contact)
  {
    Vector2 circleCenter  = CIRCLE->getBody()->getPosition();
    Real circleRadius     = CIRCLE->getRadius();

    Vector2 rectCenter    = RECT->getBody()->getPosition();
    Vector2 rectHalfSize  = RECT->getHalfSize();

    // TODO: OPTIMIZATION -> early out (separation axis -> p. 285) needed ???

    // Transform the point in the local coordinates of the rect
    // TODO: OPTIMIZATION -> in one step with Matrix2x3
    // TODO: OPTIMIZATION -> using projection axis (p. 133)
    Vector2 pointRect = circleCenter;
    pointRect -= rectCenter;
    pointRect  = RECT->getBody()->getOrientationMatrix().getInverse() *
      pointRect;

    // Clamp it to the not oriented rect
    if (pointRect.x < -rectHalfSize.x)
    {
      pointRect.x = -rectHalfSize.x;
    }
    if (pointRect.y < -rectHalfSize.y)
    {
      pointRect.y = -rectHalfSize.y;
    }

    if (pointRect.x > rectHalfSize.x)
    {
      pointRect.x = rectHalfSize.x;
    }
    if (pointRect.y > rectHalfSize.y)
    {
      pointRect.y = rectHalfSize.y;
    }


    // Re-transform the point in the world coordinates
    // TODO: OPTIMIZATION -> in one step with Matrix2x3
    // TODO: OPTIMIZATION -> keep a circleCenter transfomed and do the check
    //                       with it before the re-transform
    pointRect  = RECT->getBody()->getOrientationMatrix() *
      pointRect;
    pointRect += rectCenter;

    // Test for collision
    // TODO: check for correctness (squaredDistance == 0)
    Real squaredDistance = (pointRect - circleCenter).getSquaredMagnitude();
    if (squaredDistance <= (circleRadius * circleRadius))
    {
      // TODO: point = circle1Center ???
      contact->point = pointRect;
      contact->normal = (pointRect - circleCenter).normalize();
      contact->penetrationDepth = circleRadius - s2sqrt(squaredDistance);
      std::cout << "CIRCLE - RECT\n";

      return true;
    }

    return false;
  }


  // ---------------------------------------------------------------------------
  // Check a rect against another one
  bool NarrowPhaseDetector::testRectRect (
      RectShape* RECT1, RectShape* RECT2, Contact* contact)
  {
    return true;
  }


}
