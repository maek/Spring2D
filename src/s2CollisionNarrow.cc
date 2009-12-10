#include "../include/s2CollisionNarrow.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Check a circle against another one
  bool testCircleCircle (CircleShape* CIRCLE1, CircleShape* CIRCLE2)
  {
    Vector2 distance  = CIRCLE1->getBody()->getPosition() -
                        CIRCLE2->getBody()->getPosition();
    Real radiusSum    = CIRCLE1->getRadius() +
                        CIRCLE2->getRadius();

    // Comparing the squared distance avoid the computation of the radix
    if (distance.getSquaredMagnitude() < (radiusSum * radiusSum))
    {
      std::cout << "collision [NARROW]\n";
      return true;
    }

    return false;
  }


  // ---------------------------------------------------------------------------
  // Check a circle against a rect
  bool testCircleRect (CircleShape* CIRCLE, RectShape* RECT)
  {
    return true;
  }


  // ---------------------------------------------------------------------------
  // Check a rect against another one
  bool testRectRect (RectShape* RECT1, RectShape* RECT2)
  {
    return true;
  }


}
