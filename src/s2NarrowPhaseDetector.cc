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

        case Shape::CIRCLE :
          switch ((*contactI)->body[1]->getShape()->getType())
          {
            case Shape::CIRCLE :    // CIRCLE - CIRCLE
              collision = testCircleCircle(
                  static_cast<CircleShape*>((*contactI)->body[0]->getShape()),
                  static_cast<CircleShape*>((*contactI)->body[1]->getShape()),
                  (*contactI));
              break;

            case Shape::RECT :      // CIRCLE - RECT
              collision = testCircleRect(
                  static_cast<CircleShape*>((*contactI)->body[0]->getShape()),
                  static_cast<RectShape*>((*contactI)->body[1]->getShape()),
                  (*contactI));
              break;

            case Shape::POLYGON :   // CIRCLE - POLYGON
              collision = testCirclePolygon(
                  static_cast<CircleShape*>((*contactI)->body[0]->getShape()),
                  static_cast<PolygonShape*>((*contactI)->body[1]->getShape()),
                  (*contactI));
              break;
          }
          break;


        case Shape::RECT :
          switch ((*contactI)->body[1]->getShape()->getType())
          {
            case Shape::CIRCLE :    // RECT - CIRCLE
              collision = testCircleRect(
                  static_cast<CircleShape*>((*contactI)->body[1]->getShape()),
                  static_cast<RectShape*>((*contactI)->body[0]->getShape()),
                  (*contactI));
              break;

            case Shape::RECT :      // RECT - RECT
              break;

            case Shape::POLYGON :   // RECT - POLYGON
              break;
          }
          break;


        case Shape::POLYGON :
          switch ((*contactI)->body[1]->getShape()->getType())
          {
            case Shape::CIRCLE :    // POLYGON - RECT
              collision = testCirclePolygon(
                  static_cast<CircleShape*>((*contactI)->body[1]->getShape()),
                  static_cast<PolygonShape*>((*contactI)->body[0]->getShape()),
                  (*contactI));
              break;

            case Shape::RECT :      // POLYGON - RECT
              break;

            case Shape::POLYGON :   // POLYGON - POLYGON
              collision = testPolygonPolygon(
                  static_cast<PolygonShape*>((*contactI)->body[0]->getShape()),
                  static_cast<PolygonShape*>((*contactI)->body[1]->getShape()),
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
    // TODO: OPTIMIZATION -> using projection axis (p. 133)
    Vector2 pointRect = circleCenter;
    RECT->getBody()->transformLocal(&pointRect);

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
    RECT->getBody()->transformWorld(&pointRect);


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
  // Check a circle against a polygon
  bool NarrowPhaseDetector::testCirclePolygon (
      CircleShape* CIRCLE, PolygonShape* POLYGON, Contact* contact)
  {
    Vector2 point = supportMapping(CIRCLE, Vector2::XY);
    contact->point = point;

    return true;
  }



  // ---------------------------------------------------------------------------
  // Check a polygon against another one
  bool NarrowPhaseDetector::testPolygonPolygon(
      PolygonShape* POLYGON1, PolygonShape* POLYGON2, Contact* contact)
  {
#if 0
    Vector2 A1 = POLYGON1->getVertices()[0];
    Vector2 B1 = POLYGON1->getVertices()[1];
    Vector2 C1 = POLYGON1->getVertices()[2];

    Vector2 p = POLYGON2->getBody()->getPosition() + POLYGON2->getVertices()[0];

    // Transform the point in the local coordinates of the rect
    POLYGON1->getBody()->transformLocal(&p);

    // Find the point of minimum norm
    p = pointOfMinimumNorm(p, A1, B1, C1);

    // Re-transform the point in the world coordinates
    POLYGON1->getBody()->transformWorld(&p);

    contact->point = p;
#endif

    Vector2 point = supportMapping(POLYGON1, -Vector2::Y);
    contact->point = point;

    return true;
  }



  // ---------------------------------------------------------------------------
  // Compute the point of minimum norm for a single point (1 vertex)
  Vector2 NarrowPhaseDetector::pointOfMinimumNorm (
      const Vector2& P,
      const Vector2& A) const
  {
    return A;
  }



  // ---------------------------------------------------------------------------
  // Compute the point of minimum norm for a segment (2 vertices)
  Vector2 NarrowPhaseDetector::pointOfMinimumNorm (
      const Vector2& P,
      const Vector2& A,
      const Vector2& B) const
  {
    Vector2 AB = B - A;

    // Project P onto AB, but deferring the division
    Real numerator = dotProduct(P - A, AB);
    // If P is outside segment & on the A side
    if (numerator < 0)
    {
      return A;
    }
    Real denominator = dotProduct(AB, AB);
    // If P is outside segment & on the B side
    if (numerator > denominator)
    {
      return B;
    }
    // P is on the segment
    return (A + (numerator / denominator) * AB);

  }



  // ---------------------------------------------------------------------------
  // Compute the point of minimum norm for a triangle (3 vertices)
  Vector2 NarrowPhaseDetector::pointOfMinimumNorm (
      const Vector2& P,
      const Vector2& A,
      const Vector2& B,
      const Vector2& C) const
  {
    Vector2 AB = B - A;
    Vector2 AC = C - A;
    Vector2 AP = P - A;

    // Check if P is in vertex region outside A
    Real d1 = dotProduct(AB, AP);
    Real d2 = dotProduct(AC, AP);
    if (d1 <= 0.0 && d2 <= 0.0)
    {
      // Barycentric coordinates (1, 0, 0)
      return A;
    }

    // Check if P is in vertex region outside B
    Vector2 BP = P - B;
    Real d3 = dotProduct(AB, BP);
    Real d4 = dotProduct(AC, BP);
    if (d3 >= 0.0 && d4 <= d3)
    {
      // Barycentric coordinates (0, 1, 0)
      return B;
    }

    // Check if P is in vertex region outside C
    Vector2 CP = P - C;
    Real d5 = dotProduct(AB, CP);
    Real d6 = dotProduct(AC, CP);
    if (d6 >= 0.0 && d5 <= d6)
    {
      // Barycentric coordinates (0, 0, 1)
      return C;
    }


    // Check if P is in edge region of AB & if so return projection of P onto AB
    Real vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    {
      // Barycentric coordinates (1 - v, v, 0)
      Real v = d1 / (d1 - d3);
      return (A + v * AB);
    }

    // Check if P is in edge region of AC & if so return projection of P onto AC
    Real vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    {
      // Barycentric coordinates (1 - w, 0, w)
      Real w = d2 / (d2 - d6);
      return (A + w * AC);
    }

    // Check if P is in edge region of BC & if so return projection of P onto BC
    Real va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
    {
      // Barycentric coordinates (0, 1 - w, w)
      Real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
      return (B + w * (C - B));
    }


    // P is inside face region & return
    // u * a + v * b + w * c
    // u = va * denominator = 1.0 - v - w
    Real denominator = 1.0 / (va + vb + vc);
    Real v = vb * denominator;
    Real w = vc * denominator;
    return (A + AB * v + AC * w);

  }



  // ---------------------------------------------------------------------------
  // Return the furthest point along the given direction [CIRCLE]
  Vector2 NarrowPhaseDetector::supportMapping (
      const CircleShape* CIRCLE, Vector2 direction) const
  {
    // center + radius * direction / ||direction||
    return CIRCLE->getBody()->getPosition() +
      (CIRCLE->getRadius() * direction.normalize());
  }



  // ---------------------------------------------------------------------------
  // Return the furthest point along the given direction [POLYGON]
  Vector2 NarrowPhaseDetector::supportMapping (
      const PolygonShape* POLYGON, Vector2 direction) const
  {
    // Transform the direction in the local coordinates
    direction =
      POLYGON->getBody()->getOrientationMatrix().getInverse() * direction;

    Vector2* VERTICES   = POLYGON->getVertices();
    unsigned N_VERTICES = POLYGON->getNVertices();

    Vector2 pointCW   = VERTICES[0];
    Vector2 pointCCW  = VERTICES[0];

    Real projection;
    Real tprojection;

    // Counter-clockwise
    projection = dotProduct(pointCCW, direction);
    for (unsigned i = 1; i < N_VERTICES; ++i)
    {
      tprojection = dotProduct(VERTICES[i], direction);
      if (projection <= tprojection)
      {
        pointCCW = VERTICES[i];
        projection = tprojection;
      }
      else
      {
        break;
      }
    }

    // Clockwise
    projection = dotProduct(pointCW, direction);
    for (unsigned i = N_VERTICES - 1; i > 0; --i)
    {
      tprojection = dotProduct(VERTICES[i], direction);
      if (projection <= tprojection)
      {
        pointCW = VERTICES[i];
        projection = tprojection;
      }
      else
      {
        break;
      }
    }

    // Pick the furthest
    if (dotProduct(pointCCW, direction) > projection)
    {
      POLYGON->getBody()->transformWorld(&pointCCW);
      return pointCCW;
    }
    else
    {
      POLYGON->getBody()->transformWorld(&pointCW);
      return pointCW;
    }

  }


}
