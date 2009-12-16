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
  // Check a polygon against another one [GJK]
  // TODO : does exist a better initial point (ISA) ???
  bool NarrowPhaseDetector::testPolygonPolygon(
      PolygonShape* POLYGON1, PolygonShape* POLYGON2, Contact* contact)
  {
    Real epsilon = 0.001;
    Simplex Y;
    Vector2 v;
    Vector2 w;

    // Get a start point from the Minkowsky difference
    // TODO: grep body2pos - body1pos for the direction
    v = supportMappingMinkowsky(POLYGON1, POLYGON2,
        (POLYGON2->getBody()->getPosition() -
         POLYGON1->getBody()->getPosition()).getNormalized());
    std::cout << "v = " << v << "\n";

    while (true)
    {
      // TODO: remove for correctness ? (NO normal found)
      //if (v == Vector2::ZERO)
      //{
      //  std::cout << "collision [GJK]\n";
      //  // TODO: call EPA
      //  return true;
      //}

      // Compute a support point in direction -v
      w = supportMappingMinkowsky(POLYGON1, POLYGON2, -v.getNormalized());
      std::cout << "w = " << w << "\n";
      std::cout << "||v||^2 = " << v.getSquaredMagnitude() << "\n";
      std::cout << "dot(v, w) = " << dotProduct(v, w) << "\n";

      // If w is in Y OR
      // v.getMagnitude() - dotProduct(v, w) <= epsilon^2 * v.getMagnitude()
      if (Y.hasVertex(w) ||
          v.getSquaredMagnitude() * (1 - epsilon * epsilon) <= dotProduct(v, w))
        // dotProduct(v, w) > 0)
      {
        std::cout << "[DISTANCE] =================== " << v.getMagnitude() << "\n";
        return false;
      }

      // Add w to the simplex
      Y.expand(w);
      // Get the point of minimum norm in the simplex
      v = Y.getPointOfMinimumNorm();
      std::cout << "v = " << v << "\n";

      // Check for collision
      if (Y.hasOriginInside() == true)
      {
        std::cout << "collision [GJK] ==============================\n";
        // Here Y contain exactly 3 points
        std::cout << "dimension = " << Y.getDimension() << "\n";
        std::cout << "P[0] = " << Y.P[0] << "\n";
        std::cout << "P[1] = " << Y.P[1] << "\n";
        std::cout << "P[2] = " << Y.P[2] << "\n";
        EPA(Y, POLYGON1, POLYGON2, contact);
        // TODO: call EPA
        return true;
      }
    }

    // Cosmetic return
    return false;
  }



  // ---------------------------------------------------------------------------
  // Find the distance from the two intersecting polygons
  void NarrowPhaseDetector::EPA (
      const Simplex& Y,
      PolygonShape* POLYGON1, PolygonShape* POLYGON2,
      Contact* contact) const
  {
    std::priority_queue<Entry*, std::vector<Entry*>, PQComparison> Q;
    Vector2 v;
    Vector2 w;
    Entry* entry;
    Entry* entry1;
    Entry* entry2;

    // Build all entries
    for (int i = 0; i < 3; ++i)
    {
      entry = new Entry();
      if (entry->build(Y.P[i], Y.P[(i + 1) % 3]))
      {
        Q.push(entry);
      }
      else
      {
        delete entry;
      }
    }

    // Compute the penetration distance
    // TODO: resolve the memory leak
    while (true)
    {
      entry = Q.top();
      Q.pop();

      v = entry->v;
      // TODO: normalize ???
      w = supportMappingMinkowsky(POLYGON1, POLYGON2, v);
      std::cerr << "v = " << v << "\n";
      std::cerr << "w = " << w << "\n";
      std::cerr << (dotProduct(v, w) / v.getMagnitude() - v.getMagnitude()) << "\n";

      // Close enough
      if (dotProduct(v, w) / v.getMagnitude() - v.getMagnitude() <= 0.001)
      {
        std::cout << "[EPA] ======================== " << v.getMagnitude() << "\n";
        break;
      }


      // Split the edge
      entry1 = new Entry();
      if (entry1->build(entry->y[0], w))
      {
        Q.push(entry1);
      }
      else
      {
        delete entry1;
      }

      entry2 = new Entry();
      if (entry2->build(w, entry->y[1]))
      {
        Q.push(entry2);
      }
      else
      {
        delete entry2;
      }

      // TODO: here ???
      delete entry;
    }

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



  // ---------------------------------------------------------------------------
  // Return the furthest point along the given direction [MINKOWSKY]
  // S   (d) = S (d) - S (-d)
  //  a-b       a       b
  inline Vector2 NarrowPhaseDetector::supportMappingMinkowsky (
      const PolygonShape* POLYGON1, const PolygonShape* POLYGON2,
      Vector2 direction) const
  {
    return (supportMapping(POLYGON1, direction) -
        supportMapping(POLYGON2, -direction));
  }


}
