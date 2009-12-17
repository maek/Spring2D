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

            case Shape::POLYGON :   // CIRCLE - POLYGON
              collision = testCirclePolygon(
                  static_cast<CircleShape*>((*contactI)->body[0]->getShape()),
                  static_cast<PolygonShape*>((*contactI)->body[1]->getShape()),
                  (*contactI));
              break;

            case Shape::RECT :      // CIRCLE - RECT
              collision = testCircleRect(
                  static_cast<CircleShape*>((*contactI)->body[0]->getShape()),
                  static_cast<RectShape*>((*contactI)->body[1]->getShape()),
                  (*contactI));
              break;
          }
          break;


        case Shape::POLYGON :
          switch ((*contactI)->body[1]->getShape()->getType())
          {
            case Shape::CIRCLE :    // POLYGON - CIRCLE
              collision = testCirclePolygon(
                  static_cast<CircleShape*>((*contactI)->body[1]->getShape()),
                  static_cast<PolygonShape*>((*contactI)->body[0]->getShape()),
                  (*contactI));
              break;

            case Shape::POLYGON :   // POLYGON - POLYGON
              collision = testPolygonPolygon(
                  static_cast<PolygonShape*>((*contactI)->body[0]->getShape()),
                  static_cast<PolygonShape*>((*contactI)->body[1]->getShape()),
                  (*contactI));
              break;

            case Shape::RECT :      // POLYGON - RECT
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

            case Shape::POLYGON :   // RECT - POLYGON
              break;

            case Shape::RECT :      // RECT - RECT
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
      // TODO: fix contact points
      contact->point[0] = circle1Center;
      contact->point[1] = circle2Center;
      contact->penetrationDepth = radiusSum - s2sqrt(squaredDistance);
      std::cerr << "CIRCLE - CIRCLE\n";

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
      // TODO: fix contact points
      contact->point[0] = circleCenter;
      contact->point[1] = pointRect;
      contact->penetrationDepth = circleRadius - s2sqrt(squaredDistance);
      std::cerr << "CIRCLE - RECT\n";

      return true;
    }

    return false;
  }



  // ---------------------------------------------------------------------------
  // Check a circle against a polygon
  bool NarrowPhaseDetector::testCirclePolygon (
      CircleShape* CIRCLE, PolygonShape* POLYGON, Contact* contact)
  {
    return true;
  }



  // ---------------------------------------------------------------------------
  // Check a polygon against another one
  bool NarrowPhaseDetector::testPolygonPolygon(
      PolygonShape* POLYGON1, PolygonShape* POLYGON2, Contact* contact)
  {
    Simplex Y;
    if (GJK(Y, POLYGON1, POLYGON2) == true)
    {
      EPA(Y, POLYGON1, POLYGON2, contact);
      return true;
    }
    return false;
  }





  // ---------------------------------------------------------------------------
  // Find if two shape is intersecting [GJK]
  bool NarrowPhaseDetector::GJK (
      Simplex& Y,
      const Shape* SHAPE1, const Shape* SHAPE2) const
  {
    Vector2 v;
    Vector2 w;
    Vector2 sA;
    Vector2 sB;

    // Get a start point from the Minkowsky difference
    // TODO: grep body2pos - body1pos for the direction
    sA = SHAPE1->getSupportPoint(Vector2::X);
    sB = SHAPE1->getSupportPoint(-Vector2::X);
    v = sA - sB;
    std::cerr << "v = " << v << "\n";

    Real mu = 0;
    Real delta;
    Real vLength;
    while (v.isZero() == false)
    {
      std::cerr << "INIZIO ===========================================\n";

      // Compute a support point in direction -v
      sA = SHAPE1->getSupportPoint(-v.getNormalizedCopy());
      sB = SHAPE2->getSupportPoint(v.getNormalizedCopy());
      w = sA - sB;
      std::cerr << "w = " << w << "\n";
      std::cerr << "||v||^2 = " << v.getSquaredMagnitude() << "\n";
      std::cerr << "dot(v, w) = " << dotProduct(v, w) << "\n";


      vLength = v.getMagnitude();
      delta = dotProduct(v, w) / vLength;
      mu = std::max(mu, delta);

      // If w is in Y OR
      // v.getMagnitude() - dotProduct(v, w) <= epsilon^2 * v.getMagnitude()
      if (Y.hasVertex(w) ||
          vLength - mu <= EPSILON * vLength)
        //    v.getSquaredMagnitude() * (1 - EPSILON_2) <= dotProduct(v, w))
        // dotProduct(v, w) > 0)
      {
        return false;
      }

      // Add w to the simplex
      Y.addVertex(w, sA, sB);
      // Get the point of minimum norm in the simplex
      v = Y.calculateClosestPointToOrigin();
      std::cerr << "v = " << v << "\n";

      // Check for collision
      if (Y.includeOrigin == true)
      {
        std::cerr << "collision [GJK] ==============================\n";
        // Here Y contain exactly 3 points
        std::cerr << "size = " << Y.size << "\n";
        std::cerr << "P[0] = " << Y.vertices[0] << "\n";
        std::cerr << "P[1] = " << Y.vertices[1] << "\n";
        std::cerr << "P[2] = " << Y.vertices[2] << "\n";
        return true;
      }
      std::cerr << "FINE =============================================\n";
    }

    // Dummy return
    return false;
  }



  // ---------------------------------------------------------------------------
  // Find the distance & the intersection points of two overlapping shapes [EPA]
  void NarrowPhaseDetector::EPA (
      const Simplex& Y,
      const Shape* SHAPE1, const Shape* SHAPE2,
      Contact* contact) const
  {
    Vector2 v;
    Vector2 w;
    Vector2 sA;
    Vector2 sB;
    Edge* edge;
    Edge* edge1;
    Edge* edge2;
    std::priority_queue<Edge*, std::vector<Edge*>, EdgeCompare> Q;
    std::priority_queue<Edge*, std::vector<Edge*>, EdgeCompare> R;

    // Construct all edges
    for (int i = 0; i < 3; ++i)
    {
      edge = new Edge();
      if (edge->construct(
            Y.vertices[i],
            Y.supportPointsA[i], Y.supportPointsB[i],
            Y.vertices[(i + 1) % 3],
            Y.supportPointsA[(i + 1) % 3], Y.supportPointsB[(i + 1) % 3]))
      {
        Q.push(edge);
      }
      else
      {
        delete edge;
      }
    }

    // TODO: fix
    Real mu = HUGE_VALF;
    Real delta;
    // Compute the penetration distance
    // TODO: remove all memory leaks
    do
    {
      // TODO: print Q
      Edge* e;
      int qsize = Q.size();
      std::cerr << "Qsize = " << qsize << "\n";
      while (R.size())
        R.pop();
      for (int i = 0; i < qsize; ++i)
      {
        e = Q.top();
        std::cerr << "A[" << i << "] = " << e->endpoints[0] << "\n";
        std::cerr << "B[" << i << "] = " << e->endpoints[1] << "\n\n";
        Q.pop();
        R.push(e);
      }
      int rsize = R.size();
      std::cerr << "Rsize = " << rsize << "\n";
      for (int i = 0; i < rsize; ++i)
      {
        e = R.top();
        R.pop();
        Q.push(e);
      }
      std::cerr << "Qsize = " << Q.size() << "\n";




      edge = Q.top();
      std::cerr << "vA = " << edge->endpoints[0] << "\n";
      std::cerr << "vB = " << edge->endpoints[1] << "\n\n";
      Q.pop();

      v = edge->v;
      // TODO: normalize ???
      sA = SHAPE1->getSupportPoint(v.getNormalizedCopy());
      sB = SHAPE2->getSupportPoint(-v.getNormalizedCopy());
      w = sA - sB;
      std::cerr << "epa\n";
      std::cerr << "v = " << v << "\n";
      std::cerr << "w = " << w << "\n";
      std::cerr << "dotProduct(v, w) = " << dotProduct(v, w) << "\n";
      std::cerr << "delta = " << dotProduct(v, w) * dotProduct(v, w) / edge->key << "\n";
      std::cerr << "mu = " << mu << "\n";



      delta = dotProduct(v, w) * dotProduct(v, w) / edge->key;
      mu = (mu < delta ? mu : delta);
      std::cerr << "mu = " << mu << "\n";
      std::cerr << "condition = " << (1 + EPSILON) * (1 + EPSILON) * edge->key << "\n";

      // Close enough
      if (mu <= (1 + EPSILON) * (1 + EPSILON) * edge->key)
        //if (dotProduct(v, w) / v.getMagnitude() - v.getMagnitude() <= EPSILON)
      {
        break;
      }


      // Split the current edge
      edge1 = new Edge();
      if (edge1->construct(
            edge->endpoints[0],
            edge->supportPointsA[0], edge->supportPointsB[0],
            w,
            sA, sB))
      {
        std::cerr << "A1 = " << edge1->endpoints[0] << "\n";
        std::cerr << "B1 = " << edge1->endpoints[1] << "\n\n";
        Q.push(edge1);
      }
      else
      {
        delete edge1;
      }

      edge2 = new Edge();
      if (edge2->construct(
            w,
            sA, sB,
            edge->endpoints[1],
            edge->supportPointsA[1], edge->supportPointsB[1]))
      {
        std::cerr << "A2 = " << edge2->endpoints[0] << "\n";
        std::cerr << "B2 = " << edge2->endpoints[1] << "\n\n";
        Q.push(edge2);
      }
      else
      {
        delete edge2;
      }

      //delete edge;
    }
    while(Q.top()->key <= mu);


    std::cerr << "[EPA] ======================== " << v.getMagnitude() << "\n";
    contact->point[0] = edge->supportPointsA[0] * (1 - edge->t) +
      edge->supportPointsA[1] * edge->t;
    contact->point[1] = edge->supportPointsB[0] * (1 - edge->t) +
      edge->supportPointsB[1] * edge->t;


    delete edge;
    int qsize = Q.size();
    for (int i = 0; i < qsize; ++i)
    {
      edge = Q.top();
      Q.pop();
      delete edge;
    }

  }





  // ---------------------------------------------------------------------------
  // Compute the point of minimum norm for the simplex & automatically reduce it
  Vector2 Simplex::calculateClosestPointToOrigin()
  {
    if (size == 1) // single point (1 vertex)
    {
      std::cerr << "size = 1\n";
      std::cerr << "vertices[0] = " << vertices[0] << "\n";
      bCoordinates[0] = 1;
      return vertices[0];
    }


    else if (size == 2) // segment (2 vertices)
    {
      std::cerr << "size = 2\n";
      std::cerr << "vertices[0] = " << vertices[0] << "\n";
      std::cerr << "vertices[1] = " << vertices[1] << "\n";
      Vector2 A = vertices[0];
      Vector2 B = vertices[1];
      Vector2 O(0 ,0);

      // Check if O is in vertex region outside A
      Vector2 AB = B - A;
      Real numerator = dotProduct(O - A, AB);
      if (numerator < 0)
      {
        // Remove B & return A
        size--;
        bCoordinates[0] = 1;
        return A;
      }

      // Check if O is in vertex region outside B
      Real denominator = dotProduct(AB, AB);
      if (numerator > denominator)
      {
        // Remove A & return B
        vertices[0]       = vertices[1];
        supportPointsA[0] = supportPointsA[1];
        supportPointsB[0] = supportPointsB[1];
        size--;
        bCoordinates[0] = 1;
        return B;
      }
      // O is on AB
      // Return the projection on AB
      Real v = numerator / denominator;
      bCoordinates[0] = 1 - v;
      bCoordinates[1] = v;
      return (A + v * AB);
    }


    else // triangle (3 vertices)
    {
      std::cerr << "size = 3\n";
      std::cerr << "vertices[0] = " << vertices[0] << "\n";
      std::cerr << "vertices[1] = " << vertices[1] << "\n";
      std::cerr << "vertices[2] = " << vertices[2] << "\n";
      Vector2 A = vertices[0];
      Vector2 B = vertices[1];
      Vector2 C = vertices[2];
      Vector2 O(0 ,0);


      // Check if O is in vertex region outside A
      Vector2 AB = B - A;
      Vector2 AC = C - A;
      Vector2 AO = O - A;
      Real d1 = dotProduct(AB, AO);
      Real d2 = dotProduct(AC, AO);
      if (d1 <= 0 && d2 <= 0)
      {
        // Remove B and C & return A
        size -= 2;
        bCoordinates[0] = 1;
        return A;
      }

      // Check if O is in vertex region outside B
      Vector2 BO = O - B;
      Real d3 = dotProduct(AB, BO);
      Real d4 = dotProduct(AC, BO);
      if (d3 >= 0 && d4 <= d3)
      {
        // Remove A and C & return B
        vertices[0]       = vertices[1];
        supportPointsA[0] = supportPointsA[1];
        supportPointsB[0] = supportPointsB[1];
        size -= 2;
        bCoordinates[0] = 1;
        return B;
      }

      // Check if O is in vertex region outside C
      Vector2 CO = O - C;
      Real d5 = dotProduct(AB, CO);
      Real d6 = dotProduct(AC, CO);
      if (d6 >= 0 && d5 <= d6)
      {
        // Remove A and B & return C
        vertices[0]       = vertices[2];
        supportPointsA[0] = supportPointsA[2];
        supportPointsB[0] = supportPointsB[2];
        size -= 2;
        bCoordinates[0] = 1;
        return C;
      }


      // Check if O is in edge region of AB
      Real vc = d1 * d4 - d3 * d2;
      if (vc <= 0 && d1 >= 0 && d3 <= 0)
      {
        // Remove C & return the projection on AB
        size--;
        Real v = d1 / (d1 - d3);
        bCoordinates[0] = 1 - v;
        bCoordinates[1] = v;
        return (A + v * AB);
      }

      // Check if O is in edge region of AC
      Real vb = d5 * d2 - d1 * d6;
      if (vb <= 0 && d2 >= 0 && d6 <= 0)
      {
        // Remove B & return the projection on AC
        vertices[1]       = vertices[2];
        supportPointsA[1] = supportPointsA[2];
        supportPointsB[1] = supportPointsB[2];
        size--;
        Real v = d2 / (d2 - d6);
        bCoordinates[0] = 1 - v;
        bCoordinates[1] = v;
        return (A + v * AC);
      }

      // Check if O is in edge region of BC
      Real va = d3 * d6 - d5 * d4;
      if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
      {
        // Remove A & return the projection on BC
        vertices[0]       = vertices[1];
        supportPointsA[0] = supportPointsA[1];
        supportPointsB[0] = supportPointsB[1];
        vertices[1]       = vertices[2];
        supportPointsA[1] = supportPointsA[2];
        supportPointsB[1] = supportPointsB[2];
        size--;
        Real v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        bCoordinates[0] = 1 - v;
        bCoordinates[1] = v;
        return (B + v * (C - B));
      }


      // O is inside face region
      // Return the projection on ABC
      includeOrigin = true;
      Real denominator = 1.0 / (va + vb + vc);
      Real v = vb * denominator;
      Real w = vc * denominator;
      bCoordinates[0] = 1 - v - w;
      bCoordinates[1] = v;
      bCoordinates[2] = w;
      return (A + AB * v + AC * w);
    }

  }





  // ---------------------------------------------------------------------------
  // Construct the Edge & test if it is valid
  bool Edge::construct (
      const Vector2& A,
      const Vector2& SUPPORT_POINT_A_0, const Vector2& SUPPORT_POINT_B_0,
      const Vector2& B,
      const Vector2& SUPPORT_POINT_A_1, const Vector2& SUPPORT_POINT_B_1)
  {
    Vector2 O(0 ,0);

    // Check if O is in any vertex region
    Vector2 AB = B - A;
    t = dotProduct(O - A, AB) / dotProduct(AB, AB);
    if (t <= 0 || t >= 1)
    {
      return false;
    }

    // O is on AB
    endpoints[0]      = A;
    supportPointsA[0] = SUPPORT_POINT_A_0;
    supportPointsB[0] = SUPPORT_POINT_B_0;
    endpoints[1]      = B;
    supportPointsA[1] = SUPPORT_POINT_A_1;
    supportPointsB[1] = SUPPORT_POINT_B_1;
    v = (A + t * AB);
    key = v.getSquaredMagnitude();
    return true;
  }


}
