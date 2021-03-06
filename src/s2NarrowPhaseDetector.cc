/*
 * Copyright (C) 2010   Marco Dalla Via (maek@paranoici.org)
 *
 *  This file is part of Spring2D.
 *
 *  Spring2D is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Spring2D is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Lesser Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser Public License
 *  along with Spring2D. If not, see <http://www.gnu.org/licenses/>.
 */

#include "../include/s2NarrowPhaseDetector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Compute the exact list of collisions
  void NarrowPhaseDetector::findCollisions (
      const BodyList& bodyList, ContactList* contactList)
  {
    ContactList::iterator contactI = contactList->begin();
    bool collision;
    Body* tbody;

    while (contactI != contactList->end())
    {
      // Skip static geometry checks
      if ((*contactI)->body[0]->isStatic() &&
          (*contactI)->body[1]->isStatic())
      {
        // Delete the contact data
        delete *contactI;

        // Remove the contact
        contactI = contactList->erase(contactI);
        continue;
      }

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
              tbody                 = (*contactI)->body[0];
              (*contactI)->body[0]  = (*contactI)->body[1];
              (*contactI)->body[1]  = tbody;
              collision = testCircleRect(
                  static_cast<CircleShape*>((*contactI)->body[0]->getShape()),
                  static_cast<RectShape*>((*contactI)->body[1]->getShape()),
                  (*contactI));
              break;

            case Shape::RECT :      // RECT - RECT
              collision = testPolygonPolygon(
                  static_cast<RectShape*>((*contactI)->body[0]->getShape()),
                  static_cast<RectShape*>((*contactI)->body[1]->getShape()),
                  (*contactI));
              break;

            case Shape::POLYGON :   // RECT - POLYGON
              collision = testPolygonPolygon(
                  (*contactI)->body[0]->getShape(),
                  (*contactI)->body[1]->getShape(),
                  (*contactI));
              break;
          }
          break;


        case Shape::POLYGON :
          switch ((*contactI)->body[1]->getShape()->getType())
          {
            case Shape::CIRCLE :    // POLYGON - CIRCLE
              tbody                 = (*contactI)->body[0];
              (*contactI)->body[0]  = (*contactI)->body[1];
              (*contactI)->body[1]  = tbody;
              collision = testCirclePolygon(
                  static_cast<CircleShape*>((*contactI)->body[0]->getShape()),
                  static_cast<PolygonShape*>((*contactI)->body[1]->getShape()),
                  (*contactI));
              break;

            case Shape::RECT :      // POLYGON - RECT
              collision = testPolygonPolygon(
                  (*contactI)->body[0]->getShape(),
                  (*contactI)->body[1]->getShape(),
                  (*contactI));
              break;

            case Shape::POLYGON :   // POLYGON - POLYGON
              collision = testPolygonPolygon(
                  (*contactI)->body[0]->getShape(),
                  (*contactI)->body[1]->getShape(),
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
        contactI = contactList->erase(contactI);
        continue;
      }
      else
      {
        ++contactI;
        continue;
      }

    }

  }



  // ---------------------------------------------------------------------------
  // Check a circle against another one
  bool NarrowPhaseDetector::testCircleCircle (
      CircleShape* CIRCLE1, CircleShape* CIRCLE2, Contact* contact)
  {
    Vector2 center1 = CIRCLE1->getBody()->getPosition();
    Vector2 center2 = CIRCLE2->getBody()->getPosition();

    // Early out if the two circle is concentric (NO normal possible)
    if (center1 == center2)
    {
      return false;
    }

    Real radius1 = CIRCLE1->getRadius();
    Real radius2 = CIRCLE2->getRadius();

    Vector2 midLine = center2 - center1;
    Real squaredDistance = midLine.getSquaredMagnitude();
    Real radiusSum = radius1 + radius2;

    // Comparing the squared distance avoid the computation of the radix
    if (squaredDistance < (radiusSum * radiusSum))
    {
      contact->penetrationDepth = radiusSum - s2sqrt(squaredDistance);
      contact->point[0] = center1 + midLine.getNormalizedCopy() * radius1;
      contact->point[1] = center2 - midLine.getNormalizedCopy() * radius2;

      return true;
    }

    return false;
  }



  // ---------------------------------------------------------------------------
  // Check a circle against a rect
  bool NarrowPhaseDetector::testCircleRect (
      CircleShape* CIRCLE, RectShape* RECT, Contact* contact)
  {
    Vector2 centerC   = CIRCLE->getBody()->getPosition();
    Real radius       = CIRCLE->getRadius();

    Vector2 centerR   = RECT->getBody()->getPosition();
    Vector2 extent    = RECT->getExtent();

    // Transform the point in the local coordinates of the rect
    Vector2 centerCinR  = RECT->getBody()->transformLocal(centerC);
    Vector2 pointRect   = RECT->getBody()->transformLocal(centerC);

    // Clamp it to the rect
    if (pointRect.x < -extent.x)
    {
      pointRect.x = -extent.x;
    }
    else if (pointRect.x > extent.x)
    {
      pointRect.x = extent.x;
    }

    if (pointRect.y < -extent.y)
    {
      pointRect.y = -extent.y;
    }
    else if (pointRect.y > extent.y)
    {
      pointRect.y = extent.y;
    }


    // Test for collision
    Real squaredDistance = (pointRect - centerCinR).getSquaredMagnitude();
    // The circle center is inside the rectangle
    if (squaredDistance == 0)
    {
      // Clamp it to the rect bounds
      if (pointRect.x < 0)
      {
        if (pointRect.y < 0)
        {
          if (extent.x + pointRect.x < extent.y + pointRect.y)
          {
            pointRect.x = -extent.x;
          }
          else
          {
            pointRect.y = -extent.y;
          }
        }
        else
        {
          if (extent.x + pointRect.x < extent.y - pointRect.y)
          {
            pointRect.x = -extent.x;
          }
          else
          {
            pointRect.y = extent.y;
          }
        }
      }
      else
      {
        if (pointRect.y < 0)
        {
          if (extent.x - pointRect.x < extent.y + pointRect.y)
          {
            pointRect.x = extent.x;
          }
          else
          {
            pointRect.y = -extent.y;
          }
        }
        else
        {
          if (extent.x - pointRect.x < extent.y - pointRect.y)
          {
            pointRect.x = extent.x;
          }
          else
          {
            pointRect.y = extent.y;
          }
        }
      }

      // Re-transform the point in the world coordinates
      RECT->getBody()->transformWorld(&pointRect);

      contact->penetrationDepth = radius + (pointRect - centerC).getMagnitude();
      contact->point[0] = centerC +
        (centerC - pointRect).getNormalizedCopy() * radius;
      contact->point[1] = pointRect;

      return true;
    }
    // The circle center is outside the rectangle but they collide
    if (squaredDistance < (radius * radius))
    {
      // Re-transform the point in the world coordinates
      RECT->getBody()->transformWorld(&pointRect);

      contact->penetrationDepth = radius - s2sqrt(squaredDistance);
      contact->point[0] = centerC +
        (pointRect - centerC).getNormalizedCopy() * radius;
      contact->point[1] = pointRect;

      return true;
    }

    return false;
  }



  // ---------------------------------------------------------------------------
  // Check a circle against a polygon
  bool NarrowPhaseDetector::testCirclePolygon (
      CircleShape* CIRCLE, PolygonShape* POLYGON, Contact* contact)
  {
    Simplex W;
    Vector2 centerC = CIRCLE->getBody()->getPosition();
    Real radius     = CIRCLE->getRadius();

    Vector2 distance = GJK(W, CIRCLE, POLYGON);
    if (distance.getSquaredMagnitude() < radius * radius)
    {
      contact->penetrationDepth = radius - distance.getMagnitude();
      contact->point[0] = centerC - distance.getNormalizedCopy() * radius;
      contact->point[1] = centerC - distance;
      return true;
    }
    return false;
  }



  // ---------------------------------------------------------------------------
  // Check a polygon against another one
  bool NarrowPhaseDetector::testPolygonPolygon (
      Shape* SHAPE1, Shape* SHAPE2, Contact* contact)
  {
    Simplex W;
    if (GJK(W, SHAPE1, SHAPE2).isZero())
    {
      if (W.size == 3)
      {
        EPA(W, SHAPE1, SHAPE2, contact);
        if (contact->penetrationDepth == 0)
        {
          return false;
        }
        return true;
      }
    }
    return false;
  }





  // ---------------------------------------------------------------------------
  // Find if two shape is intersecting [GJK]
  Vector2 NarrowPhaseDetector::GJK (
      Simplex& W,
      const Shape* SHAPE1, const Shape* SHAPE2) const
  {
    Simplex Y;
    Vector2 w;
    Vector2 sA;
    Vector2 sB;

    // Get a start point from the Minkowski difference
    Vector2 v = SHAPE1->getSupportPoint0() - SHAPE2->getSupportPoint0();

    while (v.isNotZero())
    {
      // Compute a support point in direction -v
      sA = SHAPE1->getSupportPoint(-v.getNormalizedCopy());
      sB = SHAPE2->getSupportPoint(v.getNormalizedCopy());
      w = sA - sB;

      // If w is in Y || |v|^2 - dot(v, w) <= EPSILON_REL_2 * |v|^2
      if (Y.hasVertex(w) ||
          v.getSquaredMagnitude() - dot(v, w) <=
          EPSILON_REL_2 * v.getSquaredMagnitude())
      {
        // Objects don't penetrate each other
        return v;
      }

      // Add w to the simplex W
      W.addVertex(w, sA, sB);
      Y = W;
      // Get the closest point to origin from the simplex
      v = W.calculateClosestPointToOrigin();

      // Check for collisions
      if (W.size == 3)
      {
        // Collision found
        return Vector2::ZERO;
      }
    }

    // No intersection if objects touch each other but don't penetrate
    return Vector2::ZERO;
  }



  // ---------------------------------------------------------------------------
  // Find the distance & the intersection points of two overlapping shapes [EPA]
  void NarrowPhaseDetector::EPA (
      const Simplex& P,
      const Shape* SHAPE1, const Shape* SHAPE2,
      Contact* contact) const
  {
    Vector2 v;
    Vector2 vN;
    Vector2 w;
    Vector2 sA;
    Vector2 sB;
    Edge* edge;
    Edge* edge1;
    Edge* edge2;
    Real mu = INFINITE;
    EdgeQueue Q;


    // Construct all edges
    for (int i = 0; i < 3; ++i)
    {
      edge = new Edge();
      if (edge->construct(
            P.vertices[i],
            P.supportPointsA[i], P.supportPointsB[i],
            P.vertices[(i + 1) % 3],
            P.supportPointsA[(i + 1) % 3], P.supportPointsB[(i + 1) % 3]))
      {
        Q.push(edge);
      }
      else
      {
        delete edge;
      }
    }

    bool error = true;
    edge = 0;

    // Compute the penetration distance
    do
    {
      delete edge;
      edge = Q.top();
      Q.pop();

      v  = edge->v;
      vN = v.getNormalizedCopy();
      sA = SHAPE1->getSupportPoint(vN);
      sB = SHAPE2->getSupportPoint(-vN);
      w = sA - sB;

      //TODO: check for correctness
      if (mu < dot(v, w) * dot(v, w) / edge->key)
      {
        break;
      }

      mu = std::min(mu, dot(v, w) * dot(v, w) / edge->key);

      // Close enough
      if (mu <= (1 + EPSILON_REL) * (1 + EPSILON_REL) * edge->key)
      {
        error = false;
        break;
      }


      // Split the current edge
      edge1 = new Edge();
      if (edge1->construct(
            edge->endpoints[0],
            edge->supportPointsA[0], edge->supportPointsB[0],
            w,
            sA, sB) &&
          edge->key <= edge1->key && edge1->key <= mu)
      {
        if (
            Q.hasEdge(edge1) ||
            (edge->endpoints[0] == edge1->endpoints[0] &&
             edge->endpoints[1] == edge1->endpoints[1]) ||
            (edge->endpoints[0] == edge1->endpoints[1] &&
             edge->endpoints[1] == edge1->endpoints[0])
           )
        {
          delete edge1;
          error = false;
          break;
        }
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
            edge->supportPointsA[1], edge->supportPointsB[1]) &&
          edge->key <= edge2->key && edge2->key <= mu)
        {
          if (
              Q.hasEdge(edge2) ||
              (edge->endpoints[0] == edge2->endpoints[0] &&
               edge->endpoints[1] == edge2->endpoints[1]) ||
              (edge->endpoints[0] == edge2->endpoints[1] &&
               edge->endpoints[1] == edge2->endpoints[0])
             )
          {
            delete edge2;
            error = false;
            break;
          }
          Q.push(edge2);
        }
      else
      {
        delete edge2;
      }

    }
    while(Q.size() > 0 && Q.top()->key <= mu);

    // Skip collision if EPA fails
    if (error)
    {
      contact->penetrationDepth = 0;
      delete edge;
      return;
    }

    // Set the contact data
    contact->penetrationDepth = s2sqrt(edge->key);
    contact->point[0] =
      edge->supportPointsA[0] * (1 - edge->t) +
      edge->supportPointsA[1] * edge->t;
    contact->point[1] =
      edge->supportPointsB[0] * (1 - edge->t) +
      edge->supportPointsB[1] * edge->t;

    delete edge;
  }



  // ---------------------------------------------------------------------------
  // Compute the point of minimum norm for the simplex & automatically reduce it
  Vector2 Simplex::calculateClosestPointToOrigin()
  {
    if (size == 1) // single point (1 vertex)
    {
      bCoordinates[0] = 1;
      return vertices[0];
    }


    else if (size == 2) // segment (2 vertices)
    {
      Vector2 A = vertices[0];
      Vector2 B = vertices[1];

      // Check if O is in vertex region outside A
      Vector2 AB = B - A;
      Real numerator = dot(-A, AB);
      if (numerator <= 0)
      {
        // Remove B & return A
        size--;
        bCoordinates[0] = 1;
        // TODO: this should never happen
        assert(false);
        return A;
      }

      // Check if O is in vertex region outside B
      Real denominator = dot(AB, AB);
      if (numerator >= denominator)
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


      //TODO: OPTIMIZATION -> find the shortcut
#if 0
      Vector2 A = vertices[0];
      Vector2 B = vertices[1];

      Vector2 AB = B - A;

      // Check if O is in vertex region outside B
      Real lambda = dot(AB, B);
      // TODO: check also length > 0 ???
      if (lambda <= 0)
      {
        // Remove B & return A
        vertices[0]       = vertices[1];
        supportPointsA[0] = supportPointsA[1];
        supportPointsB[0] = supportPointsB[1];
        size--;
        bCoordinates[0] = 1;
        return B;
      }

      // O is on AB
      lambda /= dot(AB, AB);
      bCoordinates[0] = 1 - lambda;
      bCoordinates[1] = lambda;
      return (A + lambda * AB);
#endif
    }


    else // triangle (3 vertices)
    {
      Vector2 A = vertices[0];
      Vector2 B = vertices[1];
      Vector2 C = vertices[2];

      Vector2 AB = B - A;
      Vector2 AC = C - A;
      Vector2 BC = C - B;

      // TODO: remove
      Real snum   = -dot(AB, A);
      Real sdenom =  dot(AB, B);
      Real tnum   = -dot(AC, A);
      Real tdenom =  dot(AC, C);
      Real unum   = -dot(BC, B);
      Real udenom =  dot(BC, C);


      // Check if O is in vertex region outside C
      if (tdenom <= 0 && udenom <= 0)
      {
        // Remove A and B & return C
        vertices[0]       = vertices[2];
        supportPointsA[0] = supportPointsA[2];
        supportPointsB[0] = supportPointsB[2];
        size -= 2;
        bCoordinates[0] = 1;
        return C;
      }

      // Should not be in vertex a or b region.
      // TODO: remove
      assert(snum > 0 || tnum > 0);
      assert(sdenom > 0 || unum > 0);

      Real n = cross(AB, AC);

      // Should not be in edge ab region.
      // TODO: remove
      Real vc = n * cross(A, B);
      assert(vc > 0 || snum > 0 || sdenom > 0);


      // Check if O is in edge region of BC
      Real va = n * cross(B, C);
      if (va <= 0 && unum >= 0 && udenom >= 0 && (unum + udenom) > 0)
      {
        // Remove A & return the projection on BC
        vertices[0]       = vertices[2];
        supportPointsA[0] = supportPointsA[2];
        supportPointsB[0] = supportPointsB[2];
        size--;
        Real lambda = unum / (unum + udenom);
        bCoordinates[0] = 1 - lambda;
        bCoordinates[1] = lambda;
        return (B + lambda * BC);
      }


      // Check if O is in edge region of AC
      Real vb = n * cross(C, A);
      if (vb <= 0 && tnum >= 0 && tdenom >= 0 && (tnum + tdenom) > 0)
      {
        // Remove B & return the projection on AC
        vertices[1]       = vertices[2];
        supportPointsA[1] = supportPointsA[2];
        supportPointsB[1] = supportPointsB[2];
        size--;
        Real lambda = tnum / (tnum + tdenom);
        bCoordinates[0] = 1 - lambda;
        bCoordinates[1] = lambda;
        return (A + lambda * AC);
      }


      // O is inside the triangle
      Real denom = va + vb + vc;
      denom = 1. / denom;

      Real u = va * denom;
      Real v = vb * denom;
      Real w = 1 - u - v;
      bCoordinates[0] = u;
      bCoordinates[1] = v;
      bCoordinates[2] = w;
      return (u * A + v * B + w * C);
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
    bool valid;

    // Check if O is in any vertex region
    Vector2 AB = B - A;
    t = dot(-A, AB) / dot(AB, AB);
    if (t <= 0 || t >= 1)
    {
      valid = false;
    }
    else
    {
      valid = true;
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

    return valid;
  }


}
