#include "../include/s2CollisionSolver.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Solve collisions
  void CollisionSolver::solveCollisions (
      ContactList* frontContacts,
      ContactList* backContacts)
  {
    // Early out
    if (frontContacts->empty())
    {
      return;
    }

    // Preprocess contacts
    preprocessContacts(frontContacts, backContacts);


    // Sort in penetration depth decreasing order
    frontContacts->sort(interpenetrationCompare);

    // Solve interpenetration
    // TODO: update & sort again
    //       extract from the main list, sort in a new list & merge
    for (ContactList::iterator contactI = frontContacts->begin();
        contactI != frontContacts->end(); ++contactI)
    {
      solveInterpenetration(*contactI);
    }


    // Sort in closing velocity decreasing order
    frontContacts->sort(velocityCompare);

    // Solve velocities
    for (ContactList::iterator contactI = frontContacts->begin();
        contactI != frontContacts->end(); ++contactI)
    {
      solveVelocity(*contactI);
    }

  }



  // ---------------------------------------------------------------------------
  // Preprocess the Contact list
  void CollisionSolver::preprocessContacts (
      ContactList* frontContacts,
      ContactList* backContacts)
  {
    for (ContactList::iterator contactI = frontContacts->begin();
        contactI != frontContacts->end(); ++contactI)
    {
      // Swap if needed
      if ((*contactI)->body[0]->isStatic())
      {
        (*contactI)->swap();
      }


      // Save the contact point for persistence
      (*contactI)->persistencePoint[0][0] = (*contactI)->point[0];
      (*contactI)->body[0]->transformLocal(&(*contactI)->persistencePoint[0][0]);
      (*contactI)->persistencePoint[0][1] = (*contactI)->point[1];
      (*contactI)->body[1]->transformLocal(&(*contactI)->persistencePoint[0][1]);


      // Check the shapes for contact persistence (only RECT & POLYGON)
      if ((*contactI)->body[0]->getShape()->getType() != Shape::CIRCLE &&
          (*contactI)->body[1]->getShape()->getType() != Shape::CIRCLE)
      {
        // Search for a contact in the last frame
        // TODO: optimize
        for (ContactList::iterator contactY = backContacts->begin();
            contactY != backContacts->end(); ++contactY)
        {
          if ((*contactI)->body[0] == (*contactY)->body[0] &&
              (*contactI)->body[1] == (*contactY)->body[1])
          {
            std::cerr << "##################################################\n";
            std::cerr << "Found a previous contact\n";
            Vector2 point[2];

            switch ((*contactY)->nContacts)
            {

              case 1: // Found a previous single contact
                //std::cerr << "p[0] = " << point[0] << "\n";
                //std::cerr << "P[0] = " << (*contactY)->persistencePoint[0][0] << "\n";
                //std::cerr << point[0] - (*contactY)->persistencePoint[0][0] << "\n";
                //std::cerr << "p[1] = " << point[1] << "\n";
                //std::cerr << "P[1] = " << (*contactY)->persistencePoint[0][1] << "\n";
                //std::cerr << point[1] - (*contactY)->persistencePoint[0][1] << "\n";

                // If it is not the same
                if (
                    ((*contactI)->persistencePoint[0][0] !=
                     (*contactY)->persistencePoint[0][0]) &&
                    ((*contactI)->persistencePoint[0][1] !=
                     (*contactY)->persistencePoint[0][1])
                   )
                {
                  // Check if the previous contact is still valid
                  point[0] = (*contactY)->persistencePoint[0][0];
                  point[1] = (*contactY)->persistencePoint[0][1];
                  (*contactI)->body[0]->transformWorld(&point[0]);
                  (*contactI)->body[1]->transformWorld(&point[1]);
                  // TODO: define a constant for persistence distance
                  if (dot(point[1] - point[0], (*contactY)->normal) > 0 ||
                      (point[1] - point[0]).getMagnitude() < 0.05)
                  {
                    // Add the current point for the persistence
                    (*contactI)->persistencePoint[1][0] = (*contactY)->persistencePoint[0][0];
                    (*contactI)->persistencePoint[1][1] = (*contactY)->persistencePoint[0][1];
                    (*contactI)->nContacts = 2;
                    std::cerr << "Found a contact suitable for persistence [1]\n";
                  }
                }
                break;


              case 2: // Found a previous double (persistence) contact
                // Check if the previous contact 0 is valid
                point[0] = (*contactY)->persistencePoint[0][0];
                point[1] = (*contactY)->persistencePoint[0][1];
                (*contactI)->body[0]->transformWorld(&point[0]);
                (*contactI)->body[1]->transformWorld(&point[1]);
                // TODO: define a constant for persistence distance
                if (dot(point[1] - point[0], (*contactY)->normal) > 0 ||
                    (point[1] - point[0]).getMagnitude() < 0.05)
                {
                  (*contactI)->nContacts = 2;
                }
                else // Not valid
                {
                  (*contactI)->nContacts = 1;
                }

                // Check if the previous contact 1 is valid
                point[0] = (*contactY)->persistencePoint[1][0];
                point[1] = (*contactY)->persistencePoint[1][1];
                (*contactI)->body[0]->transformWorld(&point[0]);
                (*contactI)->body[1]->transformWorld(&point[1]);
                // TODO: define a constant for persistence distance
                if (dot(point[1] - point[0], (*contactY)->normal) > 0 ||
                    (point[0] - point[1]).getMagnitude() < 0.05)
                {
                  if ((*contactI)->nContacts == 2) // (V, V)
                  {
                    Vector2 tpoint[2];
                    Vector2 midPoint[3];

                    tpoint[0] = (*contactY)->persistencePoint[0][0];
                    (*contactI)->body[0]->transformWorld(&tpoint[0]);
                    tpoint[1] = (*contactY)->persistencePoint[0][1];
                    (*contactI)->body[1]->transformWorld(&tpoint[1]);
                    midPoint[0] = (tpoint[0] + tpoint[1]) * 0.5;

                    tpoint[0] = (*contactY)->persistencePoint[1][0];
                    (*contactI)->body[0]->transformWorld(&tpoint[0]);
                    tpoint[1] = (*contactY)->persistencePoint[1][1];
                    (*contactI)->body[1]->transformWorld(&tpoint[1]);
                    midPoint[1] = (tpoint[0] + tpoint[1]) * 0.5;

                    midPoint[2] = ((*contactI)->point[0] + (*contactI)->point[1]) * 0.5;

                    // Keep the two furthest points
                    if (dot(midPoint[0], (*contactI)->tangent) >
                        dot(midPoint[1], (*contactI)->tangent))
                    {
                      if (dot(midPoint[2], (*contactI)->tangent) >=
                          dot(midPoint[0], (*contactI)->tangent))   // (2, 0, 1)
                      {
                        (*contactI)->persistencePoint[1][0] = (*contactY)->persistencePoint[1][0];
                        (*contactI)->persistencePoint[1][1] = (*contactY)->persistencePoint[1][1];
                      }
                      else if (dot(midPoint[2], (*contactI)->tangent) <=
                          dot(midPoint[1], (*contactI)->tangent))   // (0, 1, 2)
                      {
                        (*contactI)->persistencePoint[1][0] = (*contactY)->persistencePoint[0][0];
                        (*contactI)->persistencePoint[1][1] = (*contactY)->persistencePoint[0][1];
                      }
                      else                                          // (0, 2, 1)
                      {
                        (*contactI)->persistencePoint[0][0] = (*contactY)->persistencePoint[0][0];
                        (*contactI)->persistencePoint[0][1] = (*contactY)->persistencePoint[0][1];
                        (*contactI)->persistencePoint[1][0] = (*contactY)->persistencePoint[1][0];
                        (*contactI)->persistencePoint[1][1] = (*contactY)->persistencePoint[1][1];
                      }
                    }
                    else // (1 > 0)
                    {
                      if (dot(midPoint[2], (*contactI)->tangent) >=
                          dot(midPoint[1], (*contactI)->tangent))   // (2, 1, 0)
                      {
                        (*contactI)->persistencePoint[1][0] = (*contactY)->persistencePoint[0][0];
                        (*contactI)->persistencePoint[1][1] = (*contactY)->persistencePoint[0][1];
                      }
                      else if (dot(midPoint[2], (*contactI)->tangent) <=
                          dot(midPoint[0], (*contactI)->tangent))   // (1, 0, 2)
                      {
                        (*contactI)->persistencePoint[1][0] = (*contactY)->persistencePoint[1][0];
                        (*contactI)->persistencePoint[1][1] = (*contactY)->persistencePoint[1][1];
                      }
                      else                                          // (1, 2, 0)
                      {
                        (*contactI)->persistencePoint[0][0] = (*contactY)->persistencePoint[0][0];
                        (*contactI)->persistencePoint[0][1] = (*contactY)->persistencePoint[0][1];
                        (*contactI)->persistencePoint[1][0] = (*contactY)->persistencePoint[1][0];
                        (*contactI)->persistencePoint[1][1] = (*contactY)->persistencePoint[1][1];
                      }

                    }

                    (*contactI)->point[0] =
                      ((*contactY)->persistencePoint[0][0] +
                       (*contactY)->persistencePoint[1][0]) * 0.5;
                    (*contactI)->body[0]->transformWorld(&(*contactI)->point[0]);
                    (*contactI)->point[1] =
                      ((*contactY)->persistencePoint[0][1] +
                       (*contactY)->persistencePoint[1][1]) * 0.5;
                    (*contactI)->body[1]->transformWorld(&(*contactI)->point[1]);
                    (*contactI)->penetrationDepth =
                      ((*contactI)->point[0] -
                       (*contactI)->point[1]).getMagnitude();
                    std::cerr << "Found a contact suitable for persistence [2]\n";
                    std::cerr << "Persistence (V, V)\n";
                  }
                  else // (X, V)
                  {
                    // Add the current point for the persistence
                    (*contactI)->persistencePoint[1][0] = (*contactY)->persistencePoint[1][0];
                    (*contactI)->persistencePoint[1][1] = (*contactY)->persistencePoint[1][1];
                    (*contactI)->nContacts = 2;
                    std::cerr << "Persistence (X, V)\n";
                    std::cerr << "Found a contact suitable for persistence [1]\n";
                  }
                }
                else // Not valid
                {
                  if ((*contactI)->nContacts == 2) // (V, X)
                  {
                    // Add the current point for the persistence
                    (*contactI)->persistencePoint[1][0] = (*contactY)->persistencePoint[0][0];
                    (*contactI)->persistencePoint[1][1] = (*contactY)->persistencePoint[0][1];
                    (*contactI)->nContacts = 2;
                    std::cerr << "Persistence (V, X)\n";
                    std::cerr << "Found a contact suitable for persistence [1]\n";
                  }
                  else // (X, X)
                  {
                    // Keep only the current point
                    //(*contactI)->nContacts = 1;
                    std::cerr << "Persistence (X, X)\n";
                  }
                }
                break;


              default:
                // This should never happens
                assert(false);
            }

            std::cerr << "##################################################\n";
            break;
          }
        }
      }


      // Calculate the normal
      (*contactI)->normal = ((*contactI)->point[1] - (*contactI)->point[0]);
      (*contactI)->normal.normalize();

      // Calculate the tangent
      (*contactI)->tangent = (*contactI)->normal.getPerpendicularCopy();


      // Calculate the relative points
      (*contactI)->relativeContactPoint[0] = (*contactI)->point[0] -
        (*contactI)->body[0]->getPosition();
      if ((*contactI)->body[1]->isDynamic())
      {
        (*contactI)->relativeContactPoint[1] = (*contactI)->point[1] -
          (*contactI)->body[1]->getPosition();
      }

      Vector2 velocity =
        (*contactI)->body[0]->getVelocity() +
        (*contactI)->body[0]->getRotation() *
        (*contactI)->relativeContactPoint[0].getPerpendicularCopy();
      if ((*contactI)->body[1]->isDynamic())
      {
        velocity -=
          (*contactI)->body[1]->getVelocity() +
          (*contactI)->body[1]->getRotation() *
          (*contactI)->relativeContactPoint[1].getPerpendicularCopy();
      }

      // Calculate the closing velocity
      (*contactI)->closingVelocity = dot(velocity, (*contactI)->normal);

      // Calculate the sliding velocity
      (*contactI)->slidingVelocity = dot(velocity, (*contactI)->tangent);


      // Calculate the linear inertia
      (*contactI)->linearInertia[0] = (1.0 / (*contactI)->body[0]->getMass());
      if ((*contactI)->body[1]->isDynamic())
      {
        (*contactI)->linearInertia[1] = (1.0 / (*contactI)->body[1]->getMass());
      }

      // Calculate the angular inertia
      Real tcross = cross(
          (*contactI)->relativeContactPoint[0],
          (*contactI)->normal);
      (*contactI)->angularInertia[0] = tcross * tcross /
        (*contactI)->body[0]->getMomentOfInertia();
      if ((*contactI)->body[1]->isDynamic())
      {
        tcross = cross(
            (*contactI)->relativeContactPoint[1],
            (*contactI)->normal);
        (*contactI)->angularInertia[1] = tcross * tcross /
          (*contactI)->body[1]->getMomentOfInertia();
      }
    }

  }



  // ---------------------------------------------------------------------------
  // Solve interpenetrations
  void CollisionSolver::solveInterpenetration (Contact* contact)
  {
    Vector2 Rap = contact->relativeContactPoint[0];
    Vector2 Rbp;
    Vector2 n   = contact->normal;
    Real p      = contact->penetrationDepth;

    Real lIa    = contact->linearInertia[0];
    Real lIb;
    Real aIa    = contact->angularInertia[0];
    Real aIb;

    Real tI     = lIa + aIa;

    if (contact->body[1]->isDynamic())
    {
      Rbp = contact->relativeContactPoint[1];

      lIb = contact->linearInertia[1];
      aIb = contact->angularInertia[1];

      tI += lIb + aIb;
    }


    Real itI  = (1.0 / tI);

    Real lMa =  p * lIa * itI;
    Real lMb;
    Real aMa =  p * aIa * itI;
    Real aMb;

    if (contact->body[1]->isDynamic())
    {
      lMb = -p * lIb * itI;
      aMb = -p * aIb * itI;
    }


    // Angular correction
    // TODO: recalculate preprocessing
#if 0
    const Real angularLimitConstant = 0.2;

    Real aLa = angularLimitConstant * Rap.getMagnitude();
    Real aLb;

    if (s2fabs(aMa) > aLa)
    {
      Real tM = lMa + aMa;

      (aMa > 0) ? aMa = aLa : aMa = -aLa;

      lMa = tM - aMa;
    }

    if (contact->body[1]->isDynamic())
    {
      aLb = angularLimitConstant * Rbp.getMagnitude();

      if (s2fabs(aMb) > aLb)
      {
        Real tM = lMb + aMb;

        (aMb > 0) ? aMb = aLb : aMb = -aLb;

        lMb = tM - aMb;
      }
    }
#endif


    Vector2 pa1 = contact->body[0]->getPosition();
    Vector2 pb1;
    contact->body[0]->setPosition(pa1 + lMa * n);

    Complex oa(contact->body[0]->getOrientation());
    Complex ob;

    oa.rotate(s2copysign(1, cross(Rap, n)) * aMa / Rap.getMagnitude());
    contact->body[0]->setOrientation(oa);


    if (contact->body[1]->isDynamic())
    {
      pb1 = contact->body[1]->getPosition();
      contact->body[1]->setPosition(pb1 + lMb * n);

      ob = Complex(contact->body[1]->getOrientation());

      ob.rotate(s2copysign(1, cross(Rbp, n)) * aMb / Rbp.getMagnitude());
      contact->body[1]->setOrientation(ob);
    }
  }



  // ---------------------------------------------------------------------------
  // Solve velocities
  void CollisionSolver::solveVelocity (Contact* contact)
  {
    // Restitution
    // b = before, a = after
    // e = -(v1a - v2a) / (v1b - v2b)
    contact->restitution = 0.2;
    contact->friction = 0.9;
    Vector2 n = contact->normal;

    Vector2 Rap = contact->relativeContactPoint[0];
    Vector2 Rbp;

    Vector2 Va1 = contact->body[0]->getVelocity();
    Vector2 Vb1;
    Real Wa1    = contact->body[0]->getRotation();
    Real Wb1;


    if (contact->body[1]->isDynamic())
    {
      Rbp = contact->relativeContactPoint[1];
      Vb1 = contact->body[1]->getVelocity();
      Wb1 = contact->body[1]->getRotation();
    }


    Real cV = contact->closingVelocity;

    // TODO: check =
    if (cV >= 0)
    {
      return;
    }


    Real e = contact->restitution;
    Real Ma = contact->body[0]->getMass();
    Real Mb;
    Real Ia = contact->body[0]->getMomentOfInertia();
    Real Ib;


    Mb = contact->body[1]->getMass();
    Ib = contact->body[1]->getMomentOfInertia();


    // Calculate the normal impulse
    // TODO: remove velocity caused by only acceleration (gravity)
    Real Jnorm =
      -(1 + e) * cV /
      (
       contact->linearInertia[0] +
       contact->angularInertia[0]
      );
    if (contact->body[1]->isDynamic())
    {
      // TODO: remove velocity caused by only acceleration (gravity)
      Jnorm =
        -(1 + e) * cV /
        (
         contact->linearInertia[0] +
         contact->linearInertia[1] +
         contact->angularInertia[0] +
         contact->angularInertia[1]
        );

    }


    Real sV = contact->slidingVelocity;

    // Calculate the angular inertia on the tangent
    Real angularInertia[2];
    Real tcross = cross(
        contact->relativeContactPoint[0],
        contact->tangent);
    angularInertia[0] = tcross * tcross /
      contact->body[0]->getMomentOfInertia();
    if (contact->body[1]->isDynamic())
    {
      tcross = cross(
          contact->relativeContactPoint[1],
          contact->tangent);
      angularInertia[1] = tcross * tcross /
        contact->body[1]->getMomentOfInertia();
    }

    // Calculate the tangent impulse
    Real Jtang =
      -sV /
      (
       contact->linearInertia[0] +
       angularInertia[0]
      );
    if (contact->body[1]->isDynamic())
    {
      Jtang =
        -sV /
        (
         contact->linearInertia[0] +
         contact->linearInertia[1] +
         angularInertia[0] +
         angularInertia[1]
        );
    }


    // Clamp if dynamic friction
    if (s2fabs(Jtang) > Jnorm * contact->friction)
    {
      Jtang = s2copysign(Jnorm * contact->friction, Jtang);
    }

    Vector2 J = Jnorm * contact->normal + Jtang * contact->tangent;


    // Apply the impulse (velocity)
    contact->body[0]->setVelocity(Va1 + J * (1.0 / Ma));

    // Apply the impulse (rotation)
    contact->body[0]->setRotation(Wa1 + cross(Rap, J) * (1.0 / Ia));


    if (contact->body[1]->isDynamic())
    {
      // Apply the impulse (velocity)
      contact->body[1]->setVelocity(Vb1 - J * (1.0 / Mb));
      // Apply the impulse (rotation)
      contact->body[1]->setRotation(Wb1 - cross(Rbp, J) * (1.0 / Ib));
    }

  }





  // ---------------------------------------------------------------------------
  // The compare function for the interpenetration based ordering
  bool interpenetrationCompare (Contact* CONTACT1, Contact* CONTACT2)
  {
    return (CONTACT1->penetrationDepth > CONTACT2->penetrationDepth);
  }



  // ---------------------------------------------------------------------------
  // The compare function for the velocity based ordering
  bool velocityCompare (Contact* CONTACT1, Contact* CONTACT2)
  {
    // Use < because the closing velocities are negative
    return (CONTACT1->closingVelocity < CONTACT2->closingVelocity);
  }


}
