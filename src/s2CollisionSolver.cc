#include "../include/s2CollisionSolver.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Solve collisions
  void CollisionSolver::solveCollisions (ContactList* contacts)
  {
    // Early out
    if (contacts->empty())
    {
      return;
    }

    // Preprocess contacts
    preprocessContacts(contacts);


    // Sort in penetration depth decreasing order
    contacts->sort(interpenetrationCompare);

    // Solve interpenetration
    // TODO: update & sort again
    //       extract from the main list, sort in a new list & merge
    for (ContactList::iterator contactI = contacts->begin();
        contactI != contacts->end(); ++contactI)
    {
      solveInterpenetration(*contactI);
    }


    // Sort in closing velocity decreasing order
    contacts->sort(velocityCompare);

    // Solve velocities
    for (ContactList::iterator contactI = contacts->begin();
        contactI != contacts->end(); ++contactI)
    {
      solveVelocity(*contactI);
    }

  }



  // ---------------------------------------------------------------------------
  // Preprocess the Contact list
  void CollisionSolver::preprocessContacts ( ContactList* contacts)
  {
    for (ContactList::iterator contactI = contacts->begin();
        contactI != contacts->end(); ++contactI)
    {
      // Swap if needed
      if ((*contactI)->body[0]->isStatic())
      {
        (*contactI)->swap();
      }

      // Calculate the normal
      (*contactI)->normal = ((*contactI)->point[1] - (*contactI)->point[0]);
      (*contactI)->normal.normalize();

      // Calculate the tangent
      (*contactI)->tangent = (*contactI)->normal.getPerpendicularCopy();


      // Calculate the relative points
      (*contactI)->relativeContactPoint[0] = (*contactI)->point[0] -
        (*contactI)->body[0]->getPosition();
      if ((*contactI)->body[1]->isStatic() == false)
      {
        (*contactI)->relativeContactPoint[1] = (*contactI)->point[1] -
          (*contactI)->body[1]->getPosition();
      }

      Vector2 velocity =
        (*contactI)->body[0]->getVelocity() +
        (*contactI)->body[0]->getRotation() *
        (*contactI)->relativeContactPoint[0].getPerpendicularCopy();
      if ((*contactI)->body[1]->isStatic() == false)
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
      (*contactI)->linearInertia[0] = (*contactI)->body[0]->getInverseMass();
      if ((*contactI)->body[1]->isStatic() == false)
      {
        (*contactI)->linearInertia[1] = (*contactI)->body[1]->getInverseMass();
      }

      // Calculate the angular inertia
      Real tcross = cross(
          (*contactI)->relativeContactPoint[0],
          (*contactI)->normal);
      (*contactI)->angularInertia[0] = tcross * tcross *
        (*contactI)->body[0]->getInverseMomentOfInertia();
      if ((*contactI)->body[1]->isStatic() == false)
      {
        tcross = cross(
            (*contactI)->relativeContactPoint[1],
            (*contactI)->normal);
        (*contactI)->angularInertia[1] = tcross * tcross *
          (*contactI)->body[1]->getInverseMomentOfInertia();
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

    if (contact->body[1]->isStatic() == false)
    {
      Rbp = contact->relativeContactPoint[1];

      lIb = contact->linearInertia[1];
      aIb = contact->angularInertia[1];

      tI += lIb + aIb;
    }


    Real itI  = (1 / tI);

    Real lMa =  p * lIa * itI;
    Real lMb;
    Real aMa =  p * aIa * itI;
    Real aMb;

    if (contact->body[1]->isStatic() == false)
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

    if (contact->body[1]->isStatic() == false)
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

    Complex oa = contact->body[0]->getOrientation();
    Complex ob;

    oa.rotate(s2copysign(1, cross(Rap, n)) * aMa / Rap.getMagnitude());
    contact->body[0]->setOrientation(oa);


    if (contact->body[1]->isStatic() == false)
    {
      pb1 = contact->body[1]->getPosition();
      contact->body[1]->setPosition(pb1 + lMb * n);

      ob = contact->body[1]->getOrientation();

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
    contact->friction = 0.5;
    Vector2 n = contact->normal;

    Vector2 Rap = contact->relativeContactPoint[0];
    Vector2 Rbp;

    Vector2 Va1 = contact->body[0]->getVelocity();
    Vector2 Vb1;
    Real Wa1    = contact->body[0]->getRotation();
    Real Wb1;


    if (contact->body[1]->isStatic() == false)
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
    Real iMa = contact->body[0]->getInverseMass();
    Real iMb;
    Real iIa = contact->body[0]->getInverseMomentOfInertia();
    Real iIb;


    if (contact->body[1]->isStatic() == false)
    {
      iMb = contact->body[1]->getInverseMass();
      iIb = contact->body[1]->getInverseMomentOfInertia();
    }


    // Remove velocity caused by only acceleration (gravity)
    Real aV =
      dot(contact->body[0]->getVelocityFromAcceleration(), contact->normal);

    if (contact->body[1]->isStatic() == false)
    {
      aV -=
        dot(contact->body[1]->getVelocityFromAcceleration(), contact->normal);
    }

    // Calculate the normal impulse
    Real Jnorm =
      -(1 + e) * (cV - aV) /
      (
       contact->linearInertia[0] +
       contact->angularInertia[0]
      );
    if (contact->body[1]->isStatic() == false)
    {
      Jnorm =
        -(1 + e) * (cV - aV) /
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
    angularInertia[0] = tcross * tcross *
      contact->body[0]->getInverseMomentOfInertia();
    if (contact->body[1]->isStatic() == false)
    {
      tcross = cross(
          contact->relativeContactPoint[1],
          contact->tangent);
      angularInertia[1] = tcross * tcross *
        contact->body[1]->getInverseMomentOfInertia();
    }

    // Calculate the tangent impulse
    Real Jtang =
      -sV /
      (
       contact->linearInertia[0] +
       angularInertia[0]
      );
    if (contact->body[1]->isStatic() == false)
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
      // TODO: check the theory
      Jtang = s2copysign(Jnorm * contact->friction, Jtang);
    }

    Vector2 J = Jnorm * contact->normal + Jtang * contact->tangent;

    // Apply the impulse (velocity)
    contact->body[0]->setVelocity(Va1 + J * iMa);

    // Apply the impulse (rotation)
    contact->body[0]->setRotation(Wa1 + cross(Rap, J) * iIa);


    if (contact->body[1]->isStatic() == false)
    {
      // Apply the impulse (velocity)
      contact->body[1]->setVelocity(Vb1 - J * iMb);
      // Apply the impulse (rotation)
      contact->body[1]->setRotation(Wb1 - cross(Rbp, J) * iIb);
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
