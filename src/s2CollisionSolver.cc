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
  void CollisionSolver::preprocessContacts (ContactList* contacts)
  {
    for (ContactList::iterator contactI = contacts->begin();
        contactI != contacts->end(); ++contactI)
    {
      // Calculate the normal
      (*contactI)->normal = ((*contactI)->point[1] - (*contactI)->point[0]);
      (*contactI)->normal.normalize();

      // Calculate the relative points
      (*contactI)->relativeContactPoint[0] = (*contactI)->point[0] -
        (*contactI)->body[0]->getPosition();
      (*contactI)->relativeContactPoint[1] = (*contactI)->point[1] -
        (*contactI)->body[1]->getPosition();

      // Calculate the closing velocity
      (*contactI)->closingVelocity = dot(
          (((*contactI)->body[0]->getVelocity() +
            (*contactI)->body[0]->getRotation() *
            (*contactI)->relativeContactPoint[0].getPerpendicularCopy()) -
           ((*contactI)->body[1]->getVelocity() +
            (*contactI)->body[1]->getRotation() *
            (*contactI)->relativeContactPoint[1].getPerpendicularCopy())),
          (*contactI)->normal);

      // Calculate the linear inertia
      (*contactI)->linearInertia[0] = (1.0 / (*contactI)->body[0]->getMass());
      (*contactI)->linearInertia[1] = (1.0 / (*contactI)->body[1]->getMass());

      // Calculate the angular inertia
      Real tcross = cross(
          (*contactI)->relativeContactPoint[0],
          (*contactI)->normal);
      (*contactI)->angularInertia[0] = tcross * tcross /
        (*contactI)->body[0]->getMomentOfInertia();
      tcross = cross(
          (*contactI)->relativeContactPoint[1],
          (*contactI)->normal);
      (*contactI)->angularInertia[1] = tcross * tcross /
        (*contactI)->body[1]->getMomentOfInertia();

    }

  }



  // ---------------------------------------------------------------------------
  // Solve interpenetrations
  void CollisionSolver::solveInterpenetration (Contact* contact)
  {
    std::cerr << "normal  = " << contact->normal << "\n";
    std::cerr << "penetration = " << contact->penetrationDepth << "\n";
    std::cerr << "p1 = " << contact->point[0] << "\n";
    std::cerr << "p2 = " << contact->point[1] << "\n";

    Vector2 Rap = contact->relativeContactPoint[0];
    Vector2 Rbp = contact->relativeContactPoint[1];
    Vector2 n   = contact->normal;

    Real p  = contact->penetrationDepth;


    Real lIa  = contact->linearInertia[0];
    Real lIb  = contact->linearInertia[1];

    Real aIa  = contact->angularInertia[0];
    Real aIb  = contact->angularInertia[1];


    std::cerr << "lIa = " << lIa << "\n";
    std::cerr << "lIb = " << lIb << "\n";
    std::cerr << "aIa = " << aIa << "\n";
    std::cerr << "aIb = " << aIb << "\n";

    std::cerr << "Rap = " << Rap << "\n";
    std::cerr << "Rbp = " << Rbp << "\n";



    Real tI   = lIa + aIa + lIb + aIb;
    Real itI  = (1.0 / tI);


    Real lMa =  p * lIa * itI;
    Real lMb = -p * lIb * itI;

    Real aMa =  p * aIa * itI;
    Real aMb = -p * aIb * itI;




    std::cerr << "itI = " << itI << "\n";
    std::cerr << "lMa = " << lMa * n << "\n";
    std::cerr << "lMb = " << lMb * n << "\n";
    std::cerr << "aMa = " << aMa << "\n";
    std::cerr << "aMb = " << aMb << "\n";



    // Angular correction
    // TODO: recalculate preprocessing
#if 0
    const Real angularLimitConstant = 0.2;

    Real aLa = angularLimitConstant * Rap.getMagnitude();
    Real aLb = angularLimitConstant * Rbp.getMagnitude();

    std::cerr << "aLa = " << aLa << "\n";
    std::cerr << "aLb = " << aLb << "\n";

    if (s2fabs(aMa) > aLa)
    {
      Real tM = lMa + aMa;

      (aMa > 0) ? aMa = aLa : aMa = -aLa;

      lMa = tM - aMa;
    }

    if (s2fabs(aMb) > aLb)
    {
      Real tM = lMb + aMb;

      (aMb > 0) ? aMb = aLb : aMb = -aLb;

      lMb = tM - aMb;
    }
#endif

    std::cerr << "lMa = " << lMa * n << "\n";
    std::cerr << "lMb = " << lMb * n << "\n";
    std::cerr << "aMa = " << aMa << "\n";
    std::cerr << "aMb = " << aMb << "\n";

    std::cerr << "tM = " << lMa - lMb + aMa - aMb << "\n";

    Vector2 pa1 = contact->body[0]->getPosition();
    Vector2 pb1 = contact->body[1]->getPosition();

    contact->body[0]->setPosition(pa1 + lMa * n);
    contact->body[1]->setPosition(pb1 + lMb * n);

    std::cerr << "la = " << lMa * n << "\n";
    std::cerr << "lb = " << lMb * n << "\n";

#if 1
    Complex oa(contact->body[0]->getOrientation());
    Complex ob(contact->body[1]->getOrientation());

    std::cerr << "oa = " << oa << "\n";
    std::cerr << "ob = " << ob << "\n";

    std::cerr << "+-a = " << cross(Rap, n) << "\n";
    std::cerr << "+-b = " << cross(Rbp, n) << "\n";
    std::cerr << "copysigna = " << s2copysign(1, cross(Rap, n)) << "\n";
    std::cerr << "copysignb = " << s2copysign(1, cross(Rbp, n)) << "\n";

    oa.rotate(s2copysign(1, cross(Rap, n)) * aMa / Rap.getMagnitude());
    ob.rotate(s2copysign(1, cross(Rbp, n)) * aMb / Rbp.getMagnitude());
    contact->body[0]->setOrientation(oa);
    contact->body[1]->setOrientation(ob);

    std::cerr << "oa = " << oa << "\n";
    std::cerr << "ob = " << ob << "\n";

    std::cerr << "aa = " << (aMa / Rap.getMagnitude()) << "\n";
    std::cerr << "ab = " << (aMb / Rbp.getMagnitude()) << "\n";
#endif
  }





  // ---------------------------------------------------------------------------
  // Solve velocities
  void CollisionSolver::solveVelocity (Contact* contact)
  {
    std::cerr << "normal  = " << contact->normal << "\n";
    std::cerr << "penetration = " << contact->penetrationDepth << "\n";
    std::cerr << "p1 = " << contact->point[0] << "\n";
    std::cerr << "p2 = " << contact->point[1] << "\n";

    // Restitution
    // b = before, a = after
    // e = -(v1a - v2a) / (v1b - v2b)
    contact->restitution = 1;

    Vector2 Rap = contact->relativeContactPoint[0];
    Vector2 Rbp = contact->relativeContactPoint[1];
    Vector2 n   = contact->normal;

    Vector2 Va1 = contact->body[0]->getVelocity();
    Real    Wa1 = contact->body[0]->getRotation();

    Vector2 Vb1 = contact->body[1]->getVelocity();
    Real    Wb1 = contact->body[1]->getRotation();


    Real cV = contact->closingVelocity;
    std::cerr << "cV = " << cV << "\n";

    // TODO: check =
    if (cV >= 0)
    {
      return;
    }

    Real e = contact->restitution;
    Real Ma = contact->body[0]->getMass();
    Real Mb = contact->body[1]->getMass();
    Real Ia = contact->body[0]->getMomentOfInertia();
    Real Ib = contact->body[1]->getMomentOfInertia();




    Real J =
      -(1 + e) * cV /
      (
       contact->linearInertia[0] +
       contact->linearInertia[1] +
       contact->angularInertia[0] +
       contact->angularInertia[1]
      );

    std::cerr << "J = " << J << "\n";



    // Apply the impulse (velocity)
    contact->body[0]->setVelocity(Va1 + J * n * (1.0 / Ma));
    contact->body[1]->setVelocity(Vb1 - J * n * (1.0 / Mb));

    // Apply the impulse (rotation)
    contact->body[0]->setRotation(Wa1 + cross(Rap, J * n) / Ia);
    contact->body[1]->setRotation(Wb1 - cross(Rbp, J * n) / Ib);

    std::cerr << "v1 = " << contact->body[0]->getVelocity() << "\n";
    std::cerr << "v2 = " << contact->body[1]->getVelocity() << "\n";
    std::cerr << "w1 = " << contact->body[0]->getRotation() << "\n";
    std::cerr << "w2 = " << contact->body[1]->getRotation() << "\n";
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
