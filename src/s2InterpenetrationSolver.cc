#include "../include/s2InterpenetrationSolver.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Solve interpenetrations
  void InterpenetrationSolver::solveInterpenetration (ContactSet* contacts)
  {
    // TODO: sort in penetration depth order OR preprocess it

    // TODO: remove this (debug)
    if (contacts->size() == 0)
      return;

    Contact* contact = *contacts->begin();

    // TODO: preprocess these
    contact->normal =
      (contact->point[0] - contact->point[1]).getNormalizedCopy();

    std::cerr << "normal  = " << contact->normal << "\n";
    std::cerr << "penetration = " << contact->penetrationDepth << "\n";
    std::cerr << "p1 = " << contact->point[0] << "\n";
    std::cerr << "p2 = " << contact->point[1] << "\n";


    Vector2 Rap = contact->point[0] - contact->body[0]->getPosition();
    Vector2 Rbp = contact->point[1] - contact->body[1]->getPosition();
    Vector2 n   = -contact->normal;

    Real Ma = contact->body[0]->getMass();
    Real Mb = contact->body[1]->getMass();
    Real Ia = contact->body[0]->getMomentOfInertia();
    Real Ib = contact->body[1]->getMomentOfInertia();


    Real p  = contact->penetrationDepth;

    Real lIa  = (1.0 / Ma);
    Real lIb  = (1.0 / Mb);

    Real aIa  = (cross(Rap, n) * cross(Rap, n) / Ia);
    Real aIb  = (cross(Rbp, n) * cross(Rbp, n) / Ib);

    Real tI   = lIa + aIa + lIb + aIb;
    Real itI  = (1.0 / tI);

    Vector2 pa1 = contact->body[0]->getPosition();
    Vector2 pb1 = contact->body[1]->getPosition();
    Complex oa(contact->body[0]->getOrientation());
    Complex ob(contact->body[1]->getOrientation());

    std::cerr << "itI = " << itI << "\n";
    std::cerr << "lMa = " <<  p * lIa * itI * n << "\n";
    std::cerr << "lMb = " << -p * lIb * itI * n << "\n";
    std::cerr << "aMa = " <<  p * aIa * itI  << "\n";
    std::cerr << "aMb = " << -p * aIb * itI  << "\n";

    contact->body[0]->setPosition(pa1 + p * lIa * itI * n);
    contact->body[1]->setPosition(pb1 - p * lIb * itI * n);

    oa.rotate( p * aIa * itI / Rap.getMagnitude());
    ob.rotate( p * aIb * itI / Rbp.getMagnitude());
    contact->body[0]->setOrientation(oa);
    contact->body[1]->setOrientation(ob);

  }


}
