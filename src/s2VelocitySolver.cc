#include "../include/s2VelocitySolver.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Solve velocities
  void VelocitySolver::solveVelocity (ContactSet* contacts)
  {
    // TODO: sort in closing velocity order OR preprocess it

    // TODO: remove this (debug)
    if (contacts->size() == 0)
      return;

    Contact* contact = *contacts->begin();

    // TODO: preprocess these
    contact->normal =
      (contact->point[0] - contact->point[1]).getNormalizedCopy();
    // Restitution
    // b = before, a = after
    // e = -(v1a - v2a) / (v1b - v2b)
    contact->restitution = 1;

    std::cerr << "normal  = " << contact->normal << "\n";
    std::cerr << "penetration = " << contact->penetrationDepth << "\n";
    std::cerr << "p1 = " << contact->point[0] << "\n";
    std::cerr << "p2 = " << contact->point[1] << "\n";
    std::cerr << "w1 = " << contact->body[0]->getRotation() << "\n";
    std::cerr << "w2 = " << contact->body[1]->getRotation() << "\n";



    Vector2 Va1 = contact->body[0]->getVelocity();
    Real Wa1    = contact->body[0]->getRotation();
    Vector2 Rap = contact->point[0] - contact->body[0]->getPosition();

    Vector2 Vap1 = Va1 + Wa1 * Rap.getPerpendicularCopy();

    Vector2 Vb1 = contact->body[1]->getVelocity();
    Real Wb1    = contact->body[1]->getRotation();
    Vector2 Rbp = contact->point[1] - contact->body[1]->getPosition();

    Vector2 Vbp1 = Vb1 + Wb1 * Rbp.getPerpendicularCopy();


    Vector2 Vab1 = Vap1 - Vbp1;

    Vector2 n = -contact->normal;


    Real Vr = dot(Vab1, n);
    std::cerr << "Vr = " << Vr << "\n";

    if (Vr > 0)
    {
      return;
    }

    Real e = contact->restitution;
    Real Ma = contact->body[0]->getMass();
    Real Mb = contact->body[1]->getMass();
    Real Ia = contact->body[0]->getMomentOfInertia();
    Real Ib = contact->body[1]->getMomentOfInertia();




    Real J =
      -(1 + e) * dot(Vab1, n) /
     (
      (1.0 / Ma) +
      (1.0 / Mb) +
      (cross(Rap, n) * cross(Rap, n) / Ia) +
      (cross(Rbp, n) * cross(Rbp, n) / Ib)
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


}
