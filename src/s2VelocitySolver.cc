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
    contact->restitution = 1;
    contact->normal = Vector2::X;

    std::cerr << "normal = " << contact->normal << "\n";
    std::cerr << "v1 = " << contact->body[0]->getVelocity() << "\n";
    std::cerr << "v2 = " << contact->body[1]->getVelocity() << "\n";



    // Calculate the closing velocity in the normal direction
    Real closingVelocity =
      dot(contact->body[0]->getVelocity() - contact->body[1]->getVelocity(),
          contact->normal);
    std::cerr << "closing     = " << closingVelocity << "\n";

    // Calculate the separating velocity
    Real separatingVelocity = (1 + contact->restitution) * -closingVelocity;
    std::cerr << "separating  = " << separatingVelocity << "\n";

    // Solve only if the objects are not separating
    // TODO: needs ???
    if (separatingVelocity > 0)
    {
      return;
    }

    // Calculate the impulse
    Real impulse = separatingVelocity /
      ((1 / contact->body[0]->getMass()) + (1 / contact->body[1]->getMass()));

    Vector2 impulsePerUnit = contact->normal * impulse;

    // Apply the impulse
    contact->body[0]->setVelocity(
        contact->body[0]->getVelocity() +
        impulsePerUnit * (1.0 / contact->body[0]->getMass()));
    contact->body[1]->setVelocity(
        contact->body[1]->getVelocity() -
        impulsePerUnit * (1.0 / contact->body[1]->getMass()));

    std::cerr << "v1 = " << contact->body[0]->getVelocity() << "\n";
    std::cerr << "v2 = " << contact->body[1]->getVelocity() << "\n";

  }


}
