#include "../include/s2EulerIntegrator.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Compute the new position of each body & update velocity and acceleration
  void EulerIntegrator::compute (Environment *environment) const
  {
    Environment::BodyList *bodyList = environment->getBodyList();
    Environment::BodyList::iterator body;

    for (body = bodyList->begin(); body != bodyList->end(); ++body)
    {
      // Update the position
      (*body)->position += (*body)->velocity * timeStep_;

      // Update the velocity
      (*body)->velocity += (*body)->acceleration * timeStep_;
    }

  }


}
