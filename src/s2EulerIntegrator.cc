#include "../include/s2EulerIntegrator.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Compute the new position of each body & update velocity and acceleration
  void EulerIntegrator::compute (Environment *environment) const
  {
    const Environment::BodyList bodyList = environment->getBodyList();
    Environment::BodyList::const_iterator body;

    for (body = bodyList.begin(); body != bodyList.end(); ++body)
    {
      // Update the position
      (*body)->position_ += (*body)->velocity_ * timeStep_;

      // Update the velocity
      (*body)->velocity_ += (*body)->acceleration_ * timeStep_;
    }

  }


}
