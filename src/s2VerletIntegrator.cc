#include "../include/s2VerletIntegrator.h"

static bool firstStep = true;

namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Compute the new position of each body & update velocity and acceleration
  void VerletIntegrator::compute (Environment *environment) const
  {
    Vector oldAcceleration;
    const Environment::BodyList bodyList = environment->getBodyList();
    Environment::BodyList::const_iterator body;

    for (body = bodyList.begin(); body != bodyList.end(); ++body)
    {
      // Update the acceleration
      oldAcceleration = (*body)->acceleration_;
      (*body)->acceleration_ = (*body)->netForce_ * (1.0 / (*body)->mass_);
      if (firstStep)
      {
        oldAcceleration = (*body)->acceleration_;
      }

      // Update the position
      (*body)->position_ +=
        ((*body)->velocity_ * timeStep_) +
        (0.5 * (*body)->acceleration_ * timeStep_ * timeStep_ );

      // Update the velocity
      (*body)->velocity_ +=
        0.5 * (oldAcceleration + (*body)->acceleration_) * timeStep_;


      // Clear the net force
      (*body)->clearNetForce();
    }

    firstStep = false;
  }


}
