#include "../include/s2Environment.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Create a new body in the environment
  Body* Environment::createBody (const Vector2& POSITION, const Vector2& VELOCITY)
  {
    Body *body = new Body(POSITION, VELOCITY);
    bodyList_.push_back(body);
    return body;
  }


  // ---------------------------------------------------------------------------
  // Destroy the specified body
  void Environment::destroyBody (Body* body)
  {
    bodyList_.remove(body);
    delete body;
  }


  // ---------------------------------------------------------------------------
  // Update positions, velocities and acceleration for each dynamic body
  void Environment::integrateAllBody ()
  {
    for (BodyList::iterator body = bodyList_.begin();
         body != bodyList_.end();
         ++body)
    {
      // Integrate
      (*body)->integrate(timeStep_);
    }
  }

}
