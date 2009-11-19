#include "../include/s2Environment.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Create a new body in the environment
  Body* Environment::createBody (const Vector& POSITION, const Vector& VELOCITY)
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


}
