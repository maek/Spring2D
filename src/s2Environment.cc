#include "../include/s2Environment.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Create a new body in the environment
  Body* Environment::createBody (
      const Vector2& POSITION, const Vector2& VELOCITY, const Real ORIENTATION)
  {
    Body *body = new Body(POSITION, VELOCITY, Complex(ORIENTATION));
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
  // Update positions & velocities for each dynamic body
  void Environment::integrateBodies ()
  {
    for (BodyList::iterator bodyI = bodyList_.begin(); bodyI != bodyList_.end();
        ++bodyI)
    {
      // Integrate
      (*bodyI)->integrate(timeStep_);
    }
  }


}
