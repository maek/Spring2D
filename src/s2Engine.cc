#include "../include/s2Engine.h"

namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Initialize the physics engine
  bool Engine::start ()
  {
    if (running_ == true)
    {
      return false;
    }


    running_ = true;

    return true;
  }


  // ---------------------------------------------------------------------------
  // Run a step
  bool Engine::runStep ()
  {
    if (running_ == false)
    {
      return false;
    }

    return true;
  }


  // ---------------------------------------------------------------------------
  // Destroy the physics engine
  bool Engine::turnOff ()
  {
    if (running_ == false)
    {
      return false;
    }


    running_ = false;

    return true;
  }
}
