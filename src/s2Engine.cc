#include "../include/s2Engine.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Get the environment
  Environment* Engine::getEnvironment ()
  {
    return environment_;
  }



  // ---------------------------------------------------------------------------
  // Run a complete physics step
  void Engine::runStep()
  {
    integrator_->compute(environment_);
  }
}
