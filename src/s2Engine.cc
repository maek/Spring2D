#include "../include/s2Engine.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Create a new environment
  Environment* Engine::createEnvironment ()
  {
    environment_ = new Environment();
    return environment_;
  }


}
