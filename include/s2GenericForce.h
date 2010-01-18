#ifndef __GENERIC_FORCE_H__
#define __GENERIC_FORCE_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // A generic force
  class GenericForce : public Force
  {
    public:

      // Constructor
      GenericForce (const Vector2& FORCE) : force_(FORCE) { }


      // Apply the force
      void apply (Body* BODY) const
      {
        if (BODY->isStatic())
        {
          return;
        }

        BODY->applyForce(force_);
      }


    private:

      Vector2 force_;

  };


}


#endif // __GENERIC_FORCE_H__
