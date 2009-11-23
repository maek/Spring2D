#ifndef __FORCE_H__
#define __FORCE_H__

#include "s2Settings.h"
#include "s2Body.h"
#include "s2Vector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The Force interface
  class Force
  {
    public:

      void apply () = 0;


      // Return the involved body
      const Body& getBody () const
      {
        return *body_;
      }


    protected:

      Body* body_;

  };


}


#endif // __FORCE_H__
