#ifndef __SHAPE_H__
#define __SHAPE_H__

#include "s2Settings.h"
#include "s2Body.h"

namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The shape of the body
  class Shape
  {
    public:

      // TODO: does it need ???
      friend class Body;


    public:

      // Destructor
      virtual ~Shape () { }

      // Return a pointer to the body
      Body* getBody () const
      {
        return body_;
      }


    protected:

      Body*     body_;

  };


}


#endif // __SHAPE_H__
