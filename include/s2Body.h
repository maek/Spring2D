#ifndef __BODY_H__
#define __BODY_H__

#include "s2Settings.h"
#include "s2Vector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The body entity
  class Body
  {
    public:

      // TODO: constructor with position


      // Set the body position
      void setPosition (const Vector& POSITION)
      {
        position_ = POSITION;
      }
      void setPosition (Real X, Real Y)
      {
        position_.x = X;
        position_.y = Y;
      }


      // Get the body position
      Vector getPosition () const
      {
        return position_;
      }


    private:

      Vector position_;

  };


}


#endif // __BODY_H__
