#ifndef __AABB_H__
#define __AABB_H__

#include "s2Settings.h"
#include "s2Math.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The AABB collision primitive
  class AABB
  {
    public:

      Vector2 min;
      Vector2 max;

  };



  bool testAABBAABB (AABB*, AABB*);


}


#endif // __AABB_H__
