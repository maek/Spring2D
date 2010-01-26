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



  // ---------------------------------------------------------------------------
  // Check two AABBs for collision
  inline bool testAABBAABB (const AABB* aabb1, const AABB* aabb2)
  {
    // Test X before Y because it can be more discriminatory
    if (aabb1->max.x < aabb2->min.x || aabb1->min.x > aabb2->max.x)
    {
      return false;
    }
    if (aabb1->max.y < aabb2->min.y || aabb1->min.y > aabb2->max.y)
    {
      return false;
    }

    return true;
  }


}


#endif // __AABB_H__
