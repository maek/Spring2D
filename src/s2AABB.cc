#include "../include/s2AABB.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Check two AABBs for collision
  bool testAABBAABB (AABB* aabb1, AABB* aabb2)
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
