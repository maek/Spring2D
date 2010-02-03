#ifndef __AABR_H__
#define __AABR_H__

#include "s2Settings.h"
#include "s2Math.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The AABR collision primitive
  class AABR
  {
    public:

      Vector2 min;
      Vector2 max;

  };



  // ---------------------------------------------------------------------------
  // Check two AABRs for collision
  inline bool testAABRAABR (const AABR* aabr1, const AABR* aabr2)
  {
    // Test X before Y because it can be more discriminatory
    if (aabr1->max.x < aabr2->min.x || aabr1->min.x > aabr2->max.x)
    {
      return false;
    }
    if (aabr1->max.y < aabr2->min.y || aabr1->min.y > aabr2->max.y)
    {
      return false;
    }

    return true;
  }


}


#endif // __AABR_H__
