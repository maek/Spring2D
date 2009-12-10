#include "../include/s2AABB.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Check two AABBs for collision
  // TODO: OPTIMIZATION -> int instead of Real (faster & lighter)
  bool checkAABB (AABB* aabb1, AABB* aabb2)
  {
    // Test X before Y because it can be more discriminatory
    if (s2fabs(aabb1->center_->x - aabb2->center_->x) >
        (aabb1->halfSize_.x + aabb2->halfSize_.x))
    {
      return false;
    }
    if (s2fabs(aabb1->center_->y - aabb2->center_->y) >
        (aabb1->halfSize_.y + aabb2->halfSize_.y))
    {
      return false;
    }

    return true;
  }


}
