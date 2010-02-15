/*
 * Copyright (C) 2010   Marco Dalla Via (maek@paranoici.org)
 *
 *  This file is part of Spring2D.
 *
 *  Spring2D is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Spring2D is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Lesser Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser Public License
 *  along with Spring2D. If not, see <http://www.gnu.org/licenses/>.
 */

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
