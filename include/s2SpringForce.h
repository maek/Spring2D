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

#ifndef __SPRING_FORCE_H__
#define __SPRING_FORCE_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The spring force
  class SpringForce : public DynamicEntry
  {
    public:

      // Constructor
      SpringForce (
          Body* BODY1,
          const Vector2& POINT1,
          Body* BODY2,
          const Vector2& POINT2,
          const Real REST_LENGTH,
          const Real STIFFNESS,
          const Real DAMP,
          const bool BUNGEE)
        : restLength_(REST_LENGTH), stiffness_(STIFFNESS), damp_(DAMP),
          bungee_(BUNGEE)
      {
        assert(REST_LENGTH >= 0);
        assert(STIFFNESS >= 0);
        assert(0 <= DAMP && DAMP <= 1);

        body_[0]  = BODY1;
        body_[1]  = BODY2;
        point_[0] = POINT1;
        point_[1] = POINT2;
      }


      // Return the rest length of the spring
      Real getRestLength ()
      {
        return restLength_;
      }

      // Check if it is a bungee
      bool isBungee ()
      {
        return bungee_;
      }


      // Apply the force
      void apply () const
      {
        Vector2 distance = body_[1]->getPosition() - body_[0]->getPosition();

        // Skip the bungee compression phase
        if (bungee_ && distance.getMagnitude() <= restLength_)
        {
          return;
        }

        // Calculate the spring force
        Vector2 force = stiffness_ * (distance.getMagnitude() - restLength_) *
          distance.getNormalizedCopy();


        // Apply it to the first body
        if (body_[0]->isStatic() == false)
        {
          // Damp decreases force (accelerating)
          if (dot(body_[0]->getVelocity(), force) >= 0)
          {
            body_[0]->addForceAtPoint(force * (1 - damp_), point_[0]);
          }
          // Damp increases force (decelerating)
          else
          {
            body_[0]->addForceAtPoint(force * (1 + damp_), point_[0]);
          }
        }

        // Apply it to the second body
        if (body_[1]->isStatic() == false)
        {
          // Damp decreases force (accelerating)
          if (dot(body_[1]->getVelocity(), -force) >= 0)
          {
            body_[1]->addForceAtPoint(-force * (1 - damp_), point_[1]);
          }
          // Damp increases force (decelerating)
          else
          {
            body_[1]->addForceAtPoint(-force * (1 + damp_), point_[1]);
          }
        }

      }


      // TODO: remove or make const
      const Body** getBody () const
      {
        return const_cast<const Body**>(body_);
      }
      // TODO: remove or make const
      const Vector2* getPoint () const
      {
        return point_;
      }


    private:

      Body*     body_[2];

      Vector2   point_[2];


      Real      restLength_;

      Real      stiffness_;

      Real      damp_;


      bool      bungee_;

  };


}


#endif // __SPRING_FORCE_H__
