#ifndef __SPRING_FORCE_H__
#define __SPRING_FORCE_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The endpoints data
  class SpringEnds
  {
    public:

      Body*     body[2];

      Vector2   point[2];


    public:

      // Constructor
      SpringEnds (
          Body* BODY1,
          Body* BODY2,
          const Vector2& POINT1,
          const Vector2& POINT2)
      {
        body[0]   = BODY1;
        body[1]   = BODY2;
        point[0]  = POINT1;
        point[1]  = POINT2;
      }


      // Check if this is the given spring
      bool hasBodies (Body* BODY1, Body* BODY2)
      {
        if ((BODY1 == body[0] && BODY2 == body[1]) ||
            (BODY1 == body[1] && BODY2 == body[0]))
        {
          return true;
        }

        return false;
      }

  };



  // ---------------------------------------------------------------------------
  // The spring couple of body list
  typedef std::list<SpringEnds*> SpringList;





  // ---------------------------------------------------------------------------
  // The spring force
  class SpringForce : public DynamicEntry
  {
    public:

      // Constructor
      SpringForce (
          const Real REST_LENGTH,
          const Real STIFFNESS = 1,
          const Real DAMP = 0,
          const bool BUNGEE = false)
        : restLength_(REST_LENGTH), stiffness_(STIFFNESS), damp_(DAMP),
          bungee_(BUNGEE)
      {
        assert(REST_LENGTH >= 0);
        assert(STIFFNESS >= 0);
        assert(0 <= DAMP && DAMP <= 1);
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


      // Add the given body (with points)
      bool addEndBodies (
          Body* BODY1,
          Body* BODY2,
          const Vector2& POINT1 = Vector2::ZERO,
          const Vector2& POINT2 = Vector2::ZERO)
      {
        for (SpringList::iterator springListI = springList_.begin();
            springListI != springList_.end(); ++springListI)
        {
          // If the couple is already in the spring list
          if ((*springListI)->hasBodies(BODY1, BODY2))
          {
            return false;
          }
        }

        // If the couple is a new one
        springList_.push_back(new SpringEnds(BODY1, BODY2, POINT1, POINT2));
        return true;
      }


      // Remove the given body
      bool removeEndBodies (Body* BODY1, Body* BODY2)
      {
        for (SpringList::iterator springListI = springList_.begin();
            springListI != springList_.end(); ++springListI)
        {
          // If the couple is in the spring list
          if ((*springListI)->hasBodies(BODY1, BODY2))
          {
            springList_.erase(springListI);
            return true;
          }
        }

        // If the couple is not in the spring list
        return false;
      }


      // Apply the force
      void apply () const
      {
        Vector2 force;
        Vector2 distance;

        for (SpringList::const_iterator springListI = springList_.begin();
            springListI != springList_.end(); ++springListI)
        {
          distance = (*springListI)->body[1]->getPosition() -
                     (*springListI)->body[0]->getPosition();

          // Skip the bungee compression phase
          if (bungee_ && distance.getMagnitude() <= restLength_)
          {
            continue;
          }

          // Calculate the spring force
          force = stiffness_ * (distance.getMagnitude() - restLength_) *
            distance.getNormalizedCopy();



          // Apply it to the first body
          if ((*springListI)->body[0]->isStatic() == false)
          {
            // Damp decreases force (accelerating)
            if (dot((*springListI)->body[0]->getVelocity(), force) >= 0)
            {
              (*springListI)->body[0]->addForceAtPoint(force * (1 - damp_),
                  (*springListI)->point[0]);
            }
            // Damp increases force (decelerating)
            else
            {
              (*springListI)->body[0]->addForceAtPoint(force * (1 + damp_),
                  (*springListI)->point[0]);
            }
          }

          // Apply it to the second body
          if ((*springListI)->body[1]->isStatic() == false)
          {
            // Damp decreases force (accelerating)
            if (dot((*springListI)->body[1]->getVelocity(), -force) >= 0)
            {
              (*springListI)->body[1]->addForceAtPoint(-force * (1 - damp_),
                  (*springListI)->point[1]);
            }
            // Damp increases force (decelerating)
            else
            {
              (*springListI)->body[1]->addForceAtPoint(-force * (1 + damp_),
                  (*springListI)->point[1]);
            }
          }

        }

      }


      // TODO: remove or make const
      SpringList* getSpringList ()
      {
        return &springList_;
      }


    private:

      Real restLength_;

      Real stiffness_;

      Real damp_;


      bool bungee_;


      SpringList springList_;

  };


}


#endif // __SPRING_FORCE_H__
