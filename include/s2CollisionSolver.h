#ifndef __COLLISION_SOLVER_H__
#define __COLLISION_SOLVER_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Contact.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The collisions solver
  class CollisionSolver
  {
    public:

      void solveCollisions (ContactList*);

      void solveConstraints (ContactList*);


    private:

      void preprocessContacts (ContactList*);

      void preprocessConstraintsContacts (ContactList*);


      void solveInterpenetration (Contact*);

      void solveVelocity (Contact*);

  };



  bool interpenetrationCompare (Contact*, Contact*);

  bool velocityCompare (Contact*, Contact*);


}


#endif // __COLLISION_SOLVER_H__
