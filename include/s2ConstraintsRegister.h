#ifndef __CONSTRAINTS_REGISTER_H__
#define __CONSTRAINTS_REGISTER_H__

#include "s2Settings.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // A constraint
  class Constraint
  {
    public:

      Body* body[2];

      Vector2 point[2];


      Real length;

      bool cable;


    public:

      // Constructor
      Constraint (
          Body* BODY1, const Vector2& POINT1,
          Body* BODY2, const Vector2& POINT2,
          const Real LENGTH,
          const bool CABLE)
        : length(LENGTH), cable(CABLE)
      {
        body[0]  = BODY1;
        body[1]  = BODY2;
        point[0] = POINT1;
        point[1] = POINT2;
      }
  };



  // ---------------------------------------------------------------------------
  // The constraint list
  typedef std::list<Constraint*> ConstraintsList;





  // ---------------------------------------------------------------------------
  // The constraints register
  class ConstraintsRegister
  {
    public:

      // Destructor
      ~ConstraintsRegister ()
      {
        constraintsList_.clear();
      }


      // Register the given constraint
      bool registerConstraint (Constraint* CONSTRAINT)
      {
        constraintsList_.push_back(CONSTRAINT);
        return true;
      }


      // Unregister the given constraint
      bool unregisterConstraint (Constraint* CONSTRAINT)
      {
        for (ConstraintsList::iterator constraintsListI = constraintsList_.begin();
            constraintsListI != constraintsList_.end(); ++constraintsListI)
        {
          // If the constraint is in the register
          if ((*constraintsListI) == CONSTRAINT)
          {
            constraintsList_.erase(constraintsListI);
            return true;
          }
        }

        // If the constraint is not in the register
        return false;
      }


      // Return the constraints list
      ConstraintsList* getConstraints ()
      {
        return &constraintsList_;
      }


    private:

      ConstraintsList constraintsList_;

  };


}


#endif // __CONSTRAINTS_REGISTER_H__
