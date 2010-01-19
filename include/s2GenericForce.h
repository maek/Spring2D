#ifndef __GENERIC_FORCE_H__
#define __GENERIC_FORCE_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // A generic force
  class GenericForce : public Force
  {
    public:

      // Constructor
      GenericForce (const Vector2& FORCE) : force_(FORCE) { }

      // Destructor
      ~GenericForce ()
      {
        bodyList_.clear();
      }


      // Add the given body
      bool addBody (Body* BODY)
      {
        for (BodyList::iterator bodyListI = bodyList_.begin();
            bodyListI != bodyList_.end(); ++bodyListI)
        {
          if ((*bodyListI) == BODY)
          {
            return false;
          }
        }

        bodyList_.push_back(BODY);
        return true;
      }


      // Remove the given body
      bool removeBody (Body* BODY)
      {
        for (BodyList::iterator bodyListI = bodyList_.begin();
            bodyListI != bodyList_.end(); ++bodyListI)
        {
          if ((*bodyListI) == BODY)
          {
            bodyList_.erase(bodyListI);
            return true;
          }
        }

        return false;
      }


      // Apply the force
      void apply () const
      {
        for (BodyList::const_iterator bodyListI = bodyList_.begin();
            bodyListI != bodyList_.end(); ++bodyListI)
        {
          if ((*bodyListI)->isStatic())
          {
            continue;
          }

          (*bodyListI)->addForce(force_);
        }
      }


    private:

      Vector2 force_;


      BodyList bodyList_;

  };


}


#endif // __GENERIC_FORCE_H__
