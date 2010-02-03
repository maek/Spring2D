#ifndef __GRID_H__
#define __GRID_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2Shape.h"
#include "s2AABR.h"
#include "s2BroadPhaseDetector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // A uniform grid for collision detection
  class Grid : public BroadPhaseDetector
  {
    public:

      // Constructor
      Grid (
          const int CELL_SIZE,
          const int X_EXTENT,
          const int Y_EXTENT,
          const Vector2& ORIGIN = Vector2::ZERO)
        : cellSize_(CELL_SIZE), xExtent_(X_EXTENT), yExtent_(Y_EXTENT),
          size_(X_EXTENT * Y_EXTENT), origin_(ORIGIN)
      {
        cellList_ = new BodyList[size_];
      }

      // Destructor
      ~Grid ()
      {
        delete[] cellList_;
      }


      void findCollisions (const BodyList&, ContactList*);


      // TODO: fix
    private:
    public:

      // TODO: const body vector
      BodyList*   cellList_;


      int         cellSize_;

      int         xExtent_;

      int         yExtent_;

      int         size_;

      Vector2     origin_;

  };


}


#endif // __GRID_H__
