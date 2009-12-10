#ifndef __U_GRID_H__
#define __U_GRID_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2Shape.h"
#include "s2AABB.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The uniform grid
  class UGrid
  {
    public:

      typedef std::list<Shape*> CellList;


    public:

      // Constructor
      UGrid (
          const unsigned CELL_SIZE,
          const unsigned X_EXTENT,
          const unsigned Y_EXTENT,
          const Vector2& ORIGIN = Vector2::ZERO) :
        cellSize_(CELL_SIZE),
        xExtent_(X_EXTENT),
        yExtent_(Y_EXTENT),
        xyExtent_(X_EXTENT * Y_EXTENT),
        origin_(ORIGIN)
      {
        cellList_ = new CellList[xyExtent_];
      }

      // Destructor
      ~UGrid ()
      {
        delete[] cellList_;
      }


      void clear ();

      void testBody (Shape*);


      // TODO: TESTING
      CellList* getCells () const
      {
        return cellList_;
      }


    private:

      CellList   *cellList_;


      unsigned    cellSize_;

      unsigned    xExtent_;

      unsigned    yExtent_;

      unsigned    xyExtent_;

      Vector2     origin_;

  };


}


#endif // __U_GRID_H__
