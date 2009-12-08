#include "../include/s2UGrid.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Clear the grid
  void UGrid::clear ()
  {
    for (int i = 0; i < xyExtent_; ++i)
    {
      cellList_[i].clear();
    }

  }



  // ---------------------------------------------------------------------------
  // Test for coarse collisions
  // TODO: origin ???
  void UGrid::testBody (Shape* shape)
  {
#if 0
    std::cout << primitive->getBody()->getPosition() << "\n";
    const Vector2 CENTER = primitive->getBody()->getPosition();
    const Real    RADIUS = primitive->getRadius();
    Vector2 cposition;
    Vector2 tposition;


    // Add the body to the grid [CENTER]
    cposition   = CENTER * (1.0 / cellSize_);
    cposition.x = static_cast<unsigned>(cposition.x);
    cposition.y = static_cast<unsigned>(cposition.y);
    cellList_[static_cast<unsigned>(cposition.x + cposition.y * xExtent_)].
      push_back(primitive);
    std::cout << "[" <<
      static_cast<unsigned>(cposition.x) << ", " <<
      static_cast<unsigned>(cposition.y) << "] (center)" << "\n";
#endif


    // TODO: found all other cells (ALL)
    // TODO: fix the diag cells (radius)



#if 0
    // Test for the RIGHT
    tposition   = (CENTER + Vector2(RADIUS, 0)) * (1.0 / cellSize_);
    tposition.x = static_cast<unsigned>(tposition.x);
    tposition.y = static_cast<unsigned>(tposition.y);
    if (cposition != tposition)
    {
      // Add the body to the grid
      cellList_[static_cast<unsigned>(tposition.x + tposition.y * xExtent_)].
        push_back(primitive);
      std::cout << "[" <<
        static_cast<unsigned>(tposition.x) << ", " <<
        static_cast<unsigned>(tposition.y) << "] (right)" << "\n";
    }


    // Test for the TOP-RIGHT
    tposition = (CENTER + Vector2(RADIUS, RADIUS)) * (1.0 / cellSize_);
    tposition.x = static_cast<unsigned>(tposition.x);
    tposition.y = static_cast<unsigned>(tposition.y);
    if (cposition.x != tposition.x && cposition.y != tposition.y)
    {
      // Add the body to the grid
      cellList_[static_cast<unsigned>(tposition.x + tposition.y * xExtent_)].
        push_back(primitive);
      std::cout << "[" <<
        static_cast<unsigned>(tposition.x) << ", " <<
        static_cast<unsigned>(tposition.y) << "] (top-right)" << "\n";
    }


    // Test for the TOP
    tposition = (CENTER + Vector2(0, RADIUS)) * (1.0 / cellSize_);
    tposition.x = static_cast<unsigned>(tposition.x);
    tposition.y = static_cast<unsigned>(tposition.y);
    if (cposition != tposition)
    {
      // Add the body to the grid
      cellList_[static_cast<unsigned>(tposition.x + tposition.y * xExtent_)].
        push_back(primitive);
      std::cout << "[" <<
        static_cast<unsigned>(tposition.x) << ", " <<
        static_cast<unsigned>(tposition.y) << "] (top)" << "\n";
    }


    // Test for the TOP-LEFT
    tposition = (CENTER + Vector2(-RADIUS, RADIUS)) * (1.0 / cellSize_);
    tposition.x = static_cast<unsigned>(tposition.x);
    tposition.y = static_cast<unsigned>(tposition.y);
    if (cposition.x != tposition.x && cposition.y != tposition.y)
    {
      // Add the body to the grid
      cellList_[static_cast<unsigned>(tposition.x + tposition.y * xExtent_)].
        push_back(primitive);
      std::cout << "[" <<
        static_cast<unsigned>(tposition.x) << ", " <<
        static_cast<unsigned>(tposition.y) << "] (top-left)" << "\n";
    }


    // Test for the LEFT
    tposition = (CENTER + Vector2(-RADIUS, 0)) * (1.0 / cellSize_);
    tposition.x = static_cast<unsigned>(tposition.x);
    tposition.y = static_cast<unsigned>(tposition.y);
    if (cposition != tposition)
    {
      // Add the body to the grid
      cellList_[static_cast<unsigned>(tposition.x + tposition.y * xExtent_)].
        push_back(primitive);
      std::cout << "[" <<
        static_cast<unsigned>(tposition.x) << ", " <<
        static_cast<unsigned>(tposition.y) << "] (left)" << "\n";
    }


    // Test for the BOTTOM-LEFT
    tposition = (CENTER + Vector2(-RADIUS, -RADIUS)) * (1.0 / cellSize_);
    tposition.x = static_cast<unsigned>(tposition.x);
    tposition.y = static_cast<unsigned>(tposition.y);
    if (cposition.x != tposition.x && cposition.y != tposition.y)
    {
      // Add the body to the grid
      cellList_[static_cast<unsigned>(tposition.x + tposition.y * xExtent_)].
        push_back(primitive);
      std::cout << "[" <<
        static_cast<unsigned>(tposition.x) << ", " <<
        static_cast<unsigned>(tposition.y) << "] (bottom-left)" << "\n";
    }


    // Test for the BOTTOM
    tposition = (CENTER + Vector2(0, -RADIUS)) * (1.0 / cellSize_);
    tposition.x = static_cast<unsigned>(tposition.x);
    tposition.y = static_cast<unsigned>(tposition.y);
    if (cposition != tposition)
    {
      // Add the body to the grid
      cellList_[static_cast<unsigned>(tposition.x + tposition.y * xExtent_)].
        push_back(primitive);
      std::cout << "[" <<
        static_cast<unsigned>(tposition.x) << ", " <<
        static_cast<unsigned>(tposition.y) << "] (bottom)" << "\n";
    }


    // Test for the BOTTOM-RIGHT
    tposition = (CENTER + Vector2(RADIUS, -RADIUS)) * (1.0 / cellSize_);
    tposition.x = static_cast<unsigned>(tposition.x);
    tposition.y = static_cast<unsigned>(tposition.y);
    if (cposition.x != tposition.x && cposition.y != tposition.y)
    {
      // Add the body to the grid
      cellList_[static_cast<unsigned>(tposition.x + tposition.y * xExtent_)].
        push_back(primitive);
      std::cout << "[" <<
        static_cast<unsigned>(tposition.x) << ", " <<
        static_cast<unsigned>(tposition.y) << "] (bottom-right)" << "\n";
    }
#endif



    // TODO: reverse iterator
  }


}
