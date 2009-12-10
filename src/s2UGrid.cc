#include "../include/s2UGrid.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Clear the grid
  void UGrid::clear ()
  {
    // TODO: OPTIMIZATION -> dense bit array for the cells with bodies inside
    for (unsigned i = 0; i < xyExtent_; ++i)
    {
      cellList_[i].clear();
    }

  }



  // ---------------------------------------------------------------------------
  // Test for coarse collisions
  void UGrid::testBody (Shape* shape)
  {
    int index;

    // Update the AABB
    shape->updateAABB();

    // Found the min & max of the AABB
    Vector2 position = *shape->getAABB()->center_;
    Vector2 halfSize =  shape->getAABB()->halfSize_;
    Vector2 min = (position - halfSize - origin_) * (1.0 / cellSize_);
    Vector2 max = (position + halfSize - origin_) * (1.0 / cellSize_);

    // Insert the body into the grid & check for collisions
    for (int x = static_cast<int>(min.x); x <= static_cast<int>(max.x); ++x)
    {
      for (int y = static_cast<int>(min.y); y <= static_cast<int>(max.y); ++y)
      {
        // Check for valid grid positions
        index = static_cast<int>(x + y * xExtent_);
        if (index >= 0)
        {
          // Check for collisions in the grid
          for (CellList::reverse_iterator shapeI = cellList_[index].rbegin();
              shapeI != cellList_[index].rend(); ++shapeI)
          {
            // Check the AABB for the collision
            if (checkAABB(shape->getAABB(), (*shapeI)->getAABB()) == true)
            {
              // TODO: TESTING
              // TODO: check for already existing contact &
              //       send only one per couple
              std::cout << "collision [AABB]\n";
            }
          }

          // Insert the body
          cellList_[index].push_back(shape);
        }
      }
    }

  }


}
