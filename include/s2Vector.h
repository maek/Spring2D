#ifndef __VECTOR_H__
#define __VECTOR_H__

#include "s2Common.h"


namespace Spring2D
{

  // ---------------------------------------------------------------------------
  // Class for the 2D vectors
  class Vector
  {
    public:

      Real x;
      Real y;


    public:

      // Constructor
      explicit Vector (const Real X = 0.0, const Real Y = 0.0) : x(X), y(Y) { }


      // Assignment
      Vector& operator= (const Vector& V)
      {
        x = V.x;
        y = V.y;

        return *this;
      }


      // Update addition
      Vector& operator+= (const Vector& V)
      {
        x += V.x;
        y += V.y;

        return *this;
      }

      // Update subtraction
      Vector& operator-= (const Vector& V)
      {
        x -= V.x;
        y -= V.y;

        return *this;
      }

  };



  // ---------------------------------------------------------------------------
  // Equal
  inline bool operator== (const Vector& V1, const Vector& V2)
  {
    return (V1.x == V2.x && V1.y == V2.y); 
  }

  // ---------------------------------------------------------------------------
  // Not Equal
  inline bool operator!= (const Vector& V1, const Vector& V2)
  {
    return (V1.x != V2.x || V1.y != V2.y); 
  }

  // ---------------------------------------------------------------------------
  // Print
  inline std::ostream& operator<< (std::ostream& os, const Vector& V)
  {
    return (os << "[" << V.x << ", " << V.y << "]");
  }


  // ---------------------------------------------------------------------------
  // Addition
  inline Vector operator+ (const Vector& V1, const Vector& V2)
  {
    return Vector(
        V1.x + V2.x,
        V1.y + V2.y); 
  }

  // ---------------------------------------------------------------------------
  // Subtraction
  inline Vector operator- (const Vector& V1, const Vector& V2)
  {
    return Vector(
        V1.x - V2.x,
        V1.y - V2.y); 
  }


}


#endif // __VECTOR_H__
