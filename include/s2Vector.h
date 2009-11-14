#ifndef __VECTOR_H__
#define __VECTOR_H__

#include "s2Settings.h"


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

      // Canonical vectors
      static const Vector ZERO;
      static const Vector VERSOR_X;
      static const Vector VERSOR_Y;
      static const Vector NEG_VERSOR_X;
      static const Vector NEG_VERSOR_Y;


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

      // Update scale
      Vector& operator*= (const Real& R)
      {
        x *= R;
        y *= R;

        return *this;
      }

      // Dot (scalar) product
      Real dot (const Vector& V) const
      {
        return (x * V.x + y * V.y);
      }


      // Magnitude (length)
      Real magnitude () const
      {
        return s2Sqrt(x * x + y * y);
      }

      // Squared Magnitude
      Real squaredMagnitude () const
      {
        return (x * x + y * y);
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
  // Inversion (unary minus)
  inline Vector operator- (const Vector& V)
  {
    return Vector(
        -V.x,
        -V.y);
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

  // ---------------------------------------------------------------------------
  // Scaling
  inline Vector operator* (const Vector& V, const Real& R)
  {
    return Vector(
        V.x * R,
        V.y * R);
  }
  inline Vector operator* (const Real& R, const Vector& V)
  {
    return Vector(
        V.x * R,
        V.y * R);
  }


}


#endif // __VECTOR_H__
