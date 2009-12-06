// bool isZero () const
// Vector2 normalizedCopy () const
// Vector2 perpendicular (const Vector2& V)


#ifndef __VECTOR2_H__
#define __VECTOR2_H__

#include "s2Settings.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Class for the 2D vectors
  class Vector2
  {
    public:

      Real x;
      Real y;


    public:

      // Canonical vectors
      static const Vector2 ZERO;
      static const Vector2 XY;
      static const Vector2 X_VERSOR;
      static const Vector2 Y_VERSOR;


    public:

      // Constructor
      explicit Vector2 (const Real X = 0, const Real Y = 0) : x(X), y(Y) { }


      // Assignment
      Vector2& operator= (const Vector2& V)
      {
        x = V.x;
        y = V.y;

        return *this;
      }


      // Update addition
      Vector2& operator+= (const Vector2& V)
      {
        x += V.x;
        y += V.y;

        return *this;
      }

      // Update subtraction
      Vector2& operator-= (const Vector2& V)
      {
        x -= V.x;
        y -= V.y;

        return *this;
      }

      // Update scale
      Vector2& operator*= (const Real R)
      {
        x *= R;
        y *= R;

        return *this;
      }


      // Magnitude (length)
      Real magnitude () const
      {
        return s2hypot(x, y);
      }

      // Squared Magnitude
      Real squaredMagnitude () const
      {
        return (x * x + y * y);
      }


      // Normalize
      Vector2& normalize ()
      {
        Real m = s2hypot(x, y);

        if (m > 0)
        {
          x /= m;
          y /= m;
        }

        return *this;
      }


  };



  // ---------------------------------------------------------------------------
  // Equal
  inline bool operator== (const Vector2& V1, const Vector2& V2)
  {
    return (V1.x == V2.x && V1.y == V2.y);
  }

  // ---------------------------------------------------------------------------
  // Not Equal
  inline bool operator!= (const Vector2& V1, const Vector2& V2)
  {
    return (V1.x != V2.x || V1.y != V2.y);
  }

  // ---------------------------------------------------------------------------
  // Print
  inline std::ostream& operator<< (std::ostream& os, const Vector2& V)
  {
    return (os << "[" << V.x << ", " << V.y << "]");
  }


  // ---------------------------------------------------------------------------
  // Inversion (unary minus)
  inline Vector2 operator- (const Vector2& V)
  {
    return Vector2(
        -V.x,
        -V.y);
  }


  // ---------------------------------------------------------------------------
  // Addition
  inline Vector2 operator+ (const Vector2& V1, const Vector2& V2)
  {
    return Vector2(
        V1.x + V2.x,
        V1.y + V2.y);
  }

  // ---------------------------------------------------------------------------
  // Subtraction
  inline Vector2 operator- (const Vector2& V1, const Vector2& V2)
  {
    return Vector2(
        V1.x - V2.x,
        V1.y - V2.y);
  }

  // ---------------------------------------------------------------------------
  // Scaling
  inline Vector2 operator* (const Vector2& V, const Real R)
  {
    return Vector2(
        V.x * R,
        V.y * R);
  }
  inline Vector2 operator* (const Real R, const Vector2& V)
  {
    return Vector2(
        V.x * R,
        V.y * R);
  }


  // ---------------------------------------------------------------------------
  // Dot (scalar) product
  inline Real dotProduct (const Vector2& V1, const Vector2& V2)
  {
    return (V1.x * V2.x + V1.y * V2.y);
  }

  // ---------------------------------------------------------------------------
  // Cross (vector) product
  inline Real crossProduct (const Vector2& V1, const Vector2& V2)
  {
    return (V1.x * V2.y - V1.y * V2.x);
  }

}


#endif // __VECTOR2_H__
