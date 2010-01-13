#ifndef __MATH_H__
#define __MATH_H__

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
      static const Vector2 X;
      static const Vector2 Y;
      static const Vector2 XY;


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


      // Check if is ZERO
      bool isZero () const
      {
        return (
            s2fabs(x) <= EPSILON_ABS &&
            s2fabs(y) <= EPSILON_ABS);
      }

      // Check if is not ZERO
      bool isNotZero () const
      {
        return (
            s2fabs(x) > EPSILON_ABS ||
            s2fabs(y) > EPSILON_ABS);
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
      Real getMagnitude () const
      {
        return s2hypot(x, y);
      }

      // Squared Magnitude
      Real getSquaredMagnitude () const
      {
        return (x * x + y * y);
      }


      // Normalize
      Vector2& normalize ()
      {
        Real m = this->getMagnitude();

        if (m > 0)
        {
          x /= m;
          y /= m;
        }

        return *this;
      }

      // Get a normalized copy of the vector
      Vector2 getNormalizedCopy () const
      {
        Vector2 tthis = *this;
        return tthis.normalize();
      }


      // Perpendicularize
      Vector2& perpendicularize ()
      {
        Real t = x;
        x = -y;
        y = t;
        return *this;
      }

      // Get a perpendicular copy of the vector
      Vector2 getPerpendicularCopy () const
      {
        Vector2 tthis = *this;
        tthis.x = -y;
        tthis.y = x;
        return tthis;
      }
  };



  // ---------------------------------------------------------------------------
  // Equal [Vector2]
  inline bool operator== (const Vector2& V1, const Vector2& V2)
  {
    return (
        s2fabs(V1.x - V2.x) <= EPSILON_ABS &&
        s2fabs(V1.y - V2.y) <= EPSILON_ABS);
  }

  // ---------------------------------------------------------------------------
  // Not Equal [Vector2]
  inline bool operator!= (const Vector2& V1, const Vector2& V2)
  {
    return (
        s2fabs(V1.x - V2.x) > EPSILON_ABS ||
        s2fabs(V1.y - V2.y) > EPSILON_ABS);
  }


  // ---------------------------------------------------------------------------
  // Print [Vector2]
  inline std::ostream& operator<< (std::ostream& os, const Vector2& V)
  {
    return (os << "[" << V.x << ", " << V.y << "]");
  }


  // ---------------------------------------------------------------------------
  // Inversion (unary minus) [Vector2]
  inline Vector2 operator- (const Vector2& V)
  {
    return Vector2(
        -V.x,
        -V.y);
  }


  // ---------------------------------------------------------------------------
  // Addition [Vector2]
  inline Vector2 operator+ (const Vector2& V1, const Vector2& V2)
  {
    return Vector2(
        V1.x + V2.x,
        V1.y + V2.y);
  }

  // ---------------------------------------------------------------------------
  // Subtraction [Vector2]
  inline Vector2 operator- (const Vector2& V1, const Vector2& V2)
  {
    return Vector2(
        V1.x - V2.x,
        V1.y - V2.y);
  }

  // ---------------------------------------------------------------------------
  // Scaling [Vector2]
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
  // Dot (scalar) product [Vector2]
  inline Real dot (const Vector2& V1, const Vector2& V2)
  {
    return (V1.x * V2.x + V1.y * V2.y);
  }

  // ---------------------------------------------------------------------------
  // Cross (vector) product [Vector2]
  inline Real cross (const Vector2& V1, const Vector2& V2)
  {
    return (V1.x * V2.y - V1.y * V2.x);
  }





  // ---------------------------------------------------------------------------
  // Class for the complex
  class Complex
  {
    public:

      Real r;
      Real i;


    public:

      // Canonical orientations
      static const Complex ZERO;
      static const Complex PI_4;
      static const Complex PI_2;
      static const Complex PI;


    public:

      // Constructor
      explicit Complex (const Real R, const Real I) : r(R), i(I) { }

      explicit Complex (const Real ANGLE = 0)
      {
        r = s2cos(ANGLE);
        i = s2sin(ANGLE);
      }


      // Assignment
      Complex& operator= (const Complex& C)
      {
        r = C.r;
        i = C.i;

        return *this;
      }


      // Update multiplication
      Complex& operator*= (const Complex& C)
      {
        // (a + bi) * (c + di) = (a * c - b * d) + (a * d + b * c)i
        Real tr = r;
        Real ti = i;

        r = tr * C.r - ti * C.i;
        i = tr * C.i + ti * C.r;

        return *this;
      }


      // Rotate
      Complex& rotate (const Real R)
      {
        // (a + bi) * (c + di) = (a * c - b * d) + (a * d + b * c)i
        Real c = s2cos(R);
        Real d = s2sin(R);
        Real tr = r;
        Real ti = i;

        r = tr * c - ti * d;
        i = tr * d + ti * c;

        return *this;
      }


      // Magnitude (length)
      // TODO: does it need ?
      Real getMagnitude () const
      {
        return s2hypot(r, i);
      }


      // Normalize
      Complex& normalize ()
      {
        Real m = s2hypot(r, i);

        if (m > 0)
        {
          r /= m;
          i /= m;
        }

        return *this;
      }


  };



  // ---------------------------------------------------------------------------
  // Equal [Complex]
  inline bool operator== (const Complex& C1, const Complex& C2)
  {
    return (
        C1.r == C2.r && C1.i == C2.i);
  }

  // ---------------------------------------------------------------------------
  // Not Equal [Complex]
  inline bool operator!= (const Complex& C1, const Complex& C2)
  {
    return (C1.r != C2.r || C1.i != C2.i);
  }

  // ---------------------------------------------------------------------------
  // Print [Complex]
  inline std::ostream& operator<< (std::ostream& os, const Complex& C)
  {
    return (os << "(" << C.r << " + " << C.i << "i)");
  }


  // ---------------------------------------------------------------------------
  // Multiplication [Complex]
  inline Complex operator* (const Complex& C1, const Complex& C2)
  {
    // (a + bi) * (c + di) = (a * c - b * d) + (a * d + b * c)i
    return Complex(
        C1.r * C2.r - C1.i * C2.i,
        C1.r * C2.i + C1.i * C2.r);
  }





  // ---------------------------------------------------------------------------
  // Class for the 2x2 matrices
  class Matrix2x2
  {
    public:

      Real entry[4];


    public:

      // Canonical matrices
      //static const Matrix2x2 ZERO;
      //static const Matrix2x2 IDENTITY;


    public:

      // Constructor (identity matrix)
      explicit Matrix2x2 (
          const Real E0 = 1,
          const Real E1 = 0,
          const Real E2 = 0,
          const Real E3 = 1)
      {
        entry[0] = E0;
        entry[1] = E1;
        entry[2] = E2;
        entry[3] = E3;
      }

      // Constructor (complex)
      explicit Matrix2x2 (const Complex& COMPLEX)
      {
        entry[0] =  COMPLEX.r;
        entry[1] = -COMPLEX.i;
        entry[2] =  COMPLEX.i;
        entry[3] =  COMPLEX.r;
      }


      // Assignment
      Matrix2x2& operator= (const Matrix2x2& M)
      {
        entry[0] = M.entry[0];
        entry[1] = M.entry[1];
        entry[2] = M.entry[2];
        entry[3] = M.entry[3];

        return *this;
      }


      // Update multiplication
      Matrix2x2& operator*= (const Matrix2x2& M)
      {
        Real tentry[4] = {
          entry[0] * M.entry[0] + entry[1] * M.entry[2],
          entry[0] * M.entry[1] + entry[1] * M.entry[3],
          entry[2] * M.entry[0] + entry[3] * M.entry[2],
          entry[2] * M.entry[1] + entry[3] * M.entry[3]};

        entry[0] = tentry[0];
        entry[1] = tentry[1];
        entry[2] = tentry[2];
        entry[3] = tentry[3];

        return *this;
      }


      // Return the inverse [transpose for rotations]
      Matrix2x2 getInverse ()
      {
        return Matrix2x2(
            entry[0],
            -entry[1],
            -entry[2],
            entry[3]);
      }

  };



  // ---------------------------------------------------------------------------
  // Equal [Matrix2x2]
  inline bool operator== (const Matrix2x2& M1, const Matrix2x2& M2)
  {
    return (
        M1.entry[0] == M2.entry[0] &&
        M1.entry[1] == M2.entry[1] &&
        M1.entry[2] == M2.entry[2] &&
        M1.entry[3] == M2.entry[3]);
  }

  // ---------------------------------------------------------------------------
  // Not Equal [Matrix2x2]
  inline bool operator!= (const Matrix2x2& M1, const Matrix2x2& M2)
  {
    return (
        M1.entry[0] != M2.entry[0] ||
        M1.entry[1] != M2.entry[1] ||
        M1.entry[2] != M2.entry[2] ||
        M1.entry[3] != M2.entry[3]);
  }


  // ---------------------------------------------------------------------------
  // Print [Matrix2x2]
  inline std::ostream& operator<< (std::ostream& os, const Matrix2x2& M)
  {
    return (
        os <<
        M.entry[0] << "\t" << M.entry[1] << "\n" <<
        M.entry[2] << "\t" << M.entry[3]);
  }


  // ---------------------------------------------------------------------------
  // Inversion (unary minus) [Matrix2x2]
  inline Matrix2x2 operator- (const Matrix2x2& M)
  {
    return Matrix2x2(
        -M.entry[0],
        -M.entry[1],
        -M.entry[2],
        -M.entry[3]);
  }


  // ---------------------------------------------------------------------------
  // Multiplication [Matrix2x2]
  inline Matrix2x2 operator* (const Matrix2x2& M1, const Matrix2x2& M2)
  {
    return Matrix2x2(
        M1.entry[0] * M2.entry[0] + M1.entry[1] * M2.entry[2],
        M1.entry[0] * M2.entry[1] + M1.entry[1] * M2.entry[3],
        M1.entry[2] * M2.entry[0] + M1.entry[3] * M2.entry[2],
        M1.entry[2] * M2.entry[1] + M1.entry[3] * M2.entry[3]);
  }





  // ---------------------------------------------------------------------------
  // Multiplication [Matrix2x2 & Vector2]
  inline Vector2 operator* (const Matrix2x2& M, const Vector2& V)
  {
    return Vector2(
        M.entry[0] * V.x + M.entry[1] * V.y,
        M.entry[2] * V.x + M.entry[3] * V.y);
  }


}


#endif // __MATH_H__
