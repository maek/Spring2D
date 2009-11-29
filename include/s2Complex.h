#ifndef __COMPLEX_H__
#define __COMPLEX_H__

#include "s2Settings.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Class for the complex
  class Complex
  {
    public:

      Real r;
      Real i;


    public:

      // Constructor
      explicit Complex (const Real R = 0, const Real I = 0) : r(R), i(I) { }


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
      Complex& operator*= (const Real R)
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
      Real magnitude () const
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
  // Equal
  inline bool operator== (const Complex& C1, const Complex& C2)
  {
    return (C1.r == C2.r && C1.i == C2.i);
  }

  // ---------------------------------------------------------------------------
  // Not Equal
  inline bool operator!= (const Complex& C1, const Complex& C2)
  {
    return (C1.r != C2.r || C1.i != C2.i);
  }

  // ---------------------------------------------------------------------------
  // Print
  inline std::ostream& operator<< (std::ostream& os, const Complex& C)
  {
    return (os << "(" << C.r << " + " << C.i << "i)");
  }


  // ---------------------------------------------------------------------------
  // Multiplication
  inline Complex operator* (const Complex& C1, const Complex& C2)
  {
    // (a + bi) * (c + di) = (a * c - b * d) + (a * d + b * c)i
    return Complex(
        C1.r * C2.r - C1.i * C2.i,
        C1.r * C2.i + C1.i * C2.r);
  }


}


#endif // __COMPLEX_H__